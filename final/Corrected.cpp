// =============================================================================
// 文件名: Corrected.cpp
// 功能描述: 校正数据处理模块
// 系统角色: 本文件负责通道相位校正，消除不同接收通道之间的相位不一致性。
//           由于两个接收通道(端口0和端口1)的硬件特性不完全一致，会引入
//           固有的相位偏差(通道间相位误差)，需要通过校正消除。
//
// 校正原理: 在切刀顺序中，当天线对的两个天线相同时(如{1,1})，两个通道
//           接收的是同一天线的信号(通过功分器分配)，理论上相位差应为0。
//           实际测得的相位差即为通道间的固有相位误差。
//           将后续所有相位差测量值减去该校正值，即可消除通道误差。
//
// 关键功能:
//   1. setCorrectionData - 从同天线自校准刀({1,1}=code=0)收集校正数据
//   2. calCorrectionOffset - 计算校正偏移(对应 Python compute_calibration:
//      非 GLONASS 按频点圆形均值统一偏移, GLONASS 逐卫星偏移)
//   3. getCorrectedGnssData - 对所有相位差数据应用校正
//   4. calCorrecteData - 对单个卫星数据减去校正偏移(对应 Python offset_fn)
// =============================================================================
#include "SpoofingDoa.h"
// 校正数据的相关操作

// =========================================================================
// 设置校正数据(主入口)
// 功能: 从同天线自校准刀({1,1} = Python code=0)提取校正信息，
//       计算校正偏移后对所有输入数据应用校正。
// =========================================================================
void SpoofingDoa::setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA)
{
    // 数据长度必须与切刀序列一致
    if (dataA.size() != m_cutSequence.size())
    {
        return;
    }

    // 收集同天线自校准刀的数据(天线对相同的刀，如 {1,1})
    vector<vector<SatelliteDataPhaseDiffA>> calCuts;
    for (unsigned int i = 0; i < dataA.size(); i++)
    {
        if (m_cutSequence[i][0] == m_cutSequence[i][1])
        {
            calCuts.emplace_back(dataA[i]);
        }
    }
    calCorrectionOffset(calCuts);
}

// =========================================================================
// 由同天线校正刀数据计算校正偏移 —— 对应 Python detection_lib.py compute_calibration
// 流程:
//   1. 数据筛选(对应 Python compute_calibration):
//      - 均匀分布:     有效采样数 >= min(校正刀内帧数, MIN_STABLE_SAMPLES)
//        (每刀多帧的稳定性过滤与覆盖判定已由上游 getSmoothData/calSmoothData 完成，
//         不满足的卫星载噪比已置 0，此处按 <1e-3 跳过；
//         不再要求信号从该刀第一帧(第一秒)就存在 —— Python 亦不要求);
//      - 相位差稳定:   最小覆盖弧 < STABILITY_RANGE_DEG(15°);
//   2. 偏移组合:
//      - 非 GLONASS: 每个(系统,频点)取各稳定卫星偏移的圆形均值, 存入 prn=-1;
//      - GLONASS(FDMA): 每颗卫星逐星偏移, 存入对应 prn;
//   3. 仅当本轮 {1,1} 校正刀能算出有效偏移时才重建 m_CorrectionData(与 Python 每周期
//      重算 cal_rx/cal_glo 一致); 无 {1,1} 刀或算不出时保留上一次校正, 沿用其校正。
// =========================================================================
void SpoofingDoa::calCorrectionOffset(const vector<vector<SatelliteDataPhaseDiffA>> &calCuts)
{
    // samples: typeInt -> prn -> [相位差(度), ...]
    map<int, map<int, vector<double>>> samples;  // 相位差样本(度)
    map<int, map<int, double>> sumSnr;           // 载噪比累加(仅用于日志)
    int nCalCuts = 0;

    for (unsigned int c = 0; c < calCuts.size(); ++c)
    {
        ++nCalCuts;
        for (const auto &sat : calCuts[c])
        {
            // 载噪比有效性检查(数据完整性判断，非质量门限)
            // 不满足「从该刀第一帧就存在」的卫星已在上游被置 0，此处自动跳过
            if (sat.i_Snr1 < 1e-3 || sat.i_Snr2 < 1e-3)
            {
                continue;
            }
            int typeInt = TypeInt(sat.i_Sys, sat.i_Type);
            int prn = sat.i_Prn;
            samples[typeInt][prn].emplace_back(sat.i_phase_diff * 360.0);  // 周->度
            sumSnr[typeInt][prn] += (sat.i_Snr1 + sat.i_Snr2) / 2.0;
        }
    }

    // 本轮没有 {1,1} 校正刀数据: 不重建, 沿用上一次校正继续校正
    if (nCalCuts <= 0)
    {
        return;
    }
    int required = (nCalCuts < MIN_STABLE_SAMPLES) ? nCalCuts : MIN_STABLE_SAMPLES;

    // 先写入临时容器, 仅当本轮 {1,1} 刀确实算出有效新校正时才整体替换 m_CorrectionData;
    // 否则(无有效星/全部不稳定)不清空不覆盖, 保留上一次校正值。
    map<int, map<int, SatelliteDataPhaseDiffA>> newCorrection;
    map<int, vector<double>> freqVals;  // 非 GLONASS: typeInt -> [稳定卫星偏移(度)...]

    for (auto &tkv : samples)
    {
        int typeInt = tkv.first;
        int sys = typeInt / 100;
        for (auto &skv : tkv.second)
        {
            int prn = skv.first;
            const vector<double> &samp = skv.second;

            // 1. 相位差稳定(最小覆盖弧 < STABILITY_RANGE_DEG=15°)
            if (circularSpanDeg(samp) >= STABILITY_RANGE_DEG)
            {
                continue;
            }
            // 2. 均匀分布(有效采样覆盖足够帧数)
            if ((int)samp.size() < required)
            {
                continue;
            }

            double meanDeg = circularMeanDeg(samp);
            double meanSnr = sumSnr[typeInt][prn] / samp.size();

            SatelliteDataPhaseDiffA tp;
            tp.i_Sys = sys;
            tp.i_Type = typeInt % 100;
            tp.i_Prn = prn;
            double cyc = fmod(meanDeg / 360.0, 1.0);
            if (cyc < 0)
            {
                cyc += 1.0;  // 归一化到 [0,1) 周
            }
            tp.i_phase_diff = cyc;
            tp.i_Snr1 = meanSnr;
            tp.i_Snr2 = meanSnr;

            if (sys == 1)
            {
                // GLONASS(FDMA): 逐卫星偏移
                newCorrection[typeInt][prn] = tp;
            }
            else
            {
                // 非 GLONASS: 收集用于频点级圆形均值
                freqVals[typeInt].emplace_back(meanDeg);
            }
        }
    }

    // 非 GLONASS: 每(系统,频点)统一偏移 = 稳定卫星偏移的圆形均值
    for (auto &fv : freqVals)
    {
        int typeInt = fv.first;
        double meanDeg = circularMeanDeg(fv.second);
        double cyc = fmod(meanDeg / 360.0, 1.0);
        if (cyc < 0)
        {
            cyc += 1.0;
        }
        SatelliteDataPhaseDiffA tp;
        tp.i_Sys = typeInt / 100;
        tp.i_Type = typeInt % 100;
        tp.i_Prn = -1;
        tp.i_phase_diff = cyc;
        tp.i_Snr1 = 0;
        tp.i_Snr2 = 0;
        newCorrection[typeInt][-1] = tp;
    }

    // {1,1} 校正刀没能算出任何稳定有效偏移: 保留上一次校正(不清空不覆盖), 沿用其校正。
    if (newCorrection.empty())
    {
        return;
    }

    // 整体替换为本轮新校正(与 Python 每周期重算 cal_rx/cal_glo 一致)。
    m_CorrectionData.swap(newCorrection);

    // 日志输出校正数据
    for (auto it = m_CorrectionData.begin(); it != m_CorrectionData.end(); ++it)
    {
        map<int, SatelliteDataPhaseDiffA> tpA;
        tpA = it->second;
        for (auto itt = tpA.begin(); itt != tpA.end(); ++itt)
        {
            SatelliteDataPhaseDiffA tp = itt->second;
            LogSatelliteDataPhaseDiffA(tp);
        }
    }
}

// =========================================================================
// 对所有输入数据进行相位校正
// 遍历所有切刀的所有卫星数据，分别调用calCorrecteData进行校正
// =========================================================================
void SpoofingDoa::getCorrectedGnssData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int size = (int)dataA.size();
    for (int i = 0; i < size; i++)
    {
        for (unsigned int j = 0; j < dataA[i].size(); j++)
        {
            calCorrecteData(dataA[i][j]);
        }
    }
}

// =========================================================================
// 对单个卫星数据进行相位差校正 —— 对应 Python offset_fn
// 校正策略(与 Python compute_calibration / offset_fn 语义一致):
//   - GLONASS(FDMA): 逐卫星偏移(该卫星在校正刀有偏移则减之, 否则偏移0不校正);
//   - 非 GLONASS:    统一频点偏移(prn=-1 综合值, 无则偏移0不校正);
//   无该频点校正数据时: 一律偏移 0（保持原值），对应 Python offset_fn 返回 0；
//   注: 原「测向模式无该频点校正则载噪比置 0 不进入测向」已移除，改为与 Python 一致
//       (仅当某 (系统,频点) 完全无稳定校正星时才按原相位差继续，Python 同样如此)。
// 公式: 校正后相位差 = 原始相位差 - 校正偏移
// =========================================================================
void SpoofingDoa::calCorrecteData(SatelliteDataPhaseDiffA &dataA)
{
    int typeInt = TypeInt(dataA.i_Sys, dataA.i_Type);
    int prn = dataA.i_Prn;

    // 载噪比无效
    if (dataA.i_Snr1 < 1e-3 || dataA.i_Snr2 < 1e-3)
    {
        return;
    }

    auto it = m_CorrectionData.find(typeInt);
    if (it == m_CorrectionData.end())
    {
        return;  // 不存在该频点校正数据：偏移0，保持原值（对齐 Python offset_fn）
    }

    const map<int, SatelliteDataPhaseDiffA> &tp_prnData = it->second;
    const SatelliteDataPhaseDiffA *off = nullptr;
    if (dataA.i_Sys == 1)
    {
        // GLONASS(FDMA): 逐卫星偏移; 无则该星偏移0不校正(对应 Python offset_fn 返回 0)
        auto ps = tp_prnData.find(prn);
        if (ps != tp_prnData.end())
        {
            off = &(ps->second);
        }
    }
    else
    {
        // 非 GLONASS: 统一频点偏移(prn=-1 综合值)
        auto pf = tp_prnData.find(-1);
        if (pf != tp_prnData.end())
        {
            off = &(pf->second);
        }
    }
    if (off != nullptr)
    {
        dataA.i_phase_diff = dataA.i_phase_diff - off->i_phase_diff;
    }
}
