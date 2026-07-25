// =============================================================================
// 文件名: Corrected.cpp
// 功能描述: 校正数据处理模块
// 系统角色: 本文件负责通道相位校正，消除不同接收通道之间的相位不一致性。
//           由于两个接收通道(端口0和端口1)的硬件特性不完全一致，会引入
//           固有的相位偏差(通道间相位误差)，需要通过校正消除。
//
// 校正原理: 在切刀顺序中，当天线对的两个天线相同时(如{7,7})，两个通道
//           接收的是同一天线的信号(通过功分器分配)，理论上相位差应为0。
//           实际测得的相位差即为通道间的固有相位误差。
//           将后续所有相位差测量值减去该校正值，即可消除通道误差。
//
// 关键功能:
//   1. setCorrectionData - 从同天线功分数据计算校正值并做多星平滑
//   2. calCorrectionData - 逐卫星更新校正数据(选择载噪比高的)
//   3. getCorrectedGnssData - 对所有相位差数据应用校正
//   4. calCorrecteData - 对单个卫星数据减去校正值
// =============================================================================
#include "SpoofingDoa.h"
// 校正数据的相关操作

// =========================================================================
// 设置校正数据(主入口)
// 功能: 从切刀中天线对相同的刀(同天线功分信号)提取校正信息
// 流程:
//   1. 筛选切刀中天线对相同的刀(如{7,7} -> 天线7与自身，即功分器)
//   2. 过滤不可用的校正数据(载噪比极低或两通道载噪比差异>10dB)
//   3. 逐卫星保存校正相位差到m_CorrectionData
//   4. 对每个频点的所有卫星校正数据进行多星平滑，得到综合校正值(prn=-1)
// =========================================================================
void SpoofingDoa::setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA)
{
    int size = (int)dataA.size();
    // 数据长度必须与切刀序列一致
    if (size != (int)m_cutSequence.size())
    {
        return;
    }
    SatelliteDataPhaseDiffA tp_dataA;
    for (int i = 0; i < size; i++)
    {
        // 筛选天线对相同的刀: 同一天线通过功分器分配到两个通道
        if (m_cutSequence[i][0] == m_cutSequence[i][1])
        {
            for (unsigned int j = 0; j < dataA[i].size(); j++)
            {
                tp_dataA = dataA[i][j];
                // 过滤条件:
                //   1. 载噪比极低(未收到信号)
                //   2. 两通道载噪比差异 > 10dB(功分不均匀或通道异常)
                if (tp_dataA.i_Snr1 < 1e-6 || tp_dataA.i_Snr2 < 1e-6 || fabs(tp_dataA.i_Snr1 - tp_dataA.i_Snr2) > 10)
                { // 一般情况下校正数据为同一天线功分得到的，则校正数据的载噪比应该相差不大，若校正数据的载噪比相差大于10dB,则该校正数据不可用
                    continue;
                }
                calCorrectionData(tp_dataA);  // 保存卫星校正数据
            }
        }
    }
    map<int, SatelliteDataPhaseDiffA> tp_prnData;

    int typeInt = 0;

    // 对每个频点进行多星平滑，得到综合校正值
    for (auto it = m_CorrectionData.begin(); it != m_CorrectionData.end(); ++it)
    {
        tp_prnData.clear();
        typeInt = it->first;
        tp_prnData = it->second;
        int count = 0;
        SatelliteDataPhaseDiffB tpB;
        SatelliteDataPhaseDiffA tp;
        tpB.i_Sys = typeInt / 100;
        tpB.i_Type = typeInt % 100;
        tpB.i_Prn = -1;
        // 收集该频点所有卫星的校正数据
        for (auto it2 = tp_prnData.begin(); it2 != tp_prnData.end(); ++it2)
        {
            if (it2->first == -1)
            {
                continue;  // 跳过已存在的综合校正值
            }
            tpB.i_phase_diff[count] = it2->second.i_phase_diff;
            tpB.i_Snr1[count] = it2->second.i_Snr1;
            tpB.i_Snr2[count] = it2->second.i_Snr2;
            ++count;
        }
        tpB.i_diffLen = count;
        calSmoothData(tpB, tp);  // 多星平滑(取相位差一致性最高的一组取均值)

        // 综合校正值存入prn=-1位置
        m_CorrectionData[typeInt][-1] = tp;
    }

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
// 逐卫星更新校正数据
// 功能: 将新的校正数据与已有数据比较，选择载噪比较高的保存
// 原因: 高载噪比数据具有更低的噪声，校正值更可靠
// 策略: 如果新数据的载噪比不低于旧数据+10dB，则更新
//       这里的逻辑考虑了校正数据的跳变现象(载噪比大幅降低时校正数据不可靠)
// =========================================================================
void SpoofingDoa::calCorrectionData(const SatelliteDataPhaseDiffA dataA)
{
    if (dataA.i_Snr1 < 1e-3 || dataA.i_Snr2 < 1e-3)
    {
        return;  // 载噪比无效
    }
    int typeInt = TypeInt(dataA.i_Sys, dataA.i_Type);
    int prn = dataA.i_Prn;
    map<int, SatelliteDataPhaseDiffA> tp;
    if (m_CorrectionData.find(typeInt) == m_CorrectionData.end())
    {
        // 该频点首次出现，直接存入
        tp[prn] = dataA;
        m_CorrectionData[typeInt] = tp;
    }
    else
    {

        tp = m_CorrectionData[typeInt];
        if (tp.find(prn) == tp.end())
        {
            // 该卫星首次出现，直接存入
            m_CorrectionData[typeInt][prn] = dataA;
        }
        else
        {
            if (tp[prn].i_Snr1 > (dataA.i_Snr1 + 10) && tp[prn].i_Snr2 > (dataA.i_Snr2 + 10))
            // 不同帧之间的校正数据不同，校正数据存在跳变现象，载噪比大幅度降低，校正数据跳变
            // 此时保存旧的(载噪比较高的)校正值
            {
                m_CorrectionData[typeInt][prn] = tp[prn];
            }
            else
            {
                // 更新为新的校正值
                m_CorrectionData[typeInt][prn] = dataA;
            }
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
// 对单个卫星数据进行相位差校正
// 校正策略:
//   1. 如果有该卫星的校正数据 -> 减去该卫星的校正值(逐星精确校正)
//   2. 如果无该卫星但有该频点的综合校正值(prn=-1) -> 减去综合校正值
//   3. 如果两者都没有且当前为检测模式 -> 保持原值
//   4. 如果两者都没有且当前为测向模式 -> 标记为无效(载噪比置0)
// 公式: 校正后相位差 = 原始相位差 - 校正相位差
// =========================================================================
void SpoofingDoa::calCorrecteData(SatelliteDataPhaseDiffA &dataA)
{
    int typeInt = TypeInt(dataA.i_Sys, dataA.i_Type);
    int prn = dataA.i_Prn;
    map<int, SatelliteDataPhaseDiffA> tp_prnData;
    tp_prnData.clear();
    SatelliteDataPhaseDiffA tp;

    double tp_diff = dataA.i_phase_diff;
    // 载噪比无效
    if (dataA.i_Snr1 < 1e-3 || dataA.i_Snr2 < 1e-3)
    {
        return;
    }

    if (m_CorrectionData.find(typeInt) != m_CorrectionData.end()) // 存在这个频点的校正数据
    {

        tp_prnData = m_CorrectionData[typeInt];
        if (tp_prnData.find(prn) == tp_prnData.end()) // 不存在这个卫星的校正数据
        {
            tp = tp_prnData[-1];  // 使用该频点的综合校正值
            dataA.i_phase_diff = tp_diff - tp.i_phase_diff;
        }
        else
        {
            dataA.i_phase_diff = tp_diff - tp_prnData[prn].i_phase_diff;  // 使用逐星精确校正值
        }
    }
    else if (m_Detection_Tag == 1)
    {
        // 检测模式下，没有校正数据也继续处理
        return;
    }
    else
    { // 测向模式下如果不存在这个频点的校正数据，则该频点不进入测向
        dataA.i_Snr1 = 0;
        dataA.i_Snr2 = 0;
    }
}
