// =============================================================================
// 文件名: AmpPhase.cpp
// 功能描述: 幅相法(Amplitude-Phase)测向模块
// 系统角色: 实现基于仿真阵列流型的幅相法DOA。与相关干涉仪不同，
//           幅相法不仅利用相位差信息，还利用幅度差(载噪比)信息进行测向。
//           仿真阵列流型(SimulateA)通过电磁仿真软件(如HFSS/CST)生成，
//           包含每个角度上的复数导向矢量(幅度和相位响应)。
//
// 核心算法: 将实测幅度和相位与仿真阵列流型进行匹配，计算相关系数，
//           相关性最高的角度即为估计的到达角。
//
// 关键功能:
//   1. getResultAmpPhaseDoa - 幅相法测向主函数(对各卫星逐星计算)
//   2. initSimulateA - 初始化仿真阵列流型模板
// =============================================================================
#include "pch.h"
#include "SpoofingDoa.h"
// 幅相法测向

// =========================================================================
// 幅相法测向主函数
// 算法流程(对每颗卫星):
//   1. 获取该频点的仿真阵列流型(m_SimulateA)
//   2. 构建幅度向量(ampsnr)和相位差向量(phase_diff)
//   3. 填充InterferInfo结构(包含幅度和相位差)
//   4. 调用ArithmeticDoa::calAmpPhase进行幅相法匹配
//   5. 记录匹配结果(角度、质量)到m_AngleResultData
//
// 幅度处理: 载噪比(SNR)通过10^(SNR/20)转换为线性幅度
// 相位处理: 相位差(周)乘以2π转换为弧度
// 参考通道(第0刀): 幅度取平均载噪比，相位差设为0
// =========================================================================
void SpoofingDoa::getResultAmpPhaseDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    // 清空上一轮的结果
    m_AngleResultData.clear();
    int size = dataB.size();
    int typeInt = 0;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    std::vector<std::vector<complex<double>>> tp_SimulateA;
    vector<double> diff;
    SatelliteDataPhaseDiffB tpB;
    int num1 = (int)m_cutSequence.size();
    vector<double> amp_snr;
    amp_snr.resize(num1);
    vector<double> phase_diff;
    phase_diff.resize(num1);
    double tmp1 = 0.0;
    double tmp2 = 0.0;
    InterferInfo data;
    data.i_Start = 0;
    data.i_End = 359;  // 全方向搜索
    map<int, vector<double>> tp_peseudo;

    // 设置天线对序列(与切刀顺序一致)
    for (int i = 0; i < num1; i++)
    {
        data.i_AntennaSq[i][0] = m_cutSequence[i][0];
        data.i_AntennaSq[i][1] = m_cutSequence[i][1];
    }
    data.i_Phase_Len = num1;

    // 对每颗卫星独立进行幅相法测向
    for (int i = 0; i < size; i++)
    {
        tp_alarms.clear();
        tpB = dataB[i];
        typeInt = TypeInt(tpB.i_Sys, tpB.i_Type);
        tp_SimulateA.clear();
        tp_SimulateA = m_SimulateA[typeInt];  // 获取该频点的仿真阵列流型
        double sum_snr = 0.0;
        int num2 = tpB.i_diffLen;
        int prn = dataB[i].i_Prn;
        tp_peseudo.clear();

        // 构建幅度向量和相位差向量(从第1刀开始，第0刀为参考)
        for (int j = 1; j < num2; j++)
        {
            sum_snr = sum_snr + tpB.i_Snr1[j];
            // 载噪比(dB)转换为线性幅度: 10^(SNR/20)
            amp_snr[j] = pow(10, (tpB.i_Snr2[j] / 20));
            // 相位差(周)转换为弧度: 相位差 * 2π
            phase_diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
            data.i_Amp[j] = pow(10, (tpB.i_Snr2[j] / 20));
            data.i_Phase_Diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
        }
        // 参考通道(第0刀): 相位差=0，幅度为平均载噪比
        tmp2 = sum_snr / (tpB.i_diffLen - 1);
        tmp1 = tmp2;
        amp_snr[0] = pow(10, (tmp1 / 20));
        phase_diff[0] = 0;
        data.i_Amp[0] = pow(10, (tmp1 / 20));
        data.i_Phase_Diff[0] = 0;

        double angle;
        double quality;
        diff.clear();

        // 调用底层幅相法匹配算法
        ArithmeticDoa::calAmpPhase(tp_SimulateA, data, angle, quality, diff);

        // 记录最大载噪比
        m_Max_Snr[typeInt][tpB.i_Prn] = tmp2;

        // 构建告警/测向结果
        tp_alarm.i_Prn = tpB.i_Prn;
        tp_alarm.i_Angle = (int)angle;
        tp_alarm.i_Quality = quality;

        // 按频点汇总结果到m_AngleResultData
        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            tp_alarms.emplace_back(tp_alarm);
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo[prn] = diff;  // 保存伪谱值
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
            }
        }
        else
        {
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo = m_Pseudo_Spectrum_Value[typeInt];
                tp_peseudo[prn] = diff;
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
            }
            tp_alarms = m_AngleResultData[typeInt];
            tp_alarms.emplace_back(tp_alarm);
        }

        m_AngleResultData[typeInt] = tp_alarms;
    }
}


// =========================================================================
// 初始化幅相法理论模版(仿真阵列流型)
// 流程:
//   1. 从仿真数据文件加载各频率的阵列流型数据
//   2. 对于每个工作的GNSS频点，在仿真数据中寻找频率最接近的模板
//      注: 仿真数据可能只有有限几个频率点，需要匹配最近的频率
//   3. 如果没有任何仿真数据可用，自动切换到相关干涉仪算法(m_Doa_Arithmetic=1)
// 结果: 存放在m_SimulateA中，按频点编码索引
// =========================================================================
void SpoofingDoa::initSimulateA(void){
    int typeInt = 0;
    double f = 0;
    std::map<double, std::vector<std::vector<complex<double>>>> tp_SimulateA;
    tp_SimulateA.clear();
    vector<vector<complex<double>>> simulateA;
    int num = m_All_Simulate_data_Fre.size();
    vector<double> erse_index;
    erse_index.clear();

    // 加载所有仿真频率的阵列流型
    for (int i = 0; i < num; i++){
        f = m_All_Simulate_data_Fre[i];
        simulateA.clear();
        ArithmeticDoa::calAmpPhaseSimulateA(m_Simulate_Data_file, f, simulateA);  // 从文件读取
        if (0 == simulateA.size()){
            continue;  // 该频率无仿真数据
        }
        erse_index.emplace_back(f);
        tp_SimulateA[f] = simulateA;
    }

    // 如果没有加载到任何仿真数据，自动切换到相关干涉仪算法
    if (0 == tp_SimulateA.size()){
        m_Doa_Arithmetic = 1; // 若没有阵列仿真数据，则自动转成使用相关干涉仪算法
        initTheory();
        return;
    }

    m_All_Simulate_data_Fre.clear();

    for (int i = 0; i < (int)erse_index.size(); i++){
        m_All_Simulate_data_Fre.emplace_back(erse_index[i]);
    }

    double tp_f_min = 999999e8;
    double tp_f2 = 0;
    double tp_diff = 0.0;
    double f2 = 0.0;

    // 为每个工作频点匹配最接近的仿真频率
    for (auto it = m_F.begin(); it != m_F.end(); ++it){

        typeInt = it->first;
        f = it->second;  // 工作频点实际频率
        tp_f_min = 999999e8;
        for (int i = 0; i < (int)m_All_Simulate_data_Fre.size(); i++){

            tp_f2 = m_All_Simulate_data_Fre[i];
            tp_diff = abs(tp_f2 - f);  // 频率差
            if (tp_f_min > tp_diff){
                tp_f_min = tp_diff;
                f2 = tp_f2;  // 记录最接近的仿真频率
            }

        }

        PublicSpace::Log("Fre = %.1f,f= %.1f\n", f, f2);
        m_SimulateA[typeInt] = tp_SimulateA[f2]; // 保存各个频点的仿真阵列流型(使用最接近频率的模板)
    }
}
