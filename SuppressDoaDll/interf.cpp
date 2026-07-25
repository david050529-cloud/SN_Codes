// =============================================================================
// 文件名: interf.cpp
// 功能描述: 相关干涉仪测向方法实现
//   相关干涉仪是测向系统中的核心方法，适用于单信号场景。
//   基本思想: 将实测的天线间相位差向量，与预先计算的理论相位差库
//             (覆盖0-360度所有可能入射角)进行匹配比较。
//   比较方式: 计算实测向量与每个角度的理论向量之间的相关性(相关系数)，
//             相关性最高的角度即为DOA估计结果。
//
//   关键数据结构: InterferInfo
//     - i_AntennaSq[N][2]: N组天线对的序号(1-based)
//     - i_Phase_Diff[N]:   对应的实测相位差(弧度)
//     - i_Start / i_End:   角度搜索范围(度)
//     - i_Phase_Len:       有效的天线对数量
//
//   setInterf(): 单信号相关干涉仪测向主流程
//     1. 获取相位差向量（单通道/多通道）
//     2. 全向天线: 如果相位差不全，通过天线相位重构补充
//        定向天线: 根据信号强度选择扇区天线，限制搜索范围
//     3. 构建InterferInfo并调用calInterfer匹配
//     4. 多次测向取置信度最高的结果
//
//   setInterfInfo(): 将切刀序列和相位差打包成InterferInfo结构
// =============================================================================
#include "SuppressDoa.h"

// =============================================================================
// setInterf: 单信号相关干涉仪测向主流程
// 流程说明:
//   步骤1 - 获取相位差:
//     单通道: getSigleChannelPhaseA() 通过四次移相+牛顿迭代求解
//     多通道: getMuiltChannelPhaseA() 通过复比均值 arg(mean(ch_j/ch_i))
//   步骤2 - 天线选择:
//     全向天线: 使用全部天线，相位差不全时通过天线相位重构
//     定向天线: 根据幅度选择最强信号的m_Doa_Cut_Num+1个天线
//   步骤3 - 匹配测向:
//     构建InterferInfo -> calInterfer()与理论相位差库匹配
//   步骤4 - 结果选取:
//     如果有多组天线配置，取置信度最高的结果
// =============================================================================
void SuppressDoa::setInterf()
{
    vector<vector<int>> cutSequence;

    int Start = 0; // 搜索范围开始角度,默认s
    int End = 359; // 搜索范围结束角度

    vector<double> phs;
    vector<double> A;
    vector<vector<vector<int>>> use_cuts;
    vector<vector<double>> all_phaseDiff;
    vector<int> use_cut;
    if (1 == m_channelNum) // 单通道
    {
      //  auto start = high_resolution_clock::now();
        getSigleChannelPhaseA(phs, A); // 获得切刀顺序对应的相差
     //     auto end = high_resolution_clock::now();
    //   auto d_ms = duration_cast<milliseconds>(end - start).count();//毫秒
     //  cout << "数据处理时间：" << d_ms << "ms , " << d_ms / 1000.0 << "s" << endl;
        cutSequence = m_cutSequence;
    }
    else
    {
        getMuiltChannelPhaseA(cutSequence, phs, A); // 多通道: 通过复比计算相位差
    }

    if (1 == m_antnenaType) // 定向天线，现只可通过幅度的大小来确定使用天线序号，和搜索范围
    {
        // 根据信号幅度选择用于测向的天线，同时确定角度搜索范围
        getUseAntennaByADirect(A, m_Doa_Cut_Num, use_cut, Start, End); // 天线序号范围1~N
        // 将切刀相位差重构为天线级相位
        int f = getAntennaPhase(use_cut, m_AntennaNum, cutSequence, phs);
        if (f != 0)
        {
            string tp1 = Vector2String(cutSequence);
            m_angles[0] = -1;
            m_qualities[0] = -1;
            Log("***error:get per ant phase is false:cutSequence=%s\n", tp1.c_str());
            return;
        }
        int sizes = (int)use_cut.size();
        int cutsNum = sizes - m_Doa_Cut_Num + 1; // 滑动窗口的数量
        use_cuts.clear();
        //  vectorResize(use_cuts, cutsNum, m_Doa_Cut_Num);
        vector<int> tp_antenna;
        vector<double> tp_phs;

        for (int i = 0; i < cutsNum; i++) // 在同一次测向时中，可能多次测向，选取置信度最高那个
        {
            tp_phs.clear();
            tp_phs.resize(m_AntennaNum);
            tp_antenna.clear();
            tp_antenna.resize(m_Doa_Cut_Num);
            for (int j = 0; j < m_Doa_Cut_Num; j++)
            {
                tp_antenna[j] = use_cut[j + i]; // 滑动窗口选择天线
            }
            for (int j = 0; j < m_AntennaNum; j++)
            {
                tp_phs[j] = -phs[j]; // 取反以获得正确的相位关系
            }
            getUsePhaseDiffAll(tp_antenna, cutSequence, tp_phs); // 提取相关天线对的相位差
            use_cuts.emplace_back(cutSequence);
            all_phaseDiff.emplace_back(tp_phs);
        }
    }
    else // 全向天线
    {

        use_cut.resize(m_AntennaNum);
        for (int i = 0; i < m_AntennaNum; i++) // 天线的序号
        {
            use_cut[i] = i + 1; // 1-based天线序号
        }
        int num = (int)phs.size();
        if (num < (m_AntennaNum * (m_AntennaNum - 1) / 2)) // 不是全切刀(缺少某些天线对)
        {
            // 通过天线相位重构，补充缺失的相位差信息
            int f = getAntennaPhase(use_cut, m_AntennaNum, cutSequence, phs);
            //   getAntennaPhase(cutSequence, phs, m_AntennaNum); // 每个天线对应的相位(相对的相位)
            //  getPhaseDiffAll(cutSequence, phs);               // 获得全天线相互之间的相位差
            getUsePhaseDiffAll(use_cut, cutSequence, phs); // 提取所用天线的全部相位差
            use_cuts.emplace_back(cutSequence);
            all_phaseDiff.emplace_back(phs);
        }
        else // 全切刀，直接使用
        {
            use_cuts.emplace_back(cutSequence);
            all_phaseDiff.emplace_back(phs);
        }
    }
    InterferInfo tp_info;
    vector<double> phaseDiff;
    vector<double> diff;
    double angles = 0.0;
    double qulities = 0.0;
    double max_quality = -999;
    Log("start cal angle:\n");
    cutSequence.clear();
    // 对每组天线配置分别进行相关干涉仪匹配，取置信度最高的结果
    for (int i = 0; i < use_cuts.size(); i++) // use_cuts 测向的次数？
    {
        phaseDiff = all_phaseDiff[i];
        cutSequence = use_cuts[i];
        setInterfInfo(cutSequence, phaseDiff, tp_info); // 构建InterferInfo
        tp_info.i_Start = Start;
        tp_info.i_End = End;
        ArithmeticDoa::calInterfer(m_Theory, tp_info, angles, qulities, diff); // 核心: 与理论相位库匹配
        Log("%d---angel=%.1f , qulity=%.2f\n", i + 1, angles, qulities);
        if (qulities > max_quality)
        {
            max_quality = qulities;
            m_angles[0] = angles;
            m_qualities[0] = qulities;
        }
    }
}

// =============================================================================
// setInterfInfo: 将切刀序列和相位差信息封装为InterferInfo结构
// 用于后续相关干涉仪匹配计算
// 参数:
//   cutSequence: 天线对列表，每个元素是{天线i, 天线j}
//   phs: 对应天线对的实测相位差(弧度)
//   info: (输出)封装好的InterferInfo结构
// =============================================================================
void SuppressDoa::setInterfInfo(const vector<vector<int>> cutSequence, const vector<double> phs, InterferInfo &info)
{
    int num3 = (int)phs.size();

    for (int i = 0; i < num3; i++)
    {
        info.i_AntennaSq[i][0] = cutSequence[i][0]; // 天线1序号
        info.i_AntennaSq[i][1] = cutSequence[i][1]; // 天线2序号
        info.i_Phase_Diff[i] = phs[i];               // 相位差(弧度)
        Log("---------ant1=%d,ant2=%d:  phsediff=%.2f\n", info.i_AntennaSq[i][0], info.i_AntennaSq[i][1], info.i_Phase_Diff[i] * 180 / m_PI);
    }
    info.i_Phase_Len = num3;  // 天线对数量
    info.i_Start = 0;         // 默认搜索范围 0度
    info.i_End = 359;         // 默认搜索范围 359度(全覆盖)
}

// =============================================================================
// setInterfInfo: 重载版本，直接执行相关干涉仪测向并写入结果
// 用于简单的多通道全天线场景(不再使用的主要接口)
// =============================================================================
void SuppressDoa::setInterfInfo(double &angles, double &qulities) // 设置相关干涉仪测向数据
{
    std::vector<std::vector<double>> tp_Actual;
    tp_Actual.clear();
    tp_Actual.resize(m_AntennaNum);
    for (int i = 0; i < m_AntennaNum; i++)
    {
        tp_Actual[i].resize(m_AntennaNum);
    }

    int num1, num2;
    for (int cut = 0; cut < m_cutNum; ++cut)
    {
        for (int i = 0; i < m_channelNum - 1; ++i)
        {
            for (int j = i + 1; j < m_channelNum; ++j)
            {
                num1 = m_cutSequence[cut][i] - 1;
                num2 = m_cutSequence[cut][j] - 1;
                complex<double> sum(0, 0);
                for (int k = 0; k < m_usePointNum; ++k)
                {
                    if (abs(m_Data[cut][i][k]) < MIN_ZERO)
                    {
                        sum += m_Data[cut][j][k];
                    }
                    else
                    {
                        sum += m_Data[cut][j][k] / m_Data[cut][i][k];
                    }
                }
                tp_Actual[num1][num2] = arg(sum / (double)m_usePointNum);
                //   tp_Actual[num2][num1] = -tp_Actual[num1][num2];
            }
        }
    }
    vector<vector<int>> cutSequence;
    vector<int> tp_ctse;
    vector<double> phaseDiff;

    for (int i = 0; i < m_channelNum - 1; ++i)
    {
        tp_ctse.clear();
        tp_ctse.resize(2);
        for (int j = i + 1; j < m_channelNum; ++j)
        {
            if (tp_Actual[i][j] < MIN_ZERO)
            {
                tp_ctse[0] = i + 1;
                tp_ctse[1] = j + 1;
                phaseDiff.emplace_back(tp_Actual[i][j]);
                cutSequence.emplace_back(tp_ctse);
            }
        }
    }
    ArithmeticDoa::setUseAntennaAndPhaseAll(cutSequence, phaseDiff);
    InterferInfo tp_info;
    int Start = 0; // 搜索范围开始角度,默认
    int End = 359; // 搜索范围结束角度
    int num3 = (int)phaseDiff.size();
    for (int i = 0; i < num3; i++)
    {
        tp_info.i_AntennaSq[i][0] = cutSequence[i][0];
        tp_info.i_AntennaSq[i][1] = cutSequence[i][1];
        tp_info.i_Phase_Diff[i] = phaseDiff[i];
    }

    tp_info.i_Start = Start;
    tp_info.i_End = End;
    tp_info.i_Phase_Len = num3;
    vector<double> diff;
    ArithmeticDoa::calInterfer(m_Theory, tp_info, angles, qulities, diff); // 执行相关干涉仪匹配
}