// =============================================================================
// 文件名: SuppressDoa.cpp
// 功能描述: SuppressDoa类的核心实现
//   本文件实现测向系统的主流程方法，包括:
//     - 构造函数: 初始化项目参数，从配置文件加载所有配置
//     - initSuppressDoa: 设置工作频率，计算理论相位差或加载阵列流型
//     - setCorrectionData: 计算通道校正系数（幅相校准）
//     - setData: 接收IQ数据并按切刀-通道-采样点三维组织存储
//     - getAngleSuppressDoa: 对外返回测向结果（角度、置信度、幅度）
//     - setAngleSuppressDoa: 内部测向算法调度，根据配置选择具体测向方法
//   依赖: ArithmeticDoa基类提供理论相位差计算和相关干涉仪匹配
// =============================================================================
#include "SuppressDoa.h"
using namespace std;

// =============================================================================
// 构造函数: 初始化SuppressDoa测向对象
// 参数:
//   pointNum: FFT采样点数（一帧数据的采样点数）
//   startPointNum: 有效数据的起始采样点（跳过滤波器暂态响应/拖尾）
//   endPointNum: 有效数据的结束采样点
// 流程:
//   1. 读取日志配置
//   2. InitProject: 根据项目类型设置默认的切刀序列、天线数、通道数等
//   3. 参数合法性检查
//   4. 计算可用点数 m_usePointNum
//   5. 从配置文件读取用户可配置的参数
// =============================================================================
SuppressDoa::SuppressDoa(int pointNum, int startPointNum, int endPointNum)
{
    string configAdr = "./DoaBSuppressConfig.txt"; // 默认配置文件路径
    getLogFlg(configAdr);                          // 读取日志配置
    InitProject();                                 // 根据项目类型初始化默认参数
    m_pointNum = pointNum;
    m_startPointNum = startPointNum;
    m_endPointNum = endPointNum;
    if (m_pointNum < 0)
    {
        cout << "***error:pointNum=" << m_pointNum << endl;
        Log("error:pointNum=%d\n", m_pointNum);
        return;
    }
    if (startPointNum < 0)
    {
        m_startPointNum = 0;
    }
    if (endPointNum > pointNum - 1)
    {
        m_endPointNum = pointNum - 1;
    }
    // 可用采样点数: 剔除首尾拖尾后的有效数据长度
    m_usePointNum = m_endPointNum - m_startPointNum + 1;

    setConfigData(configAdr);
}

SuppressDoa::~SuppressDoa()
{
    LogClose(); // 关闭日志文件
}

// =============================================================================
// 初始化测向系统
// 参数: f - 工作频率(Hz)
// 返回值: 0=成功, -1=频率无效, -2=切刀配置无效
// 流程:
//   1. 设置工作频率 m_F
//   2. 单通道模式: 初始化IQ积分队列 + 读取移相误差系数文件
//   3. 初始化校正系数为1（复数域单位元）
//   4. 根据算法类型计算理论值:
//      算法1/3: 计算理论相位差矩阵 m_Theory
//      算法5(DML): 计算理论相位差 + 加载仿真阵列流型 m_SimulateA
// =============================================================================
int SuppressDoa::initSuppressDoa(double f)
{
    if (f > 0)
    {
        m_F = f;
    }
    else
    {
        Log("***error: F=%.1f\n", f);
        return -1;
    }
    m_cutNum = (int)m_cutSequence.size(); // 切刀次数
    string cutSequenceStr = Vector2String(m_cutSequence);

    if (m_cutNum < 0 || m_cutNum == 0)
    {
        Log("***error: cutNum=%d,cutSequence=%s\n", m_cutNum, cutSequenceStr.c_str());
        return -2;
    }
    // 定向天线: 根据频率从文件读取对应的阵列半径R
    if (1 == m_antnenaType)
    {
        setR(m_F, m_Radr);
    }

    // tp_channel: 校正系数数组长度
    // 多通道模式: 长度=通道数(m_channelNum)
    // 单通道模式: 长度=移相次数(m_ShiftPhaseNum=4)
    int tp_channel = m_channelNum;
    if (1 == m_channelNum)
    {
        initAmDesity();                                     // 初始化IQ幅度积分队列(deque)
        sigleChannel4shiftInitl(m_shiftPhase_Coefficeicent_Adr); // 从CSV文件读取移相误差系数
        tp_channel = m_ShiftPhaseNum;                       // 单通道时校正系数按移相次数设置


    }
    Log("cutNum=%d,cutSequence=%s\n", m_cutNum, cutSequenceStr.c_str());
    Log("F=%.1f,R=%.4f\n", m_F, m_R);
    Log("channelNum=%d,antennaType=%d\n", m_channelNum, m_antnenaType);
    Log("startPointNum= %d,endPointNum=%d,usePointNum=%d\n", m_startPointNum, m_endPointNum, m_usePointNum);
    m_Correction.resize(tp_channel);
    for (int i = 0; i < tp_channel; i++)
    {
        m_Correction[i] = COMPLEX_ONE; // 初始化为1，无校正状态
    }
    // 相关干涉仪测向/联合对角化+干涉仪: 需要理论相位差库
    if (1 == m_Doa_Arithmetic || 3 == m_Doa_Arithmetic) // 相关干涉仪测向，联合对角化+干涉仪
    {
        m_Theory.clear();
        ArithmeticDoa::calPhaseTheory(f, m_R, m_AntennaNum, m_Theory); // 理论相位差
    }
    else if (m_Doa_Arithmetic==5) // DML测向(GN560定向天线): 需要理论相位差 + 仿真阵列流型
    {
        m_Theory.clear();
        ArithmeticDoa::calPhaseTheory(f, m_R, m_AntennaNum, m_Theory); // 理论相位差
        m_SimulateA.clear();
        ArithmeticDoa::calAmpPhaseSimulateA(m_Simulate_Data_file,  f, m_SimulateA); // 理论相位差
    }
    return 0;
}

// =============================================================================
// 设置通道校正数据
// 功能: 计算各通道相对于参考通道(通道0)的幅相校正系数
// 原理:
//   校正系数 = E[ch_j(k) / ch_0(k)] 对所有有效采样点k取平均
//   即计算每个通道与参考通道的复数比值（包含幅度比和相位差）
//   该比值在后续测向时用于补偿通道间的不一致性
// 参数:
//   data: short数组，IQ数据交替存储，排列顺序为[通道][采样点][IQ交替]
//   length: data数组的长度
// 注意事项:
//   - 仅当参考通道数据非零时才参与累加（防止除零）
//   - 校正系数为复数，幅值表示幅度校正，相位表示相位校正
//   - 校正系数会被归一化保护：若幅值过小则置为1
// =============================================================================
int SuppressDoa::setCorrectionData(short *data, int length)
{

    vector<vector<complex<double>>> correctionData(m_channelNum);

    correctionData.resize(m_channelNum);
    for (int j = 0; j < m_channelNum; ++j)
    {
        correctionData[j].resize(m_usePointNum);
    }

    int start = 0;

    // 从short数组中提取IQ复数数据
    // 数据排列: [通道0的所有采样点IQ], [通道1的所有采样点IQ], ...
    for (int j = 0; j < m_channelNum; ++j)
    {
        start = (j * m_pointNum + m_startPointNum) * 2;
        for (int k = 0; k < m_usePointNum; ++k)
        {
            correctionData[j][k] = complex<double>(data[start], data[start + 1]);
            start += 2;
        }
    }

    // 计算校正系数: m_Correction[j] = mean(ch_j / ch_0)
    for (int j = 0; j < m_channelNum; ++j)
    {
        complex<double> sum(0.0, 0.0);
        int count = 0;
        for (int k = 0; k < m_usePointNum; ++k)
        {
            // 如果参考通道数据不为0，则计算校正系数
            if (abs(correctionData[0][k]) > MIN_ZERO)
            {
                sum = sum + (correctionData[j][k] / correctionData[0][k]);
                count++;
            }
        }
        m_Correction[j] = sum / (double)(count);
        Log("m_channelNum --%d ---m_Correction = %.2f \n", j + 1, arg(m_Correction[j]) * 180 / m_PI);

        // 防止数据除零导致后续计算出错
        if (abs(m_Correction[j]) < MIN_ZERO)
        {
            m_Correction[j] = COMPLEX_ONE;
        }
    }

    saveData(1, data, length);

    Log("set correction data is sucess...\n");
    return 0;
}

// =============================================================================
// 设置测向数据
// 功能: 将IQ数据按切刀-通道-采样点三维结构组织存储到 m_Data
// 数据排列规则:
//   多通道: [切刀0][通道0的所有IQ], [切刀0][通道1的所有IQ], [切刀0][通道2的所有IQ],
//           [切刀1][通道0的所有IQ], ...
//   单通道: 类似，但tp_channel = m_ShiftPhaseNum (4次移相)
// 每对相邻short表示一个复数IQ采样点(data[0]=I, data[1]=Q)
// 数据会除以校正系数 m_Correction[j] 进行通道校正
// 参数:
//   data: short数组，IQ交替存储
//   length: 数据总长度
// =============================================================================
int SuppressDoa::setData(short *data, int length)
{

    int tp_channel = m_channelNum;
    if (m_channelNum == 1)
    {
        tp_channel = m_ShiftPhaseNum; // 单通道时转换为4次移相的维度
    }
    m_Data.clear();
    // 重新分配三维存储空间: [切刀数][通道数/移相次数][采样点数]
    m_Data.resize(m_cutNum);
    for (int i = 0; i < m_cutNum; ++i)
    {
        m_Data[i].resize(tp_channel);
        for (int j = 0; j < tp_channel; ++j)
        {
            m_Data[i][j].resize(m_usePointNum);
        }
    }
    int start = 0;

    for (int i = 0; i < m_cutNum; i++)
    {
        for (int j = 0; j < tp_channel; j++)
        {
            if (abs(m_Correction[j]) < MIN_ZERO)
            {
                m_Correction[j] = COMPLEX_ONE; // 校正系数保护
            }
            //   Log("correction:--%d,  %.2f\n", j + 1, m_Correction[j]);
            // 计算当前切刀当前通道的起始位置偏移
            start = (i * tp_channel * m_pointNum + j * m_pointNum + m_startPointNum) * 2;

            for (int k = 0; k < m_usePointNum; k++)
            {
                // 读取IQ数据并除以校正系数（若有校正数据，则先设置校正数据，若没有校正数据，则为1）
                m_Data[i][j][k] = complex<double>(data[start], data[start + 1]) / m_Correction[j]; // 若有校正数据，则先设置校正数据，若没有校正数据，则为1

                start += 2;
            }
        }
    }
    saveData(2, data, length);

    Log("set doa data is sucess...\n");

    return 0;
}

// =============================================================================
// 设置有效数据的起止采样点
// startPointNum: 起始采样点，用于跳过滤波器暂态响应(拖尾)
// endPointNum: 结束采样点
// 参数保护: start >= 0, end > start, 确保合理范围
// =============================================================================
int SuppressDoa::setStartEndPointNum(int startPointNum, int endPointNum)
{
    m_startPointNum = startPointNum;
    m_endPointNum = endPointNum;
    if (startPointNum < 0 || m_startPointNum > m_pointNum)
    {
        m_startPointNum = 0;
    }
    if (m_endPointNum < m_startPointNum)
    {
        m_endPointNum = m_pointNum - 1;
    }
    m_usePointNum = m_endPointNum - m_startPointNum + 1;
    Log("startPointNum= %d,endPointNum=%d,usePointNum=%d\n", m_startPointNum, m_endPointNum, m_usePointNum);
    return 0;
}

// =============================================================================
// 多通道模式: 计算所有切刀的相位差和幅度
// 算法:
//   对每个切刀的每组通道对(i,j):
//     - 计算复比 sum = ch_j / ch_i 的均值（对采样点平均）
//     - 相位差 = arg(mean(ch_j/ch_i))  即两天线接收信号的相位差
//     - 幅度 = 20*log10(mean(|ch_j|))  即该天线的接收功率(dB)
// 参数(输出):
//   cutSequence: 每对天线的序号(用于构建InterferInfo)
//   phaseDiff: 对应的相位差(弧度)
//   A: 各天线的幅度(dB)
// =============================================================================
void SuppressDoa::getMuiltChannelPhaseA(vector<vector<int>> &cutSequence, vector<double> &phaseDiff, vector<double> &A)
{
    vector<int> tp_ctse;
    vector<double> tp_A;

    cutSequence.clear();
    phaseDiff.clear();
    tp_ctse.clear();
    tp_ctse.resize(2);
    tp_A.clear();
    tp_A.resize(m_AntennaNum);
    A.resize(m_AntennaNum);
    int num1, num2;
    double tp_diff = 0.0;
    double sum_A = 0.0;
    for (int cut = 0; cut < m_cutNum; ++cut)
    {
        for (int i = 0; i < m_channelNum - 1; ++i)
        {
            for (int j = i + 1; j < m_channelNum; ++j)
            {
                num1 = m_cutSequence[cut][i] - 1; // 转为0-based序号
                num2 = m_cutSequence[cut][j] - 1;
                complex<double> sum(0.0, 0.0);
                sum_A = 0.0;
                for (int k = 0; k < m_usePointNum; ++k) // 相对第一通道的相位差
                {
                    if (abs(m_Data[cut][i][k]) < MIN_ZERO)
                    {
                        sum += m_Data[cut][j][k]; // 防止除零，直接用ch_j值
                    }
                    else
                    {
                        sum += m_Data[cut][j][k] / m_Data[cut][i][k]; // 复比，相位差=arg(ch_j/ch_i)
                    }
                    sum_A += sqrt(m_Data[cut][j][k].real()* m_Data[cut][j][k].real()+ m_Data[cut][j][k].imag()* m_Data[cut][j][k].imag());
                }

                tp_diff = arg(sum / (double)m_usePointNum); // 复数均值的辐角=相位差
                double tp2 = tp_diff * 180 / m_PI;
                // tp_Actual[num1][num2] = arg(sum / (double)m_usePointNum);
                tp_ctse[0] = num2 + 1;
                tp_ctse[1] = num1 + 1;
                Log("------cut = %d, channel1 = %d,channel2 = %d, sum = %.2f+%0.2fi,  phsdiff=%.2f\n", cut + 1, tp_ctse[0], tp_ctse[1], sum.real(), sum.imag(), Round3600(tp2));

                phaseDiff.emplace_back(tp_diff);
                cutSequence.emplace_back(tp_ctse);
                A[num2] = 20 * log10(sum_A / (double)m_usePointNum); // 幅度转为dB
            }
        }
    }

    Log("get Muilt channel phase diff and Am sucess...\n");
}
// void SuppressDoa::setData(short *data, int length)
// {

//     if (m_channelNum == 1)
//     {
//         setSigleChannelDoaData(data, length);
//     }
//     else
//     {
//         m_Data.clear();
//         m_Data.resize(m_cutNum);
//         for (int i = 0; i < m_cutNum; ++i)
//         {
//             m_Data[i].resize(m_channelNum);
//             for (int j = 0; j < m_channelNum; ++j)
//             {
//                 m_Data[i][j].resize(m_usePointNum);
//             }
//         }
//         int start = 0;
//         for (int i = 0; i < m_cutNum; i++)
//         {
//             for (int j = 0; j < m_channelNum; j++)
//             {
//                 start = (i * m_channelNum * m_pointNum + j * m_pointNum + m_startPointNum) * 2;
//                 for (int k = 0; k < m_usePointNum; k++)
//                 {
//                     m_Data[i][j][k] = complex<double>(data[start], data[start + 1]) / m_Correction[i][j]; // 若有校正数据，则先设置校正数据，若没有校正数据，则为1
//                     start += 2;
//                 }
//             }
//         }
//     }
//     saveData(2, data, length);
//     Log("set doa data is sucess...");
// }

// =============================================================================
// 根据频率设置阵列半径R
// 定向天线: 不同频率对应不同的等效阵列孔径（半径）
// 从文件中读取频率-半径对应表，线性查找当前频率所属范围的半径值
// 参数: f - 工作频率(Hz), adr - 半径数据文件路径
// =============================================================================
void SuppressDoa::setR(const double f, const string adr)
{
    if (m_antnenaType != 0) // 仅定向天线需要动态设置半径
    {

        vector<Rs> mRs;
        mRs.clear();
        getRData(adr, mRs); // 从文件读取频率-半径对应表
        for (int i = 0; i < mRs.size(); i++)
        {
            if (f >= mRs[i].i_starF && f <= mRs[i].i_endF) // 查找频率所属范围
            {
                m_R = mRs[i].i_r;
                break;
            }
        }
    }
    if (m_R < 0 || m_R == 0)
    {
        Log("error:m_R=%.5f\n", m_R);
    }
}
// =============================================================================
// 设置同频信号个数
// 参数: signalNum - 同频信号的个数（>=1）
// 用途: 告诉测向系统当前场景有多少个同频信号
//       单信号时使用相关干涉仪直接测向
//       多信号时使用联合对角化进行盲源分离后再测向
// =============================================================================
int SuppressDoa::setSignalNum(int signalNum)
{
    if (signalNum < 1)
    {
        m_signalNum = 1;
        Log("***warn:set is signalNum=%d\n", signalNum);
        return -1;
    }
    else
    {
        m_signalNum = signalNum;
        Log("signalNum=%d\n", signalNum);
        return 0;
    }
}

int SuppressDoa::getSpectrumSupressDoa(int &signalNum, short *spectrumData, int &length) // 频谱数据
{
    return 0;
}

// =============================================================================
// 获取测向结果（对外接口）
// 功能: 执行测向计算并返回结果
// 参数(输出):
//   signalNum: 检测到的信号个数
//   angles: 各信号的来波方向(度, 0-360)
//   qualities: 各信号的测向置信度(0-1, 越高越可靠)
//   amplitudes: 各信号的幅度(dB)
// 注意: qualities会乘上m_qulity_Coefficeicent进行信号强度修正
// =============================================================================
int SuppressDoa::getAngleSuppressDoa(int &signalNum, double *angles, double *qualities, double *amplitudes)
{
    setAngleSuppressDoa(); // 执行实际测向计算
    Log("-----------------------------signalNum=%d-----------------------\n", m_signalNum);
    signalNum = m_signalNum;

    for (int i = 0; i < m_signalNum; i++)
    {
        Log("%d-angles=%.1f,qualities=%.2f\n", i + 1, m_angles[i], m_qualities[i]);
        angles[i] = m_angles[i];
        qualities[i] = m_qualities[i]*m_qulity_Coefficeicent; // 质量系数修正
        amplitudes[i] = m_amplitudes[i];
    }
    Log("---------------------------------------------------------------\n");
    return 0;
}

// =============================================================================
// 测向算法调度中心
// 根据当前配置选择执行哪种测向算法
// 调度策略:
//   - 算法1(相关干涉仪): 直接执行setInterf()单信号测向
//   - 算法3(联合对角化+干涉仪):
//       单信号 -> 退化为相关干涉仪 setInterf()
//       多信号 -> 联合对角化盲分离 getJointdiagByInterf()
//   - 算法5(DML): 由setInterf()根据天线类型内部处理
//   - 单通道模式 + 单信号: setInterf() (使用四次移相求解相位差)
//   - 单通道模式 + 多信号: getMuiltSignalBySigleChannel() (FFT+联合对角化)
// 各测向方法相互独立，结果写入 m_angles[], m_qualities[], m_amplitudes[]
// =============================================================================
int SuppressDoa::setAngleSuppressDoa()
{

    if (1 == m_Doa_Arithmetic) // 相关干涉仪测向
    {
        setInterf();
    }
    if (3 == m_Doa_Arithmetic) // 联合对角化+相关干涉仪测向
    {
        if (1 == m_signalNum)
        {

            setInterf(); // 若只有一个信号则使用相关干涉仪
        }
        else
        {
            getJointdiagByInterf(); // 多信号
        }
    }
    if (m_channelNum == 1)
    {
        if (1 == m_signalNum)
        {
           // auto start = high_resolution_clock::now();

            setInterf(); // 若只有一个信号则使用相关干涉仪
           // auto end = high_resolution_clock::now();
           // auto d_ms = duration_cast<milliseconds>(end - start).count();//毫秒
           // cout << "数据处理时间：" << d_ms << "ms , " << d_ms / 1000.0 << "s" << endl;
        }
        else
        {
            getMuiltSignalBySigleChannel(); // 多信号
        }
    }
    return 0;
}
// int SuppressDoa::getAngleSuppressDoa(int &signalNum, double *angles, double *qualities, double *amplitudes)
// {
//     vector<double> tp_angles;
//     vector<double> tp_qualities;
//     if (1 == m_Doa_Arithmetic)
//     {
//         signalNum = 1;
//         setInterfInfo(angles[0], qualities[0]);
//     }
//     if (3 == m_Doa_Arithmetic)
//     {
//         if (m_signalNum == 1)
//         {
//             signalNum = 1;
//             setInterfInfo(angles[0], qualities[0]);
//         }
//         else
//         {
//             getJointdiagByInterf(tp_angles, tp_qualities);

//             signalNum = (int)tp_angles.size();
//             for (int i = 0; i < signalNum; i++)
//             {
//                 angles[i] = tp_angles[i];
//                 qualities[i] = tp_qualities[i];
//             }
//         }
//     }
//     return 0;
// }

// =============================================================================
// 根据信号强度选择用于测向的天线（定向天线专用）
// 算法: 按幅度排序，选择信号最强的天线，然后向两侧扩展相邻天线
//   直到选够 m_Doa_Cut_Num+1 个天线
// 参数:
//   A: 各天线的接收幅度(dB)
//   useIndex: (输出)按信号强度排序后的天线序号列表
//   startAngle/endAngle: (输出)由最强天线位置确定的角度搜索范围
// 注意: 定向天线每个天线覆盖特定扇区，信号最强的天线方向即为大致来波方向
//       相邻天线用于精确定向，搜索范围限制在最强天线附近
// =============================================================================
void SuppressDoa::getUseAntennaBySignalStrength(const vector<double> A, vector<int> &useIndex, int &startAngle, int &endAngle)
{
    useIndex.clear();
    int size = A.size();
    vector<int> index;
    index.clear();
    index.resize(size);

    for (int i = 0; i < size; i++)
    {
        index[i] = i;
    }
    // 按幅度降序排序: A大的天线排前面
    std::sort(index.begin(), index.end(), [&](int i, int j)
              { return A[i] > A[j]; });
    int max_index = index[0];  // 信号最强的天线序号
    int last_index = 0;
    int next_index = 0;
    vector<int> tp_index;
    tp_index.emplace_back(max_index);
    int size2 = 1;
    // 环形搜索: 从最强天线向两侧交替扩展，直到满足所需数量
    while (size2 < m_Doa_Cut_Num + 1)
    {
        last_index = (tp_index[0] - 1 + size) % size;        // 左侧邻居（环形）
        next_index = (tp_index[size2 - 1] + 1) % size;       // 右侧邻居（环形）
        for (int i = 1; i < size; i++)
        {
            if (index[i] == last_index)
            {
                tp_index.insert(tp_index.begin() + 0, index[i]); // 插入左侧
                break;
            }
            if (index[i] == next_index)
            {
                tp_index.emplace_back(index[i]); // 追加右侧
                break;
            }
        }
        size2 = tp_index.size();
    }
    useIndex = tp_index;
    // 根据选出的天线确定角度搜索范围
    calAngleSerchRange(tp_index, m_cutSequence, m_AntennaNum, max_index, startAngle, endAngle);
}

// =============================================================================
// 定向天线: 将幅度和相位差数据打包成InterferInfo列表
// 流程: 根据信号强度选择天线 -> 提取对应切刀的相位差 -> 构建InterferInfo
// =============================================================================
void SuppressDoa::setInterferInfoDataDirect(const vector<double> Am, const vector<double> thete, vector<InterferInfo> &inferInfoData)
{
    inferInfoData.clear();
    vector<int> useIndex;
    useIndex.clear();
    int startAngle;
    int endAngle;
    getUseAntennaBySignalStrength(Am, useIndex, startAngle, endAngle);

    int size = useIndex.size();
    vector<vector<int>> tp_antenna;
    vector<double> tp_diff;
    tp_antenna.resize(m_Doa_Cut_Num);
    tp_diff.resize(m_Doa_Cut_Num);
    int num = 0;
    InterferInfo tp_info;

    while (size >= m_Doa_Cut_Num)
    {
        for (int i = 0; i < m_Doa_Cut_Num; i++)
        {
            tp_antenna[i] = m_cutSequence[useIndex[i]];
            tp_diff[i] = thete[useIndex[i]];
        }
        setUseAntennaAndPhaseAll(tp_antenna, tp_diff);

        useIndex.erase(useIndex.begin() + 0);
        for (int i = 0; i < tp_diff.size(); i++)
        {
            tp_info.i_Phase_Diff[i] = tp_diff[i];
            tp_info.i_AntennaSq[i][0] = tp_antenna[i][0];
            tp_info.i_AntennaSq[i][1] = tp_antenna[i][1];
        }
        tp_info.i_Phase_Len = tp_diff.size();
        tp_info.i_Start = startAngle;
        tp_info.i_End = endAngle;
        inferInfoData.emplace_back(tp_info);
        size = useIndex.size();
    }
}

void SuppressDoa::setTheoryPhaseDiff()
{
    m_Theory.clear();
    calPhaseTheory(m_F, m_R, m_AntennaNum, m_Theory);
}

void SuppressDoa::setAngleInterferDoa(const vector<InterferInfo> inferInfoData)
{
    double angle;
    double max_qulity = -999;
    Log("get angle:\n");
    vector<double> diff;
    for (int i = 0; i < inferInfoData.size(); i++)
    {
        double tp_angle;
        double tp_qulity = 0;
        calInterfer(m_Theory, inferInfoData[i], tp_angle, tp_qulity, diff);
        if (tp_qulity > max_qulity)
        {
            angle = tp_angle;
            max_qulity = tp_qulity;
        }
        Log("startAngle = %d,endAngle=  %d,angle = %.1f,qulity=%.2f\n",
            inferInfoData[i].i_Start, inferInfoData[i].i_End, tp_angle, tp_qulity);
        for (int j = 0; j < inferInfoData[i].i_Phase_Len; j++)
        {
            Log("  ant1=%d,ant2=%d,phasediff=%.2f\n",
                inferInfoData[i].i_AntennaSq[j][0], inferInfoData[i].i_AntennaSq[j][1], inferInfoData[i].i_Phase_Diff[j]);
        }
    }

    m_signalNum = 1;
    m_angles[0] = Round3600(angle * 10) / 10;
    m_qualities[0] = max_qulity;
}

// =============================================================================
// 保存原始数据到二进制文件（调试用）
// 文件命名规则: 频率_采样点数_天线数_通道数_[校正/测向]Data.dat
// 参数: dataFlg=1保存校正数据, dataFlg=2保存测向数据
// =============================================================================
void SuppressDoa::saveData(int dataFlg, short *data, int length)
{
    std::ostringstream oss;
    oss << to_string((int)m_F) << "_" << m_pointNum << "_" << m_AntennaNum << "_" << m_channelNum;
    int Len = m_pointNum * m_channelNum * m_cutNum * 2;
    int tp_Len = 0;

    switch (dataFlg)
    {
    case 1:

        tp_Len = m_pointNum * m_channelNum * 2;
        if (length > tp_Len)
        {
            tp_Len = Len;
        }
        oss << "_CorrectionData.dat";
        Log("save correction data len= %d\n", tp_Len);
        break;
    case 2:
        oss << "_DoaData.dat";
        Log("save doa data len= %d\n", Len);
        break;
    default:
        std::cerr << "非法的 dataFlg 值: " << dataFlg << std::endl;
        return;
    }
    std::string fileName = oss.str();
    // length 输入的是IQ的长度，IQ由实部虚部组成，在short数组中，数据长度应该乘以2
    saveArrayToBinary(fileName, data, Len);
}


void SuppressDoa::getLogFlg(const string adr)
{
    map<string, string> configMap;
    int flg = readConfigtxt(adr, configMap);
    if (0 != flg)
    {
        return;
    }
    getMapData(configMap, "logAdr", m_LogFile);
    getMapData(configMap, "logFlg", m_logFlg);
    string path = ".";
    path.append(m_LogFile);
    LogCreat(path);
   // PublicSpace::Log("Suppress Version:%s\n", Version);
}

void SuppressDoa::setConfigData(const string adr)
{
    map<string, string> configMap;
    int flg = readConfigtxt(adr, configMap);
    if (0 != flg)
    {
        return;
    }

    getMapData(configMap, "antennaNum", m_AntennaNum);
    PublicSpace::Log("antennaNum=%d\n", m_AntennaNum);

    getMapData(configMap, "antennaType", m_antnenaType);
    PublicSpace::Log("antennaType=%d\n", m_antnenaType);

    getMapData(configMap, "channelNum", m_channelNum);
    PublicSpace::Log("channelNum=%d\n", m_channelNum);

    getMapData(configMap, "AccumulationTimeFactor", m_angle_AccumulationTimeFactor);
    PublicSpace::Log("AccumulationTimeFactor=%.2f\n", m_angle_AccumulationTimeFactor);

    getMapData(configMap, "simulateFile", m_Simulate_Data_file);
    PublicSpace::Log("simulateFile=%s\n", m_Simulate_Data_file.c_str());

    getMapData(configMap, "AMDesityNum", m_Am_Density_Num);
    PublicSpace::Log("AMDesityNum=%.2f\n", m_Am_Density_Num);

    getMapData(configMap, "shiftPhaseCoefficeicentAdr", m_shiftPhase_Coefficeicent_Adr);
    PublicSpace::Log("shiftPhaseCoefficeicentAdr=%s\n", m_shiftPhase_Coefficeicent_Adr.c_str());

    getMapData(configMap, "doaCutNum", m_Doa_Cut_Num);
    PublicSpace::Log("doaCutNum=%d\n", m_Doa_Cut_Num);

    getMapData(configMap, "RAllNum", m_RAll_Num);
    PublicSpace::Log("RAllNum=%d\n", m_RAll_Num);

    getMapData(configMap, "RAllPointNum", m_RAll_PointNum);
    PublicSpace::Log("RAllPointNum=%d\n", m_RAll_PointNum);

    getMapData(configMap, "virtualFlag", m_Virtual_Flag);
    PublicSpace::Log("virtualFlag=%d\n", m_Virtual_Flag);

    getMapData(configMap, "virtualMultiple", m_Virtual_Multiple);
    PublicSpace::Log("virtualMultiple=%.2f\n", m_Virtual_Multiple);

    getMapData(configMap, "doaArithmetic", m_Doa_Arithmetic);
    PublicSpace::Log("doaArithmetic=%d\n", m_Doa_Arithmetic);

    getMapData(configMap, "saveDataFlg", m_save_data_Flg);
    PublicSpace::Log("saveDataFlg=%d\n", m_save_data_Flg);

    string tp;
    int flg2 = getMapData(configMap, "cutThw", tp);
    if (0 == flg2)
    {
        string2Vector(tp, m_cutSequence);
    }

    string tp2;
    vector<vector<double>> tp4;
    int flg3 = getMapData(configMap, "SimulateFre", tp2);
    if (0 == flg3)
    {
        string2Vector(tp, tp4);
    }
    // Log("m_cutSequence=%s\n", m_Snr_Threshold);

    if (0 == m_antnenaType)
    {
        getMapData(configMap, "r", m_R);
    }
    else
    {
        getMapData(configMap, "Radr", m_Radr);
    }
}
int SuppressDoa::setRadr(string adr)
{
    m_Radr = adr;
    setR(m_F, m_Radr);
    return 0;
}

// =============================================================================
// 根据项目类型初始化默认参数
// 各项目硬件平台配置不同:
//
// GN930 (三代):
//   - 3通道7天线，全向天线，UCA圆形阵列
//   - 切刀序列 {{1,2,3}, {1,4,5}, {1,6,7}, {2,4,6}, {2,5,7}, {3,4,7}, {3,5,6}}
//     每次切刀连接3个天线到3个通道（三元组模式）
//   - 默认使用联合对角化算法(算法3)，支持同频多信号测向
//   - 11个伪协方差矩阵用于联合对角化
//
// GN930U (三代升级):
//   - 2通道7天线，全向天线
//   - 切刀序列二元组模式: 通道0固定接天线1(参考)，通道1依次切换天线2~7
//   - 默认使用相关干涉仪算法(算法1)
//
// GN560 (小型化):
//   - 单通道7天线，定向天线(有方向性增益)
//   - 切刀序列环形连接: {1,2}, {2,3}, ..., {7,1}
//   - 4次移相(0/90/180/270)
//   - 5帧IQ幅度积分用于降噪
//   - 使用4个相邻天线进行测向
//   - 默认使用DML算法(算法5)
// =============================================================================
void SuppressDoa::InitProject()
{
    switch (m_Project_flg)
    {
    case GN930:
        m_cutSequence.clear();
        m_Doa_Arithmetic = 3;
        // 三元组切刀: 每次切刀连接3个天线，共6种组合覆盖7元阵列
        m_cutSequence = {{1, 2, 3}, {1, 4, 5}, {1, 6, 7}, {2, 4, 6}, {2, 5, 7}, {3, 4, 7}, {3, 5, 6}};
        m_AntennaNum = 7;
        m_channelNum = 3;
        m_R = 0.1865;   // 阵列半径(米)
        m_antnenaType = 0; // 全向天线
        m_RAll_Num = 11;   // 11个伪协方差矩阵
        m_RAll_PointNum = 1;
        break;
    case GN930U:
        m_cutSequence.clear();
        m_Doa_Arithmetic = 1;
        // 二元组切刀: 通道0接天线1(参考)，通道1接天线2~7
        m_cutSequence = {{1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
        m_AntennaNum = 7;
        m_antnenaType = 0; // 全向天线
        m_channelNum = 2;
        m_R = 0.2; // 阵列半径(米)
        break;
    case GN560:
        m_cutSequence.clear();
        m_Doa_Arithmetic = 1;
        // 环形切刀: 相邻天线依次配对
        m_cutSequence = {{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 1}};
        m_AntennaNum = 7;
        m_antnenaType = 1; // 定向天线
        m_channelNum = 1;  // 单通道
        m_Am_Density_Num = 5; // 幅度积分的次数,取值范围大于0
        m_ShiftPhaseNum = 4;  // 单通道移相次数，一般情况移相4次
        m_Doa_Cut_Num = 4;    // 用于测向的天线数
        m_R = 0.18;           // 阵列半径(米)
        m_Doa_Arithmetic = 5; // 覆盖为DML算法(实际GN560使用DML+定向天线)
        m_Simulate_Data_file = "./SimulateData/"; // 仿真阵列流型数据路径
        break;
    default:
        break;
    }
}