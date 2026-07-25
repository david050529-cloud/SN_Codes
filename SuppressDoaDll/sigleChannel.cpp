// =============================================================================
// 文件名: sigleChannel.cpp
// 功能描述: 单通道测向处理模块
//
//   单通道体制简介:
//   单通道测向系统中，只有一个接收通道，通过电子开关(切刀)分时连接
//   不同的天线。每次切刀连接两个天线(天线对)，对每个天线对进行4次移相
//   操作(0,90,180,270度)，采集IQ数据。
//
//   单通道的优势: 硬件成本低，体积小，适用于小型化设备(GN560)
//   单通道的挑战: 不能同时采集所有天线数据，需要移相来恢复两个通道的
//                 相位差信息，且移相器存在非理想性(幅度/相位误差)
// =============================================================================
//
// =============================================================================
// 单通道四次移相测相原理 (calPhaseBy4shiftPhase):
//
//   模型假设:
//     设有两路信号分别来自天线A1和A2(切刀连接的两个天线):
//       s1(t) = A1 * exp(j*phi1(t))   -> 天线1的信号
//       s2(t) = A2 * exp(j*phi2(t))   -> 天线2的信号
//
//   经过第j次移相(0,90,180,270度)后，合路输出为:
//       y_j(t) = s1(t) + errorA_j * exp(j*fai_j) * s2(t)
//     其中:
//       errorA_j: 第j次移相的幅度误差(线性系数，理想值为1)
//       fai_j:    第j次移相的相移量(理想值为 0, pi/2, pi, 3pi/2)
//
//   四次移相测量的功率值(即IQ平方和):
//       Am[j] = |A1 + A2 * errorA_j * exp(j*(delta_phi + fai_j))|^2
//
//     其中 delta_phi = phi2 - phi1 是两天线的相位差(待求)
//
//   展开:
//       Am[j] = A1^2 + (errorA_j * A2)^2 + 2*A1*errorA_j*A2*cos(delta_phi + fai_j)
//
//   对于理想情况(errorA_j=1, fai_j = 0,90,180,270):
//       Am[0] = A1^2 + A2^2 + 2*A1*A2*cos(delta_phi)
//       Am[1] = A1^2 + A2^2 - 2*A1*A2*sin(delta_phi)
//       Am[2] = A1^2 + A2^2 - 2*A1*A2*cos(delta_phi)
//       Am[3] = A1^2 + A2^2 + 2*A1*A2*sin(delta_phi)
//
//   传统解法 (无误差校正):
//       (Am[0] - Am[2]) + j*(Am[1] - Am[3])
//       = 4*A1*A2*[cos(delta_phi) + j*sin(delta_phi)] = 4*A1*A2*exp(j*delta_phi)
//       所以 delta_phi = arg((Am[0]-Am[2]) + j*(Am[1]-Am[3]))
//
//   带误差校正的Newton-Raphson迭代法:
//     由于实际移相器存在幅度误差(errorA_j != 1)和相位误差(fai_j偏离理想值)，
//     上述传统解法精度不够。需要使用非线性最小二乘迭代求解:
//
//     待求未知数: x = [A1, A2, delta_phi]
//     残差方程 (j=0..3):
//       F_j(x) = A1^2 + (errorA_j*A2)^2 + 2*A1*errorA_j*A2*cos(delta_phi+fai_j) - Am[j]
//
//     雅可比矩阵 (4x3):
//       dF_j/dA1 = 2*A1 + 2*errorA_j*A2*cos(delta_phi+fai_j)
//       dF_j/dA2 = 2*errorA_j^2*A2 + 2*errorA_j*A1*cos(delta_phi+fai_j)
//       dF_j/d(delta_phi) = -2*A1*errorA_j*A2*sin(delta_phi+fai_j)
//
//     Newton更新:
//       J^T * J * dx = -J^T * F  ->  dx = -(J^T*J)^{-1} * J^T * F
//       x_{new} = x - dx
//       A1 = |A1|, A2 = |A2| (非负约束)
//
// =============================================================================
//
// =============================================================================
// IQ幅度积分 (IQ Density Accumulation):
//
//   目的: 单通道每次移相采集的数据信噪比可能较低，通过多帧累积提高
//         IQ平方和的估计精度，从而改善相位差计算精度。
//
//   实现: 使用deque作为滑动窗口
//     - initAmDesity(): 创建m_Am_Density_Num个全零矩阵
//     - calDesityIqAm(): 队首弹出旧帧，队尾推入新帧(滑动窗口)
//     - getIqAm(): 将窗口内所有帧按元素累加，得到平滑后的IQ平方和
//
//   效果: 相当于对IQ功率做m_Am_Density_Num帧的移动平均
//         窗口越大，平滑效果越好，但响应速度越慢
//
// =============================================================================
#include "SuppressDoa.h"
#include "fft.h"

// =============================================================================
// getSigleChannelPhaseA: 单通道测向主流程
// 流程: IQ平方和 -> 幅度积分 -> 求解相位差和幅度
// 参数(输出):
//   phs: 各切刀天线对的相位差(弧度)
//   A:   各天线的总幅度(两根天线幅度之和)
// =============================================================================
void SuppressDoa::getSigleChannelPhaseA(vector<double> &phs, vector<double> &A)
{
    vector<vector<double>> Am;
    calIqSquareSum(Am);
    calDesityIqAm(Am);
    getIqAm(Am);
    vector<double> A1;
    vector<double> A2;
    getPhaseBy4shiftPhase(Am, phs, A1, A2);
    int size = (int)A1.size();
    A.clear();
    vector<double>().swap(A);
    A.resize(size);
    for (int i = 0; i < size; i++)
    {
        A[i] = A1[i] + A2[i];
        //    +A2[i];
    }
    Log("get sigle channel phase diff and Am sucess...\n");
}

// =============================================================================
// calIqSquareSum: 计算IQ平方和(I^2 + Q^2)
// 功能: 对每个切刀、每次移相、所有采样点计算I^2+Q^2之和
//   Am[i][j] = sum_k( |data[i][j][k]|^2 )
// 物理意义: Am[i][j]是切刀i在第j次移相时接收到的总功率
//   功率值反映了两个天线信号叠加后的幅度信息
//
// 附加功能: 根据最大平均幅度动态调整测向质量系数
//   maxMean越大(信号越强)，测向置信度系数越接近1.0
//   maxMean < 1  -> 系数0.1 (极弱信号)
//   1 <= maxMean < 5  -> 系数0.5 (弱信号)
//   5 <= maxMean < 50 -> 系数0.8 (中等信号)
//   maxMean >= 50     -> 系数1.0 (强信号,默认)
// =============================================================================
void SuppressDoa::calIqSquareSum(vector<vector<double>> &Am)
{
    Am.clear();
    vectorResize(Am, m_cutNum, m_ShiftPhaseNum);
    int start = 0;
    double sum = 0.0;
    for (int i = 0; i < m_cutNum; i++)
    {
        for (int j = 0; j < m_ShiftPhaseNum; j++)
        {
            sum = 0.0;
            for (int k = 0; k < m_usePointNum; k++)
            {
                double tp_img = m_Data[i][j][k].imag();
                double tp_real = m_Data[i][j][k].real();
                // I^2 + Q^2 = |IQ|^2 复数的模平方
                sum += tp_img * tp_img + tp_real * tp_real;
            }
            Am[i][j] = sum; // 总功率
        }
    }
    //计算获得测向质量系数: 根据信号功率动态调整置信度权重
    double maxMean = 0.0;
    for (int i = 0; i < m_cutNum; i++)
    {
        double mean = meanAm(Am[i]);
        if (mean > maxMean)
        {
            maxMean = mean;
        }
    }
    if (0<maxMean <1.0)
    {
        m_qulity_Coefficeicent = 0.1;  // 极弱信号，降低置信度
    }
    else if (maxMean >=1.0 && maxMean <5)
    {
        m_qulity_Coefficeicent = 0.5;  // 弱信号
    }
    else if (maxMean >= 5 && maxMean < 50)
    {
        m_qulity_Coefficeicent = 0.8;  // 中等信号
    }
    // else: maxMean >= 50, 保持在默认值1.0
    Log("get IQ data square sum :\n");
    for (int i = 0; i < m_cutNum; i++)
    {
        Log("%d--", i + 1);
        for (int j = 0; j < m_ShiftPhaseNum; j++)
        {
            Log("Am=%.2f,  ", Am[i][j]);
        }
        Log("\n");
    }
}

// =============================================================================
// calDesityIqAm: IQ幅度积分(密度累积)
// 实现滑动窗口: 队首弹出最旧的数据帧，队尾推入最新数据帧
// 窗口大小 = m_Am_Density_Num (默认5帧)
// 效果: 多帧移动平均，平滑噪声对IQ平方和的影响
// =============================================================================
void SuppressDoa::calDesityIqAm(const vector<vector<double>> Am)
{
    m_Am_Density.pop_front();    // 弹出最旧帧
    m_Am_Density.push_back(Am);  // 推入最新帧
}
// =============================================================================
// Variance: 计算归一化IQ平方和的方差
// 用于衡量数据的波动程度(数据质量评估)
// =============================================================================
double SuppressDoa::Variance(const vector<double>& data)
{
    if (data.empty())
    {
        return -1;
    }
    double sum = 0.0;
    int sizes = data.size();
    for (int i = 0; i < sizes; i++)
    {
        sum += data[i]/ m_usePointNum; // 归一化到每个采样点
    }
    double mean = sum / sizes; // 均值
    double sumSqDiff = 0.0;
    for (int i = 0; i < sizes; i++)
    {
        sumSqDiff += pow(data[i] / m_usePointNum - mean, 2); // 偏差平方和
    }
    return sumSqDiff / sizes; // 方差 = 偏差平方和 / 样本数
}

// =============================================================================
// meanAm: 计算归一化的幅度均值
// 归一化因子: 1/(m_usePointNum * m_bandwidth_coefficient)
// bandwidth_coefficient = bandwidth/10000, 带宽越大，归一化后的功率越小
// 返回值: 每采样点的平均功率
// =============================================================================
double SuppressDoa::meanAm(const vector<double>& data)
{
    if (data.empty())
    {
        return -1;
    }
    double sum = 0.0;
    int sizes = data.size();
    for (int i = 0; i < sizes; i++)
    {
        sum += data[i];
    }
    // 归一化: 除以采样点数和带宽系数
    sum  = sum / m_usePointNum / m_bandwidth_coefficient;
    double mean = sum / sizes;
    return mean;
}

// =============================================================================
// getIqAm: 获取积分后的IQ平方和
// 功能: 将滑动窗口(deque)中所有帧的Am按元素累加
// 效果: 相当于m_Am_Density_Num帧的移动平均
// 特殊处理: 如果窗口大小=1，则直接返回最新一帧
// 调试输出: 当移相4次时，额外输出 Am0+Am180 和 Am90+Am270
//   这两项在理想情况下分别等于 2*(A1^2+A2^2) (直流分量)
// =============================================================================
void SuppressDoa::getIqAm(vector<vector<double>> &Am)
{
    Am.clear();
    vectorResize(Am, m_cutNum, m_ShiftPhaseNum);
    if (1 == m_Am_Density_Num)
    {
        Am = m_Am_Density[0]; // 无积分，直接使用最新帧
        return;
    }
    for (int i = 0; i < m_cutNum; i++)
    {
        for (int j = 0; j < m_ShiftPhaseNum; j++)
        {
            Am[i][j] = 0.0; // 清零
        }
    }
    vector<vector<double>> tp;
    // 将所有帧累加(不除以帧数，保持与单帧相同的量级用于后续迭代)
    for (int k = 0; k < m_Am_Density_Num; k++)
    {
        tp = m_Am_Density[k];
        for (int i = 0; i < m_cutNum; i++)
        {
            for (int j = 0; j < m_ShiftPhaseNum; j++)
            {
                Am[i][j] += tp[i][j];
            }
        }
    }
    Log("get density IQ data square sum :\n");
    for (int i = 0; i < m_cutNum; i++)
    {
        Log("%d--", i + 1);
        for (int j = 0; j < m_ShiftPhaseNum; j++)
        {
            Log("Am=%.2f,  ", Am[i][j]);
        }
        // 当4次移相时: Am0+Am180 和 Am90+Am270 应对称相等
        // 理想: Am[0]+Am[2] = Am[1]+Am[3] = 2*(A1^2+A2^2)
        if (m_ShiftPhaseNum == 4)
        {
            Log("Am0+Am180=%.2f,  Am90+Am270=%.2f", Am[i][0] + Am[i][2], Am[i][1] + Am[i][3]);
        }
        Log("\n");
    }
}

// =============================================================================
// initAmDesity: 初始化IQ幅度积分队列
// 创建m_Am_Density_Num个全零矩阵的deque作为滑动窗口
// 在第一次接收数据前需要调用，确保deque中有足够的帧数
// =============================================================================
void SuppressDoa::initAmDesity()
{
    if (m_Am_Density_Num < 1)
    {
        m_Am_Density_Num = 1;
    }
    m_Am_Density.clear();
    vector<vector<double>> tp;
    vectorResize(tp, m_cutNum, m_ShiftPhaseNum);
    for (int i = 0; i < m_cutNum; i++)
    {
        for (int j = 0; j < m_ShiftPhaseNum; j++)
        {
            tp[i][j] = 0.0;
        }
    }

    for (int i = 0; i < m_Am_Density_Num; i++)
    {
        m_Am_Density.push_back(tp); // 填充全零矩阵
    }
    Log("Am_Density_Num=%d,init density is sucess...\n", m_Am_Density_Num);
}

// =============================================================================
// getPhaseBy4shiftPhase: 由积分后的IQ平方和计算相位差
// 对每个切刀天线对:
//   1. 查找对应的移相误差参数(m_shift_phase_arg)
//   2. 调用带误差校正的calPhaseBy4shiftPhase求解相位差和幅度
// 参数(输出):
//   phs: 各切刀的相位差(弧度)
//   A1:  天线1的幅度
//   A2:  天线2的幅度
// =============================================================================
void SuppressDoa::getPhaseBy4shiftPhase(const vector<vector<double>> Am, vector<double> &phs, vector<double> &A1, vector<double> &A2)
{
    phs.clear();
    A1.clear();
    A2.clear();
    phs.resize(m_cutNum);
    A1.resize(m_cutNum);
    A2.resize(m_cutNum);
    double tp_theta = 0.0; // 相位差
    double tp_A1 = 0.0;    // 幅度
    double tp_A2 = 0.0;
    double sumA = 0.0;

    double errorA[4] = {0.0};
    double fai[4] = {0.0};
    double tp_Am[4] = {0.0};
    // string a = Vector2String(m_cutSequence);
    Log("sigle channeal get phase by shift phase:cutNum=%d\n", m_cutNum);
    int index = -1;
    for (int i = 0; i < m_cutNum; i++)
    {
        index = -1;
        // 查找当前切刀对应的移相误差参数
        for (int k = 0; k < m_cutNum; k++)
        {
            if (m_shift_phase_arg[k].channel1 == m_cutSequence[i][0] && m_shift_phase_arg[k].channel2 == m_cutSequence[i][1])
            {
                index = k;
                break;
            }
        }

        if (index == -1)
        {
            Log("***eror:shift coffect data is loss...\n");
            return;
        }
        // 提取该切刀的误差参数和测量值
        for (int j = 0; j < m_ShiftPhaseNum; j++)
        {
            errorA[j] = m_shift_phase_arg[index].amChang[j]; // 幅度误差(线性)
            fai[j] = m_shift_phase_arg[index].shiftPhs[j];   // 相移误差(弧度)
            tp_Am[j] = Am[i][j];                              // 积分后的IQ平方和
        }
        Log("%d, %d------", m_shift_phase_arg[index].channel1, m_shift_phase_arg[index].channel2);
        // 带误差校正的Newton-Raphson迭代求解
        calPhaseBy4shiftPhase(errorA, fai, tp_Am, tp_theta, tp_A1, tp_A2);
        phs[i] = tp_theta;
        A1[i] = tp_A1;
        A2[i] = tp_A2;
        Log("----theta=%.2f,  A1=%.2f,  A2=%.2f,  A1+A2=%.2f\n", phs[i] * 180 / m_PI, A1[i], A2[i], A1[i] + A2[i]);
    }
}

// =============================================================================
// calPhaseBy4shiftPhase (带误差校正版本)
// 使用Newton-Raphson(Gauss-Newton)非线性最小二乘法求解相位差
//
// 待求未知量: x = [A1, A2, delta_phi]
//   A1, A2: 两天线的信号幅度
//   delta_phi: 两天线间的相位差
//
// 残差方程 (j=0..3, 对应4次移相):
//   F_j(x) = A1^2 + (errorA_j*A2)^2 + 2*A1*errorA_j*A2*cos(delta_phi+fai_j) - Am_j
//
// 雅可比矩阵 J (4×3):
//   dF_j/dA1 = 2*A1 + 2*errorA_j*A2*cos(delta_phi+fai_j)
//   dF_j/dA2 = 2*errorA_j^2*A2 + 2*errorA_j*A1*cos(delta_phi+fai_j)
//   dF_j/d(delta_phi) = -2*A1*errorA_j*A2*sin(delta_phi+fai_j)
//
// 迭代: dx = (J^T*J)^{-1} * J^T * F  ->  x = x - dx
// 约束: A1, A2 非负；delta_phi 在 [0, 2*pi) 范围内
//
// 初始值: 传统解析方法结果作为初值，加速收敛
// =============================================================================
void SuppressDoa::calPhaseBy4shiftPhase(const double *errorA, const double *fai, const double *Am, double &theta, double &A1, double &A2) // 利用四次移相计算相位差
{

    int max_iter = 100;            // 最大迭代次数
    double tolerance = 1e-5;      // 收敛容限
    double x[3] = { 75.0, 150.0, 0.0 }; // 初始值 [A1, A2, delta_phi]

    // 用传统方法计算初始值，加速迭代收敛
    calPhaseBy4shiftPhase( Am,x[2],x[0], x[1]);
    Eigen::MatrixXf F(4, 1);  // 残差向量
    Eigen::MatrixXf J(4, 3);  // 雅可比矩阵
    int tp_i;
    double tp_tolerance = tolerance;
    for (int i = 0; i < max_iter; i++)
    {
        tp_i = i;

        // 计算残差F和雅可比J
        for (int j = 0; j < 4; j++)
        {
            F(j, 0) = pow(x[0], 2) + pow((errorA[j] * x[1]), 2) + 2 * x[0] * errorA[j] * x[1] * cos(x[2] + fai[j]) - Am[j];
            J(j, 0) = 2 * x[0] + 2 * errorA[j] * x[1] * cos(x[2] + fai[j]);
            J(j, 1) = 2 * pow(errorA[j], 2) * x[1] + 2 * errorA[j] * x[0] * cos(x[2] + fai[j]);
            J(j, 2) = -2 * x[0] * errorA[j] * x[1] * sin(x[2] + fai[j]);
        }

        // Gauss-Newton法: dx = (J^T*J)^{-1} * J^T * F
        Eigen::MatrixXf transJ, H, B, intvH, tx;
        transJ = J.transpose();
        H = transJ * J;    // Hessian近似 J^T*J (3x3)

        B = transJ * F;    // 梯度 J^T*F (3x1)

        intvH = H.inverse(); // 求逆 (3x3矩阵求逆，计算量小)
        tx = intvH * B;    // 更新步长 (3x1)
        int flag = 1;

        // 检查收敛: 所有更新分量的绝对值是否都小于容限
        for (int k1 = 0; k1 < tx.rows(); k1++)
        {
            if (abs(tx(k1, 0)) > tolerance)
            {
                flag = 0;
                tp_tolerance = abs(tx(k1, 0));
            }
        }
        if (flag == 1)
        {
            break; // 收敛，退出迭代
        }
        else
        {
            // 更新变量
            for (int k2 = 0; k2 < tx.rows(); k2++)
            {
                x[k2] = x[k2] - tx(k2, 0);
            }
            // 约束处理
            x[0] = abs(x[0]); // A1 >= 0
            x[1] = abs(x[1]); // A2 >= 0
            int tp = floor(x[2] / (2 * m_PI));
            x[2] = x[2] - tp * 2 * m_PI; // delta_phi 归一化到 [0, 2*pi)
        }
    }
    Log("tolerance=%.8f,max_iter=%d", tp_tolerance, tp_i);
    A1 = x[0];
    A2 = x[1];
    theta = x[2];
}

// =============================================================================
// calPhaseBy4shiftPhase (传统解析方法，无误差校正)
// 适用于理想移相器或作为迭代法的初始值估计
//
// 公式推导:
//   设 Am[j] = |A1 + A2 * exp(j*(delta_phi + fai_j))|^2
//   理想情况下 fai = {0, pi/2, pi, 3*pi/2}:
//
//   Am[0] - Am[2] = 4*A1*A2*cos(delta_phi)
//   Am[1] - Am[3] = 4*A1*A2*sin(delta_phi)  (实际是 -4*A1*A2*sin(delta_phi),
//                                               取负变为 +4*A1*A2*sin)
//
//   构造复数: (Am[0]-Am[2]) + j*(Am[1]-Am[3]) = 4*A1*A2*exp(j*delta_phi)
//   所以 delta_phi = arg((Am[0]-Am[2]) + j*(Am[1]-Am[3]))
//
//   幅度求解:
//     AA2 = 4*A1*A2 (通过差值反推)
//     A2A2 = (Am[0]+Am[1]+Am[2]+Am[3])/4 = A1^2+A2^2 (直流分量)
//     解方程组: A1+A2 = sqrt(A2A2 + AA2/2)
//               |A1-A2| = sqrt(A2A2 - AA2/2)
//     最终: A1 = [(A1+A2) + (A1-A2)] / 2
//           A2 = [(A1+A2) - (A1-A2)] / 2
// =============================================================================
void SuppressDoa::calPhaseBy4shiftPhase(const double* Am, double& theta, double& A1, double& A2)
{
    // 构造复数: real = Am0-Am2, imag = Am1-Am3
    complex<double>tp(Am[0]-Am[2],Am[1]-Am[3]);
    double tp_phs = arg(tp); // delta_phi = arg(差值复数)
    double AA2;
    double A2A2;
    double A1_A2;
    // 通过 cos/sin 反推 4*A1*A2 (处理除零情况)
    if (abs(cos(tp_phs)) < 10e-5)
    {
        AA2 = (Am[1] - Am[3]) / sin(tp_phs);
    }
    else
    {
        AA2 = (Am[0] - Am[2]) / cos(tp_phs);
    }
    // A1^2+A2^2 = 四项功率的均值
    A2A2 = (Am[0]+Am[1]+Am[2]+Am[3]) / 4;
    // |A1-A2| = sqrt(|AA2 - A2A2|)   [取绝对值防止数值误差]
    A1_A2 = sqrt(abs(AA2 - A2A2));

    // A1+A2 = sqrt(A2A2 + AA2)
    double A1_A2_2 = sqrt(A2A2 + AA2);
    // 解二元一次方程组
    A1 = (A1_A2 + A1_A2_2) / 2; // 取较大的为A1
    A2 = (A1_A2_2 - A1_A2) / 2; // 较小的为A2
    theta = -tp_phs;            // 取负(与迭代法约定一致)
}
// =============================================================================
// 设置移相误差系数文件的路径
// =============================================================================
int SuppressDoa::setSignaleChannelCoefficeicentadr(string adr)
{
    m_shiftPhase_Coefficeicent_Adr = adr;

    return 0;
}

// =============================================================================
// 设置采样带宽，用于计算功率归一化系数
// bandwidth_coefficient = bandwidth / 10000
// 带宽越宽，信号功率按比例增大，需要归一化
// =============================================================================
int SuppressDoa::setBandwidth(int bandwidth)//设置采样带宽
{
    if (bandwidth > 0)
    {
        m_bandwidth_coefficient = bandwidth / 1e4;
        return 0;
    }
    return -1;
}

// =============================================================================
// sigleChannel4shiftInitl: 从CSV文件读取并初始化单通道移相误差参数
//
// 文件格式:
//   第一行: 列标签 "频率, 1-2-0-dB, 1-2-90-dB, 1-2-180-dB, 1-2-270-dB, ..."
//          标签含义: 天线1-天线2-移相度数-类型(dB=幅度, deg=相位)
//   后续行: 频率1, 值1, 值2, ...
//          频率2, 值1, 值2, ...
//
// 处理流程:
//   1. 解析列标签，提取每个天线对的移相参数标签
//   2. 查找当前工作频率m_F在频率表中的位置
//   3. 使用线性插值(linearInterpolation)计算当前频率对应的参数
//   4. dB值转为线性值: 10^(dB/20)
//   5. 度值转为弧度: deg * pi/180
//
// 示例标签: "1-2-0-dB" 表示天线1和天线2之间，0度移相时的幅度(dB)
//          "1-2-90-deg" 表示天线1和天线2之间，90度移相时的相位误差(度)
// =============================================================================
void SuppressDoa::sigleChannel4shiftInitl(const string adr)
{
    vector<string> tpVec;
    tpVec.clear();
    int flg = 0;
    flg = readFile(adr, tpVec);

    if (flg != 0)
    {
        Log("***error:get sigle channel error paranmeter adr is false!!!\n");
        return;
    }
    // 切刀对应的天线标签
    vector<string> a1;
    a1.clear();
    /// vector<string>().swap(a1);
    string &a = tpVec[0];
    split(a1, a, ',');
    int tmp_row = a1.size();
    int rowNum = tmp_row - 2;
    int *channel1= new int[rowNum]; // 切刀对应的天线序号
    int *channel2 = new int[rowNum];
    int *tmp_shiftPhs = new int[rowNum]; // 移相度数
   // string *dbDeg = {};      // 幅度还是相位标签
    vector<string>dbDeg;
    dbDeg.clear();

    for (int j = 2; j < tmp_row; j++)
    {
        // cout << row << endl;
        if (a1[j].empty())
        {
            channel1[j - 2] = channel1[j - 3];
            channel2[j - 2] = channel2[j - 3];
        }
        else
        {
            vector<string> a3;
            a3.clear();
            vector<string>().swap(a3);
            split(a3, a1[j], '-');
            if (a3.size() > 1)
            {
                channel1[j - 2] = stod(a3[0]);
                channel2[j - 2] = stod(a3[1]);
                tmp_shiftPhs[j - 2] = stod(a3[2]);
               // dbDeg[j - 2] = a3[3];
                dbDeg.emplace_back(a3[3]);
            }
            a3.clear();
            vector<string>().swap(a3);
        }
    }

    a1.clear();
    vector<string>().swap(a1);
    double *tmp_error1 = new double[rowNum + 1];
    // double all_error[rowNum];
    double tmp_fre = 0.0;
    int cutNum = m_cutSequence.size();
    ShiftPhaseArg *tmp_shift_phase_arg = new ShiftPhaseArg[cutNum];
    for (int i = 0; i < cutNum; i++)
    {
        tmp_shift_phase_arg[i].channel1 = m_cutSequence[i][0];
        tmp_shift_phase_arg[i].channel2 = m_cutSequence[i][1];
    }

    // 注意：频率是否超过频段范围
    for (int i = 1; i < tpVec.size(); i++)
    {
        string &a1 = tpVec[i];
        vector<string> a2;
        a2.clear();
        vector<string>().swap(a2);
        split(a2, a1, ',');
        double fre = stod(a2[0]);
        if (fre >= m_F)
        {
            for (int j = 1; j < a2.size(); j++)
            {
                double tmp1 = tmp_error1[j - 1];
                double tmp2 = stod(a2[j]);
                double tmp = linearInterpolation(tmp_fre, tmp1, fre, tmp2, m_F);
                //  cout << "--->" << tmp << endl;
                if (j != 1)
                {

                    for (int k = 0; k < cutNum; k++)
                    {

                        if (tmp_shift_phase_arg[k].channel1 == channel1[j - 2] && tmp_shift_phase_arg[k].channel2 == channel2[j - 2])
                        {
                            if (dbDeg[j - 2].compare("dB") == 0 || dbDeg[j - 2].compare("DB") == 0)
                            {

                                if (tmp_shiftPhs[j - 2] == 0)
                                {
                                    tmp_shift_phase_arg[k].amChang[0] = tmp;
                                }
                                if (tmp_shiftPhs[j - 2] == 90)
                                {
                                    tmp_shift_phase_arg[k].amChang[1] = tmp;
                                }
                                if (tmp_shiftPhs[j - 2] == 180)
                                {
                                    tmp_shift_phase_arg[k].amChang[2] = tmp;
                                }
                                if (m_ShiftPhaseNum == 4)
                                {
                                    if (tmp_shiftPhs[j - 2] == 270)
                                    {
                                        tmp_shift_phase_arg[k].amChang[3] = tmp;
                                    }
                                }
                            }
                            if (dbDeg[j - 2].compare("deg") == 0 || dbDeg[j - 2].compare("DEG") == 0)
                            {

                                if (tmp_shiftPhs[j - 2] == 0)
                                {
                                    tmp_shift_phase_arg[k].shiftPhs[0] = tmp;
                                }
                                if (tmp_shiftPhs[j - 2] == 90)
                                {
                                    tmp_shift_phase_arg[k].shiftPhs[1] = tmp;
                                }
                                if (tmp_shiftPhs[j - 2] == 180)
                                {
                                    tmp_shift_phase_arg[k].shiftPhs[2] = tmp;
                                }
                                if (m_ShiftPhaseNum == 4)
                                {
                                    if (tmp_shiftPhs[j - 2] == 270)
                                    {
                                        tmp_shift_phase_arg[k].shiftPhs[3] = tmp;
                                    }
                                }
                            }
                            break;
                        }
                    }
                }
            }
            break;
        }

        for (int k1 = 1; k1 < a2.size(); k1++)
        {
            tmp_error1[k1 - 1] = stod(a2[k1]);
            tmp_fre = fre;
        }
    }

    // m_shift_phase_arg = tmp_shift_phase_arg;
    for (int s = 0; s < cutNum; s++)
    {
        m_shift_phase_arg[s].channel1 = tmp_shift_phase_arg[s].channel1;
        m_shift_phase_arg[s].channel2 = tmp_shift_phase_arg[s].channel2;
        for (int s2 = 0; s2 < m_ShiftPhaseNum; s2++)
        {
            double tmp2 = tmp_shift_phase_arg[s].amChang[s2] / 20;
            m_shift_phase_arg[s].amChang[s2] = pow(10, tmp2);
            m_shift_phase_arg[s].shiftPhs[s2] = tmp_shift_phase_arg[s].shiftPhs[s2] * m_PI / 180;
        }
    }
    tpVec.clear();
    vector<string>().swap(tpVec);
    a1.clear();
    vector<string>().swap(a1);
}

// =============================================================================
// getFFTAm: 单通道多信号测向的FFT处理方法
//
// 目的: 对于每个天线对(切刀)，利用4次移相在每个采样点求解相位差和幅度，
//       然后通过FFT将不同频率分量分离，再IFFT重构每个天线的IQ时域信号。
//
// 算法流程:
//   步骤1: 对每个切刀、每次移相的时域数据做FFT变换
//          Am[i][j][k] = FFT后第k个频点的幅度^2 (I^2+Q^2)
//
//   步骤2: 对FFT的每个频点k:
//          - 使用4次移相的4个Am值，调用calPhaseBy4shiftPhase求解
//            该频点处的相位差(tp_theta)和幅度(tp_A1, tp_A2)
//          - 通过相位差累积，重建该天线接收入射信号的等效相位
//            (利用相邻切刀的相位关系进行相位连接)
//
//   步骤3: IFFT变换: 将恢复的每个天线的频域IQ数据变换回时域
//
//   步骤4: 输出 Am_Phs[天线索引][采样点] = 恢复的时域IQ复数数据
//          供后续联合对角化分离和测向使用
//
// 关键点:
//   - 单通道不能同时采样所有天线，但通过对每个采样点执行4次移相求解，
//     可以逐点恢复各天线接收信号的相对关系
//   - FFT用于频域分离同频但不同调制频率的分量
//   - 相位连接: phs2[i][k] = phs2[i-1][k] - phs[i-1][k] 保证相位连续性
// =============================================================================
void SuppressDoa::getFFTAm(vector<vector<complex<double>>> &Am_Phs)
{ // 计算fft对应的每个点的幅度

    splab::Vector<complex<double>> yn,syn;
    yn.resize(m_usePointNum);
    vector<vector<vector<double>>> Am;
    Am.resize(m_cutNum);
    double tp_theta = 0.0; // 相位差
    double tp_A1 = 0.0;    // 幅度
    double tp_A2 = 0.0;
    double errorA[4] = {0.0};
    double fai[4] = {0.0};
    double tp_Am[4] = {0.0};
    vector<vector<double>> A1;
    vector<vector<double>> A2;
    vector<vector<double>> phs;
    vector<vector<double>> phs2;
   // vector<vector<complex<double>>> Am_Phs;
    A1.resize(m_AntennaNum);
    A2.resize(m_AntennaNum);
    phs.resize(m_AntennaNum);
    phs2.resize(m_AntennaNum);
    Am_Phs.resize(m_AntennaNum);
    splab::Vector<complex<double>> iyn,siyn;

    // 步骤1: 对每个切刀的每次移相数据做FFT，计算每个频点的幅度平方
    for (int i = 0; i < m_cutNum; i++)
    {
        Am[i].resize(m_ShiftPhaseNum);
        A1[i].resize(m_usePointNum);
        A2[i].resize(m_usePointNum);
        phs[i].resize(m_usePointNum);
        phs2[i].resize(m_usePointNum);
        Am_Phs[i].resize(m_usePointNum);

        for (int j = 0; j < m_ShiftPhaseNum; j++)
        {
            Am[i][j].resize(m_usePointNum);
            for (int k = 0; k < m_usePointNum; k++)
            {
                yn[k] = m_Data[i][j][k]; // 时域IQ数据
            }
            syn =  splab::fft(yn); // FFT变换到频域
            for (int k = 0; k < m_usePointNum; k++)
            {
                double tp_img = syn[k].imag();
                double tp_real = syn[k].real();
                Am[i][j][k] = tp_img * tp_img + tp_real * tp_real; // 频域功率谱
            }
        }
    }
    int index = -1;

    // 步骤2: 对每个频点逐点求解相位差和幅度
    iyn.resize(m_usePointNum);
    for (int i = 0; i < m_cutNum; i++)
    {
        index = -1;
        // 查找当前切刀的移相误差参数
        for (int k = 0; k < m_cutNum; k++)
        {
            if (m_shift_phase_arg[k].channel1 == m_cutSequence[i][0] && m_shift_phase_arg[k].channel2 == m_cutSequence[i][1])
            {
                index = k;
                break;
            }
        }
        if (index == -1)
        {
            Log("***eror:shift coffect data is loss...\n");
            return;
        }
        // 对每个频点执行4次移相求解
        for (int k = 0; k < m_usePointNum; k++)
        {
            for (int j = 0; j < m_ShiftPhaseNum; j++)
            {
                errorA[j] = m_shift_phase_arg[index].amChang[j];
                fai[j] = m_shift_phase_arg[index].shiftPhs[j];
                tp_Am[j] = Am[i][j][k];
            }
            calPhaseBy4shiftPhase(errorA, fai, tp_Am, tp_theta, tp_A1, tp_A2); // Newton迭代求解
            double tp_diff_phs = 0.0;
            A1[i][k] = tp_A1;
            A2[i][k] = tp_A2;
            phs[i][k] = tp_theta;
            // 相位累积: 利用相邻切刀的相位关系连接
            if (i > 0)
            {
                tp_diff_phs = phs2[i-1][k] - phs[i-1][k]; // 相位差累积
            }
            // 构造该天线在该频点的IQ复数 (使用A1幅度和累积相位)
            iyn[k].imag(tp_A1*sin(tp_diff_phs));
            iyn[k].real(tp_A1 * cos(tp_diff_phs));
            phs2[i][k] = tp_diff_phs;
        }
        // 步骤3: IFFT变换回时域
        siyn = splab::ifft(iyn);
        for (int k = 0; k < m_usePointNum; k++)
        {
            Am_Phs[i][k] = siyn[k]; // 恢复的时域IQ数据
        }
    }
}

// =============================================================================
// getMuiltSignalBySigleChannel: 单通道多信号测向主入口
// 流程:
//   1. getFFTAm: FFT频域分离 + 逐点4次移相求解 + IFFT重构
//   2. getJointdiagByAP: 对重构的7天线IQ数据执行联合对角化+幅相法测向
// 注意: 单通道模式下所有7个天线作为全通道处理(切刀只有1刀 = 全天线)
// =============================================================================
void SuppressDoa::getMuiltSignalBySigleChannel()
 {
    vector<vector<complex<double>>> Am_Phs;
    getFFTAm(Am_Phs); // FFT处理获得各天线的时域IQ信号
    vector < vector<vector<complex<double>>>>Am_Phs2;
    Am_Phs2.resize(1);
    Am_Phs2[0] = Am_Phs;
    // 全天线切刀(7个天线全部作为"一刀")
    vector<vector<int>> CutSequence = { {1,2,3,4,5,6,7} };
    // 联合对角化分离 + 幅相法测向
    SuppressDoa::getJointdiagByAP(Am_Phs2,
        CutSequence);
}
/*void SuppressDoa::getIFFTData(const vector<vector<double>> A1, const vector<vector<double>> phs, vector<vector<double>> &ifftIQ)
{
    int sizes = phs.size();
    ifftIQ.clear();
    ifftIQ.resize(sizes);

    for (int i = 0; i < sizes; i++)
    {
        int len = phs[i].size();
        Vector<complex<double>> yn(len);
        for (int j = 0; j < len; j++)
        {
            complex<double> tp_cdata = A1[i][j] * exp(phs[i][j] * sqrt(-1));

            yn[j] = tp_cdata;
        }
        ifft(yn);
    }
}*/
