// =============================================================================
// 文件名: SuppressDoa.h
// 功能描述: 抑制式测向（SuppressDoa）模块的主类声明
// 本模块支持多种测向体制，包括:
//   - 相关干涉仪测向（单信号）
//   - 联合对角化 + 相关干涉仪多信号测向（同频多信号分离）
//   - 单通道四次移相 + FFT 多信号测向
//   - 幅相法测向（定向天线/全向天线）
//   - DML/MUSIC 测向预留
// 适用项目: GN902 / GN930 / GN930U / GN560 系列
// =============================================================================
#pragma once
#include <math.h>
#include <stdarg.h>
#include <vector>
#include <queue>
#include <map>
#include <mutex>
#include <time.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <iostream>
#include <complex>

#include<chrono>
#include "../publicFunctionDoa/publicFunctionDoa.h"
#include "../Arithmetic/arithmetic.h"
using namespace chrono;
using namespace Eigen;
using namespace std;
using namespace PublicSpace;
extern char Version[];

// 匿名命名空间，定义全局常量
namespace
{
    const complex<double> COMPLEX_ZERO = complex<double>(0.0, 0.0); // 复数零
    const complex<double> COMPLEX_ONE = complex<double>(1.0, 0.0);  // 复数单位元 1+0i
    const double MIN_ZERO = 10e-11;                                  // 极小值阈值，用于防止除零
}

#pragma pack(1)
// =============================================================================
// 数据结构: ShiftPhaseArg
// 描述: 单通道四次移相误差参数，用于移相误差校正
// 单通道体制下，通过电子开关（切刀）在不同天线之间切换，每次切换会引入
// 幅度和相位误差，需要预先标定这些误差并在计算相位差时进行补偿。
//
// 成员说明:
//   channel1 / channel2 : 切刀所连接的两个天线序号（1-based）
//                         例如 channel1=1, channel2=2 表示天线1和天线2之间的通道
//   shiftPhs[4]          : 四次移相的实际相位值（弧度）
//                         理想值为 {0, pi/2, pi, 3*pi/2}，即 {0, 90, 180, 270}度
//                         因移相器非理想性，实际值会有偏差，需标定后使用
//   amChang[4]           : 四次移相对应的幅度变化（线性值，非dB）
//                         理想值为 {1, 1, 1, 1}，因移相器引入幅度误差
//                         从文件读取为dB值，初始化时转换为线性值: 10^(dB/20)
// =============================================================================
struct ShiftPhaseArg
{
    int channel1; // 切刀对应的天线序号（第一通道）
    int channel2; // 切刀对应的天线序号（第二通道）
    double shiftPhs[4]; // 移相相位，移相相位依次为0、90、180、270
    double amChang[4];  // 幅度变化
};
#pragma pack()

// =============================================================================
// SuppressDoa 类: 抑制式测向核心类
// 继承自 ArithmeticDoa，提供完整的测向数据处理流水线:
//   1. 初始化 (initSuppressDoa): 根据频率/阵列参数计算理论相位差
//   2. 设置校正数据 (setCorrectionData): 计算通道间幅相校正系数
//   3. 设置测向数据 (setData): 接收IQ数据并按切刀组织
//   4. 测向计算 (getAngleSuppressDoa): 根据配置选择算法执行DOA估计
//
// 支持的测向算法 (m_Doa_Arithmetic):
//   1 = 相关干涉仪测向 (单信号)
//   2 = 幅相法 (单信号)
//   3 = 联合对角化 + 干涉仪 (多信号盲分离 + 相关干涉仪)
//   4 = MUSIC (预留)
//   5 = DML (确定性最大似然，用于GN560定向天线)
//   6 = 沃森瓦特 (预留)
//   7 = 比幅测向 (预留)
//
// 天线体制支持:
//   - 全向天线 (m_antnenaType = 0): 相位差-幅度，全360度搜索
//   - 定向天线 (m_antnenaType = 1): 根据信号强度选择天线扇区，限定搜索范围
//
// 通道体制支持:
//   - 单通道 (m_channelNum = 1): 通过电子开关(切刀)分时切换天线 + 4次移相
//   - 多通道 (m_channelNum > 1): 多通道同步采集，通过切刀覆盖所有天线对
//
// 数据组织: m_Data[切刀索引][通道/移相次数索引][采样点索引]
// =============================================================================
class SuppressDoa : public ArithmeticDoa
{
public:
    // 构造函数: 设置采样点数、起始/结束采样点位置
    // pointNum: FFT采样点数，startPointNum/endPointNum用于截去拖尾
    SuppressDoa(int pointNum, int startPointNum, int endPointNum);

    ~SuppressDoa();

    // 初始化测向: 设置工作频率f(Hz)，计算理论相位差/阵列流型
    int initSuppressDoa(double f); // 初始化

    // 设置测向IQ数据: data数组按[切刀][通道][IQ交替]顺序排列
    // length为IQ数据对的总长度（即实部+虚部的组数）
    int setData(short *data, int length); // 设置数据

    // 设置通道校正数据: 计算各通道相对于参考通道的幅相校正系数
    // 校正系数 m_Correction = mean(channel_j / channel_0) 按采样点平均
    int setCorrectionData(short *data, int length); // 设置校正数据

    // 设置同频信号个数: 本模块支持同频多信号盲分离测向
    int setSignalNum(int signalNum); // 设置信源个数

    // 获取测向结果: 输出信号数、角度、置信度、幅度
    int getAngleSuppressDoa(int &signalNum, double *angles, double *qualities, double *amplitudes);

    int getSpectrumSupressDoa(int &signalNum, short *spectrumData, int &length); // 频谱数据

    int setStartEndPointNum(int startPointNum, int endPointNum);

    int setRadr(string adr); // 设置定向天线各个频率对应的孔径 文件

    int setSignaleChannelCoefficeicentadr(string adr); // 设置单通道移相误差系数文件的地址

    int setBandwidth(int bandwidth);//设置采样带宽

private:

    // ============================================================================
    // 方法分组说明:
    // 相关干涉仪 (interf.cpp)       - 全向/定向天线的相关干涉仪匹配测向
    // 联合对角化 (jointdiag.cpp)     - 同频多信号盲分离(伪协方差矩阵+联合对角化)
    // 单通道 (sigleChannel.cpp)      - 4次移相、移相误差、迭代计算相位差及幅度
    //                                 IQ幅度积分、FFT多信号处理
    // 多通道                          - 通道数-切刀数（不包含校正数据）-切刀顺序
    // 定向天线                        - 幅相法/相关干涉仪，基于幅度选择天线扇区
    // 积分                            - 单通道IQ幅度积分(密度累积)、测向结果角度积分
    // ============================================================================

    // ============================================================================
    // interf.cpp: 相关干涉仪测向方法
    //   核心思想: 将实测相位差向量与理论相位差库进行匹配，找到最优匹配角度
    //   评估指标: 相关系数/欧氏距离，最大值对应最可能的来波方向
    // ============================================================================
    // 相关干涉仪测向主流程: 获取相位差 -> 选择天线 -> 构建干涉仪数据结构 -> 匹配搜索
    void setInterf();
    // 设置相关干涉仪测向数据（内部重载，用于直接写入结果）
    void setInterfInfo(double &angles, double &qulities); // 设置相关干涉仪测向数据
    // 设置相关干涉仪测向数据: 将切刀序列和相位差封装成InterferInfo结构
    void setInterfInfo(const vector<vector<int>> cutSequence, const vector<double> phs, InterferInfo &info);

    // ============================================================================
    // jointdiag.cpp: 联合对角化、多信号测向方法
    //   原理: 利用多个伪协方差矩阵的联合对角化，实现同频多信号的盲源分离
    //   流程: 构建伪协方差矩阵(RAll) -> 特征分解初始化 -> 联合对角化求解混矩阵W
    //         -> 获得分离后的阵列流型A -> 对每个分离信号执行相关干涉仪测向
    // ============================================================================
    // 获得所有伪协方差矩阵: m_RAll_Num个延迟相关的伪协方差矩阵
    void getRAll(vector<MatrixXcd> &RAll); // 获得所有的伪协方差
    // 联合对角化获得实际阵列流型(分离后的导向矢量)
    void getJointA(const vector<MatrixXcd> &RAll, MatrixXcd &A, int sigNum); // 获得实际阵列流行
    // Uwedge算法: 联合对角化的核心迭代求解，计算解混矩阵W
    // 基于Newton-Raphson迭代，最小化非对角元素的Frobenius范数
    // 输入M: 多个对角化后的矩阵堆叠，输出W: 解混矩阵
    void Uwedge(const MatrixXcd &M, MatrixXcd &W); // 计算解混矩阵
    // 联合对角化 + 相关干涉仪测向（默认切刀序列）
    void getJointdiagByInterf(vector<double> &angles, vector<double> &qualities); // 联合对角化+相关干涉仪
    // 联合对角化 + 相关干涉仪测向（使用成员变量）
    void getJointdiagByInterf();
    // 联合对角化 + 幅相法测向: 由外部提供数据和切刀序列
    void getJointdiagByAP(const std::vector<std::vector<std::vector<std::complex<double>>>> data,
                          const vector<vector<int>> CutSequence);//联合对角化+幅相法
    // 获得伪协方差矩阵（外部数据和切刀序列版本）
    void getRAll(vector<MatrixXcd> &RAll,
                 const std::vector<std::vector<std::vector<std::complex<double>>>> data,
                 const vector<vector<int>> CutSequence);

    // ============================================================================
    // sigleChannel.cpp: 单通道测向方法
    //   单通道体制: 一个接收通道通过电子开关(切刀)分时连接不同天线
    //   每个切刀位置(两个天线)进行4次移相(0,90,180,270度)采集IQ数据
    //   IQ平方和 -> IQ幅度积分(密度累积) -> 牛顿迭代求解相位差和幅度
    //   多信号: 对每个采样点做4次移相求解 -> FFT频域分离 -> 联合对角化测向
    // ============================================================================

    // 利用四次移相计算相位差（带误差校正的Newton-Raphson迭代法）
    // errorA[4]: 四次移相的幅度误差系数(线性值)
    // fai[4]: 四次移相的实际相移值(弧度)
    // Am[4]: 四次移相测量的IQ平方和(功率值)
    // 输出: theta=两天线之间的相位差, A1/A2=两个天线的信号幅度
    // 核心方程: |A1*exp(j*phi1) + A2*errorA[j]*exp(j*(phi2+fai[j]))|^2 = Am[j], j=0..3
    void calPhaseBy4shiftPhase(const double *errorA, const double *fai, const double *Am, double &theta, double &A1, double &A2); // 利用四次移相计算相位差

    // 传统方法计算相位差（无误差校正，适用于理想移相器）
    // 利用公式: (Am[0]-Am[2]) + j*(Am[1]-Am[3]) 直接计算相位差和幅度
    void calPhaseBy4shiftPhase(const double* Am, double& theta, double& A1, double& A2);//传统方法计算相位差

    // 单通道相关干涉仪算法主流程
    void setSigleChannelInterferDoa(); // 单通道相关干涉仪算法

    // 从文件读取并解析单通道移相误差参数，线性插值到当前工作频率
    void sigleChannel4shiftInitl(const string adr); // 获取移相误差参数

    // 初始化IQ幅度积分队列: 创建m_Am_Density_Num个全零矩阵的deque
    void initAmDesity(); // 初始化积分

    // 计算IQ平方和: 对每个切刀的每次移相，累加I^2+Q^2
    void calIqSquareSum(vector<vector<double>> &Am); // 计算IQ平方和

    // IQ幅度积分(密度累积): 将当前帧的Am推入deque队尾，队首弹出
    // 实现滑动窗口式的多帧累积
    void calDesityIqAm(const vector<vector<double>> Am); // IQ 幅度积分

    // 获取积分后的IQ平方和: 将deque中所有帧的Am按元素累加
    void getIqAm(vector<vector<double>> &Am); // 获得积分后的IQ平方和

    // 由积分后的IQ平方和计算相位差: 调用calPhaseBy4shiftPhase
    void getPhaseBy4shiftPhase(const vector<vector<double>> Am, vector<double> &phs, vector<double> &A1, vector<double> &A2);

    // 单通道测向主流程: 计算IQ平方和 -> 积分 -> 求解相位差和幅度
    void getSigleChannelPhaseA(vector<double> &phs, vector<double> &A); // 获得单通道测向的相位差和幅度信息

    // 计算IQ平方和的方差（用于评估数据质量）
    double Variance(const vector<double> &data);

    // 计算IQ平方和的均值（归一化到采样点数，用于评估信号强度）
    double meanAm(const vector<double>& data);

    // 单通道多信号测向: 对每个采样点执行4次移相求解 -> FFT -> IFFT重构 -> 联合对角化
    // Am_Phs[天线索引][采样点]: 每个天线恢复的IQ时域信号
    void getFFTAm(vector<vector<complex<double>>>& Am_Phs);

    // 单通道多信号测向主入口: FFT处理 + 联合对角化 + 幅相法
    void getMuiltSignalBySigleChannel();

    // ============================================================================
    // 定向天线相关方法
    //   定向天线有方向性，不同扇区(天线)的信号强度不同
    //   通过比较各天线的接收幅度，选择信号最强的几个天线及其相邻天线进行测向
    //   同时根据最强天线位置限定角度搜索范围
    // ============================================================================
    // 定向天线: 将幅度和相位差数据组装成InterferInfo结构用于测向
    void setInterferInfoDataDirect(const vector<double> Am, const vector<double> thete, vector<InterferInfo> &inferInfoData); // 定向天线，用于测向的数据

    // 根据信号强度选择用于测向的天线: 幅度排序后选择最强天线及其相邻天线
    // A: 各天线的接收幅度(dB), useIndex: 输出选用的天线序号
    // startAngle/endAngle: 输出角度搜索范围
    void getUseAntennaBySignalStrength(const vector<double> A, vector<int> &useIndex, int &startAngle, int &endAngle); // 由信号强度得到用于测向的切刀序号

    // 多通道: 计算所有切刀的相位差和幅度
    // cutSequence: 输出实际的切刀天线对, phaseDiff: 输出相位差(弧度), A: 输出幅度(dB)
    void getMuiltChannelPhaseA(vector<vector<int>> &cutSequence, vector<double> &phaseDiff, vector<double> &A); // 多通道，获得相位差和幅度

    // 重新计算理论相位差（根据当前频率和阵列半径）
    void setTheoryPhaseDiff();

    // 由InterferInfo列表执行相关干涉仪DOA估计，选取置信度最高的结果
    void setAngleInterferDoa(const vector<InterferInfo> inferInfoData);

    // 根据频率设置阵列半径R（定向天线从文件读取频率-半径对应关系）
    void setR(const double f, const string adr);

    // 保存原始IQ数据到文件（用于调试和数据回放）
    void saveData(int dataFlg, short *data, int length); // 保存原始数据

    // 从配置文件读取所有参数
    void setConfigData(const string adr);

    // 根据参数选择使用的算法进行测向
    // 原则：不同测向方法之间互不干扰
    // 参数包括：定向天线/全向天线，使用的算法编号
    // 选择逻辑:
    //   - 算法1(干涉仪): 单信号 -> setInterf()
    //   - 算法3(联合对角化): 单信号 -> setInterf(), 多信号 -> getJointdiagByInterf()
    //   - 单通道: 单信号 -> setInterf(), 多信号 -> getMuiltSignalBySigleChannel()
    int setAngleSuppressDoa(); // 根据参数选择使用的算法进行测向，原则：不同测向方法之间互不干扰；参数包括：定向天线/全向天线，使用算法

    // 根据项目类型初始化默认参数 (GN902/GN930/GN930U/GN560)
    void InitProject();

    // 读取日志配置
    void getLogFlg(const string adr);
    // 文件写入

private:
    // ============================================================================
    // 项目类型枚举
    //   GN902:  二代产品
    //   GN930:  三代产品 (3通道7天线, 联合对角化)
    //   GN930U: 三代升级 (2通道7天线, 参考通道+切换)
    //   GN560:  小型化产品 (单通道7天线, 定向天线, 4次移相, DML测向)
    // ============================================================================
    enum ProjectNum
    {
        GN902,
        GN930,
        GN930U,
        GN560
    };
    int m_Project_flg = GN560; // 项目
    int m_Doa_Arithmetic = 1;  // 测向所用的算法；1 相关干涉仪；2 幅相法；3 多信号测向：联合对角化+干涉仪；4 MUSIC ；5 DML;6 沃森瓦特；7 比辐测向

    // ============================================================================
    // 通用参数
    // ============================================================================
    // vector<vector<int>> m_cutSequence = {{1, 2, 3}, {1, 4, 5}, {1, 6, 7}, {2, 4, 6}, {2, 5, 7}, {3, 4, 7}, {3, 5, 6}};
    // 切刀顺序: 每个子vector表示一次切刀所连接的天线序号(1-based)
    // 例如 {1,2} 表示天线1和天线2被同时连接到通道
    // 不包含校正通道的序列，校正数据通过setCorrectionData单独设置
    vector<vector<int>> m_cutSequence = {{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 1}}; // 切刀顺序,不带校正的序列
    int m_signalNum = 1;                                                                          // 信号个数（同频多信号场景）
    int m_AntennaNum = 7;                                                                         // 天线个数
    int m_antnenaType = 0;                                                                        // 天线类型，0 为全向天线，1 为定向天线
    double m_F = 0.0;                                                                             // 测向频率(Hz)
    int m_pointNum = 1024;                                                                        // 采样点数（FFT点数）
    int m_startPointNum;                                                                          // 开始点数，防止出现拖尾现象（滤波器暂态响应）
    int m_endPointNum;                                                                            // 结束点数
    int m_usePointNum;                                                                            // 可用点数 = endPointNum - startPointNum + 1
    int m_channelNum = 3;                                                                         // 通道数（单通道=1, 多通道>=2）
    int m_cutNum;                                                                                 // 切刀次数 = m_cutSequence.size()
    string m_LogFile = "/suppressDoaLog_";       // 日志文件路径
    vector<double> m_angle_Density;              // 测向结果角度积分（角度域密度累积，提高稳定性）
    double m_angle_AccumulationTimeFactor = 0.9; // 测向结果积分遗忘系数（指数平滑因子）

    // ============================================================================
    // 理论与仿真参数
    // ============================================================================
    std::vector<std::vector<double>> m_Theory;             // 保存理论相位差（天线对间的理论相位差矩阵，NxN）
    std::vector<std::vector<complex<double>>> m_SimulateA; // 保存仿真阵列流型（用于DML/幅相法匹配测向）
    string m_Simulate_Data_file = "/SimulateData/";        // 阵列仿真数据路径

    // ============================================================================
    // 单通道参数
    // ============================================================================
    //  vector<vector<double>> m_Am_Density; // 幅度积分数据 [旧版本: 简单vector]
    // IQ幅度积分队列: 使用deque实现滑动窗口累积，窗口大小=m_Am_Density_Num
    // 每次新数据到来时队首弹出旧数据、队尾推入新数据，实现多帧平滑降噪
    deque<vector<vector<double>>> m_Am_Density;
    int m_Am_Density_Num = 5;                                  // 幅度积分的次数(滑动窗口大小),取值范围大于0
    string m_shiftPhase_Coefficeicent_Adr = "./Doa_R_Arg.csv"; // 移相误差系数保存路径(CSV格式)
    int m_ShiftPhaseNum = 4;                                   // 单通道移相次数，一般情况移相4次(0/90/180/270)
    ShiftPhaseArg m_shift_phase_arg[10];                       // 单通道移相误差参数数组（最多10组切刀天线对）
    float m_qulity_Coefficeicent = 1.0;//测向质量的系数（根据信号强度动态调整，强信号=0.8, 弱信号=0.1）
    float m_bandwidth_coefficient = 1.0;//由采样带宽计算功率系数(用于归一化幅度，bandwidth/1e4)

    // ============================================================================
    // 多通道参数
    // ============================================================================
    // 校正系数: 长度=通道数(或单通道时的移相次数)
    // m_Correction[j] = mean(ch_j / ch_0) 第j通道相对参考通道的幅相校正系数
    std::vector<std::complex<double>> m_Correction;                     // 校正系数,只有一帧数据，vector是通道数量
    // 测向数据三维存储: [切刀索引][通道/移相次数索引][采样点索引]
    // 切刀索引: 0 ~ m_cutNum-1, 通道索引: 0 ~ m_channelNum-1(单通道时为0~m_ShiftPhaseNum-1)
    std::vector<std::vector<std::vector<std::complex<double>>>> m_Data; // 测向数据

    // ============================================================================
    // 全向天线参数
    // ============================================================================
    double m_R = 0.186;             // 全向天线时的阵列半径(米)，GN930默认0.1865m
                                    // 阵列半径影响理论相位差的计算: phase = 2*pi*R*sin(theta)/lambda
    string m_Radr = "./Radius.txt"; // 定向天线阵列孔径保存的路径（频率-半径对应表）

    // ============================================================================
    // 定向天线参数
    // ============================================================================
    int m_Doa_Cut_Num = 3; // 用于测向的切刀数（定向天线选取的最强信号天线数+相邻天线）

    // ============================================================================
    // 联合对角化参数
    // ============================================================================
    // 伪协方差矩阵个数: 每个矩阵对应一个不同的延迟k (k=0,1,...,m_RAll_Num-1)
    // 延迟协方差矩阵 R(k) = X(:, 0:N-k) * X(:, k:N)^H
    // 数量越多，联合对角化的分离效果越好，但计算量增大
    int m_RAll_Num = 11;     // 伪协方差的个数（延迟步数）= 不同延迟的协方差矩阵数量
    int m_RAll_PointNum = 1; // 构建伪协方差时延迟的间隔点数（相邻矩阵的延迟差为1个采样点）

    // ============================================================================
    // 测向结果存储
    // ============================================================================
    double m_angles[10] = {0.0};     // 测向结果（角度，度，0-360）：最多支持10个同频信号
    double m_qualities[10] = {0.0};  // 测向质量（置信度，0-1之间，越高表示测向结果越可靠）
    double m_amplitudes[10] = {0.0}; // 信号幅度（暂未启用）

    // ============================================================================
    // 虚拟矩阵参数
    // ============================================================================
    int m_Virtual_Flag = 0;           // 是否加入虚拟阵列（0=不使用虚拟扩展，1=使用虚拟阵列提高分辨率）
    double m_Virtual_Multiple = 0.94; // 虚拟阵列缩放倍数（虚拟阵元间距系数，用于扩展阵列孔径）
    int m_IQ_Density_Num = 5;         // IQ积分次数（IQ域的多帧累积，与幅度积分配合使用）
};