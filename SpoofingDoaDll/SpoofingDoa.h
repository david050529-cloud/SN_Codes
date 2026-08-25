#pragma once

// =============================================================================
// 文件名: SpoofingDoa.h
// 功能描述: 欺骗干扰测向(DOA)主类的头文件
// 系统角色: 本文件定义了欺骗测向系统的核心数据结构和主类SpoofingDoa，
//           包含卫星数据结构、欺骗检测结果结构、以及测向算法的全部接口。
// 整体流程: 数据输入(GNSSData) → 数据预处理(相位差计算) → 校正(校正数据补偿)
//           → 欺骗检测(相位差/角度/载噪比三方法) → DOA测向计算(干涉仪/幅相法)
//           → 伪谱积分 → 结果输出(SpoofingResult)
// 依赖模块: 继承自ArithmeticDoa(arithmetic.h)，提供底层数学运算支持
// =============================================================================

// 测向算法相关
#include "pch.h"
#include <math.h>
#include <stdarg.h>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <mutex>
#include <complex>
#include <algorithm>
#include <cstring>
#include <random>

// #include "../publicFunctionDoa.h"
#include "../Arithmetic/arithmetic.h"
using namespace std;
using namespace PublicSpace;

constexpr double PI = 3.141592653;

extern char Version[];
#pragma pack(1)

// =============================================================================
// 数据结构定义
// =============================================================================

// -----------------------------------------------------------------------------
// SatelliteData: 单颗卫星的原始观测数据
// 由GNSS接收机输出，包含伪距、载噪比、载波相位、多普勒等基本信息
// -----------------------------------------------------------------------------
struct SatelliteData
{
    int i_Prn;      // 卫星号(PRN编号，GPS为1-32，BDS为1-63等)
    int i_Sys;      // 卫星系统(0=GPS, 1=GLONASS, 2=SBAS, 3=Galileo, 4=BDS, 5=QZSS)
    int i_Type;     // 卫星频点(如GPS L1C/A=0, L5=2, BDS B1I=0, B2a=12等)
    double i_Psr;   // 伪距(单位:米)，用于定位解算的距离观测量
    float i_Snr;    // 卫星载噪比(单位:dB-Hz)，反映信号强度
    double i_Phase; // 卫星载波相位(单位:周)，高精度相位观测量
    double i_Dop;   // 载波多普勒(单位:Hz)，反映相对运动速度
};

// -----------------------------------------------------------------------------
// SatelliteDataPhaseDiffA: 单颗卫星在两个通道间的相位差数据(细粒度)
// 用于单帧数据的相位差表示，包含第一通道和第二通道的载噪比及载波相位差
// 注:这是第一级相位差数据结构，由原始GNSSData计算得到
// -----------------------------------------------------------------------------
struct SatelliteDataPhaseDiffA
{
    int i_Prn;           // 卫星号
    int i_Sys;           // 卫星系统
    int i_Type;          // 卫星频点
    float i_Snr1;        // 第一通道的载噪比(单位:dB-Hz)
    float i_Snr2;        // 第二通道的载噪比(单位:dB-Hz)
    double i_phase_diff; // 载波相位差，单位：周
};

// -----------------------------------------------------------------------------
// SatelliteDataPhaseDiffB: 单颗卫星在各切刀位置的相位差数据(粗粒度)
// 用于多帧数据的汇总，每个数组元素对应一个切刀序号
// 切刀顺序决定天线对的切换顺序：如{1,2}表示天线1与天线2构成一个干涉基线
// 注:这是第二级相位差数据结构，由多个SatelliteDataPhaseDiffA合并得到
// -----------------------------------------------------------------------------
struct SatelliteDataPhaseDiffB
{
    int i_Prn;  // 卫星号
    int i_Sys;  // 卫星系统
    int i_Type; // 卫星频点
    int i_diffLen;                           // 有效相位差数量(有效切刀数)
    float i_Snr1[100] = {0.0};               // 第一通道的载噪比，索引对应切刀序号
    float i_Snr2[100] = {0.0};               // 第二通道的载噪比，与切刀序号一一对应，若该刀没有数据则为0
    double i_phase_diff[100] = {0.0};        // 载波相位差(单位:周)，与切刀序号一一对应
    // int i_useAntenna[20][2];              // 天线序号，与载噪比、相位差一一对应
};

// -----------------------------------------------------------------------------
// AlarmData: 单颗卫星的告警/测向结果数据
// 存储每颗被判定为欺骗信号的卫星的详细信息
// -----------------------------------------------------------------------------
struct AlarmData
{
    int i_Prn;        // 欺骗的卫星号
    float i_Snr;      // 载噪比(单位:dB-Hz)
    int i_Angle;      // 测向结果(单位:度，0-359)，即到达角估计值
    double i_Quality; // 测向质量(0-100)，值越大表示测向结果可靠性越高
};

// -----------------------------------------------------------------------------
// SatelliteAngle: 一个频点(系统+频点类型)的欺骗检测汇总结果
// 汇总该频点下所有被检测为欺骗的卫星信息
// -----------------------------------------------------------------------------
struct SatelliteAngle
{
    int i_Sys;       // 卫星系统
    int i_Type;      // 卫星频点
    int i_Alarm = 0; // 报警标识，0=不为欺骗，1=为欺骗
    double i_Angle;  // 该频点的测向结果(单位:度)，综合多颗卫星的到达角估计
    int i_Count;     // 被干扰卫星数(该频点被检测为欺骗的卫星颗数)
    AlarmData i_AlarmData[32]; // 最多32颗卫星的详细告警数据
};

// -----------------------------------------------------------------------------
// SpoofingResult: 欺骗测向的最终输出结果
// 包含多个频点的欺骗检测结果，每个频点包含到达角和被欺骗卫星列表
// 这是DLL接口向外部返回的主要数据结构
// -----------------------------------------------------------------------------
struct SpoofingResult
{
    int i_Count; // 返回多个结果，频点数(存在欺骗的频点个数)
    SatelliteAngle i_SatelliteAngle[24]; // 最多24个频点的测向结果
};


// -----------------------------------------------------------------------------
// GNSSData: GNSS原始观测数据输入结构
// 包含两个接收通道(两端口)的所有卫星观测量
// 端口0通常为参考天线，端口1为另一个天线，通过切刀开关在不同天线间切换
// 注:这是DLL接口接收外部数据的主要输入结构
// -----------------------------------------------------------------------------
struct GNSSData
{
    int i_PortOneNum;             // 第一个通道(端口0)接收的信息条数
    SatelliteData i_PortOne[500]; // 第一通道中所有卫星信息，规定每次最多传输500条

    int i_PortTwoNum;             // 第二个通道(端口1)接收的信息条数
    SatelliteData i_PortTwo[500]; // 第二通道中所有卫星信息，规定每次最多传输500条
};


// -----------------------------------------------------------------------------
// SingleDeceptiveResult: 由外部欺骗检测算法传入的单个欺骗检测结果
// 用于与其他欺骗检测模块的信息交互，联合筛选测向结果
// -----------------------------------------------------------------------------
struct SingleDeceptiveResult
{
    int r_Sys;     // 欺骗的系统
    int r_Type;    // 欺骗的频点
    int r_Angel;   // 欺骗的角度(单位:度)
    int prncount;  // 此频点欺骗卫星数量
    int r_Prn[30]; // 欺骗的卫星号列表(最多30颗)
};

#pragma pack()

// =============================================================================
// SpoofingDoa 主类
// 继承自 ArithmeticDoa，实现欺骗干扰检测与测向的完整流程
//
// 核心功能:
// 1. 欺骗检测 - 通过相位差、角度、载噪比三种方法检测欺骗信号的存在
// 2. 到达角估计(DOA) - 利用相关干涉仪或幅相法估计欺骗信号的方向
// 3. 校正数据处理 - 校正通道间相位不一致性
// 4. 伪谱积分 - 累积多帧测向结果提高测向精度和稳定性
// 5. 跳半周优化 - 检测并修复载波相位跳变问题
// =============================================================================
class SpoofingDoa : public ArithmeticDoa
{
public:
    SpoofingDoa(void);

    ~SpoofingDoa(void);

    void Init(void); // 初始化: 读取配置、初始化频率表、理论相位差、检测历史记录等

    // 设置相位差阈值(欺骗检测门限)
    // @param threshold: 卫星颗数阈值，达到该数量则判定为欺骗
    // @param phsThreshold: 相位差阈值(单位:度)，相位差在该范围内的卫星判定为同源
    // @param sys: 卫星系统(-1表示所有系统)
    // @param type: 卫星频点(-1表示所有频点)
    void setThresholdDetectionDoa(int threshold, double phsThreshold, int sys, int type);

    // 设置需要加入载噪比进行欺骗检测的频点
    void setTypeDetectionBySnr(int thresholdNum, int sys, int type);

    // 设置GNSS观测数据，驱动测向流程
    // @param data: GNSS原始观测数据数组
    // @param dataLen: 数据长度(1=仅欺骗检测, 2=带校正的欺骗检测, >2=完整测向)
    void setGNSSData(const GNSSData *data, int dataLen);

    // 获取欺骗测向的最终结果
    int getAngleSpoofingDoa(SpoofingResult &result);

    // 设置配置文件路径(已废弃，实际使用默认路径)
    void setConfigTxtAdr(const char *adr);

    // 设置切刀顺序(射频开关切换顺序)
    // RF开关按照此顺序在不同天线间切换，形成多个干涉基线用于测向
    void setCutSquence(int len, const int *cutSq);

    // 设置外部欺骗检测结果(用于联合筛选)
    void setSpoofingDetecteResult(const vector<SingleDeceptiveResult> detectResult);

    // 设置欺骗检测记录数(连续多少帧检测为欺骗才最终判定)
    void setDetectionRecordNum(int num);

private:
    // =========================================================================
    // 各功能模块的私有函数(按源文件分组)
    // =========================================================================

    // ---- PreparationData.cpp: 数据预处理 ----
    // 计算两个通道间的相位差(细粒度)，处理卫星匹配和重复检测
    void getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA);
    // 将多帧相位差数据汇总为B格式(粗粒度)，按卫星聚合并处理缺失数据
    void getSatelliteDataPhaseDiffB(const vector<vector<SatelliteDataPhaseDiffA>> &dataA, vector<SatelliteDataPhaseDiffB> &dataB);
    // 清空SatelliteDataPhaseDiffB结构的所有字段
    void clearSatelliteDataPhaseDiffB(SatelliteDataPhaseDiffB &dataB);
    // 将同一系统同一频点的卫星数据分类存储到map中
    void getSatelliteDataByType(const std::vector<SatelliteDataPhaseDiffA> &dataA, std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT);

    // ---- InitData.cpp: 初始化模块 ----
    // 根据项目号初始化硬件参数(天线数、切刀数、阵列半径等)
    void initProject(void);
    // 生成频点唯一编码: sys*100+type
    int TypeInt(int sys, int type);
    // 初始化所有GNSS频点的频率映射表(m_F)
    void initType(void);
    // 初始化所有频点的欺骗检测阈值(卫星颗数和相位差阈值)
    void initDetectionThreshold(int threshold, double phsThreshold);
    // 初始化欺骗检测历史记录(滑动窗口队列)
    void initRecords(void);
    void paraProjectGN902();
    void paraProjectGN930();

    // ---- Corrected.cpp: 校正数据处理 ----
    // 设置校正数据:从同天线功分的通道数据计算相位校正值
    void setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA);
    // 逐卫星更新校正数据(选择载噪比较高的数据保存)
    void calCorrectionData(const SatelliteDataPhaseDiffA dataA);
    // 对所有输入数据进行相位校正(减去校正值消除通道误差)
    void getCorrectedGnssData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    // 对单个卫星数据进行相位差校正
    void calCorrecteData(SatelliteDataPhaseDiffA &dataA);

    // ---- Directed.cpp: 定向天线测向 ----
    // 定向天线:由载噪比决定用于测向的天线(选择信号最强的天线扇区)
    void getUseAntennaBySnr(SatelliteDataPhaseDiffB &dataB, int &startAngle, int &endAngle);
    // 定向天线:构建用于干涉仪测向的天线对与相位差数据
    void setInterferInfoDataDirect(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData);

    // ---- Omni.cpp: 全向天线测向 ----
    // 全向天线:构建用于干涉仪测向的天线对与相位差数据
    void setInterferInfoDataOmni(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData);

    // ---- AmpPhase.cpp: 幅相法测向 ----
    // 使用幅相法进行DOA计算(利用仿真阵列流型进行匹配)
    void getResultAmpPhaseDoa(vector<SatelliteDataPhaseDiffB> dataB);
    // 初始化仿真阵列流型(从仿真数据文件读取)
    void initSimulateA(void);

    // ---- Interfer.cpp: 相关干涉仪测向 ----
    // 使用相关干涉仪方法进行DOA计算
    void getResultInterferDoa(vector<SatelliteDataPhaseDiffB> dataB);
    // 初始化理论相位差模板(基于阵列几何参数计算)
    void initTheory(void);
    // 初始化基于阵列仿真相位差的理论模板(从仿真数据文件读取)
    void initTheoryBySimulatePhase(void);

    // ---- Alarm.cpp: 欺骗检测与告警 ----
    // 利用相位差检测欺骗信号:统计相位差相近的卫星数量，超过阈值则告警
    void calAlarmByPhaseDiff(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm);
    // 利用示向度(到达角)判断是否为欺骗:统计角度相近的测向结果数量
    void calAlarmByAngle(int typeInt, const std::vector<AlarmData> &data, std::vector<int> &alarmIndex, int &alarm, double &angle);
    // 利用载噪比检测欺骗信号:欺骗信号通常具有相同或相近的载噪比
    void calAlarmBySnr(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm);
    // 设置欺骗检测记录(滑动窗口)，需连续多帧检测为欺骗才最终判定
    void setDetectionRecords(int typeInt, int &alarm);
    // 带校正数据的欺骗检测(先校正再检测)
    void setCorrectDetectionDataAlarm(const GNSSData *data, int dataLen);

    // ---- SpectrumDesity.cpp: 伪谱积分 ----
    // 对测向结果进行伪谱积分累积，提高稳定性
    void setSpoofingResultPseudoSpectrumDesity(SpoofingResult &result);
    // 多星伪谱合成:对多颗卫星的伪谱求和，获取最终角度估计
    void getPseudoSpectrumDesity(vector<vector<double>> diff, double &angle);
    // 按类型计算伪谱(对欺骗信号施加加权)
    void getPseudoSpectrumType(const vector<vector<double>> diff, vector<double> &meanDiff, int alarm);

    // =========================================================================
    // 综合调度与辅助函数
    // =========================================================================

    // 设置测向数据(主流程入口):相位差计算→校正→检测→测向→结果
    void setDataAngle(const GNSSData *data, int dataLen);
    // 设置告警数据(欺骗检测模式入口):仅进行欺骗检测不测向
    void setDataAlarm(const GNSSData data);
    // 获取检测结果并汇总到m_AngleResultData
    void getAlarm(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT);
    // 设置各频点的阵列半径(全向天线使用统一半径，定向天线使用频率相关半径)
    void setR(const string adr);
    // 读取配置文件并解析所有配置参数
    void setConfigData(const string adr);
    // 根据天线对选择构建InterferInfo结构(排除无效数据，应用虚拟阵列)
    void calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg);
    // 多帧数据平滑(排除异常值后取均值)
    void getSmoothData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    // 对单颗卫星多帧相位差进行稳定性过滤 + 跳半周处理 + 圆形均值
    // requireFromStart=true 时额外要求该卫星从起始帧就存在（校正用）
    void calSmoothData(SatelliteDataPhaseDiffB dataB, SatelliteDataPhaseDiffA &dataA, bool requireFromStart = false);
    // 取最后一帧数据(不进行平滑时使用)
    void getEndFramData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    // 组装最终输出的SpoofingResult
    void setSpoofingResult(SpoofingResult &result);
    // 对每个频点的所有卫星进行干涉仪测向并汇总结果
    void calAngle(std::map<int, std::map<int, InterferInfo>> inferInfoData);
    // 对每刀数据进行欺骗检测，只保留检测为欺骗的信号用于测向
    void getSpoofingDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    // 根据欺骗检测结果筛选卫星数据
    void setSpoofingDetectionData(std::vector<SatelliteDataPhaseDiffA> &spoofingData);
    // 根据外部欺骗检测结果筛选数据
    void getDataByDetectionResult(const GNSSData data, GNSSData &detetectData);
    // 保存原始GNSS数据到二进制文件
    void saveGNSSData(const GNSSData *data, int dataLen);
    // 根据外部欺骗检测结果筛选测向结果:将通过检测但未测向的频点加入告警
    void getSpoofingResultByDetection(SpoofingResult &result);
    // 检测是否存在跳半周现象:比较当前帧与历史帧的相位差余弦值
    int getDetection180(int typeInt, int prn, InterferInfo &info);
    // 全排列组合方式优化跳半周:尝试所有±0.5周的相位补偿
    void setPermutationOptimiz180(SatelliteDataPhaseDiffB dataB, const vector<vector<double>> phaseTheory, double &angle, double &qulity);
    // 对所有卫星进行跳半周优化测向
    void getOptimizResultDoa180(vector<SatelliteDataPhaseDiffB> dataB);

    // ---- 角度工具（静态，对应 Python simplified_detection 的角度函数）----
    static double normalizeAngle180(double deg);                  // 归一化到 [-180°,180°)
    static double circularMeanDeg(const std::vector<double> &degs); // 圆形均值(度)
    static double circularSpanDeg(const std::vector<double> &degs); // 最小覆盖弧跨度(度)
    static double foldHalfCycle(double deg);                       // 折叠到 [0°,180°)
    static double circularSpan180Deg(const std::vector<double> &degs); // 半周圆上的跨度(度)

    // ---- 循环切刀检测流程（对应 Python cyclic_phase_detection.py 主流程）----
    void resetCyclicDetection(void);   // 清空连续报警计数与跟踪状态
    // 每刀相位差聚类检测 + 跨刀连续确认 + 测向持续跟踪（只保留确认欺骗的卫星用于测向）
    void getCyclicDetectionData(std::vector<std::vector<SatelliteDataPhaseDiffA>> &dataA);

    // =========================================================================
    // 日志输出函数(WriteLog.cpp)
    // =========================================================================
    void LogSatelliteDataPhaseDiffB(const SatelliteDataPhaseDiffB tp);
    void LogSatelliteDataPhaseDiffB(const vector<SatelliteDataPhaseDiffB> &dataB);
    void LogSatelliteDataPhaseDiffA(const vector<SatelliteDataPhaseDiffA> &dataA);
    void LogSatelliteDataPhaseDiffA(const SatelliteDataPhaseDiffA dataA);
    void LogSpoofingResult(const SpoofingResult result);
    void LogGNSSData(const GNSSData data, int i);
    void LogSatelliteDataPhaseDiffType(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT);

private:
    // =========================================================================
    // 项目枚举:不同硬件平台有不同的天线布局和切刀顺序
    // =========================================================================
    enum ProjectNum
    {
        GN902,   // GN902设备: 全向天线，7阵元
        GN930U,  // GN930U设备: 全向天线，7阵元
        GN930,   // GN930设备: 全向天线，7阵元，双板卡
        GN560    // GN560设备: 定向天线，7阵元
    };

    // =========================================================================
    // 核心数据成员
    // =========================================================================

    // ---- 频率与阵列参数 ----

    // m_F: 保存各个频点的频率(Hz)
    // key = TypeInt(sys, type)频点编码, value = 频率(Hz)
    // 例如: BDS B1I 编码为 400, 频率为 1561.098e6 Hz
    std::map<int, double> m_F;

    // m_Detection_Threshold: 欺骗检测阈值 - 卫星颗数
    // 当相位差相近的卫星数量达到此阈值时，判定为欺骗
    // 按频点独立设置，不同频点可有不同阈值
    std::map<int, int> m_Detection_Threshold;

    // m_Detection_PhsThreshold: 欺骗检测阈值 - 相位差大小(单位:周)
    // 两星相位差在此范围内认为"相近"，两颗卫星可能来自同一方向
    std::map<int, double> m_Detection_PhsThreshold;

    // m_Detection_snrThrehold: 是否使用载噪比进行欺骗检测
    // -1=不使用, 1=使用，按频点配置
    std::map<int, int> m_Detection_snrThrehold;

    // m_R: 保存各个频点的阵列半径(米)
    // 全向天线:所有频点使用相同半径(m_omni_R)
    // 定向天线:不同频点可能使用不同半径，从配置文件读取
    std::map<int, double> m_R;

    // ---- 检测记录与模板数据 ----

    // m_Detection_Records: 欺骗检测历史记录(滑动窗口队列)
    // key=频点编码, value=最近N帧的检测结果(1=欺骗,0=正常)
    // 需要连续N帧全部检测为欺骗才最终判定，避免虚警
    std::map<int, std::deque<int>> m_Detection_Records;

    // m_Theory: 保存各个频点的理论相位差模板
    // key=频点编码, value=360个角度(0-359度)的理论相位差向量
    // 每个角度对应一组天线对的相位差期望值，用于相关干涉仪匹配
    // 格式: vector[angle][antenna_pair_index]
    std::map<int, std::vector<std::vector<double>>> m_Theory;

    // m_SimulateA: 保存各个频点的仿真阵列流型(导向矢量)
    // key=频点编码, value=360个角度的复数导向矢量
    // 用于幅相法DOA:将实测相位差与仿真阵列流型进行匹配
    std::map<int, std::vector<std::vector<complex<double>>>> m_SimulateA;

    // ---- 校正与处理结果 ----

    // m_CorrectionData: 保存校正数据
    // 第一层key=频点编码, 第二层key=卫星号(prn=-1表示多星平滑后的综合校正值)
    // 校正数据由同一天线功分信号(切刀顺序中天线对相同的刀)计算得到
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>> m_CorrectionData;

    // m_Pseudo_Spectrum_Value: 保存各星各角度的伪谱值
    // 第一层key=频点编码, 第二层key=卫星号(prn), value=360个角度的伪谱值
    // 伪谱值反映实测相位差与理论相位差的匹配程度
    std::map<int, std::map<int, std::vector<double>>> m_Pseudo_Spectrum_Value;

    // m_AngleResultData: 同一频点的所有卫星测向结果
    // key=频点编码, value=该频点所有卫星的测向结果(AlarmData列表)
    // 这是测向计算的中间结果，最终汇总为SpoofingResult输出
    std::map<int, std::vector<AlarmData>> m_AngleResultData;

    // m_Max_Snr: 频点中各卫星的最大载噪比
    // 第一层key=频点编码, 第二层key=卫星号(prn), value=最大载噪比(dB-Hz)
    std::map<int, std::map<int, double>> m_Max_Snr;

    // ---- 外部交互 ----

    // m_detectResult: 外部欺骗检测结果(由其他欺骗检测模块传入)
    // 用于联合筛选，将外部检测结果与本模块测向结果进行交叉验证
    vector<SingleDeceptiveResult> m_detectResult;

    // m_Angle_Accumulate: 伪谱积分累积器(360维向量)
    // 每帧测向结果通过加权累加到各角度bin中，实现多帧累积
    vector<double> m_Angle_Accumulate;

    // m_infoData180: 历史测向数据，用于检测跳半周现象
    // 通过比较当前帧与历史帧的相位差余弦值变化判断是否跳半周
    std::map<int, std::map<int, InterferInfo>> m_infoData180;

    // =========================================================================
    // 循环切刀欺骗检测状态（对应 Python cyclic_phase_detection.py）
    // =========================================================================

    // 载噪比质量门限(dB)：两端口载噪比都需达标（对应 CNR_MIN_DB）
    static const double CNR_MIN_DB;
    // 稳定性阈值(度)：相位差最小覆盖弧 < 该值判为稳定（对应 STABILITY_RANGE_DEG）
    static const double STABILITY_RANGE_DEG;
    // 每刀至少需要的有效采样帧数才判为稳定（对应 MIN_STABLE_SAMPLES，帧不足时按实际帧数）
    static const int MIN_STABLE_SAMPLES;

    // 连续报警计数：typeInt -> 连续被判为欺骗的刀数；跨刀连续出现 p 次才确认（对应 consecutive）
    std::map<int, int> m_ConsecutiveAlarm;

    // 测向持续跟踪状态（对应 tracking）：确认欺骗后进入跟踪，每轮测向更新最近一次 DOA，
    // 直到该 (系统,频点) 不再报警才停止跟踪（最终记录保留最近一次 DOA）。
    struct TrackingInfo
    {
        std::set<int> cluster_sats;   // 可疑卫星号集合
        double doa_deg = -1.0;        // 最近一次测向角度(-1 表示无)
        double quality = -1.0;        // 最近一次测向质量
    };
    std::map<int, TrackingInfo> m_Tracking;   // typeInt -> 跟踪状态

protected:
    // =========================================================================
    // 配置与控制参数
    // =========================================================================

    // m_Detection_Recodds_Num: 欺骗检测记录个数
    // 连续多少帧检测为欺骗信号才最终判定，默认为1(每帧独立判定)
    int m_Detection_Recodds_Num = 1;

    // m_Project_flg: 项目号，决定硬件平台和天线配置
    // 默认为GN930U(全向天线，7阵元)
    int m_Project_flg = GN930U;

    // m_AntennaNum: 阵列中的天线阵元个数(默认为7)
    int m_AntennaNum = 7;

    // m_Angle_Threshold: 同一方向的阈值(单位:度)
    // 两星到达角在此范围内被认为是同一方向(±m_Angle_Threshold度)
    double m_Angle_Threshold = 3;

    // m_Phasediff_Threshold: 相位差相近阈值(单位:度)
    // ±m_Phasediff_Threshold度内被认为相位差相近
    double m_Phasediff_Threshold = 5;

    // m_antnenaType: 天线类型
    // 0 = 全向天线(omni-directional), 1 = 定向天线(directional)
    int m_antnenaType = 0;

    // m_OneCut_Frams: 一刀中采集数据的帧数
    // 射频开关每个位置停留时间内采集的观测帧数
    int m_OneCut_Frams = 1;

    // m_Smooth_Flag: 是否进行多帧平滑
    // 0=不平滑(取最后一帧), 1=平滑(排除异常值后取均值)
    int m_Smooth_Flag = 0;

    // m_Doa_Cut_Num: 用于测向的刀数(有效干涉基线数量)
    int m_Doa_Cut_Num = 7;

    // m_Detection_Threshold_Num: 欺骗检测的卫星颗数默认阈值
    int m_Detection_Threshold_Num = 4;

    // m_PseudoSpectrum_Flag: 是否进行伪谱积分
    // 0=不积分, 1=积分(多帧累积提高测向稳定性)
    int m_PseudoSpectrum_Flag = 0;

    // m_Doa_Detection_Flag: 测向数据中的欺骗检测模式
    // 0=不检测欺骗，对所有数据进行测向
    // 1=对每刀数据独立检测，不合并
    // 2=对每刀数据检测并合并，仅对检测为欺骗的信号进行测向
    int m_Doa_Detection_Flag = 0;

    // m_Cyclic_Detection_Flag: 是否使用循环切刀检测流程（对应 cyclic_phase_detection.py）
    // 0=使用原有检测流程(m_Doa_Detection_Flag)
    // 1=使用新流程：每刀聚类 + 跨刀连续确认 + 测向持续跟踪
    int m_Cyclic_Detection_Flag = 0;

    // m_Save_Original_Flg: 是否保存原始GNSS数据
    // 0=不保存, 1=保存到二进制文件
    int m_Save_Original_Flg = 0;

    // m_Save_Oringial_Num: 原始数据保存个数
    int m_Save_Oringial_Num = 0;

    // m_Snr_Threshold: 载噪比阈值(dB-Hz)
    // 低于该载噪比的卫星数据将被删除，不参与测向
    double m_Snr_Threshold = 0.0;

    // m_Qulity_Threshold: 测向质量阈值
    // 低于该值的测向结果被丢弃(质量分0-100，100为最优)
    double m_Qulity_Threshold = 0.0;

    // m_Doa_Arithmetic: 测向所用的算法
    // 1=相关干涉仪(使用理论相位差模板)
    // 2=幅相法(使用仿真阵列流型)
    // 3=相关干涉仪+阵列仿真数据(使用仿真相位差模板)
    int m_Doa_Arithmetic = 1;

    // m_Doa_Cut_min_Num: 用于测向的相位差的最少数量
    // 有效天线对数量低于此值时，该卫星不参与测向
    int m_Doa_Cut_min_Num = 6;

    // m_LogFile: 日志文件路径前缀
    string m_LogFile = "./spoofingDoaLog_";

    // m_Accumulate_multiplier: 测向结果积分衰减因子(0-1)
    // 每帧伪谱积分累加后乘以该因子实现指数衰减，避免历史数据影响过大
    double m_Accumulate_multiplier = 0;

    // m_Simulate_Data_file: 阵列仿真数据文件路径(用于幅相法和仿真相位干涉仪)
    string m_Simulate_Data_file = "/simulateData/";

    // m_Delete_Prn_Flag: 是否删除存在缺失刀数据的卫星
    // 0=不删除(保留部分相位差有效的数据), 1=删除(要求全部刀的相位差均有效)
    int m_Delete_Prn_Flag = 1;

    // m_Secondary_Doa_Flag: 是否进行二次确认测向
    // 0=进行(在干涉仪结果基础上使用虚拟干涉仪再次测向), 1=不进行
    int m_Secondary_Doa_Flag = 0;

    // m_Virtual_Flag: 是否加入虚拟阵列(扩展天线孔径)
    // 0=不使用, 1=使用(通过数学变换构造虚拟阵元，提高测向精度)
    int m_Virtual_Flag = 0;

    // m_Detection180_Qulity_Threshold: 跳半周修复的质量改善阈值
    // 修复后的测向质量需比原结果提升超过此值才采用修复结果
    double m_Detection180_Qulity_Threshold = 10.0;

    // m_Virtual_Multiple: 虚拟阵列扩展倍数(默认0.94)
    double m_Virtual_Multiple = 0.94;

    // m_Screen_detection_Flag: 是否根据外部欺骗检测结果筛选数据
    // 0=不筛选, 1=筛选(只对已检测为欺骗的频点进行测向)
    int m_Screen_detection_Flag = 0;

    // m_getUseAntennaBySnr_Flag: 是否利用载噪比确定用于测向的天线序号
    // 0=不利用(按固定扇区划分), 1=利用(选择载噪比最高的天线区域)
    int m_getUseAntennaBySnr_Flag = 0;

    // m_Receiver_num: 接收机的个数(默认为1个)
    int m_Receiver_num = 1;

    // m_Radr: 定向天线半径配置文件路径
    string m_Radr;

    // m_omni_R: 全向天线阵列半径(米)，所有频点统一使用
    // 不同项目有不同的默认值: GN902=0.1865, GN930U=0.2, GN560=0.18
    double m_omni_R = 0.1865;

    // m_Detection_Tag: 当前数据帧的类型标记
    // 0=有多帧数据用于测向(完整DOA模式)
    // 1=只有少量数据用于欺骗检测(仅检测不测向)
    int m_Detection_Tag = 0;

    // m_All_Simulate_data_Fre: 阵列仿真数据中包含的所有频率(Hz)
    // 用于为实际工作频点寻找最接近的仿真频率
    vector<double> m_All_Simulate_data_Fre = {1176e6, 1279e6, 1561e6, 1602e6};

    // m_cutSequence: 切刀顺序(射频开关切换顺序)
    // 每一对{a,b}表示天线a与天线b构成一个干涉基线
    // 默认顺序[{7,7},{1,2},{1,3},{1,4},{7,7},{1,5},{1,6},{1,7}]
    // 其中{7,7}表示同天线自检(用于校正数据计算)
    vector<vector<int>> m_cutSequence = {{7, 7}, {1, 2}, {1, 3}, {1, 4}, {7, 7}, {1, 5}, {1, 6}, {1, 7}};
};
