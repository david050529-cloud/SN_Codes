#pragma once

// =============================================================================
// 文件名: GN902.h
// 功能描述: GN902 欺骗检测/测向模块 —— 自包含头文件(消除编译隔离)
//
// 本文件将以下头文件的全部类型/类声明与常量逐字收纳, 配合 GN902.cpp
// (收纳全部函数实现) 即可单独编译, 无需再引用 Arithmetic/、publicFunctionDoa/、
// SpoofingDoaDll/ 等外部源文件与头文件(仅依赖 C++ 标准库):
//   - 接口共享结构 (SatelliteData / GNSSData / AlarmData / SatelliteAngle / SpoofingResult)
//   - 引擎中间结构 (SatelliteDataPhaseDiffA/B)
//   - 测向算法基类 ArithmeticDoa (相关干涉仪 / 幅相法 / 虚拟阵列)
//   - 公共工具命名空间 PublicSpace (日志 / 峰值检测 / 角度规整 / 文件读取)
//   - 主类 SpoofingDoa 与接口类 GN902
// =============================================================================

// ---- 平台头文件 (原 framework.h / pch.h) ----
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// ---- 标准库 ----
#include <math.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <assert.h>
#include <cmath>
#include <ctime>
#include <string>
#include <vector>
#include <queue>
#include <map>
#include <set>
#include <mutex>
#include <complex>
#include <algorithm>
#include <cstring>
#include <random>
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <chrono>
#include <stdexcept>
#include <utility>

using namespace std;

#define m_PI 3.14159265358979323846
#define m_C 3e8
constexpr double PI = 3.141592653;


// =============================================================================
// 公共工具命名空间 PublicSpace (原 publicFunctionDoa.h)
// =============================================================================
namespace PublicSpace
{
    /**
     * @brief 日志记录控制标志
     * @note 0 - 不记录日志（默认值）
     *       1 - 记录日志，文件以覆盖模式("w")打开
     *       2 - 记录日志，文件以追加模式("a")打开
     */
    extern int m_logFlg;

    /**
     * @brief 原始数据保存控制标志
     * @note 1 - 保存原始数据到二进制文件；其他值(默认0) - 不保存
     */
    extern int m_save_data_Flg;

    // ==================== 数据操作 ====================
    void findPeaks(const std::vector<double> data, vector<int> &index2, vector<double> &vaules2); // 获取峰值
    void split(vector<string> &result, string str, char str1); // 字符串分割
    void trim(string &str); // 去掉字符串中的空格
    int readFile(const string adr, vector<string> &data); // 读取文件

    // ==================== 数学工具 ====================
    double getNorm(vector<complex<double>> data); // 计算复数的L2范式
    double Round3600(double x); // 将角度规整到[0,360)度范围
    int Round360(int x);

    // ==================== 配置文件与时间 ====================
    string getNowTime(); // 获取当前时间

    // ==================== 日志系统 ====================
    void LogCreat(const string path); // 创建日志文件
    void Log(const char *format, ...); // 编写日志（支持可变参数格式化）

    // ==================== 模板函数 ====================

    /**
     * @brief 将数组数据以二进制格式追加写入文件
     */
    template <typename T>
    void saveArrayToBinary(const std::string& filename, const T* data, size_t size)
    {
        if (!m_save_data_Flg)
        {
            return;
        }
        try
        {
            std::ofstream file(filename, std::ios::binary | std::ios::app);
            if (!file)
            {
                throw std::runtime_error("无法打开文件进行写入！");
            }
            file.write(reinterpret_cast<const char*>(data), size * sizeof(T));
            if (file.fail())
            {
                throw std::runtime_error("写入文件时出错！");
            }
            std::cout << "数据已成功保存到: " << filename << std::endl;
        }
        catch (const std::exception& e)
        {
            std::cerr << e.what() << std::endl;
        }
    }

}

using namespace PublicSpace;


// =============================================================================
// 测向算法基类所需结构 (原 Arithmetic/arithmetic.h)
// =============================================================================
#pragma pack(1)

/**
 * @struct InterferInfo
 * @brief 测向输入数据结构体，封装一次测向所需的全部原始信息
 */
struct InterferInfo // 用于测向的相关信息
{
    int i_Start;              ///< 搜索范围开始角度（度），闭区间
    int i_End;                ///< 搜索范围结束角度（度），闭区间
    int i_Phase_Len;          ///< 有效的相位差/幅度数据条数
    int i_AntennaSq[200][2];  ///< 相位差对应的天线对序号，天线序号从 1 算起
    double i_Phase_Diff[200]; ///< 实测相位差数组（弧度）
    double i_Amp[200];        ///< 实测幅度数组（线性值）
};

/**
 * @struct Rs
 * @brief 频率-阵列半径对应关系
 */
struct Rs
{
    double i_starF; ///< 起始频率（Hz）
    double i_endF;  ///< 结束频率（Hz）
    double i_r;     ///< 该频率区间对应的阵列半径（米）
};

#pragma pack()

/**
 * @class ArithmeticDoa
 * @brief 测向算法核心类，封装相关干涉仪、幅相法、虚拟阵列扩展等测向方法
 */
class ArithmeticDoa
{
public:
    ArithmeticDoa(void);
    ~ArithmeticDoa(void);

protected:
    /*==================相关干涉仪测向==================*/
    void calPhaseTheory(const double f, const double r, const int antnnaNum, std::vector<std::vector<double>> &theory);
    void calPseudoByInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, vector<double> &diff);
    void calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2);

    /*==================幅相法测向==================*/
    int getSimulatePhase(const string path, const double f, std::vector<std::vector<double>> &phase);
    int getSimulateAmp(const string path, const double f, std::vector<std::vector<double>> &amp);
    void calAmpPhaseSimulateA(const string path, const double f, vector<vector<complex<double>>>& simulateA);
    void calAmpPhase(vector<vector<complex<double>>> simulateA, const InterferInfo data, double &angle, double &quality, vector<double> &diff);
    void getA(const vector<double> amp, const vector<double> phase, vector<complex<double>> &A);
    int getModeData(const string path, vector<vector<double>> &data);
    void calPseudoByAmpPhase(const vector<vector<complex<double>>> simulateA, vector<complex<double>> theory_A, vector<double> &diff);
    bool existAmpPhsFile(const string path, const double f);

    /*==================虚拟阵列扩展（Virtual Array）==================*/
    void getVirtual(const double virMultiple, vector<vector<int>> &antnna, vector<double> &phase_diff);
    void getVirtualTheory(const int antnnaNum, const std::vector<std::vector<double>> tp_theory, const double virMultiple, std::vector<std::vector<double>> &virtualTheory);
    int getVirtualAntNum(const int ant1, const int ant2);
    void calSecondDoaByVirInterf(const vector<vector<double>> phaseTheory, const double virMultiple, vector<double> diff, InterferInfo data, double &angle, double &quality);

    /*==================天线选择与相位处理工具方法==================*/
    void calAngleSerchRange(const vector<int> index, const vector<vector<int>> cutSequence, const int AntennaNum, int max_index, int &startAngle, int &endAngle);
    void setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff);
    int getRData(const string adr, vector<Rs> &mR);

    /*==================测向质量评估==================*/
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff, int starAngle, int endAngle);
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff);

};


// =============================================================================
// 宿主/接口共享结构 (原 GN902.h, 保持与 DLL 边界 ABI 一致)
// =============================================================================
struct SatelliteData
{
    int i_Prn;      // 卫星号（PRN）
    int i_Sys;      // 卫星系统编码
    int i_Type;     // 频点编码
    double i_Psr;   // 伪距（米）
    float i_Snr;    // 载噪比（dB-Hz）
    double i_Phase; // 载波相位（周）
    double i_Dop;   // 多普勒频移（Hz）
    // 添加司南欺骗标识：0=正常，1=欺骗
    int i_SpoofingFlag; // 0=正常，1=欺骗
};

struct GNSSData
{
    int i_PortOneNum;             // Port1卫星条数
    SatelliteData i_PortOne[500]; // Port1所有卫星数据
    int i_PortTwoNum;             // Port2卫星条数
    SatelliteData i_PortTwo[500]; // Port2所有卫星数据
};

struct AlarmData
{
    // 卫星号（PRN），取值范围与卫星系统 i_Sys 对应：
    //   GPS(0)      : 1~32
    //   GLONASS(1)  : 1~24（扩展号 65~85）
    //   SBAS(2)     : 120~158
    //   Galileo(3)  : 1~36
    //   BDS(4)      : 1~63（BDS-2: 1~16, BDS-3: 17~63）
    //   QZSS(5)     : 193~202
    int i_Prn;
    float i_Snr;      // 载噪比
    int i_Angle;      // 测向角度
    double i_Quality; // 测向质量（0-100）
};


struct SatelliteAngle
{
    int i_Sys;       // 卫星系统
    int i_Type;      // 卫星频点
    int i_Alarm;     // 报警标识：0=正常，1=欺骗
    double i_Angle;  // 欺骗信号来向角度（度）
    int i_Count;     // 被欺骗卫星数
    AlarmData i_AlarmData[32]; // 最多32颗报警卫星详情
};

struct SpoofingResult
{
    int i_Count;                          // 报警频点数
    SatelliteAngle i_SatelliteAngle[24];  // 最多24个频点的结果
};




// =============================================================================
// 引擎中间结构 (原 SpoofingDoa.h, 参与通道间数据/单星欺骗结果)
// =============================================================================
#pragma pack(1)

// 单颗卫星在两个通道间的相位差数据(细粒度)
struct SatelliteDataPhaseDiffA
{
    int i_Prn;           // 卫星号
    int i_Sys;           // 卫星系统
    int i_Type;          // 卫星频点
    float i_Snr1;        // 第一通道的载噪比(单位:dB-Hz)
    float i_Snr2;        // 第二通道的载噪比(单位:dB-Hz)
    double i_phase_diff; // 载波相位差，单位：周
};

// 单颗卫星在各切刀位置的相位差数据(粗粒度)
struct SatelliteDataPhaseDiffB
{
    int i_Prn;  // 卫星号
    int i_Sys;  // 卫星系统
    int i_Type; // 卫星频点
    int i_diffLen;                           // 有效相位差数量(有效切刀数)
    float i_Snr1[100] = {0.0};               // 第一通道的载噪比，索引对应切刀序号
    float i_Snr2[100] = {0.0};               // 第二通道的载噪比，与切刀序号一一对应
    double i_phase_diff[100] = {0.0};        // 载波相位差(单位:周)，与切刀序号一一对应
};

#pragma pack()

// =============================================================================
// SpoofingDoa 主类
// 继承自 ArithmeticDoa，实现欺骗干扰检测与测向的完整流程
// =============================================================================
class SpoofingDoa : public ArithmeticDoa
{
public:
    SpoofingDoa(void);
    ~SpoofingDoa(void);

    /**
     * @brief 初始化引擎: 建立频率表、理论相位差模板、检测历史记录等全部运行状态
     * @note 构造后即可按项目默认值运行(不读配置文件), 运行参数已内置在 Init 中
     */
    void Init(void);

    /**
     * @brief 设置某系统频点的欺骗检测阈值
     * @param sys         卫星系统编码(0=GPS,1=GLONASS,2=SBAS,3=Galileo,4=BDS,5=QZSS)
     * @param type        频点编码
     * @param threshold   卫星数阈值(相位差聚成一簇的最小卫星数), -1 表示不修改
     * @param phsThreshold 相位差检测阈值(度), <=0 表示不修改
     * @note 当 sys==-1 且 type==-1 时, 用给定 threshold/phsThreshold 初始化全部频点
     */
    void setThresholdDetectionDoa(int sys, int type, int threshold, double phsThreshold);

    /**
     * @brief 喂入一批 GNSS 原始数据并触发检测/测向
     * @param data    GNSS 数据数组指针
     * @param dataLen 数据条数: 1=仅检测, 2=检测+校正, >2=按切刀序列测向
     */
    void setGNSSData(const GNSSData *data, int dataLen);

    /**
     * @brief 取出最近一轮的测向/报警结果
     * @param result 输出结果(含各频点报警角度、卫星明细)
     * @return 0=成功
     */
    int getAngleSpoofingDoa(SpoofingResult &result);

    /**
     * @brief 设置切刀(天线对)序列
     * @param len   数组长度(须为偶数, 每 2 个整数组成一对 {通道1天线, 通道2天线})
     * @param cutSq 天线对序列, 两值相等表示校正刀(不参与测向)
     */
    void setCutSquence(int len, const int *cutSq);

    /**
     * @brief 设置连续报警确认次数
     * @param num 次数(1~10), 超出范围按 1 处理
     */
    void setDetectionRecordNum(int num);

    /**
     * @brief 配置循环切刀运行方式
     * @param cyclic      是否启用循环切刀检测
     * @param oneCutFrams 每个切刀帧数(>0 才生效, 0=沿用旧值)
     * @param smooth      是否多帧平滑(true=平滑, false=取末帧)
     * @param omniR       全向天线阵列半径(米, >0 且为全向天线时重建理论模板)
     */
    void configCyclicRuntime(bool cyclic, int oneCutFrams, bool smooth, double omniR);

private:
    // ---- PreparationData.cpp: 数据预处理 ----
    void getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA);
    void getSatelliteDataPhaseDiffB(const vector<vector<SatelliteDataPhaseDiffA>> &dataA, vector<SatelliteDataPhaseDiffB> &dataB);
    void clearSatelliteDataPhaseDiffB(SatelliteDataPhaseDiffB &dataB);
    void getSatelliteDataByType(const std::vector<SatelliteDataPhaseDiffA> &dataA, std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT);

    // ---- InitData.cpp: 初始化模块 ----
    void initProject(void);
    int TypeInt(int sys, int type);
    void initType(void);
    void initDetectionThreshold(int threshold, double phsThreshold);

    // ---- Corrected.cpp: 校正数据处理 ----
    void setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA);
    void calCorrectionOffset(const vector<vector<SatelliteDataPhaseDiffA>> &calCuts);
    void getCorrectedGnssData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    void calCorrecteData(SatelliteDataPhaseDiffA &dataA);

    // ---- Directed.cpp: 定向天线测向 ----
    void getUseAntennaBySnr(SatelliteDataPhaseDiffB &dataB, int &startAngle, int &endAngle);
    void setInterferInfoDataDirect(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData);

    // ---- Omni.cpp: 全向天线测向 ----
    void setInterferInfoDataOmni(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData);

    // ---- AmpPhase.cpp: 幅相法测向 ----
    void getResultAmpPhaseDoa(vector<SatelliteDataPhaseDiffB> dataB);
    void initSimulateA(void);

    // ---- Interfer.cpp: 相关干涉仪测向 ----
    void getResultInterferDoa(vector<SatelliteDataPhaseDiffB> dataB);
    void initTheory(void);
    void initTheoryBySimulatePhase(void);

    // ---- Alarm.cpp: 欺骗检测与告警 ----
    void calAlarmByPhaseDiff(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm);
    void setCorrectDetectionDataAlarm(const GNSSData *data, int dataLen);

    // ---- SpectrumDesity.cpp: 伪谱积分 ----
    void setSpoofingResultPseudoSpectrumDesity(SpoofingResult &result);
    void getPseudoSpectrumDesity(vector<vector<double>> diff, double &angle);
    void getPseudoSpectrumType(const vector<vector<double>> diff, vector<double> &meanDiff, int alarm);

    // ---- 综合调度与辅助函数 ----
    void setDataAngle(const GNSSData *data, int dataLen);
    void setDataAlarm(const GNSSData data);
    void getAlarm(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT);
    void setR(const string adr);
    void calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg);
    void getSmoothData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    void calSmoothData(SatelliteDataPhaseDiffB dataB, SatelliteDataPhaseDiffA &dataA);
    void getEndFramData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    void setSpoofingResult(SpoofingResult &result);
    void calAngle(std::map<int, std::map<int, InterferInfo>> inferInfoData);
    void getSpoofingDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    void setSpoofingDetectionData(std::vector<SatelliteDataPhaseDiffA> &spoofingData);
    void saveGNSSData(const GNSSData *data, int dataLen);
    int getDetection180(int typeInt, int prn, InterferInfo &info);

    // ---- 角度工具（静态，对应 Python detection_lib 的角度函数）----
    static double normalizeAngle180(double deg);
    static double circularMeanDeg(const std::vector<double> &degs);
    static double circularSpanDeg(const std::vector<double> &degs);
    static double circularSpan180Deg(const std::vector<double> &degs);

    // ---- 循环切刀检测流程（对应 Python detection_main.py / detection_lib.py）----
    void resetCyclicDetection(void);
    void getCyclicDetectionData(std::vector<std::vector<SatelliteDataPhaseDiffA>> &dataA);
    void accumulateBaselines(const std::vector<SatelliteDataPhaseDiffB> &dataB);
    void getCrossCycleDataB(std::vector<SatelliteDataPhaseDiffB> &doaDataB);

    // ---- 日志输出函数(WriteLog.cpp) ----
    void LogSatelliteDataPhaseDiffB(const SatelliteDataPhaseDiffB tp);
    void LogSatelliteDataPhaseDiffB(const vector<SatelliteDataPhaseDiffB> &dataB);
    void LogSatelliteDataPhaseDiffA(const vector<SatelliteDataPhaseDiffA> &dataA);
    void LogSatelliteDataPhaseDiffA(const SatelliteDataPhaseDiffA dataA);
    void LogSpoofingResult(const SpoofingResult result);
    void LogGNSSData(const GNSSData data, int i);
    void LogSatelliteDataPhaseDiffType(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT);

private:
    enum ProjectNum
    {
        GN902,   // GN902设备: 全向天线，7阵元
        GN930U,  // GN930U设备: 全向天线，7阵元
        GN930,   // GN930设备: 全向天线，7阵元，双板卡
        GN560    // GN560设备: 定向天线，7阵元
    };

    // ---- 频率与阵列参数 ----
    std::map<int, double> m_F;                        ///< 各频点载波频率(Hz), key=TypeInt(sys,type)=sys*100+type
    std::map<int, int> m_Detection_Threshold;         ///< 各频点欺骗检测卫星数阈值(相位差聚簇的最小卫星数)
    std::map<int, double> m_Detection_PhsThreshold;   ///< 各频点相位差检测阈值(单位:周)
    std::map<int, double> m_R;                        ///< 各频点阵列半径(米)

    // ---- 检测记录与模板数据 ----
    std::map<int, std::vector<std::vector<double>>> m_Theory;                 ///< 各频点理论相位差模板 [360角度][阵元]
    std::map<int, std::vector<std::vector<complex<double>>>> m_SimulateA;     ///< 各频点仿真幅相数据(幅相法测向用)

    // ---- 校正与处理结果 ----
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>> m_CorrectionData;         ///< 通道校正相位差(按频点/PRN)
    std::map<int, std::map<int, std::vector<double>>> m_Pseudo_Spectrum_Value;      ///< 各频点各星伪谱向量(伪谱积分用)
    std::map<int, std::vector<AlarmData>> m_AngleResultData;                        ///< 测向结果: 各频点报警卫星列表
    std::map<int, std::map<int, double>> m_Max_Snr;                                 ///< 各频点各星最大信噪比

    // ---- 外部交互 ----
    vector<double> m_Angle_Accumulate;                              ///< 伪谱角度累加器(360维, 跨轮累计)
    std::map<int, std::map<int, InterferInfo>> m_infoData180;       ///< 180° 模糊度检测的历史相位差

    // =========================================================================
    // 循环切刀欺骗检测状态（对应 Python detection_lib.py）
    // =========================================================================
    static const double CNR_MIN_DB;              ///< 参与检测的最小载噪比(dB-Hz)
    static const double STABILITY_RANGE_DEG;     ///< 相位差稳定范围阈值(度)
    static const int MIN_STABLE_SAMPLES;         ///< 判定稳定所需最小样本数

    std::map<int, int> m_ConsecutiveAlarm;       ///< 各频点连续报警计数

    struct TrackingInfo
    {
        std::set<int> cluster_sats;   ///< 被跟踪的欺骗卫星簇(PRN集合)
        double doa_deg = -1.0;        ///< 当前跟踪的测向角度(度, -1=未确定)
        double quality = -1.0;        ///< 当前跟踪的测向质量(0-100, -1=未确定)
    };
    std::map<int, TrackingInfo> m_Tracking;      ///< 各频点欺骗跟踪状态

    std::map<int, std::map<int, SatelliteDataPhaseDiffB>> m_Baselines;  ///< 跨周期累计的基线相位差(补缺刀用)

protected:
    // =========================================================================
    // 配置与控制参数 (Configure / control parameters)
    //
    // 说明: 以下参数控制欺骗检测与测向引擎的行为。多数在 Init() 中会被项目默认值
    //       覆盖, 也可通过公开接口(如 configCyclicRuntime / setThresholdDetectionDoa)
    //       在运行时调整。取值 0/1 的参数均为开关(0=关闭, 1=开启)。
    // =========================================================================

    /// 连续报警确认次数: 某频点需连续 m_Detection_Recodds_Num 次检测到欺骗
    /// 才将其卫星簇纳入跟踪, 用于滤除偶发跳变(对应 Python 的 ALARM_CONSECUTIVE_P)。
    int m_Detection_Recodds_Num = 1;

    /// 项目/设备型号(ProjectNum 枚举): 决定阵元数、切刀序列、阵列半径等默认参数。
    /// GN902/GN930U=全向7阵元, GN930=全向7阵元双板卡, GN560=定向7阵元。
    int m_Project_flg = GN930U;

    /// 阵列阵元数量(7)。
    int m_AntennaNum = 7;

    /// 默认相位差检测阈值(度): 用于 initDetectionThreshold 初始化各频点阈值。
    /// 实际各频点阈值存于 m_Detection_PhsThreshold(单位:周)。
    double m_Phasediff_Threshold = 5;

    /// 天线类型: 0=全向天线(默认), 1=定向天线。决定测向流程走全向还是定向分支。
    int m_antnenaType = 0;

    /// 每个切刀位置的帧数: >1 时启用多帧平滑或取末帧处理(由 m_Smooth_Flag 决定方式)。
    int m_OneCut_Frams = 1;

    /// 平滑标志: 1=对同一切刀多帧相位差做圆周均值平滑; 0=直接取末帧。
    int m_Smooth_Flag = 0;

    /// 测向所需的最大切刀(天线对)数量(定向天线按信噪比选天线时使用)。
    int m_Doa_Cut_Num = 7;

    /// 默认欺骗检测卫星数阈值(相位差聚簇的最小卫星数)。
    int m_Detection_Threshold_Num = 2;

    /// 伪谱积分开关: 1=输出多星综合伪谱角度(累计+加权), 0=关闭。
    int m_PseudoSpectrum_Flag = 0;

    /// 测向过程中的欺骗检测开关(非循环切刀模式下的检测分支)。
    int m_Doa_Detection_Flag = 0;

    /// 循环切刀检测开关: 1=启用跨轮循环切刀检测与跟踪(主流程), 0=关闭。
    int m_Cyclic_Detection_Flag = 0;

    /// 是否打印原始 GNSSData 日志: 0=否, 1=是。
    int m_Save_Original_Flg = 0;

    /// 参与测向/检测的最小载噪比阈值(dB-Hz): 低于该值的天线对/卫星被剔除。
    double m_Snr_Threshold = 0.0;

    /// 测向质量阈值(0-100): 质量低于该值的测向结果被丢弃(不参与报警)。
    double m_Qulity_Threshold = 0.0;

    /// 测向算法选择: 1=相关干涉仪(默认), 2=幅相法, 3=仿真相位模板干涉仪。
    int m_Doa_Arithmetic = 1;

    /// 测向所需的最小有效切刀数: 有效切刀数低于该值则本轮不测向。
    int m_Doa_Cut_min_Num = 6;

    /// 日志文件路径前缀(完整路径 = 前缀 + 序号 + ".log")。
    string m_LogFile = "./spoofingDoaLog_";

    /// 伪谱角度累加器衰减系数: 每轮结束后 m_Angle_Accumulate[i] *= 该系数(0=每轮清零)。
    double m_Accumulate_multiplier = 0;

    /// 仿真幅相数据文件目录(幅相法/仿真相位模板测向时读取 A-*/P-*.csv)。
    string m_Simulate_Data_file = "/simulateData/";

    /// 是否剔除信噪比不全(某切刀 SNR 缺失)的卫星: 1=剔除, 0=保留。
    int m_Delete_Prn_Flag = 1;

    /// 二次测向开关: 1=用虚拟阵列对 180° 模糊度做二次判定, 0=关闭。
    int m_Secondary_Doa_Flag = 0;

    /// 虚拟阵列扩展开关: 1=生成虚拟阵元参与测向, 0=关闭。
    int m_Virtual_Flag = 0;

    /// 180° 模糊度判定质量差阈值: 若候选 180° 解质量超出主解该阈值则采用该解。
    double m_Detection180_Qulity_Threshold = 10.0;

    /// 虚拟阵列倍数: 虚拟相位差 = 实阵元相位差 × 该系数。
    double m_Virtual_Multiple = 0.94;

    /// 定向天线按信噪比选天线开关: 1=按信噪比挑选测向天线, 0=按固定顺序。
    int m_getUseAntennaBySnr_Flag = 0;

    /// 阵列半径配置文件路径(定向天线 m_antnenaType==1 时读取频率-半径表)。
    string m_Radr;

    /// 全向天线阵列半径(米): 直接决定理论相位差模板。
    double m_omni_R = 0.1865;

    /// 检测标签: 1=本批仅做欺骗检测(不测向), 0=正常测向。
    int m_Detection_Tag = 0;

    /// 仿真幅相数据的频率列表(Hz): 幅相法/仿真相位模板按最近频率匹配。
    vector<double> m_All_Simulate_data_Fre = {1176e6, 1279e6, 1561e6, 1602e6};

    /// 切刀(天线对)序列: 每项 {通道1天线, 通道2天线}; 两值相等表示校正刀(不参与测向)。
    vector<vector<int>> m_cutSequence = {{7, 7}, {1, 2}, {1, 3}, {1, 4}, {7, 7}, {1, 5}, {1, 6}, {1, 7}};
};


// =============================================================================
// GN902 接口类（保持与 interface.h / interface.cpp 调用的签名一致）
// 对外 C 接口 Create_GN902 / SetThresholdDetection_GN902 / SetData_GN902 /
// GetResult_GN902 / Release_GN902 最终都落到本类方法上。
// =============================================================================
class GN902
{
public:
    GN902();   ///< 构造: 创建引擎并初始化循环切刀运行参数
    ~GN902();  ///< 析构: 释放引擎

    /**
     * @brief 设置欺骗检测阈值
     * @param phsDiffThreshold       位相差检测阈值(度)
     * @param satelliteCountThreshold 卫星数阈值(相位差聚簇的最小卫星数)
     * @param cutCountThreshold       连续确认刀数(>0 时生效, 对应连续报警确认次数)
     * @param sysEnum                 卫星系统编码
     * @param typeEnum                频点编码
     */
    void SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum);

    /**
     * @brief 流式喂入一帧数据
     * @param data     本帧 GNSS 数据
     * @param cutIdx_1 通道1天线索引(参考天线, 固定为 1)
     * @param cutIdx_2 通道2天线索引(1=校正刀, 2..7=六测向刀)
     * @note 校正刀再次出现时自动判定上一轮结束并触发检测+测向
     */
    void SetData(const GNSSData* data, int cutIdx_1, int cutIdx_2);

    /**
     * @brief 取最近一轮测向结果
     * @param result 输出结果
     */
    void GetResult(SpoofingResult& result);

private:
    void Detect();  ///< 整轮组批并喂入引擎, 完成循环切刀欺骗检测与跟踪
    void Doa();     ///< 从引擎取出本轮测向结果
};
