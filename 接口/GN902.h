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
//   - 测向算法基类 ArithmeticDoa (相关干涉仪)
//   - 公共工具命名空间 PublicSpace (日志 / 角度规整)
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

    // ==================== 数学工具 ====================
    int Round360(int x); // 将整数角度规整到[0,360)范围

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

    /*==================天线对/基线扩展==================*/
    void setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

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
    // 标识为1，对应系统频点为欺骗信号
    // 当前这一帧数据，取卫星号的并集
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

    void Init(void); // 初始化: 读取配置、初始化频率表、理论相位差、检测历史记录等

    void setThresholdDetectionDoa(int sys, int type, int threshold, double phsThreshold);
    void setGNSSData(const GNSSData *data, int dataLen);
    int getAngleSpoofingDoa(SpoofingResult &result);
    void setCutSquence(int len, const int *cutSq);
    void setDetectionRecordNum(int num);
    void configCyclicRuntime(bool cyclic, int oneCutFrams, bool smooth, double omniR);

private:
    // ---- PreparationData.cpp: 数据预处理 ----
    void getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA);
    void getSatelliteDataPhaseDiffB(const vector<vector<SatelliteDataPhaseDiffA>> &dataA, vector<SatelliteDataPhaseDiffB> &dataB);
    void clearSatelliteDataPhaseDiffB(SatelliteDataPhaseDiffB &dataB);
    void getSatelliteDataByType(const std::vector<SatelliteDataPhaseDiffA> &dataA, std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT);

    // ---- InitData.cpp: 初始化模块 ----
    int TypeInt(int sys, int type);
    void initType(void);
    void initDetectionThreshold(int threshold, double phsThreshold);

    // ---- Corrected.cpp: 校正数据处理 ----
    void setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA);
    void calCorrectionOffset(const vector<vector<SatelliteDataPhaseDiffA>> &calCuts);
    void getCorrectedGnssData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    void calCorrecteData(SatelliteDataPhaseDiffA &dataA);

    // ---- Omni.cpp: 全向天线测向 ----
    void setInterferInfoDataOmni(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData);

    // ---- Interfer.cpp: 相关干涉仪测向 ----
    void getResultInterferDoa(vector<SatelliteDataPhaseDiffB> dataB);
    void initTheory(void);

    // ---- Alarm.cpp: 欺骗检测与告警 ----
    void calAlarmByPhaseDiff(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm);

    // ---- 综合调度与辅助函数 ----
    void setDataAngle(const GNSSData *data, int dataLen);
    void setR(void);
    void calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg);
    void getSmoothData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    void calSmoothData(SatelliteDataPhaseDiffB dataB, SatelliteDataPhaseDiffA &dataA);
    void getEndFramData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    void setSpoofingResult(SpoofingResult &result);
    void calAngle(std::map<int, std::map<int, InterferInfo>> inferInfoData);
    void saveGNSSData(const GNSSData *data, int dataLen);

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
    // ---- 频率与阵列参数 ----
    std::map<int, double> m_F;
    std::map<int, int> m_Detection_Threshold;
    std::map<int, double> m_Detection_PhsThreshold;
    std::map<int, double> m_R;

    // ---- 检测模板数据 ----
    std::map<int, std::vector<std::vector<double>>> m_Theory;

    // ---- 校正与处理结果 ----
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>> m_CorrectionData;
    std::map<int, std::vector<AlarmData>> m_AngleResultData;
    std::map<int, std::map<int, double>> m_Max_Snr;

    // =========================================================================
    // 循环切刀欺骗检测状态（对应 Python detection_lib.py）
    // =========================================================================
    static const double CNR_MIN_DB;
    static const double STABILITY_RANGE_DEG;
    static const int MIN_STABLE_SAMPLES;

    std::map<int, int> m_ConsecutiveAlarm;

    struct TrackingInfo
    {
        std::set<int> cluster_sats;
        double doa_deg = -1.0;
        double quality = -1.0;
    };
    std::map<int, TrackingInfo> m_Tracking;

    std::map<int, std::map<int, SatelliteDataPhaseDiffB>> m_Baselines;

protected:
    // =========================================================================
    // 配置与控制参数
    // =========================================================================
    int m_Detection_Recodds_Num = 1;
    int m_AntennaNum = 7;
    double m_Phasediff_Threshold = 5;
    int m_OneCut_Frams = 1;
    int m_Smooth_Flag = 0;
    int m_Detection_Threshold_Num = 2;
    int m_Cyclic_Detection_Flag = 0;
    int m_Save_Original_Flg = 0;
    double m_Snr_Threshold = 0.0;
    double m_Qulity_Threshold = 0.0;
    int m_Doa_Cut_min_Num = 6;
    string m_LogFile = "./spoofingDoaLog_";
    int m_Delete_Prn_Flag = 1;
    double m_omni_R = 0.1865;
    vector<vector<int>> m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
};


// =============================================================================
// GN902 接口类（保持与 interface.h / interface.cpp 调用的签名一致）
// =============================================================================
class GN902
{
public:
    GN902();
    ~GN902();
    void SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum);
    void SetData(const GNSSData* data, int cutIdx_1, int cutIdx_2);
    void GetResult(SpoofingResult& result);
private:
    void Detect();
    void Doa();
};
