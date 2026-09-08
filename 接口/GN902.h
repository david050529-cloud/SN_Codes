#pragma once

// =============================================================================
// 文件名: GN902.h
// 功能描述: GN902 欺骗检测/测向模块 —— 自包含头文件(消除编译隔离)
//
// 本文件将以下头文件的全部类型/类声明与常量逐字收纳, 配合 GN902.cpp
// (收纳全部函数实现) 即可单独编译, 无需再引用 Arithmetic/、publicFunctionDoa/、
// SpoofingDoaDll/ 等外部源文件与头文件(仅依赖 C++ 标准库):
//   - 接口共享结构 (SatelliteData / GNSSData / AlarmData / SatelliteAngle / SpoofingResult)
//   - 引擎中间结构 (SatelliteDataPhaseDiffA/B、SingleDeceptiveResult)
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
    double linearInterpolation(double x1, double y1, double x2, double y2, double x); // 线性插值
    double Round3600(double x); // 将角度规整到[0,360)度范围
    int Round360(int x);

    // ==================== 配置文件与时间 ====================
    int readConfigtxt(const string adr, map<string, string> &configMap); // 读取配置文件
    string getNowTime(); // 获取当前时间

    // ==================== 日志系统 ====================
    void getLogCont(int id); // 设置日志编号ID
    void LogCreat(const string path); // 创建日志文件
    void Log(const char *format, ...); // 编写日志（支持可变参数格式化）
    void LogClose(void); // 关闭日志文件

    // ==================== 模板函数 ====================

    /**
     * @brief 将二维vector序列化为 "{{...},{...},{...}}" 格式的字符串
     */
    template <typename vec>
    std::string Vector2String(const std::vector<std::vector<vec>> &vectorData)
    {
        std::ostringstream oss;
        oss << "{";
        for (size_t i = 0; i < vectorData.size(); ++i)
        {
            oss << "{";
            for (size_t j = 0; j < vectorData[i].size(); ++j)
            {
                if (j > 0)
                {
                    oss << ",";
                }
                oss << vectorData[i][j];
            }
            oss << "}";
        }
        oss << "}";
        return oss.str();
    }

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

    /**
     * @brief 将二维vector调整为指定大小（先清空再resize）
     */
    template <typename V>
    void vectorResize(std::vector<std::vector<V>>& data, int cols, int rows)
    {
        vector<std::vector<V>>().swap(data);
        data.resize(cols);
        for (int i = 0; i < cols; i++)
        {
            data[i].resize(rows);
        }
    }

    // 从配置map中读取数据
    template<typename Value>
    int getMapData(const map<string, string>& configMap, const string key, Value& values) {
        auto it = configMap.find(key);
        if (it == configMap.end()) {
            return -1;
        }

        std::istringstream iss(it->second);
        if (!(iss >> values) || !iss.eof())
        {
            return -2;
        }
        return 0;
    }

    // 字符串转换为二维数组，将{{}， {}， {} }保存的切刀字符串数组转换为二维vector数组
    template<typename vec>
    void string2Vector(const string str, vector<vector<vec>>& result) {
        result.clear();
        vector<vec> tp_reult;
        vector<string> tp1;
        split(tp1, str, '}');
        vector<string> tp2;
        vector<string> tp3;
        for (int i = 0; i < (int)tp1.size(); i++)
        {
            tp2.clear();
            tp_reult.clear();
            split(tp2, tp1[i], '{');
            for (int j = 0; j < (int)tp2.size(); j++)
            {
                tp3.clear();
                split(tp3, tp2[j], ',');
                for (int k = 0; k < (int)tp3.size(); k++)
                {
                    std::istringstream iss(tp3[k]);
                    vec tp4;
                    if (!(iss >> tp4) || !iss.eof())
                    {
                        return;
                    }
                    tp_reult.emplace_back(tp4);
                }
            }
            result.emplace_back(tp_reult);
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
    void calAmpPhase(const vector<vector<complex<double>>> simulateA, const vector<complex<double>> actualA, double& angle, double& quality, vector<double>& diff);
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
    int getAntennaPhase(const vector<int> use_ant, const int antennNUm, vector<vector<int>> &cutSequence, vector<double> &phaseDiff);
    void getUsePhaseDiffAll(const vector<int> use_cut, vector<vector<int>> &cutSequence, vector<double> &phaseDiff);
    void getUseAntennaByADirect(const vector<double> A, const int use_cut_num, vector<int> &use_cut, int &startAngle, int &endAngle);
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

// 由外部欺骗检测算法传入的单个欺骗检测结果
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
// =============================================================================
class SpoofingDoa : public ArithmeticDoa
{
public:
    SpoofingDoa(void);
    ~SpoofingDoa(void);

    void Init(void); // 初始化: 读取配置、初始化频率表、理论相位差、检测历史记录等

    void setThresholdDetectionDoa(int sys, int type, int threshold, double phsThreshold);
    void setTypeDetectionBySnr(int thresholdNum, int sys, int type);
    void setGNSSData(const GNSSData *data, int dataLen);
    int getAngleSpoofingDoa(SpoofingResult &result);
    void setConfigTxtAdr(const char *adr);
    void setCutSquence(int len, const int *cutSq);
    void setSpoofingDetecteResult(const vector<SingleDeceptiveResult> detectResult);
    void setDetectionRecordNum(int num);
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
    void paraProjectGN902();
    void paraProjectGN930();

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
    void getDataByDetectionResult(const GNSSData data, GNSSData &detetectData);
    void saveGNSSData(const GNSSData *data, int dataLen);
    void getSpoofingResultByDetection(SpoofingResult &result);
    int getDetection180(int typeInt, int prn, InterferInfo &info);
    void setPermutationOptimiz180(SatelliteDataPhaseDiffB dataB, const vector<vector<double>> phaseTheory, double &angle, double &qulity);
    void getOptimizResultDoa180(vector<SatelliteDataPhaseDiffB> dataB);

    // ---- 角度工具（静态，对应 Python detection_lib 的角度函数）----
    static double normalizeAngle180(double deg);
    static double circularMeanDeg(const std::vector<double> &degs);
    static double circularSpanDeg(const std::vector<double> &degs);
    static double foldHalfCycle(double deg);
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
    std::map<int, double> m_F;
    std::map<int, int> m_Detection_Threshold;
    std::map<int, double> m_Detection_PhsThreshold;
    std::map<int, int> m_Detection_snrThrehold;
    std::map<int, double> m_R;

    // ---- 检测记录与模板数据 ----
    std::map<int, std::vector<std::vector<double>>> m_Theory;
    std::map<int, std::vector<std::vector<complex<double>>>> m_SimulateA;

    // ---- 校正与处理结果 ----
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>> m_CorrectionData;
    std::map<int, std::map<int, std::vector<double>>> m_Pseudo_Spectrum_Value;
    std::map<int, std::vector<AlarmData>> m_AngleResultData;
    std::map<int, std::map<int, double>> m_Max_Snr;

    // ---- 外部交互 ----
    vector<SingleDeceptiveResult> m_detectResult;
    vector<double> m_Angle_Accumulate;
    std::map<int, std::map<int, InterferInfo>> m_infoData180;

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
    int m_Project_flg = GN930U;
    int m_AntennaNum = 7;
    double m_Angle_Threshold = 3;
    double m_Phasediff_Threshold = 5;
    int m_antnenaType = 0;
    int m_OneCut_Frams = 1;
    int m_Smooth_Flag = 0;
    int m_Doa_Cut_Num = 7;
    int m_Detection_Threshold_Num = 2;
    int m_PseudoSpectrum_Flag = 0;
    int m_Doa_Detection_Flag = 0;
    int m_Cyclic_Detection_Flag = 0;
    int m_Save_Original_Flg = 0;
    int m_Save_Oringial_Num = 0;
    double m_Snr_Threshold = 0.0;
    double m_Qulity_Threshold = 0.0;
    int m_Doa_Arithmetic = 1;
    int m_Doa_Cut_min_Num = 6;
    string m_LogFile = "./spoofingDoaLog_";
    double m_Accumulate_multiplier = 0;
    string m_Simulate_Data_file = "/simulateData/";
    int m_Delete_Prn_Flag = 1;
    int m_Secondary_Doa_Flag = 0;
    int m_Virtual_Flag = 0;
    double m_Detection180_Qulity_Threshold = 10.0;
    double m_Virtual_Multiple = 0.94;
    int m_Screen_detection_Flag = 0;
    int m_getUseAntennaBySnr_Flag = 0;
    int m_Receiver_num = 1;
    string m_Radr;
    double m_omni_R = 0.1865;
    int m_Detection_Tag = 0;
    vector<double> m_All_Simulate_data_Fre = {1176e6, 1279e6, 1561e6, 1602e6};
    vector<vector<int>> m_cutSequence = {{7, 7}, {1, 2}, {1, 3}, {1, 4}, {7, 7}, {1, 5}, {1, 6}, {1, 7}};
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
