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
#include <cstddef>
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
constexpr double PI = 3.14159265358979323846;

// =============================================================================
// 卫星系统 / 频点编码 → 可读名称(字母)映射，用于日志打印
// =============================================================================

// 卫星系统编码 → 名称
inline const char* GetSysName(int sys)
{
    switch (sys)
    {
    case 0: return "GPS";
    case 1: return "GLONASS";
    case 2: return "SBAS";
    case 3: return "Galileo";
    case 4: return "BDS";
    case 5: return "QZSS";
    default: return "UNKNOWN";
    }
}

// 频点编码 → 名称(字母)
inline const char* GetTypeName(int sys, int type)
{
    int key = sys * 100 + type;
    switch (key)
    {
    // GPS
    case 0:   return "L1_CA";
    case 2:   return "L5C";
    case 5:   return "L2_P";
    case 9:   return "L2_P_codeless";
    case 14:  return "L5_Q";
    case 16:  return "L1C";
    case 17:  return "L2_C";
    // GLONASS
    case 100: return "G1";
    case 101: return "G2";
    case 105: return "G2_P";
    case 106: return "G3";
    // SBAS
    case 200: return "L1_CA";
    case 206: return "L5C";
    // Galileo
    case 301: return "E1_B";
    case 302: return "E1C";
    case 307: return "E6C";
    case 312: return "E5a_Q";
    case 317: return "E5b_Q";
    case 320: return "AltBOC_Q";
    // BDS
    case 400: return "B1I";
    case 417: return "B2I";
    case 402: return "B3I";
    case 408: return "B1C";
    case 412: return "B2a";
    case 419: return "B2b";
    case 434: return "B1X";
    case 449: return "B3I";
    case 447: return "B3I";
    // QZSS
    case 500: return "L1_CA";
    case 514: return "L5_Q";
    case 517: return "L2_C";
    case 516: return "L1C";
    default:  return "UNKNOWN";
    }
}


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
    // void Log(const char *format, ...); // 编写日志（支持可变参数格式化）

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
#pragma pack(push, 1)

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

#pragma pack(pop)

/**
 * @class ArithmeticDoa
 * @brief 测向算法核心类，封装相关干涉仪测向方法(理论模板生成、相关匹配、质量评估)
 */
class ArithmeticDoa
{
public:
    ArithmeticDoa(void);
    ~ArithmeticDoa(void);

protected:
    /*==================相关干涉仪测向==================*/

    /**
     * @brief 计算均匀圆阵的理论相位差模板 [360角度][阵元]
     * @param f         载波频率(Hz)
     * @param r         阵列半径(米)
     * @param antnnaNum 阵元数量
     * @param theory    输出: 理论相位差模板(弧度)
     */
    void calPhaseTheory(const double f, const double r, const int antnnaNum, std::vector<std::vector<double>> &theory);

    /**
     * @brief 计算搜索范围内实测相位差与理论模板的相关度伪谱
     * @param phaseTheory 理论相位差模板
     * @param data        实测信息(天线对、相位差、搜索范围)
     * @param diff        输出: 各角度相关度
     */
    void calPseudoByInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, vector<double> &diff);

    /**
     * @brief 相关干涉仪测向: 逐角度余弦相关匹配, 取最高相关峰对应角度
     * @param phaseTheory 理论相位差模板
     * @param data        实测信息(天线对、相位差、搜索范围)
     * @param angle       输出: 最匹配来波角度(度)
     * @param quality     输出: 测向质量(0-100)
     * @param diff2       输出: 各角度相关度伪谱
     */
    void calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2);

    /*==================天线对/基线扩展==================*/

    /**
     * @brief 由已知天线对相位差做传递闭合, 扩展出更多天线对基线
     * @param cutSequence 天线对序列(输入, 并追加推导出的新天线对)
     * @param phaseDiff   对应相位差(输入, 并追加推导出的新相位差)
     */
    void setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    /*==================测向质量评估==================*/

    /**
     * @brief 计算测向质量分(0-100): 理论伪谱与实测伪谱的归一化相关, 限定角度范围
     * @param diffTheory 理论伪谱
     * @param diff       实测伪谱
     * @param starAngle  起始角度(度)
     * @param endAngle   结束角度(度)
     */
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff, int starAngle, int endAngle);

    /**
     * @brief 计算测向质量分(0-100): 全 360° 范围版本
     * @param diffTheory 理论伪谱
     * @param diff       实测伪谱
     */
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff);

};


// =============================================================================
// 宿主/接口共享结构 (原 GN902.h, 保持与 DLL 边界 ABI 一致)
// =============================================================================
#pragma pack(push, 1)
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

// 单端口最多卫星条数 = i_PortOne/i_PortTwo 数组长度。
// 引擎内所有按 i_PortOneNum/i_PortTwoNum 遍历的循环都必须先夹到该范围。
constexpr int GN902_MAX_PORT_SAT = 500;

struct GNSSData
{
    int i_PortOneNum;                            // Port1卫星条数(有效范围 0..GN902_MAX_PORT_SAT)
    SatelliteData i_PortOne[GN902_MAX_PORT_SAT]; // Port1所有卫星数据
    int i_PortTwoNum;                            // Port2卫星条数(有效范围 0..GN902_MAX_PORT_SAT)
    SatelliteData i_PortTwo[GN902_MAX_PORT_SAT]; // Port2所有卫星数据
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
    int i_Angle;   // 测向角度；-1 表示本轮该星未测出来向角度
    double i_Quality; // 测向质量（0-100）；-1 表示本轮该星未测出来向角度
};


struct SatelliteAngle
{
    int i_Sys;       // 卫星系统
    int i_Type;      // 卫星频点
    int i_Alarm;     // 报警标识：0=正常，1=欺骗
    double i_Angle;  // 欺骗信号来向角度（度）；-1 表示本轮该频点未测出来向角度
    int i_Count;     // 被欺骗卫星数(= 跟踪到的欺骗卫星簇大小)，与是否测出角度无关
    AlarmData i_AlarmData[32]; // 最多32颗报警卫星详情
};

struct SpoofingResult
{
    int i_Count;                          // 报警频点数
    SatelliteAngle i_SatelliteAngle[24];  // 最多24个频点的结果
};


/**
 * @struct AlarmMoment
 * @brief 一个"报警时刻"的测向结果。
 *
 * 测向结果的输出单位不是"测向轮"，而是"某个切刀时刻某频点给出报警"这一事件：
 * 每个报警时刻立即用当前已累积的基线(基线只由各报警时刻的相位差累积而成)测向，
 * 生成一条记录。该时刻尚未测出角度时 i_Angle 记 -1，但 i_Count/i_AlarmData 照常给出。
 */
struct AlarmMoment
{
    int i_Cut;                 // 该时刻的切刀序号(1..6 = 测向刀 {1,2}..{1,7})
    int i_Sys;                 // 卫星系统
    int i_Type;                // 卫星频点
    int i_Alarm;               // 报警标识：1=欺骗
    double i_Angle;            // 该时刻测出的来向角度(度)；-1=该时刻尚未测出
    int i_Count;               // 该时刻报警的欺骗卫星数
    AlarmData i_AlarmData[32]; // 该时刻报警的卫星明细(未测出角度的卫星 Angle/Quality=-1)
};




// =============================================================================
// 引擎中间结构 (原 SpoofingDoa.h, 参与通道间数据/单星欺骗结果)
// =============================================================================

// 单颗卫星在两个通道间的相位差数据(细粒度, 单帧)
struct SatelliteDataPhaseDiffA
{
    int i_Prn;           // 卫星号
    int i_Sys;           // 卫星系统
    int i_Type;          // 卫星频点
    float i_Snr1;        // 第一通道(PortOne)的载噪比(单位:dB-Hz)
    float i_Snr2;        // 第二通道(PortTwo)的载噪比(单位:dB-Hz)
    double i_phase_diff; // 载波相位差(单位:周, [0,1)); 符号=PortOne−PortTwo; =−1 表示仅单端口出现
};

// 单颗卫星在各切刀位置的相位差数据(粗粒度, 一轮)
struct SatelliteDataPhaseDiffB
{
    int i_Prn;  // 卫星号
    int i_Sys;  // 卫星系统
    int i_Type; // 卫星频点
    int i_diffLen;                           // 有效相位差数量(= 一轮切刀数, 通常7)
    float i_Snr1[100] = {0.0};               // 第一通道(PortOne)的载噪比，索引=切刀序号
    float i_Snr2[100] = {0.0};               // 第二通道(PortTwo)的载噪比，与切刀序号一一对应
    double i_phase_diff[100] = {0.0};        // 载波相位差(单位:周)，符号=PortOne−PortTwo，索引=切刀序号
};

#pragma pack(pop)

// =============================================================================
// ABI 自检(编译期)
// 上面这些结构体是跨 DLL/so 边界的: 宿主在自己的栈上定义 SpoofingResult/GNSSData,
// 库通过引用往里写(GetResult_GN902 里的 result = it->second->result)。
// 因此"宿主头文件"与"库头文件"必须是同一次构建的同一份: 只要差一个字节,
// 库就会按自己的尺寸写穿调用方的栈对象 —— 正是
//     *** stack smashing detected ***: terminated
// 的成因(历史上 AlarmData::i_Angle 从 double 改 int、外加 #pragma pack(1)
// 使 SpoofingResult 由 19208 变 15940 字节，新旧混用即触发)。
// 下面把尺寸与关键字段偏移钉死: 任何人再改对齐/字段/数组长度，
// 都会在这里编译期报错，而不是留到运行时崩。
// =============================================================================
static_assert(sizeof(SatelliteData) == 44, "ABI 变更: SatelliteData");
static_assert(sizeof(GNSSData) == 44008, "ABI 变更: GNSSData");
static_assert(sizeof(AlarmData) == 20, "ABI 变更: AlarmData");
static_assert(sizeof(SatelliteAngle) == 664, "ABI 变更: SatelliteAngle");
static_assert(sizeof(SpoofingResult) == 15940, "ABI 变更: SpoofingResult");
static_assert(sizeof(AlarmMoment) == 668, "ABI 变更: AlarmMoment");

static_assert(offsetof(SatelliteData, i_Psr) == 12, "ABI 变更: SatelliteData::i_Psr");
static_assert(offsetof(SatelliteData, i_Phase) == 24, "ABI 变更: SatelliteData::i_Phase");
static_assert(offsetof(GNSSData, i_PortTwoNum) == 22004, "ABI 变更: GNSSData::i_PortTwoNum");
static_assert(offsetof(AlarmData, i_Angle) == 8, "ABI 变更: AlarmData::i_Angle");
static_assert(offsetof(AlarmData, i_Quality) == 12, "ABI 变更: AlarmData::i_Quality");
static_assert(offsetof(SatelliteAngle, i_Angle) == 12, "ABI 变更: SatelliteAngle::i_Angle");
static_assert(offsetof(SatelliteAngle, i_AlarmData) == 24, "ABI 变更: SatelliteAngle::i_AlarmData");
static_assert(offsetof(SpoofingResult, i_SatelliteAngle) == 4, "ABI 变更: SpoofingResult::i_SatelliteAngle");

/**
 * @brief ABI 指纹(由上面各结构体尺寸算出, 编译期常量)。
 *
 * 宿主启动时与库导出的 GetAbiSignature_GN902() 比对:
 *   不相等 ⇒ 正在使用的头文件与实际加载的库不是同一次构建
 *            (典型: 改了结构体但宿主加载的还是旧 .so/.dll)，必须重新编译库。
 * 此时若继续调用 GetResult_GN902，库会按自己的结构体尺寸写宿主的栈对象，
 * 触发 *** stack smashing detected *** 或静默的字段错位。
 * @return 指纹值
 */
inline unsigned int GN902AbiSignature(void)
{
    unsigned int s = 0u;
    s = s * 131u + (unsigned int)sizeof(SatelliteData);
    s = s * 131u + (unsigned int)sizeof(GNSSData);
    s = s * 131u + (unsigned int)sizeof(AlarmData);
    s = s * 131u + (unsigned int)sizeof(SatelliteAngle);
    s = s * 131u + (unsigned int)sizeof(SpoofingResult);
    s = s * 131u + (unsigned int)sizeof(AlarmMoment);
    return s;
}

// =============================================================================
// SpoofingDoa 主类
// 继承自 ArithmeticDoa，实现欺骗干扰检测与测向的完整流程。
// 一轮处理流程见 GN902.cpp 文件头部"模块级说明"：提取相位差 → 平滑/取末帧 →
// 通道校正 → 聚类检测+跟踪 → 跨周期基线累积 → 相关干涉仪测向。
// =============================================================================
class SpoofingDoa : public ArithmeticDoa
{
public:
    SpoofingDoa(void);
    ~SpoofingDoa(void);

    /**
     * @brief 初始化引擎: 建立频率表、理论相位差模板、检测历史记录等全部运行状态
     * @note 构造后即可按默认值运行(不读配置文件), 运行参数已内置在 Init 中
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
     * @param data    GNSS 数据数组(按切刀序列排列)
     * @param dataLen 数据条数(切刀数量)
     */
    void setGNSSData(const GNSSData *data, int dataLen);

    /**
     * @brief 采集固定基线(如天线对{8,9}/{9,8})的两端口逐星载波相位差(原始)
     * @param data  单帧 GNSS 数据(两通道)
     * @param dataA 输出: 各卫星相位差, 仅含两端口同时出现的卫星(相位差单位: 周)
     * @note 仅做采集: 不做信噪比过滤/通道校正/检测/测向
     */
    void collectPhaseDiffData(const GNSSData &data, std::vector<SatelliteDataPhaseDiffA> &dataA);

    /**
     * @brief 取出最近一轮的测向/报警结果
     * @param result 输出结果(含各频点报警角度、卫星明细)
     * @return 0=成功
     */
    int getAngleSpoofingDoa(SpoofingResult &result);

    /**
     * @brief 取最近一轮内所有"报警时刻"的测向结果
     * @return 按时刻先后排列的报警记录(每个给出报警的切刀时刻、每个报警频点一条)
     * @note 记录以"时刻是否给出报警"为准生成，不按整轮罗列已跟踪频点。
     */
    const std::vector<AlarmMoment> &getAlarmMoments(void) const;

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
     * @param omniR       全向天线阵列半径(米, >0 时重建理论模板)
     */
    void configCyclicRuntime(bool cyclic, int oneCutFrams, bool smooth, double omniR);

private:
    // ---- PreparationData.cpp: 数据预处理 ----
    void getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA, bool snrFilter = true);
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

    /**
     * @brief 取某频点某星的载噪比(用于报警卫星明细)
     * @param typeInt 频点编码(sys*100+type)
     * @param prn     卫星号
     * @return 最大载噪比(dB-Hz); 无数据返回 0.0
     * @note 优先取跨周期累积基线中该星任意刀位的最大值(不要求凑齐 6 条基线),
     *       回退到测向时记录的 m_Max_Snr。避免未测出角度的卫星信噪比恒为 0。
     */
    double getSatMaxSnr(int typeInt, int prn);
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

    /**
     * @brief 取指定频点、指定卫星集合的跨周期累积基线(用于单个报警时刻的实时测向)
     * @param typeInt  频点编码(sys*100+type)
     * @param sats     该时刻给出报警的卫星集合
     * @param doaDataB 输出: 这些卫星已累积的基线(未累积到基线的卫星被跳过)
     */
    void getCrossCycleDataBByType(int typeInt, const std::set<int> &sats,
                                  std::vector<SatelliteDataPhaseDiffB> &doaDataB);

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
    std::map<int, double> m_F;                        ///< 各频点载波频率(Hz), key=TypeInt(sys,type)=sys*100+type
    std::map<int, int> m_Detection_Threshold;         ///< 各频点欺骗检测卫星数阈值(相位差聚簇的最小卫星数)
    std::map<int, double> m_Detection_PhsThreshold;   ///< 各频点相位差检测阈值(单位:周)
    std::map<int, double> m_R;                        ///< 各频点阵列半径(米)

    // ---- 检测模板数据 ----
    std::map<int, std::vector<std::vector<double>>> m_Theory;   ///< 各频点理论相位差模板 [360角度][阵元]

    // ---- 校正与处理结果 ----
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>> m_CorrectionData;   ///< 通道校正相位差(按频点/PRN)
    std::map<int, std::vector<AlarmData>> m_AngleResultData;                  ///< 测向结果: 各频点报警卫星列表
    std::map<int, std::map<int, double>> m_Max_Snr;                           ///< 各频点各星最大信噪比

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

    std::map<int, std::map<int, SatelliteDataPhaseDiffB>> m_Baselines;  ///< 跨周期累计的基线相位差(只由"给出报警的时刻"累积)

    std::vector<AlarmMoment> m_AlarmMoments;  ///< 最近一轮内各"报警时刻"的测向结果(每轮开头清空)

protected:
    // =========================================================================
    // 配置与控制参数 (Configure / control parameters)
    // 说明: 以下参数控制欺骗检测与测向引擎的行为。多数在 Init() 中会被默认值覆盖,
    //       也可通过公开接口(如 configCyclicRuntime / setThresholdDetectionDoa)
    //       在运行时调整。取值 0/1 的参数均为开关(0=关闭, 1=开启)。
    // =========================================================================

    /// 连续报警确认次数: 某频点需连续 m_Detection_Recodds_Num 次检测到欺骗
    /// 才将其卫星簇纳入跟踪, 用于滤除偶发跳变(对应 Python 的 ALARM_CONSECUTIVE_P)。
    int m_Detection_Recodds_Num = 1;

    /// 阵列阵元数量(7)。
    int m_AntennaNum = 7;

    /// 默认相位差检测阈值(度), 用于 initDetectionThreshold 初始化各频点阈值。
    double m_Phasediff_Threshold = 5;

    /// 每个切刀位置的帧数: >1 时启用多帧平滑或取末帧处理(由 m_Smooth_Flag 决定方式)。
    /// 实际使用中每刀只喂最后一秒(1帧)，即 oneCutFrams=1，平滑/取末帧路径不生效。
    int m_OneCut_Frams = 1;

    /// 平滑标志: 1=对同一切刀多帧相位差做圆周均值平滑; 0=直接取末帧。
    int m_Smooth_Flag = 0;

    /// 默认欺骗检测卫星数阈值(相位差聚簇的最小卫星数)。
    int m_Detection_Threshold_Num = 2;

    /// 循环切刀检测开关: 1=启用跨轮循环切刀检测与跟踪(主流程)。
    int m_Cyclic_Detection_Flag = 0;

    /// 是否打印原始 GNSSData 日志: 0=否, 1=是。
    int m_Save_Original_Flg = 0;

    /// 参与测向/检测的最小载噪比阈值(dB-Hz): 低于该值的天线对/卫星被剔除。
    double m_Snr_Threshold = 0.0;

    /// 测向质量阈值(0-100): 质量低于该值的测向结果被丢弃(不参与报警)。
    double m_Qulity_Threshold = 10.0;

    /// 测向所需的最小有效切刀数: 有效切刀数低于该值则本轮不测向。
    int m_Doa_Cut_min_Num = 6;

    /// 日志文件路径前缀(完整路径 = 前缀 + 序号 + ".log")。
    string m_LogFile = "./spoofingDoaLog_";

    /// 是否剔除信噪比不全(某切刀 SNR 缺失)的卫星: 1=剔除, 0=保留。
    int m_Delete_Prn_Flag = 1;

    /// 全向天线阵列半径(米): 直接决定理论相位差模板。
    double m_omni_R = 0.1865;

    /// 切刀(天线对)序列: 每项 {通道1天线, 通道2天线}; 两值相等表示校正刀(不参与测向)。
    /// 顺序对齐 Python CODE_TO_PAIR: 校正 {1,1} + 六测向刀 {1,2}..{1,7}。
    vector<vector<int>> m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
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
     * @param sysEnum                 卫星系统编码
     * @param typeEnum                频点编码
     */
    void SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, int sysEnum, int typeEnum);

    /**
     * @brief 设置连续切刀数(连续报警确认次数)
     * @param thresholdCount 连续报警确认次数(>0 时生效, 对应引擎 m_Detection_Recodds_Num)
     * @note 由 SetCutnumThreshold_GN902 接口调用, 覆盖引擎默认值(2);
     *       不再从 txt 配置文件读取。
     */
    void SetCutnumThreshold(int thresholdCount);

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

    /**
     * @brief 取最近一轮内所有"报警时刻"的测向结果
     * @param out 输出: 按时刻先后排列的报警记录
     */
    void GetAlarmMoments(std::vector<AlarmMoment>& out);

private:
    void Detect();  ///< 整轮组批并喂入引擎, 完成循环切刀欺骗检测与跟踪
    void Doa();     ///< 从引擎取出本轮测向结果
};
