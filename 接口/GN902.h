#pragma once

// =============================================================================
// 文件名: GN902.h
// 功能描述: GN902 欺骗检测/测向模块 —— 自包含头文件(消除编译隔离)
//
// 本文件收纳了以下文件的全部类型/类声明与常量, 配合 GN902.cpp(收纳全部函数实现)
// 即可单独编译, 无需再引用下列原文件:
//   - GN902.h           : 宿主/接口共享结构 (SatelliteData / GNSSData /
//                         AlarmData / SatelliteAngle / SpoofingResult) 与 GN902 接口类
//   - SpoofingDoa.h     : 引擎中间结构 (SatelliteDataPhaseDiffA/B、
//                         SingleDeceptiveResult) 与主类 SpoofingDoa 声明
//   - arithmetic.h      : ArithmeticDoa 基类 + InterferInfo / Rs 结构
//   - publicFunctionDoa.h: PublicSpace 命名空间(工具函数声明 + 模板)
//
// 完全自包含: GN902.h + GN902.cpp 两个文件即可编译, 无任何外部头文件库依赖。
// 原 Arithmetic/ 与 publicFunctionDoa/ 静态库已内联; 原依赖的 Eigen 与 shlwapi
// 仅被已注释掉的 polyfit / getCurrentExecutablePath 使用, 实际未用到, 已移除。
// 仅依赖系统标准库 + windows.h。
// =============================================================================

// ---- 平台头文件 (原 framework.h / pch.h) ----
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

// ---- 标准库 ----
#include <math.h>
#include <stdarg.h>
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


// ---- 内联自 publicFunctionDoa.h 的公共工具库(声明 + 模板) ----
// 说明: 原 publicFunctionDoa.h 依赖 Eigen(<Eigen/Dense>) 与 shlwapi(PathRemoveFileSpec)，
//       但二者仅被已注释掉的 polyfit / getCurrentExecutablePath 使用，本模块实际未用到，
//       故在此一并移除，GN902.h/.cpp 不再依赖任何外部头文件库(仅系统头 + windows.h)。
#include <iomanip>
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <time.h>
#include <cmath>
#include <ctime>
#include <chrono>
#include <stdexcept>

#define m_PI 3.14159265358979323846
#define m_C 3e8

using namespace std;

namespace PublicSpace
{
    // ==================== 以下为当前项目组(Spoofing)与Suppress组均未使用的函数，暂时注释保留 ====================
    // /**
    //  * @brief 当前可执行文件所在的目录路径
    //  * @note 默认值为 "."，由 getCurrentExecutablePath() 在程序启动时赋值
    //  *       用于拼接配置文件、日志文件等相对路径的基准目录
    //  */
    // extern string m_CurrentPath;

    /**
     * @brief 日志记录控制标志
     * @note 0 - 不记录日志（默认值）
     *       1 - 记录日志，文件以覆盖模式("w")打开，每次运行会清空旧日志
     *       2 - 记录日志，文件以追加模式("a")打开，新日志追加到文件末尾
     *       全局控制所有日志输出，设为0时 getNowTime() 也返回空串以提高性能
     */
    extern int m_logFlg;

    /**
     * @brief 原始数据保存控制标志
     * @note 1 - 保存原始数据到二进制文件
     *       其他值(默认0) - 不保存原始数据
     *       saveArrayToBinary() 模板函数会检查此标志决定是否实际写入
     */
    extern int m_save_data_Flg; // 是否保存原始数据,1 保存，其他不保存

    // /**
    //  * @brief 获取当前可执行文件所在的目录路径（跨平台）
    //  * @return 返回可执行文件所在目录的绝对路径字符串，失败时返回 "."
    //  * @note Windows下使用GetModuleHandle/GetModuleFileName获取，
    //  *       Linux下使用dladdr/dirname获取
    //  */
    // std::string getCurrentExecutablePath(void); // 获得当前执行程序的路径

    // ==================== 数据操作 ====================

    /**
     * @brief 从一维数据序列中查找所有峰值（局部极大值）
     * @param data 输入的一维数据序列
     * @param index2 输出参数，峰值在原数据中的索引位置，按峰值大小降序排列
     * @param vaules2 输出参数，与index2对应的峰值数值，按降序排列
     * @note 首尾元素如果满足峰值条件也会被检测，但要求首元素同时大于第二个和最后一个元素
     */
    void findPeaks(const std::vector<double> data, vector<int> &index2, vector<double> &vaules2); // 获取峰值

    /**
     * @brief 按指定分隔符对字符串进行分割
     * @param result 输出参数，存放分割后的子字符串数组（自动去除空串和首尾空格）
     * @param str 输入的待分割字符串
     * @param str1 分隔字符
     * @note 内部会调用 trim() 去除每个子串的首尾空格
     */
    void split(vector<string> &result, string str, char str1); // 字符串分割

    /**
     * @brief 去除字符串首尾的空白字符（空格、制表符等）
     * @param str 输入/输出参数，原地修改的字符串
     */
    void trim(string &str); // 去掉字符串中的空格

    /**
     * @brief 读取文本文件内容，将每个空白分隔的单词存入字符串数组
     * @param adr 文件路径（相对或绝对路径）
     * @param data 输出参数，存放读取到的所有单词（按空白字符分割）
     * @return 0 - 读取成功; -1 - 文件无法打开（路径错误或文件不存在）
     */
    int readFile(const string adr, vector<string> &data); // 读取文件

    // ==================== 数学工具 ====================

    // /**
    //  * @brief 多项式最小二乘拟合（BdSVD求解）
    //  * @param x 自变量数据点构成的Eigen列向量
    //  * @param y 因变量数据点构成的Eigen列向量
    //  * @param n 拟合多项式的最高次数
    //  * @param coeffs 输出参数，拟合得到的多项式系数（从常数项到n次项），长度为n+1
    //  * @note 构造Vandermonde矩阵后使用BDCSVD分解求解，数值稳定性好
    //  */
    // void polyfit(const VectorXd &x, const VectorXd &y, const int n, VectorXd &coeffs); // 曲线拟合

    /**
     * @brief 计算复数向量的L2范数（欧几里得范数）
     * @param data 输入的复数向量
     * @return 返回 sqrt(sum(|data[i]|^2))，即所有复数模平方和的平方根
     */
    double getNorm(vector<complex<double>> data); // 计算复数的L2范式（欧几里得距离）

    /**
     * @brief 线性插值：已知两点(x1,y1)和(x2,y2)，求x处的线性插值
     * @param x1 第一个点的横坐标
     * @param y1 第一个点的纵坐标
     * @param x2 第二个点的横坐标
     * @param y2 第二个点的纵坐标
     * @param x 待插值的横坐标位置
     * @return x处的线性插值结果 y1 + (x-x1)*(y2-y1)/(x2-x1)
     * @throws std::invalid_argument 当x1≈x2时抛出异常（分母为零）
     */
    double linearInterpolation(double x1, double y1, double x2, double y2, double x); // 线性插值

    /**
     * @brief 将角度值规整到 [0, 3600) 范围内（以十分之一度为单位）
     * @param x 输入的角度值（度），内部会先乘以10转为十分之一度
     * @return 规整后的角度值（度），范围 [0.0, 360.0)
     * @note 常用于天线阵列的角度归一化处理
     */
    double Round3600(double x); // 将角度规整到[0,360)度范围（内部使用1/10度精度）

    /**
     * @brief 将整数角度值规整到 [0, 360) 范围内
     * @param x 输入的角度值（度，整型）
     * @return 规整后的角度值（度），范围 [0, 360)
     */
    int Round360(int x);

    // ==================== 配置文件与时间 ====================

    /**
     * @brief 读取键值对格式的配置文件（格式：key = value #注释）
     * @param adr 配置文件的路径
     * @param configMap 输出参数，存放读取到的键值对（键和值已去除首尾空格）
     * @return 0 - 读取成功; -1 - 文件无法打开
     * @note 支持以 '#' 开头的行内注释，'=' 为键值分隔符
     */
    int readConfigtxt(const string adr, map<string, string> &configMap); // 读取配置文件

    /**
     * @brief 获取当前系统时间字符串（精确到毫秒）
     * @return 格式为 "YYYY-MM-DD HH:MM:SS.mmm" 的时间字符串
     *         当 m_logFlg == 0 时直接返回空串以提高性能
     * @note 内部使用 std::chrono 获取高精度时间
     */
    string getNowTime(); // 获取当前时间

    // ==================== 日志系统 ====================
    // 日志系统设计说明：
    //   每个测向算法对象通过唯一的ID(由getLogCont设置)对应一个日志文件。
    //   日志文件路径由LogCreat中的path参数与ID拼接而成，格式为: path + ID + ".log"
    //   m_logFlg控制全局日志行为: 0=关闭, 1=覆盖写入, 2=追加写入
    //   Log()使用可变参数格式(类似printf)写入日志，内部加互斥锁保证线程安全
    //   使用流程: getLogCont(id) -> LogCreat(path) -> Log(...) -> LogClose()

    /**
     * @brief 设置当前日志编号ID，用于区分不同测向算法对象的日志
     * @param id 日志索引编号，一般对应测向算法对象的唯一ID
     * @note 此函数将id转为字符串存入内部变量m_LogCount，后续Log/LoCreat/LogClose都基于此ID操作
     */
    void getLogCont(int id); // 设置日志编号ID

    /**
     * @brief 创建并打开日志文件
     * @param path 日志文件存放目录路径，实际文件名 = path + m_LogCount + ".log"
     * @note 仅在 m_logFlg != 0 时执行
     *       m_logFlg == 1: 以覆盖模式"w"打开（清空旧内容）
     *       m_logFlg == 2: 以追加模式"a"打开（保留旧内容）
     *       文件句柄存入 m_Log_fp 映射表中，键为 m_LogCount
     */
    void LogCreat(const string path); // 创建日志文件

    /**
     * @brief 向当前日志文件写入格式化的日志内容
     * @param format 格式化字符串（与 printf 格式相同），支持可变参数
     * @note 仅在 m_logFlg != 0 时执行
     *       内部使用互斥锁(m_fileMutex)保证多线程写入安全
     *       每次写入后调用 fflush 确保内容立即落盘
     */
    void Log(const char *format, ...); // 编写日志（支持可变参数格式化）

    /**
     * @brief 关闭当前日志文件并清理资源
     * @note 关闭 m_LogCount 对应的文件句柄，并从 m_Log_fp 映射表中移除
     *       内部使用互斥锁保证线程安全
     */
    void LogClose(void); // 关闭日志文件


    template <typename Value>
    int getMapData(const map<string, string>& configMap, const string key, Value& values);

    // void string2IntVector(const string str, vector<vector<int>> &result); // 将string转为vector

    // void string2IntVector(const string str, vector<vector<double>> &result);

    /**
     * @brief 将 "{{...},{...},{...}}" 格式的字符串解析为二维vector
     * @tparam vec 内部元素的数据类型（int, double, float等支持流提取操作符的类型）
     * @param str 输入字符串，格式为嵌套花括号包围的逗号分隔数值，如 "{{1,2},{3,4}}"
     * @param result 输出参数，解析后的二维vector
     * @note 解析过程：先按 '}' 分割外层，再按 '{' 分割内层，最后按 ',' 分割数值
     *       任一数值转换失败则立即返回（不抛出异常）
     */
    template <typename vec>
    void string2Vector(const string str, vector<vector<vec>>& result);


    /**
     * @brief 将二维vector序列化为 "{{...},{...},{...}}" 格式的字符串
     * @tparam vec 内部元素的数据类型
     * @param vectorData 输入的二维vector
     * @return 格式化后的字符串，如 "{{1,2},{3,4}}"（与 string2Vector 互为逆操作）
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

    // /**
    //  * @brief 将二维复数vector调整为指定维度，并将所有元素初始化为 (1.0 + 0.0i)
    //  * @param data 输入/输出参数，要调整大小的二维复数vector
    //  * @param cols 目标列数（第一维大小）
    //  * @param rows 目标行数（第二维大小）
    //  * @note 先清空原数据，再resize并填充默认值(1.0, 0.0)
    //  */
    // void VectorResizeToOne(std::vector<std::vector<std::complex<double>>> &data, int cols, int rows); // 复数vector初始化

    /**
     * @brief 将数组数据以二进制格式追加写入文件
     * @tparam T 数组元素的数据类型
     * @param filename 目标文件路径
     * @param data 指向数组首地址的指针
     * @param size 数组元素个数
     * @note 受全局标志 m_save_data_Flg 控制，仅当该标志为 1 时才实际写入
     *       文件以追加模式(ios::app)打开，多次调用不会覆盖之前的数据
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
            // 以二进制追加模式打开文件
            std::ofstream file(filename, std::ios::binary | std::ios::app);
            if (!file)
            {
                throw std::runtime_error("无法打开文件进行写入！");
            }

            // 写入数据
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
     * @tparam V 内部元素的数据类型
     * @param data 输入/输出参数，要调整大小的二维vector
     * @param cols 目标列数（第一维大小）
     * @param rows 目标行数（第二维大小）
     * @note 先通过 swap 技巧彻底释放旧内存，再逐行 resize
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
    // @param configMap 输入配置map
    // @param key 输入键
    // @param values 输出值
    // @return 0 - 正确运行 -1 - 无效键 -2 - 无效值
    template<typename Value>
    int getMapData(const map<string, string>& configMap, const string key, Value& values) {
        auto it = configMap.find(key);
        if (it == configMap.end()) {
            return -1;
        }

        std::istringstream iss(it->second);
        if (!(iss >> values) || !iss.eof())
        {
            // 如果转换失败或者转换后流中还有剩余字符，则认为转换失败
            return -2;
        }
        return 0;
    }


    // 字符串转换为二维数组，将{{}， {}， {} }保存的切刀字符串数组转换为二位vector数组
    // @param str 输入字符串数组
    // @param result 输出切刀数组
    template<typename vec>
    void string2Vector(const string str, vector<vector<vec>>& result) {
        result.clear();
        vector<vec> tp_reult;
        vector<string> tp1;
        split(tp1, str, '}');
        vector<string> tp2;
        vector<string> tp3;
        // int cout = 0;
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
                        // 如果转换失败或者转换后流中还有剩余字符，则认为转换失败
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

constexpr double PI = 3.141592653;

// ---- 内联自 arithmetic.h: InterferInfo / Rs / ArithmeticDoa 基类 ----
#pragma pack(1)

/**
 * @struct InterferInfo
 * @brief 测向输入数据结构体，封装一次测向所需的全部原始信息
 *
 * 该结构体作为 calInterfer / calAmpPhase 等核心测向函数的输入参数，
 * 包含了实测数据的相位差、幅度以及搜索范围等关键信息。
 *
 * 天线序号约定：天线从 1 开始编号（与用户侧一致），内部计算时减 1 转为 0-based 索引。
 * 相位差定义：i_Phase_Diff[k] = 天线 i_AntennaSq[k][0] 的相位 减去 天线 i_AntennaSq[k][1] 的相位
 */
struct InterferInfo // 用于测向的相关信息
{
    int i_Start;              ///< 搜索范围开始角度（度），闭区间，用于限定相关峰搜索的范围以提高效率或适应定向天线
    int i_End;                ///< 搜索范围结束角度（度），闭区间
    int i_Phase_Len;          ///< 有效的相位差/幅度数据条数（即 i_AntennaSq、i_Phase_Diff、i_Amp 的有效长度）
    int i_AntennaSq[200][2];  ///< 相位差对应的天线对序号，相位差 = 天线[0]的相位 - 天线[1]的相位，天线序号从 1 算起
    double i_Phase_Diff[200]; ///< 实测相位差数组（弧度），与 i_AntennaSq 一一对应
    double i_Amp[200];        ///< 实测幅度数组（线性值），幅相法测向时使用，与 i_AntennaSq 一一对应
};

/**
 * @struct Rs
 * @brief 频率-阵列半径对应关系，存储某一频率区间内对应的阵列孔径（半径）
 *
 * 天线阵列的有效半径（孔径）可能随频率变化（例如不同频段使用不同的阵列校准数据），
 * 该结构体记录了一个频率区间 [i_starF, i_endF] 与其对应的阵列半径 i_r 的映射关系。
 * 频率单位为 Hz，半径单位为米。
 */
struct Rs
{
    double i_starF; ///< 起始频率（Hz）
    double i_endF;  ///< 结束频率（Hz）
    double i_r;     ///< 该频率区间对应的阵列半径（米），用于理论相位差计算
};


#pragma pack()
/**
 * @class ArithmeticDoa
 * @brief 测向算法核心类，封装相关干涉仪、幅相法、虚拟阵列扩展等测向方法
 *
 * 该类不包含数据成员（无状态设计），所有方法均为算法工具函数。
 * 调用流程通常为：
 *   1. 构造 InterferInfo 结构体，填入实测相位差/幅度数据
 *   2. 调用 calPhaseTheory 计算理论相位差，或调用 calAmpPhaseSimulateA 读取仿真阵列流型
 *   3. 调用 calInterfer（相关干涉仪）或 calAmpPhase（幅相法）执行测向
 *   4. 获取测向角度（angle）和测向质量（quality）
 *
 * 两种测向方法的核心差异：
 *   - 相关干涉仪：仅使用相位差信息，通过余弦相关度匹配，计算速度快，适用于均匀圆阵
 *   - 幅相法：同时使用幅度和相位信息，通过复数阵列流型内积匹配，对非理想天线方向图有更好的适应性
 *
 * 虚拟阵列扩展（getVirtual / calSecondDoaByVirInterf）：
 *   用于解决大孔径阵列中的相位模糊问题。当阵元间距超过半波长时，相位差可能发生"跳半周"，
 *   导致在伪谱中出现多个峰值。通过构造虚拟阵元缩短基线长度，可消除模糊得到真实角度。
 */
class ArithmeticDoa
{
public:
    ArithmeticDoa(void);
    ~ArithmeticDoa(void);

protected:
    /*==================相关干涉仪测向==================*/
    /**
     * @brief 计算均匀圆阵的理论相位（以阵列中心为参考点）
     *
     * 对于 N 元均匀圆阵，第 i 个阵元（从1开始编号）在角度 ang 处的理论相位为：
     *   phase[i] = 2*pi*f*r/c * cos(i*2*pi/N - ang*pi/180)
     * 其中 f 为频率，r 为阵列半径，c 为光速，ang 为来波方向角（度）。
     * 该公式基于平面波假设和远场条件推导。
     *
     * @param f 信号频率（Hz）
     * @param r 阵列半径（米），即圆阵的物理半径
     * @param antnnaNum 天线阵元数量
     * @param theory 输出的理论相位矩阵，theory[ang][i] 表示角度 ang（0-359度）处第 i 个阵元的理论相位（弧度）
     */
    void calPhaseTheory(const double f, const double r, const int antnnaNum, std::vector<std::vector<double>> &theory);

    /**
     * @brief 计算理论伪谱（仅用于测向质量评估，不用于角度搜索）
     *
     * 对搜索范围内的每个角度，计算该角度下的理论相位差与实际相位差的余弦相关度之和，
     * 形成一条 360 点（覆盖 0-359 度）的伪谱曲线。
     * 伪谱值越高，表示该角度的理论值与实测值越匹配。
     *
     * @param phaseTheory 理论相位差矩阵
     * @param data 实测相位差数据（相位差在 i_Phase_Diff 中，天线对在 i_AntennaSq 中）
     * @param diff 输出的理论伪谱数组，diff[ang] 存储角度 ang 处的相关度累加值
     */
    void calPseudoByInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, vector<double> &diff);

    /**
     * @brief 相关干涉仪测向主算法 -- 通过相位差相关匹配确定来波方向
     *
     * 算法原理（逐步说明）：
     *   1. 在搜索范围 [startAngle, endAngle] 内遍历每个候选角度 ang
     *   2. 对每个候选角度，根据理论相位差矩阵 phaseTheory 计算各天线对的理论相位差
     *   3. 计算理论相位差与实际相位差的余弦相关度之和：sumDiff = sum(cos(theory - actual))
     *      -- 使用余弦函数而非直接做差，天然处理了相位的 2*pi 周期性
     *   4. 找到 sumDiff 最大值对应的角度，即为 DOA 估计结果
     *   5. 基于最优角度的理论伪谱与实际伪谱计算测向质量（相关系数）
     *
     * 余弦相关度的物理意义：当理论值与实测值完全一致时，cos(0)=1，sumDiff 取最大值 N；
     * 当两者相差 180 度时，cos(pi)=-1，sumDiff 取最小值 -N。
     *
     * @param phaseTheory 理论相位矩阵，phaseTheory[ang][i] = 角度 ang 处天线 i 的理论相位（弧度）
     * @param data 实测数据，包含相位差数组、天线对序号、搜索范围等
     * @param angle 输出的测向角度（度），取余弦相关度最大处的角度值
     * @param quality 输出的测向质量（0-100），反映结果的可靠性
     * @param diff2 输出的加权伪谱数组，diff2[ang] = (sumDiff/N+1)/2 * quality
     */
    void calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2);

    /*==================幅相法测向==================*/
    /**
     * @brief 从仿真数据文件中读取指定频率的相位数据
     * @param path 仿真数据文件目录路径（末尾带路径分隔符）
     * @param f 目标频率（Hz），用于构造文件名
     * @param phase 输出的相位矩阵，phase[ang][i] = 角度 ang 处天线 i 的相位（弧度）
     * @return 0 成功，非 0 失败（文件不存在或格式错误）
     */
    int getSimulatePhase(const string path, const double f, std::vector<std::vector<double>> &phase);

    /**
     * @brief 从仿真数据文件中读取指定频率的幅度数据
     * @param path 仿真数据文件目录路径（末尾带路径分隔符）
     * @param f 目标频率（Hz），用于构造文件名
     * @param amp 输出的幅度矩阵，amp[ang][i] = 角度 ang 处天线 i 的幅度（dB）
     * @return 0 成功，非 0 失败
     */
    int getSimulateAmp(const string path, const double f, std::vector<std::vector<double>> &amp);

    /**
     * @brief 结合幅度和相位仿真数据，构建完整的阵列流型矩阵（复数形式）
     *
     * 处理流程：
     *   1. 根据频率 f 查找最近的可用的仿真数据文件（通过 existAmpPhsFile 查找）
     *   2. 读取幅度数据（dB）并转换为线性值：linear = 10^(dB/20)
     *   3. 读取相位数据（度）并转换为弧度（取反以适配坐标系约定）
     *   4. 调用 getA 将幅度和相位组合为复数阵列流型：A = amplitude * exp(j*phase)
     *
     * @param path 仿真数据文件目录路径
     * @param f 目标频率（Hz）
     * @param simulateA 输出的阵列流型矩阵，simulateA[ang][i] 表示角度 ang 处第 i 个天线的复数响应
     */
    void calAmpPhaseSimulateA(const string path, const double f, vector<vector<complex<double>>>& simulateA);

    /**
     * @brief 幅相法测向主算法（版本一：通过 InterferInfo 传入实测数据）
     *
     * 算法原理：
     *   1. 从 InterferInfo 中提取实测幅度和相位差，构造实测阵列流型向量 actual
     *   2. 对每个候选角度 ang，从仿真阵列流型中提取各天线对的比值作为该角度下的理论阵列流型：
     *      tp_A[k] = simulateA[ang][ant1-1] / simulateA[ang][ant2-1]
     *      -- 使用天线对的比值而非绝对值，消除参考相位不一致的影响
     *   3. 计算理论阵列流型与实际阵列流型的归一化内积（余弦相似度）：
     *      diff = |sum(conj(tp_A[i]) * actual[i])| / (norm(tp_A) * norm(actual))
     *   4. 取相似度最大的角度作为 DOA 估计
     *   5. 基于最优角度的理论伪谱与实际伪谱计算测向质量
     *
     * 相比于相关干涉仪法的优势：同时利用了幅度信息，对不同天线的增益差异更敏感，
     * 在非理想天线方向图场景下有更好的角度分辨能力。
     *
     * @param simulateA 仿真阵列流型矩阵（复数），simulateA[ang][i] = 天线 i 在角度 ang 的复数响应
     * @param data 实测数据，包含幅度（i_Amp）和相位差（i_Phase_Diff）
     * @param angle 输出的测向角度（度）
     * @param quality 输出的测向质量（0-100）
     * @param diff 输出的相似度曲线，diff[ang] = 角度 ang 处的归一化内积 * 100
     */
    void calAmpPhase(const vector<vector<complex<double>>> simulateA, const InterferInfo data, double &angle, double &quality, vector<double> &diff);

    /**
     * @brief 幅相法测向主算法（版本二：直接传入复数阵列流型向量）
     *
     * 与版本一的核心算法相同，但输入直接是复数阵列流型 actualA，适用于已预处理好的场景。
     * 注意：此版本中角度输出取负值（angle = -ang），与版本一的坐标系约定可能不同。
     *
     * @param simulateA 仿真阵列流型矩阵
     * @param actualA 实测阵列流型向量（复数）
     * @param angle 输出的测向角度（度），注意此处取负
     * @param quality 输出的测向质量（0-100）
     * @param diff 输出的相似度曲线
     */
    void calAmpPhase(const vector<vector<complex<double>>> simulateA, const vector<complex<double>> actualA, double& angle, double& quality, vector<double>& diff);

    /**
     * @brief 将幅度和相位组合为复数形式的阵列流型
     *
     * A[i] = amp[i] * exp(j * phase[i]) = amp[i] * (cos(phase[i]) + j*sin(phase[i]))
     *
     * @param amp 幅度数组（线性值，非 dB）
     * @param phase 相位数组（弧度）
     * @param A 输出的复数阵列流型向量，长度为天线数
     */
    void getA(const vector<double> amp, const vector<double> phase, vector<complex<double>> &A);

    /**
     * @brief 从 CSV 文件中读取模板（仿真）数据
     *
     * 文件格式：每行以角度（度）开头，后跟各天线的参数值，以逗号分隔。
     * 例如：0, -1.2, 0.5, 2.1 表示角度 0 度时，天线 1、2、3 的值分别为 -1.2, 0.5, 2.1。
     *
     * @param path 模板数据文件的完整路径
     * @param data 输出的模板数据矩阵，data[ang][i] = 角度 ang 处天线 i 的值
     * @return 0 成功，非 0 失败
     */
    int getModeData(const string path, vector<vector<double>> &data);

    /**
     * @brief 根据幅相法计算理论伪谱（用于测向质量评估）
     *
     * 对每个角度 ang，计算仿真阵列流型 simulateA[ang] 与最优匹配阵列流型 theory_A 的
     * 归一化内积（复数余弦相似度），形成伪谱曲线。该伪谱用于后续的测向质量计算。
     *
     * @param simulateA 仿真阵列流型矩阵（360 x 天线数）
     * @param theory_A 最优匹配角度对应的阵列流型向量（作为"理论值"）
     * @param diff 输出的理论伪谱，diff[ang] = 内积相似度 * 100
     */
    void calPseudoByAmpPhase(const vector<vector<complex<double>>> simulateA, vector<complex<double>> theory_A, vector<double> &diff);

    /**
     * @brief 检查指定路径和频率的仿真数据文件是否存在
     * @param path 仿真数据目录路径
     * @param f 频率（Hz）
     * @return true 存在，false 不存在
     */
    bool existAmpPhsFile(const string path, const double f);

    /*==================虚拟阵列扩展（Virtual Array）==================*/
    /**
     * @brief 根据虚拟倍数扩展虚拟阵元及其相位差
     *
     * 虚拟阵列原理：
     *   对于每一对实际天线 (ant_a, ant_b) 及其相位差 diff = phase(a) - phase(b)：
     *   - 生成虚拟天线 ant_virtual = getVirtualAntNum(ant_b, ant_a)，即编号为 b*10 + a
     *   - 虚拟相位差 = diff * virMultiple，即按倍率缩放相位差
     *   - 添加天线对 (ant_b, ant_virtual) 和对应的缩放后相位差
     *   - 同时生成对称的虚拟天线和负相位差
     *
     * 这样，原有的 N 对天线扩展后产生更多的天线对，等效于扩大了阵列孔径。
     * virMultiple < 1 时可用于缩小基线（解模糊），virMultiple > 1 时可用于扩展基线（提高精度）。
     *
     * @param virMultiple 虚拟倍率，实际相位差乘以该倍率得到虚拟相位差
     * @param antnna 输入/输出的天线对列表，输入为原始天线对，输出包含原始+虚拟天线对
     * @param phase_diff 输入/输出的相位差数组，与 antnna 同步更新
     */
    void getVirtual(const double virMultiple, vector<vector<int>> &antnna, vector<double> &phase_diff);

    /**
     * @brief 根据虚拟倍率扩展理论相位矩阵，生成虚拟阵元的理论相位
     *
     * 对每个角度，遍历所有天线对 (i, j)，生成虚拟天线编号，并计算虚拟理论相位：
     *   tp_diff = virMultiple * (theory[i] - theory[j])       // 缩放的相位差
     *   virtualTheory[ang][virtualIndex] = theory[j] - tp_diff  // 虚拟阵元的理论相位
     *
     * @param antnnaNum 实际天线数量
     * @param tp_theory 原始理论相位矩阵
     * @param virMultiple 虚拟倍率
     * @param virtualTheory 输出的虚拟理论相位矩阵，列数为 antnnaNum*10 + antnnaNum
     */
    void getVirtualTheory(const int antnnaNum, const std::vector<std::vector<double>> tp_theory, const double virMultiple, std::vector<std::vector<double>> &virtualTheory);

    /**
     * @brief 生成虚拟阵元的编号
     *
     * 编码规则：virtualAntNum = ant1 * 10 + ant2
     * 这是一种将二维天线对索引编码为一维编号的简单哈希方式，
     * 编码结果用作虚拟理论相位矩阵的列索引。
     *
     * @param ant1 第一天线编号（从 1 开始）
     * @param ant2 第二天线编号（从 1 开始）
     * @return 虚拟阵元编号
     */
    int getVirtualAntNum(const int ant1, const int ant2);

    /**
     * @brief 利用相关干涉仪 + 虚拟阵元进行二次测向，解决相位模糊（"跳半周"）问题
     *
     * 算法流程（逐步说明）：
     *   1. 预处理：将伪谱 diff 中小于 90 的值置零，滤除噪声
     *   2. 寻峰：在伪谱中搜索峰值，若只有 0 或 1 个峰值则直接返回（无模糊问题）
     *   3. 选基：从天线对中选择余弦差异最大的若干对作为"鉴别基"，这些天线对在候选角度间有最好的区分度
     *   4. 虚拟扩展：分别对两个候选峰值角度，用鉴别基的天线对进行虚拟阵列扩展
     *   5. 对比判定：计算实际相位差与两个候选角度下虚拟理论相位差的余弦相关度之和
     *      - 选择相关度更高的角度作为最终结果
     *   6. 质量评估：基于最终选定的角度重新计算测向质量
     *
     * "跳半周"问题的本质：
     *   当阵元间距 d > lambda/2 时，相位差 delta_phi = 2*pi*d*sin(theta)/lambda 的绝对值
     *   可能超过 pi，导致实际测量的相位差被截断到 [-pi, pi] 范围内（相位缠绕），
     *   造成理论值与实测值的匹配出现多个候选角度。虚拟阵列通过缩短等效基线来消除模糊。
     *
     * @param phaseTheory 理论相位矩阵
     * @param virMultiple 虚拟倍率（通常 < 1，用于缩短等效基线）
     * @param diff 第一次测向得到的伪谱数组（360 点）
     * @param data 实测数据（天线对和相位差信息）
     * @param angle 输出的最终测向角度（度）
     * @param quality 输出的测向质量（0-100）
     */
    void calSecondDoaByVirInterf(const vector<vector<double>> phaseTheory, const double virMultiple, vector<double> diff, InterferInfo data, double &angle, double &quality);

    /*==================天线选择与相位处理工具方法==================*/
    /**
     * @brief 计算定向天线的角度搜索范围
     *
     * 定向天线需要根据最大信号天线位置限制搜索范围，避免在不可能的方向上浪费时间。
     * 根据天线间差值判断信号指向，结合 perAngle = 360/antennaNum 计算角度区间。
     *
     * @param index 用于测向的天线序号列表
     * @param cutSequence 天线对序列（切刀顺序）
     * @param AntennaNum 总天线数
     * @param max_index 载噪比最大的天线对索引
     * @param startAngle 输出的搜索起始角度（度）
     * @param endAngle 输出的搜索结束角度（度）
     */
    void calAngleSerchRange(const vector<int> index, const vector<vector<int>> cutSequence, const int AntennaNum, int max_index, int &startAngle, int &endAngle);

    /**
     * @brief 根据天线序号组合生成所有可能的相位差对
     *
     * 利用传递性原理，从已知相位差推导出未知的相位差组合：
     *   - 若已知 diff(A,B) 和 diff(B,C)，则 diff(A,C) = diff(A,B) + diff(B,C)
     *   - 若已知 diff(A,B) 和 diff(A,C)，则 diff(B,C) = diff(A,C) - diff(A,B)
     *
     * 此方法迭代地扩展相位差集合，直到无法生成新的天线对为止。
     *
     * @param cutSequence 输入/输出的天线对序列
     * @param phaseDiff 输入/输出的相位差数组
     */
    void setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    // ==================== 以下为当前项目组(Spoofing)与Suppress组均未使用的函数，暂时注释保留 ====================
    // /**
    //  * @brief 从相位差数据还原各天线的绝对相位（假设第一个天线的相位为 0）
    //  *
    //  * 适用于天线对按顺序排列（相邻天线对）的场景。
    //  *
    //  * @param cutSequence 天线对序列
    //  * @param phaseDiff 相位差数组（输入为天线对间差值，输出为各天线绝对相位）
    //  * @param antennNUm 天线总数
    //  */
    // void getAntennaPhase(vector<vector<int>> &cutSequence, vector<double> &phaseDiff, int antennNUm);

    /**
     * @brief 从相位差数据还原各天线的绝对相位（指定天线顺序版本）
     *
     * 适用于任意天线对排列的场景。以 use_ant 中第一个天线为参考（相位设为 0），
     * 逐步推导其余天线的绝对相位。若某一对天线间无直接或间接相位差关系，则返回失败。
     *
     * @param use_ant 参与测向的天线序号列表
     * @param antennNUm 天线总数
     * @param cutSequence 天线对序列
     * @param phaseDiff 相位差数组（输入为天线对间差值，输出为各天线绝对相位）
     * @return 0 成功，-1 失败（天线对关系不连通）
     */
    int getAntennaPhase(const vector<int> use_ant, const int antennNUm, vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    // /**
    //  * @brief 生成所有天线对两两之间的相位差（全排列）
    //  *
    //  * 从各天线的绝对相位出发，计算所有 C(N,2) 对天线之间的相位差：
    //  *   diff[i][j] = phase[i] - phase[j]
    //  *
    //  * @param cutSequence 输出的天线对序列（全排列，共 N*(N-1)/2 对）
    //  * @param phaseDiff 输入的各天线绝对相位，输出为对应的相位差数组
    //  */
    // void getPhaseDiffAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    /**
     * @brief 生成指定天线之间所有可能的相位差组合
     *
     * 与 getPhaseDiffAll 类似，但仅针对 use_cut 中指定的天线子集。
     *
     * @param use_cut 参与的天线序号列表
     * @param cutSequence 输出的天线对序列
     * @param phaseDiff 输入的绝对相位数组，输出为指定天线对之间的相位差
     */
    void getUsePhaseDiffAll(const vector<int> use_cut, vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    /**
     * @brief 定向天线模式：根据幅度信息选择用于测向的天线并计算搜索范围
     *
     * 处理流程：
     *   1. 按幅度从大到小对天线排序，找出信号最强的天线
     *   2. 以最强天线为中心，向两侧选取相邻天线，构成测向天线子集
     *   3. 根据选中的天线位置计算定向搜索的角度范围
     *
     * 特别考虑了饱和情况：当天线存在饱和时，幅度最大的天线不一定是正对信号源的天线，
     * 因此采用逐步扩展的策略，覆盖信号源可能的方向。
     *
     * @param A 各天线的幅度数组
     * @param use_cut_num 需要选择的测向天线数
     * @param use_cut 输出的选中天线序号列表
     * @param startAngle 输出的搜索起始角度
     * @param endAngle 输出的搜索结束角度
     */
    void getUseAntennaByADirect(const vector<double> A, const int use_cut_num, vector<int> &use_cut, int &startAngle, int &endAngle);

    /**
     * @brief 读取频率-半径配置文件，获取各频段对应的阵列半径
     *
     * 文件格式：每两行为一组，第一行为频率区间（如 "100-200" 或 "100~200"，单位 MHz），
     * 第二行为对应的阵列半径（单位米）。
     *
     * @param adr 配置文件路径
     * @param mR 输出的频率-半径映射数组
     * @return 0 成功，-1 文件读取失败，-2 格式错误
     */
    int getRData(const string adr, vector<Rs> &mR);

    /*==================测向质量评估==================*/
    /**
     * @brief 计算测向质量（全局版本），基于理论伪谱与实际伪谱的相关系数
     *
     * 测向质量定义：quality = (corr + 1) / 2 * 100
     * 其中 corr = sum(diffTheory[i] * diff[i]) / (sqrt(sum(diffTheory^2)) * sqrt(sum(diff^2)))
     * 即归一化互相关系数（余弦相似度），取值范围 [-1, 1]，映射到 [0, 100]。
     *
     * 质量值越高，说明最优角度的理论伪谱与整体伪谱的一致性越好，测向结果越可靠。
     *
     * @param diffTheory 理论伪谱（最优匹配角度下的理论值构成的伪谱）
     * @param diff 实际伪谱（实测数据与各角度理论值匹配得到的伪谱）
     * @return 测向质量值（0-100）
     */
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff);

    /**
     * @brief 计算测向质量（区域版本），仅在指定角度范围内计算相关系数
     *
     * 与全局版本逻辑相同，但限定在 [starAngle, endAngle] 范围内计算。
     * 适用于定向天线场景，仅关注信号可能出现的角度区间。
     *
     * @param diffTheory 理论伪谱
     * @param diff 实际伪谱
     * @param starAngle 搜索起始角度（度，闭区间）
     * @param endAngle 搜索结束角度（度，闭区间）
     * @return 测向质量值（0-100）
     */
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff, int starAngle, int endAngle);

};


// =============================================================================
// 宿主/接口共享结构 (原 GN902.h, 默认对齐, 保持与 DLL 边界 ABI 一致)
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
// 引擎中间结构 (原 SpoofingDoa.h, 参与通道间数据/单星欺骗结果) 与主类 SpoofingDoa
// =============================================================================
#pragma pack(1)

// =============================================================================
// 数据结构定义
// =============================================================================

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
// 注: AlarmData / SatelliteAngle / SpoofingResult / GNSSData 由 GN902.h 定义(见文件头)。
// -----------------------------------------------------------------------------

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
    // @param sys: 卫星系统(-1表示所有系统)
    // @param type: 卫星频点(-1表示所有频点)
    // @param threshold: 卫星颗数阈值，达到该数量则判定为欺骗
    // @param phsThreshold: 相位差阈值(单位:度)，相位差在该范围内的卫星判定为同源
    // 参数顺序: (系统sys, 频点type, 卫星颗数阈值threshold, 相位差阈值phsThreshold)
    void setThresholdDetectionDoa(int sys, int type, int threshold, double phsThreshold);

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

    // 设备 wrapper(GN902 等)在 Init 之后覆盖循环切刀运行参数(供外部驱动整轮流式输入)
    // @param cyclic      是否启用循环切刀检测(对应当前 Python 循环切刀流程)
    // @param oneCutFrams 每刀帧数; >0 时覆盖 m_OneCut_Frams
    // @param smooth      是否启用刀内多帧平滑(稳定性过滤 + 圆形均值)
    // @param omniR       全向阵列半径(m); >0 时按该半径重建 m_R 与测向理论模板
    //                    (GN902 = 0.1865，与 Python ARRAY_RADIUS 一致; <=0 表示保持当前)
    void configCyclicRuntime(bool cyclic, int oneCutFrams, bool smooth, double omniR);

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
    void paraProjectGN902();
    void paraProjectGN930();

    // ---- Corrected.cpp: 校正数据处理 ----
    // 设置校正数据:从同天线自校准刀({1,1}=code=0)提取校正信息并计算偏移
    void setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA);
    // 计算校正偏移(对应 Python compute_calibration: 非 GLONASS 按频点圆形均值, GLONASS 逐卫星)
    void calCorrectionOffset(const vector<vector<SatelliteDataPhaseDiffA>> &calCuts);
    // 对所有输入数据进行相位校正(减去校正值消除通道误差)
    void getCorrectedGnssData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    // 对单个卫星数据进行相位差校正(对应 Python offset_fn)
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
    // 根据天线对选择构建InterferInfo结构(排除无效数据，应用虚拟阵列)
    void calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg);
    // 多帧数据平滑(排除异常值后取均值)
    void getSmoothData(vector<vector<SatelliteDataPhaseDiffA>> &dataA);
    // 对单颗卫星多帧相位差进行稳定性过滤 + 跳半周处理 + 圆形均值
    // （对齐 Python check_stability：不要求信号从该刀第一帧就存在，只要求有效采样覆盖
    //   min(刀内帧数, MIN_STABLE_SAMPLES) 帧，见 detection_lib.compute_calibration/build_vectors_and_detect）
    void calSmoothData(SatelliteDataPhaseDiffB dataB, SatelliteDataPhaseDiffA &dataA);
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

    // ---- 角度工具（静态，对应 Python detection_lib 的角度函数）----
    static double normalizeAngle180(double deg);                  // 归一化到 [-180°,180°)
    static double circularMeanDeg(const std::vector<double> &degs); // 圆形均值(度)
    static double circularSpanDeg(const std::vector<double> &degs); // 最小覆盖弧跨度(度)
    static double foldHalfCycle(double deg);                       // 折叠到 [0°,180°)
    static double circularSpan180Deg(const std::vector<double> &degs); // 半周圆上的跨度(度)

    // ---- 循环切刀检测流程（对应 Python detection_main.py / detection_lib.py 循环切刀主流程）----
    void resetCyclicDetection(void);   // 清空连续报警计数与跟踪状态
    // 每刀相位差聚类检测 + 跨刀连续确认 + 测向持续跟踪（只保留确认欺骗的卫星用于测向）
    void getCyclicDetectionData(std::vector<std::vector<SatelliteDataPhaseDiffA>> &dataA);
    // 把本轮 dataB 合并进跨周期基线(相位差跨轮复用做测向，对应 Python baselines)
    void accumulateBaselines(const std::vector<SatelliteDataPhaseDiffB> &dataB);
    // 取出当前跟踪中(系统,频点)的跨周期累积基线，作为测向输入
    void getCrossCycleDataB(std::vector<SatelliteDataPhaseDiffB> &doaDataB);

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
    // 循环切刀欺骗检测状态（对应 Python detection_lib.py）
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

    // 跨周期测向基线累积：typeInt -> prn -> 累积相位差(每刀位保留最新有效值，跨轮复用做测向)。
    // 对应 Python detection_main 的 baselines：相位差可跨周期复用，前提是校正偏移相邻轮稳定。
    std::map<int, std::map<int, SatelliteDataPhaseDiffB>> m_Baselines;

protected:
    // =========================================================================
    // 配置与控制参数
    // =========================================================================

    // m_Detection_Recodds_Num: 跨刀连续确认刀数
    // 循环切刀检测中连续多少刀检测为欺骗信号才最终判定
    // (getCyclicDetectionData 中 m_ConsecutiveAlarm 累计阈值)，默认为1(每刀独立判定)
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
    // 判定为严格大于(bestCount > 阈值，触发≥阈值+1)，对应 Python 默认 countThreshold=2(触发≥3)
    int m_Detection_Threshold_Num = 2;

    // m_PseudoSpectrum_Flag: 是否进行伪谱积分
    // 0=不积分, 1=积分(多帧累积提高测向稳定性)
    int m_PseudoSpectrum_Flag = 0;

    // m_Doa_Detection_Flag: 测向数据中的欺骗检测模式
    // 0=不检测欺骗，对所有数据进行测向
    // 1=对每刀数据独立检测，不合并
    // 2=对每刀数据检测并合并，仅对检测为欺骗的信号进行测向
    int m_Doa_Detection_Flag = 0;

    // m_Cyclic_Detection_Flag: 是否使用循环切刀检测流程（对应 detection_main.py / detection_lib.py）
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


// =============================================================================
class GN902
{
public:
    GN902();
    ~GN902();
    void SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum);
    void SetData(const GNSSData* data, int cutIdx, int endFlag);
    void GetResult(SpoofingResult& result);
private:
    void Detect();
    void Doa();
};
