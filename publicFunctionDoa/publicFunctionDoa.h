/**
 * @file publicFunctionDoa.h
 * @brief 测向(Direction of Arrival, DOA)算法公共函数库头文件
 *
 * 本文件声明了测向系统中各模块共用的工具函数、模板函数和全局变量。
 * 主要功能模块包括：
 *   1. 数学工具：多项式拟合、峰值检测、向量范数计算、线性插值、角度规整
 *   2. 日志系统：支持按对象ID分文件记录运行日志，可控制日志开关和写入模式
 *   3. 配置文件解析：读取键值对格式的配置文件并进行类型转换
 *   4. 字符串与容器工具：字符串分割、格式转换、二维向量维度调整
 *   5. 跨平台路径获取：兼容Windows和Linux的可执行文件路径获取
 *   6. 数据保存：支持将数组以二进制格式追加写入文件，受全局开关控制
 *
 * 依赖：Eigen库(线性代数)、C++标准库、Windows平台需链接Shlwapi.lib
 *
 * @note 所有函数声明在 PublicSpace 命名空间下
 */
#pragma once
#include <iomanip>
#include <iostream>
#include <complex>
#include <fstream>
#include <sstream>
#include <vector>
#include <assert.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <cmath>
#include <ctime>
#include <time.h>
#include <algorithm>
#include <map>
#include <string>
#include <mutex>
#include <stdarg.h>
#include <chrono>
#include <Eigen/Dense>

#ifdef _WIN32
#include <windows.h>
#include <shlwapi.h> // 需要链接到 Shlwapi.lib
#else
#include <dlfcn.h>
#include <libgen.h>
#include <cstring>
#endif
#define m_PI 3.14159265358979323846
#define m_C 3e8
using namespace std;
using namespace Eigen;

namespace PublicSpace
{
    /**
     * @brief 当前可执行文件所在的目录路径
     * @note 默认值为 "."，由 getCurrentExecutablePath() 在程序启动时赋值
     *       用于拼接配置文件、日志文件等相对路径的基准目录
     */
    extern string m_CurrentPath;

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

    /**
     * @brief 获取当前可执行文件所在的目录路径（跨平台）
     * @return 返回可执行文件所在目录的绝对路径字符串，失败时返回 "."
     * @note Windows下使用GetModuleHandle/GetModuleFileName获取，
     *       Linux下使用dladdr/dirname获取
     */
    std::string getCurrentExecutablePath(void); // 获得当前执行程序的路径

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

    /**
     * @brief 多项式最小二乘拟合（BdSVD求解）
     * @param x 自变量数据点构成的Eigen列向量
     * @param y 因变量数据点构成的Eigen列向量
     * @param n 拟合多项式的最高次数
     * @param coeffs 输出参数，拟合得到的多项式系数（从常数项到n次项），长度为n+1
     * @note 构造Vandermonde矩阵后使用BDCSVD分解求解，数值稳定性好
     */
    void polyfit(const VectorXd &x, const VectorXd &y, const int n, VectorXd &coeffs); // 曲线拟合

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

    /**
     * @brief 将二维复数vector调整为指定维度，并将所有元素初始化为 (1.0 + 0.0i)
     * @param data 输入/输出参数，要调整大小的二维复数vector
     * @param cols 目标列数（第一维大小）
     * @param rows 目标行数（第二维大小）
     * @note 先清空原数据，再resize并填充默认值(1.0, 0.0)
     */
    void VectorResizeToOne(std::vector<std::vector<std::complex<double>>> &data, int cols, int rows); // 复数vector初始化

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