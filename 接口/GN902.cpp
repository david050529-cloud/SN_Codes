// =============================================================================
// 文件名: GN902.cpp
// 功能描述: GN902 欺骗检测/测向模块 —— 自包含实现文件(消除编译隔离)
//
// 本文件收纳了以下原独立源文件的全部函数实现(逐字保留, 便于跨函数内联优化):
//   核心引擎   SpoofingDoa.cpp      InitData.cpp     PreparationData.cpp
//              Corrected.cpp        Interf.cpp       AmpPhase.cpp
//              Omni.cpp             Directed.cpp     Alarm.cpp
//              SpectrumDesity.cpp   WriteLog.cpp
//   算法基类   arithmetic.cpp       (ArithmeticDoa 相关干涉仪/幅相法/虚拟阵列)
//   工具库     publicFunctionDoa.cpp (PublicSpace 日志/峰值/字符串/角度等工具函数)
//   接口实现   GN902.cpp (GN902 类: 实例状态机 + 整轮帧缓冲 + 检测/测向调度)
//
// 所有类型与类声明见配套头文件 GN902.h; 无外部依赖(原 Arithmetic / publicFunctionDoa
// 静态库均已内联, Eigen/shlwapi 依赖已移除)。
// =============================================================================
#include "GN902.h"

// =============================================================================
// == 内联自 publicFunctionDoa.cpp —— PublicSpace 工具函数实现 ==================
// =============================================================================
﻿/**
 * @file publicFunctionDoa.cpp
 * @brief 测向(Direction of Arrival, DOA)算法公共函数库实现文件
 *
 * 本文件实现了测向系统各模块共用的工具函数，详细功能如下：
 *   1. 数据操作：峰值检测(findPeaks)、字符串分割(split)、空格修剪(trim)、文件读取(readFile)
 *   2. 数学工具：多项式最小二乘拟合(polyfit，使用BDCSVD)、复数L2范数(getNorm)、
 *      线性插值(linearInterpolation)、角度规整(Round360/Round3600)
 *   3. 日志系统：支持按ID管理多个日志文件，全局开关控制，线程安全的格式化日志输出
 *      - getLogCont(id) 设置日志ID -> LogCreat(path) 创建文件 -> Log(...) 写入 -> LogClose() 关闭
 *   4. 配置解析：readConfigtxt 读取 "key=value" 格式配置，支持 '#' 行内注释
 *   5. 时间工具：getNowTime() 获取精确到毫秒的系统时间字符串
 *   6. 向量工具：VectorResizeToOne 二维复数vector维度调整与初始化
 *   7. 跨平台：getCurrentExecutablePath Windows/Linux获取可执行文件所在目录
 *
 * 依赖：Eigen库、C++标准库、Windows平台需 Shlwapi.lib (PathRemoveFileSpec)
 */
using namespace std;
namespace PublicSpace
{
    // ==================== 命名空间全局变量定义 ====================

    // ==================== 以下为当前项目组(Spoofing)与Suppress组均未使用的函数，暂时注释保留 ====================
    // /**
    //  * @brief 当前可执行文件所在目录路径，默认为当前目录 "."
    //  * @note 程序启动时通过 getCurrentExecutablePath() 获取实际路径并更新
    //  */
    // string m_CurrentPath = ".";
    // // extern string m_CurrentPath;

    /**
     * @brief 日志文件句柄映射表
     * @note key为日志编号字符串(m_LogCount)，value为对应的FILE*文件指针
     *       每个测向算法对象(通过ID区分)维护一个独立的日志文件句柄
     */
    map<string, FILE *> m_Log_fp;

    /**
     * @brief 当前活动的日志编号
     * @note 由 getLogCont(int id) 设置，Log/LoCreat/LogClose 都基于此值定位对应的日志文件
     *       空字符串表示尚未设置日志编号
     */
    string m_LogCount = "";

    /**
     * @brief 日志记录控制标志
     * @note 0 - 不记录日志（默认值，所有日志函数直接返回）
     *       1 - 记录日志，以覆盖模式("w")打开文件，每次运行清空旧日志
     *       2 - 记录日志，以追加模式("a")打开文件，旧日志保留
     */
    int m_logFlg = 0; // 是否打印日志，0 不打印日志；1 打印日志

    /**
     * @brief 原始数据保存控制标志
     * @note 1 - 保存原始数据; 其他值(默认0) - 不保存
     *       saveArrayToBinary 模板函数检查此标志
     */
    int m_save_data_Flg = 0;

    /**
     * @brief 日志文件写入互斥锁，保证多线程环境下日志写入的原子性
     */
    std::mutex m_fileMutex;

    /**
     * @brief 查找一维数据序列中的峰值（局部极大值），按峰值大小降序排列输出
     *
     * 算法流程：
     *   1. 遍历内部元素(索引1到size-2)，如果某个元素大于左右邻居，则标记为峰值
     *   2. 检查边界情况：首元素需同时大于第二个和最后一个元素；末元素需同时大于倒数第二个和首个元素
     *   3. 将找到的峰值按数值大小降序排序后输出
     *
     * @param data 输入的一维数据序列
     * @param index2 输出参数，峰值在原数据中的索引位置，按峰值大小降序排列
     * @param vaules2 输出参数，与index2对应的峰值数值，按降序排列
     */
    void findPeaks(const std::vector<double> data, vector<int> &index2, vector<double> &vaules2)
    {
        if (data.empty())
            return;
        int tp = -1;
        vector<int> tp_index;
        vector<int> index;
        vector<double> vaules;
        for (size_t i = 1; i < data.size() - 1; ++i)
        {
            if (data[i] > data[i - 1] && data[i] > data[i + 1])
            {
                // 当前元素比它的左右邻居都大，是一个峰值
                index.emplace_back(i);
                vaules.emplace_back(data[i]);
                tp = tp + 1;
                tp_index.emplace_back(tp);
            }
        }

        // 处理边界情况（如果第一个或最后一个元素是峰值）
        if (data.size() > 1 && (data[0] > data[1]) && (data[0] > data[data.size() - 1]))
        {
            index.emplace_back(0);
            vaules.emplace_back(data[0]);
            tp = tp + 1;
            tp_index.emplace_back(tp);
        }
        if (data.size() > 1 && (data[data.size() - 1] > data[data.size() - 2]) && (data[data.size() - 1] > data[0]))
        {
            index.emplace_back(data.size() - 1);
            vaules.emplace_back(data[data.size() - 1]);
            tp = tp + 1;
            tp_index.emplace_back(tp);
        }
        std::sort(tp_index.begin(), tp_index.end(), [&](int i, int j)
                  { return vaules[i] > vaules[j]; });

        index2.clear();
        vaules2.clear();
        int num = (int)tp_index.size();
        index2.resize(num);
        vaules2.resize(num);
        for (int i = 0; i < num; i++)
        {
            index2[i] = index[tp_index[i]];
            vaules2[i] = vaules[tp_index[i]];
        }
    }

    // /**
    //  * @brief 多项式最小二乘拟合
    //  *
    //  * 算法流程：
    //  *   1. 构造 Vandermonde 矩阵 A: A(i,j) = x(i)^j (j从0到n)
    //  *   2. 使用 BDCSVD (Bidiagonal Divide and Conquer SVD) 分解求解最小二乘问题 A * coeffs = y
    //  *   3. BDCSVD 方法在大规模数据下具有较好的数值稳定性和计算效率
    //  *
    //  * @param x 自变量数据点(Eigen列向量)
    //  * @param y 因变量数据点(Eigen列向量)
    //  * @param n 拟合多项式最高次数
    //  * @param coeffs 输出参数，多项式系数[常数项, 一次项, ..., n次项]，长度为n+1
    //  */
    // void polyfit(const VectorXd &x, const VectorXd &y, const int n, VectorXd &coeffs)
    // {
    //     int numData = x.size();
    //     MatrixXd A(numData, n + 1);
    //
    //     // 构造Vandermonde矩阵
    //     for (int i = 0; i < numData; ++i)
    //     {
    //         A(i, 0) = 1;
    //         for (int j = 1; j <= n; ++j)
    //         {
    //             A(i, j) = A(i, j - 1) * x(i);
    //         }
    //     }
    //     // 使用最小二乘法求解
    //     coeffs = A.bdcSvd(ComputeThinU | ComputeThinV).solve(y);
    // }

    /**
     * @brief 计算复数向量的L2范数（欧几里得范数）
     *
     * 公式: sqrt(sum(|data[i]|^2))
     * 其中 |z| = sqrt(real(z)^2 + imag(z)^2)
     *
     * @param data 输入的复数向量
     * @return 返回L2范数值
     */
    double getNorm(const vector<complex<double>> data)
    {
        double sum_of_squares = 0.0;
        for (int i = 0; i < (int)data.size(); i++)
        {
            sum_of_squares += std::norm(data[i]);
        }
        return std::sqrt(sum_of_squares);
    }

    /**
     * @brief 线性插值：已知两点(x1,y1)和(x2,y2)，求x处的线性插值
     *
     * 公式: y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
     *
     * @param x1, y1 第一个已知点的坐标
     * @param x2, y2 第二个已知点的坐标
     * @param x 待插值点的横坐标
     * @return x处的插值结果
     * @throws std::invalid_argument 当 x1 和 x2 过于接近时(|x1-x2| < 1e-10)抛出异常
     */
    double linearInterpolation(double x1, double y1, double x2, double y2, double x) // 线性插值
    {
        if (fabs(x1 - x2) < 1e-10)
        {
            throw std::invalid_argument("x1 and x2 cannot be the same.");
        }
        return y1 + (x - x1) * ((y2 - y1) / (x2 - x1));
    }

    /**
     * @brief 按指定分隔符将字符串分割为子串数组
     *
     * 使用 stringstream + getline 按分隔符逐段读取，每段经 trim() 去除首尾空格后，
     * 非空字符串才加入结果数组
     *
     * @param result 输出参数，存放分割后的子字符串数组
     * @param str 输入的待分割字符串
     * @param str1 分隔字符
     */
    void split(vector<string> &result, string str, char str1) // 字符串分割
    {
        stringstream ss(str);
        string word;

        while (getline(ss, word, str1))
        {
            trim(word);
            if (!word.empty())
            {
                result.push_back(word);
            }
        }
    }

    /**
     * @brief 原地去除字符串首尾的空白字符（空格、制表符、换行符等）
     *
     * 使用 erase + find_if 分别从头部和尾部删除空白字符：
     *   - 头部：找到第一个非空白字符，删除之前的所有字符
     *   - 尾部：从反向找到第一个非空白字符，删除之后的所有字符
     *
     * @param str 输入/输出参数，原地修改的字符串
     */
    void trim(string &str)
    {
        str.erase(str.begin(), find_if(str.begin(), str.end(), [](unsigned char ch)
                                       { return !isspace(ch); }));

        str.erase(find_if(str.rbegin(), str.rend(), [](unsigned char ch)
                          { return !isspace(ch); })
                      .base(),
                  str.end());
    }


    /**
     * @brief 读取文本文件，将内容按空白字符分割后存入字符串数组
     *
     * 逐行读取文件，每行再按空白字符(空格/制表符等)分割单词，依次存入data中
     *
     * @param adr 文件路径（相对或绝对路径）
     * @param data 输出参数，存放读取到的所有单词
     * @return 0 - 正常读取完成; -1 - 文件路径错误或文件不存在
     */
    int readFile(const string adr, vector<string> &data){
        string path = "";
        path.append(adr);
        data.clear();
        ifstream in(path);
        if (!in.is_open()){
            cout << "*error:adr not found !!!" << adr << endl;
            return -1;
        }

        string line;
        string tp = "";

        while (getline(in, line)){
            istringstream strm(line);
            while (strm >> tp){
                data.emplace_back(tp);
            }
        }
        in.close();
        return 0;
    }



    /**
     * @brief 将浮点角度值规整到 [0, 360) 度范围内
     *
     * 算法：先将角度乘以10转为十分之一度单位，用 fmod 求模后处理负值，
     *       再除以10还原为度。这样可以避免浮点直接求模的精度问题。
     *
     * @param x 输入的角度值（度）
     * @return 规整后的角度值（度），范围 [0.0, 360.0)
     */
    double Round3600(double x)
    {
        x = std::fmod(x * 10, 3600); // 求模
        if (x < 0)
        {
            x += 3600;
        }
        return x / 10.0;
    }
    /**
     * @brief 将整数角度值规整到 [0, 360) 度范围内
     *
     * 算法：使用取模运算(x % 360)，若结果为负则加上360使其落入正数范围
     *
     * @param x 输入的整数角度值（度）
     * @return 规整后的角度值（度），范围 [0, 360)
     */
    int Round360(int x)
    {

        x = x % 360;
        if (x < 0)
        {
            x += 360;
        }
        return x;
    }
    
    
    /**
     * @brief 读取键值对格式的配置文件
     *
     * 文件格式说明：
     *   每行格式为 "key = value #注释"
     *   以 '=' 分隔键和值，以 '#' 标识行内注释（'#'之后的内容被忽略）
     *   键和值会经 trim() 去除首尾空格
     *
     * @param adr 配置文件路径
     * @param configMap 输出参数，存放键值对映射表
     * @return 0 - 读取成功; -1 - 文件无法打开
     */
    int readConfigtxt(const string adr, map<string, string> &configMap){
        // map<string, string> configMap;
        string path = adr;
        // m_CurrentPath;
        // path.append(adr);
        ifstream configFile(path);
        if (!configFile.is_open()){
            cout << "**error:configtxt not have!!!" << path << endl;
            return -1;
        }

        string line;
        while (getline(configFile, line)){
            istringstream iss(line);
            string key, value;
            if (getline(iss, key, '=') && getline(iss, value, '#'))
            {
                trim(key);
                trim(value);
                configMap[key] = value;
            }
        }
        configFile.close();
        return 0;
    }


    /**
     * @brief 获取当前系统时间（精确到毫秒）
     *
     * 实现细节：
     *   1. 使用 std::chrono::system_clock 获取高精度时间点
     *   2. 转换为本地时间结构后格式化为 "YYYY-MM-DD HH:MM:SS"
     *   3. 从 time_point 中提取毫秒部分，拼接为 ".mmm" 后缀
     *
     * @return 格式为 "YYYY-MM-DD HH:MM:SS.mmm" 的时间字符串
     *         当 m_logFlg == 0 时，为节省性能直接返回空字符串
     */
    string getNowTime() {

        if (m_logFlg != 0){
            // 使用 chrono 获取当前时间点
            auto now = std::chrono::system_clock::now();

            // 将时间点转换为 time_t 类型
            std::time_t current_time = std::chrono::system_clock::to_time_t(now);

            // 转换为本地时间结构
            std::tm *local_time = std::localtime(&current_time);

            // 获取毫秒部分
            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto epoch = now_ms.time_since_epoch();
            auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
            long long milliseconds = value.count() % 1000;

            // 格式化输出（包含毫秒）
            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", local_time);

            // 组合包含毫秒的完整时间字符串
            std::stringstream ss;
            ss << buffer << "." << std::setfill('0') << std::setw(3) << milliseconds;
            return ss.str();
        }
        else{
            return "";
        }
    }


    /**
     * @brief 设置当前活动的日志编号ID
     *
     * 将整数id转为字符串存入 m_LogCount，后续 LogCreat/Log/LogClose 都基于此ID操作。
     * 日志文件的实际名称格式为: path + m_LogCount + ".log"
     *
     * 典型使用场景：每个测向算法对象拥有唯一ID，通过 getLogCont 切换到对应的日志文件
     *
     * @param id 日志索引编号，一般对应测向算法对象的唯一标识ID
     */
    void getLogCont(int id){
        m_LogCount = "";
        m_LogCount.append(to_string(id));
        return;
    }


    /**
     * @brief 创建并打开日志文件
     *
     * 行为说明：
     *   仅在 m_logFlg != 0 时执行
     *   - m_logFlg == 1: 先以覆盖模式("w")打开文件写入一行 "Log File create" 并关闭，
     *                    再以追加模式("a")重新打开，句柄存入 m_Log_fp 映射表
     *   - m_logFlg == 2: 直接以追加模式("a")打开，保留旧日志内容
     *
     *   文件名格式: path + m_LogCount + ".log"
     *   文件句柄以 m_LogCount 为键存入 m_Log_fp 映射表
     *
     * @param path 日志文件存放目录路径（不含文件名）
     */
    void LogCreat(const string path) {
        if (0 != m_logFlg){
            string logstr = ".log";
            string tp_path = "";
            tp_path.append(path);
            tp_path.append(m_LogCount);
            tp_path.append(logstr);

            FILE *fp; 
            
            // 清零
            if (1 == m_logFlg) {
                fp = fopen(tp_path.c_str(), "w");
            }
            // 追加
            if (2 == m_logFlg) {
                fp = fopen(tp_path.c_str(), "a"); 
            }

            if (fp == nullptr){
                return;
            }

            fprintf(fp, "Log File create\n");
            fclose(fp);

            FILE *m_fp = fopen(tp_path.c_str(), "a");
            m_Log_fp[m_LogCount] = m_fp;
            if (fp == nullptr){
                return;
            }

        }
    }
    
    /**
     * @brief 向当前日志文件写入格式化内容（类似 printf 的可变参数接口）
     *
     * 线程安全说明：
     *   使用 std::lock_guard<std::mutex> 加锁保护，确保多线程写入时不会交错
     *   每次写入后调用 fflush 确保日志内容立即刷新到磁盘
     *
     * 前置条件：需要先调用 getLogCont(id) 和 LogCreat(path) 初始化日志文件
     *
     * @param format 格式化字符串（与 printf 格式兼容），后续参数为可变参数
     * @note 仅在 m_logFlg != 0 时执行，否则直接返回
     */
    void Log(const char *format, ...){
        std::lock_guard<std::mutex> lock(m_fileMutex);

        if (0 != m_logFlg){
            FILE *m_fp = m_Log_fp[m_LogCount];
            if (nullptr == m_fp){
                // 处理错误，如返回、记录日志或抛出异常
                cout << "Log is null:" << m_LogCount << endl;
                return;
            }
            va_list args;
            va_start(args, format);
            vfprintf(m_fp, format, args);
            va_end(args);
            fflush(m_fp);
        }

    }

    /**
     * @brief 关闭当前日志文件并释放资源
     *
     * 行为：
     *   1. 加互斥锁保护
     *   2. 关闭 m_LogCount 对应的文件句柄
     *   3. 从 m_Log_fp 映射表中移除该条目
     *
     * @note 如果 m_LogCount 对应的文件句柄为空(NULL)，则跳过关闭操作
     */
    void LogClose(void)
    {
        std::lock_guard<std::mutex> lock(m_fileMutex);

        FILE *m_fp = m_Log_fp[m_LogCount];
        if (NULL != m_fp)
        {
            fclose(m_fp);
        }
        m_Log_fp.erase(m_LogCount);
        // FILE *m_fp;
        //  for (auto it = m_Log_fp.begin(); it != m_Log_fp.end(); ++it)
        //  {
        //      m_fp = it->second;
        //      if (NULL != m_fp)
        //      {
        //          fclose(m_fp);
        //      }
        //  }
    }
    // /**
    //  * @brief 将二维复数vector调整为指定维度(cols x rows)，所有元素初始化为 (1.0 + 0.0i)
    //  *
    //  * @param data 输入/输出参数，要调整的二维复数vector，调用后旧数据被清空
    //  * @param cols 目标列数（外层vector的大小）
    //  * @param rows 目标行数（内层vector的大小）
    //  */
    // void VectorResizeToOne(std::vector<std::vector<std::complex<double>>> &data, int cols, int rows)
    // {
    //     // 清空容器
    //     data.clear();
    //
    //     data.resize(cols, std::vector<std::complex<double>>(rows, std::complex<double>(1.0, 0.0)));
    // }
    // /**
    //  * @brief 获取当前可执行文件所在的目录路径（跨平台实现）
    //  *
    //  * Windows: 使用 GetModuleHandle(NULL) + GetModuleFileName + PathRemoveFileSpec
    //  *          需要链接 Shlwapi.lib (PathRemoveFileSpec函数)
    //  * Linux:   使用 dladdr + dirname 获取动态库加载地址并提取目录路径
    //  *
    //  * @return 可执行文件所在目录的绝对路径，失败时返回 "."
    //  */
    // std::string getCurrentExecutablePath(void)
    // { // 根据不同的操作系统采用不同的方法来获取当前可执行文件的目录路径
    // #ifdef _WIN32
    //     char path[MAX_PATH];
    //     HMODULE hModule = GetModuleHandle(NULL);
    //     if (hModule != NULL)
    //     {
    //         GetModuleFileName(hModule,path, sizeof(path));
    //         PathRemoveFileSpec(path); // 移除文件名，只保留目录
    //         return path;
    //     }
    // #else
    //     Dl_info dl_info;
    //     if (dladdr(reinterpret_cast<void *>(getCurrentExecutablePath), &dl_info))
    //     {
    //         char *dir_path = strdup(dl_info.dli_fname);
    //         std::string directory = dirname(dir_path);
    //         free(dir_path); // 释放分配的内存
    //         dir_path = NULL;
    //         return directory;
    //     }
    // #endif
    //     return ".";
    // }
   
}

// =============================================================================
// == 内联自 arithmetic.cpp —— ArithmeticDoa 核心测向算法实现 ===================
// =============================================================================
﻿/**
 * @file arithmetic.cpp
 * @brief 测向算法核心实现 -- 相关干涉仪、幅相法、虚拟阵列扩展等
 *
 * 本文件实现了 ArithmeticDoa 类中声明的所有测向算法方法，包括：
 *
 * ---- 相关干涉仪测向 (Correlative Interferometer DOA) ----
 *   核心思想：将实测相位差向量与每个候选角度的理论相位差向量做相关匹配，
 *   相关度最高的角度即为来波方向。使用余弦函数作为相关度量，天然处理相位周期性。
 *   相关公式：correlation(ang) = sum_i cos( theory_phase_diff_i(ang) - actual_phase_diff_i )
 *   优点：计算简单、速度快、对噪声有一定鲁棒性。
 *   局限：仅使用相位信息，在阵元间距大于半波长时可能出现相位模糊（跳半周）。
 *
 * ---- 幅相法测向 (Amplitude-Phase DOA) ----
 *   核心思想：同时利用幅度和相位信息，通过复数阵列流型的内积匹配进行测向。
 *   阵列流型匹配公式：similarity(ang) = |sum_i conj(simulate_A[ang][i]) * actual_A[i]| / (norm(simulate_A) * norm(actual))
 *   即归一化复数内积的模，取值范围 [0, 1]，1 表示完全匹配。
 *   优点：同时利用幅度信息、对不同天线增益差异敏感、适合非理想天线方向图。
 *   局限：依赖仿真/校准数据、对天线方向图精度要求较高。
 *
 * ---- 虚拟阵列扩展 (Virtual Array Expansion) ----
 *   核心思想：将实测的相位差按虚拟倍率 virMultiple 缩放，生成等效的新"虚拟阵元"和对应的相位差，
 *   从而在不增加物理天线的条件下改变等效基线长度。
 *   - virMultiple < 1：缩短等效基线，用于消除相位模糊（解决"跳半周"问题）
 *   - virMultiple > 1：增加等效孔径，用于提高角度分辨率
 *
 * ---- 测向质量 (DOA Quality) ----
 *   基于归一化互相关系数计算：quality = (corr + 1) / 2 * 100
 *   corr = sum(diffTheory[i] * diff[i]) / (sqrt(sum(diffTheory^2)) * sqrt(sum(diff^2)))
 *   quality 的取值范围为 [0, 100]，越高表示测向结果越可靠。
 *
 * ---- 关键物理常数（定义于 publicFunctionDoa.h） ----
 *   m_PI = 3.14159265358979323846  -- 圆周率
 *   m_C  = 3e8                     -- 光速 (m/s)，用于相位-距离转换
 *
 * ---- 坐标系约定 ----
 *   - 角度以度为单位，范围 [0, 360)，0 度 = 正北/正前方（取决于阵列安装方向）
 *   - 相位以弧度为单位，范围通常为 [-pi, pi]（实际测量）或任意值（理论计算）
 *   - 天线从 1 开始编号，内部计算时转换为 0-based 索引
 */

using namespace std;
// using namespace PublicSpace;
ArithmeticDoa::ArithmeticDoa()
{
    // m_CurrentPath = getCurrentExecutablePath();
}

ArithmeticDoa::~ArithmeticDoa()
{
}

/**
 * @brief 虚拟阵列扩展 -- 根据虚拟倍率生成虚拟阵元及对应的相位差
 *
 * 算法原理（逐步说明）：
 *   对于每一对原始天线 (A, B) 及其相位差 diff_AB = phase(A) - phase(B)：
 *
 *   步骤 1 -- 正向虚拟扩展：
 *     - 生成虚拟阵元编号：virtualAnt = getVirtualAntNum(B, A) = B*10 + A
 *       含义：以天线 B 为基准，向 A 的方向外推一个虚拟阵元
 *     - 虚拟相位差：virtualDiff = diff_AB * virMultiple
 *       含义：将原始 AB 之间的相位差按倍率缩放，相当于改变了等效基线长度
 *     - 添加天线对 (B, virtualAnt)，相位差为 virtualDiff
 *       含义：虚拟阵元的相位 = 天线 B 的相位 + virtualDiff
 *
 *   步骤 2 -- 反向虚拟扩展：
 *     - 生成对称的虚拟阵元编号：virtualAnt = getVirtualAntNum(A, B) = A*10 + B
 *     - 虚拟相位差：virtualDiff = -diff_AB * virMultiple
 *     - 添加天线对 (A, virtualAnt)，相位差为 -virtualDiff
 *       含义：另一个方向的虚拟扩展，保证对称性
 *
 * 虚拟阵列的物理意义：
 *   如果 virMultiple = 0.5，等效于将基线长度缩小一半，可用于消除大孔径阵列的相位模糊。
 *   如果 virMultiple = 2.0，等效于将基线长度扩大一倍，可提高角度分辨率。
 *   每个原始天线对产生 2 个虚拟天线对，总天线对数从 N 变为 3N。
 *
 * @param virMultiple 虚拟倍率，乘以原始相位差得到虚拟相位差
 * @param antnna 天线对列表 [输入] 原始天线对 / [输出] 原始+虚拟天线对
 * @param phase_diff 相位差数组 [输入] 原始相位差 / [输出] 原始+虚拟相位差
 */
void ArithmeticDoa::getVirtual(const double virMultiple, vector<vector<int>> &antnna, vector<double> &phase_diff)
{
    // 保存原始天线对和相位差的副本，因为后续 emplace_back 会修改容器
    vector<vector<int>> tp_antnna;
    tp_antnna = antnna;
    vector<double> tp_diff;
    tp_diff = phase_diff;
    int count = tp_antnna.size(); // 原始天线对数量，循环只遍历原始数据
    int virtualAnt = 0;
    double virtualDiff = 0.0;
    vector<int> tp;
    tp.resize(2);
    for (int i = 0; i < count; i++)
    {
        // ---- 正向虚拟扩展：以天线对中的第二个天线为基准，向第一个天线的方向外推 ----
        virtualAnt = getVirtualAntNum(tp_antnna[i][0], tp_antnna[i][1]); // 虚拟天线编号 = 天线1*10 + 天线2
        virtualDiff = tp_diff[i] * virMultiple;                          // 按倍率缩放相位差
        tp[0] = tp_antnna[i][1];                                         // 新天线对：天线B
        tp[1] = virtualAnt;                                              // 新天线对：虚拟天线
        tp_antnna.emplace_back(tp);                                      // 追加到天线对列表
        tp_diff.emplace_back(virtualDiff);                               // 追加对应的虚拟相位差

        // ---- 反向虚拟扩展：对称方向的外推 ----
        virtualAnt = getVirtualAntNum(tp_antnna[i][1], tp_antnna[i][0]); // 对称虚拟天线编号
        virtualDiff = -tp_diff[i] * virMultiple;                         // 取反的缩放相位差
        tp[0] = tp_antnna[i][0];                                         // 新天线对：天线A
        tp[1] = virtualAnt;                                              // 新天线对：虚拟天线
        tp_antnna.emplace_back(tp);                                      // 追加到天线对列表
        tp_diff.emplace_back(virtualDiff);                               // 追加对应的虚拟相位差
    }

    // 用扩展后的数据替换原始数据
    antnna.clear();
    phase_diff.clear();
    antnna = tp_antnna;
    phase_diff = tp_diff;
}

/**
 * @brief 根据虚拟倍率扩展理论相位矩阵，生成虚拟阵元的理论相位
 *
 * 算法原理：
 *   对于每一对不同的天线 (i, j)（i != j）：
 *     1. 计算虚拟天线编号 index = getVirtualAntNum(i+1, j+1) - 1，作为虚拟阵元在线性化矩阵中的列偏移
 *        (i+1, j+1 是因为内部循环使用 0-based 索引，而虚拟天线编号从 1 开始)
 *     2. 计算缩放的相位差：tp_diff = virMultiple * (theory[ang][i] - theory[ang][j])
 *        含义：将天线 i 和 j 之间的相位差按虚拟倍率缩放
 *     3. 计算虚拟阵元的理论相位：tp_diff2 = theory[ang][j] - tp_diff
 *        含义：从天线 j 的实际相位出发，减去（按倍率缩放的）相位差，得到虚拟阵元的理论相位
 *
 * 虚拟理论相位矩阵的结构：
 *   virtualTheory 的列数为 antnnaNum*10 + antnnaNum，这是因为虚拟天线编号 i*10+j 的最大值
 *   对于 antnnaNum 个天线，最大编号为 (antnnaNum-1)*10 + antnnaNum = antnnaNum*10
 *   （实际上不会用到所有位置，仅 index = i*10+j 的位置有值，其余为 0）
 *
 * @param antnnaNum 实际物理天线数量
 * @param tp_theory 原始理论相位矩阵，tp_theory[ang][i] = 角度 ang 处天线 i 的理论相位
 * @param virMultiple 虚拟倍率
 * @param virtualTheory 输出的虚拟理论相位矩阵，virtualTheory[ang][index] = 角度 ang 处虚拟阵元 index 的理论相位
 */
void ArithmeticDoa::getVirtualTheory(const int antnnaNum, const std::vector<std::vector<double>> tp_theory, const double virMultiple, std::vector<std::vector<double>> &virtualTheory) // 获得虚拟矩阵的理论相位
{
    // 虚拟阵元的数量上限：最大编号 = (antnnaNum-1)*10 + antnnaNum，实际使用 antnnaNum*10 个位置
    int num = antnnaNum * 10 + antnnaNum;
    int index = 0;
    virtualTheory.clear();
    virtualTheory.resize(360);
    // 遍历所有 360 个角度
    for (int ang = 0; ang < 360; ang++)
    {
        virtualTheory[ang].resize(num);
        // 遍历所有天线对 (i, j)，i != j
        for (int i = 0; i < antnnaNum; i++)
        {
            for (int j = 0; j < antnnaNum; j++)
            {
                if (i == j)
                {
                    continue; // 跳过同一天线自身，自身的相位差为 0，无意义
                }
                // 计算虚拟天线在线性化矩阵中的索引 (编码规则: ant1*10+ant2)
                index = getVirtualAntNum(i + 1, j + 1) - 1;
                // 缩放的相位差：virMultiple * (相位i - 相位j)
                double tp_diff = virMultiple * (tp_theory[ang][i] - tp_theory[ang][j]);
                // 虚拟阵元的理论相位 = 天线j的实际相位 - (缩放后的相位差)
                // 物理含义：从天线 j 的方向反向外推，得到虚拟阵元位置的理论相位
                double tp_diff2 = tp_theory[ang][j] - tp_diff;
                virtualTheory[ang][index] = tp_diff2;
            }
        }
    }
}

/**
 * @brief 计算定向天线的角度搜索范围
 *
 * 算法原理：
 *   定向天线（如对数周期天线）在不同方向上有不同的增益，信号只能从特定角度范围入射。
 *   因此需要根据信号最强的天线位置，限定DOA搜索范围以提高测向效率和精度。
 *
 * 步骤：
 *   1. 计算每个天线对应的角度跨度：perAngle = 360 / AntennaNum
 *   2. 判断信号最强的天线对（max_index 对应的天线对）中哪一端朝向信号源：
 *      - 天线编号差值 antenna_diff = ant[0] - ant[1]
 *      - 若差值绝对值 > AntennaNum/2，说明跨越了 0 度（360度）边界，需要特殊处理
 *      - 根据差值符号确定 startAntenna（信号来向起始天线）和 endAntenna（信号来向终止天线）
 *   3. 根据 max_index 在 index 中的位置（首/尾/中间），确定搜索范围：
 *      - 若是首元素：搜索范围 = [start角度-10, end角度]
 *      - 若是尾元素：搜索范围 = [start角度-1间距, end角度-1间距+10]
 *      - 若是中间元素：搜索范围 = [start角度-半间距, end角度+半间距]
 *   4. 若起始角度大于结束角度（跨越0度边界），将起始角度减去360度
 *
 * @param index 参与测向的天线对序号列表（按某种顺序排列，如载噪比排序）
 * @param cutSequence 天线对序列（切刀顺序），cutSequence[i] = {天线1编号, 天线2编号}
 * @param AntennaNum 天线总数
 * @param max_index 载噪比最大的天线对在 cutSequence 中的索引
 * @param startAngle 输出的搜索起始角度（度），可能为负值（表示从某角度到0度）
 * @param endAngle 输出的搜索结束角度（度）
 */
void ArithmeticDoa::calAngleSerchRange(const vector<int> index, const vector<vector<int>> cutSequence, const int AntennaNum, int max_index, int &startAngle, int &endAngle) // 计算角度搜索范围，主要用于定向天线
{                                                                                                                                                                           // cutSequence 切刀顺序
    // AntennaNum 天线数
    // index 用于测向的切刀序号
    // max_index 载噪比最大的序号
    int size = (int)index.size();
    int perAngle = (int)(360 / AntennaNum); // 每个天线所占的角度宽度
    int startAntenna = -1;
    int endAntenna = -1;
    float th = AntennaNum / 2.0; // 判断跨越 0 度的阈值
    int antenna_diff = cutSequence[max_index][0] - cutSequence[max_index][1];

    // 根据天线编号差值判断信号来向，确定起始天线和终止天线
    // 四种情况分别处理，考虑了圆周上天线编号的循环性
    if (antenna_diff < (0 - th))  // 差值小于 -N/2：天线1在0度附近，天线2在其后
    {
        startAntenna = cutSequence[max_index][1];
        endAntenna = cutSequence[max_index][0];
    }
    if (antenna_diff > th)  // 差值大于 N/2：跨越 360 度边界
    {
        startAntenna = cutSequence[max_index][0];
        endAntenna = cutSequence[max_index][1];
    }
    if (-th < antenna_diff && antenna_diff < 0)  // 差值在 (-N/2, 0) 区间：天线1在前面
    {
        startAntenna = cutSequence[max_index][0];
        endAntenna = cutSequence[max_index][1];
    }
    if (antenna_diff > 0 && antenna_diff < th)  // 差值在 (0, N/2) 区间：天线2在前面
    {
        startAntenna = cutSequence[max_index][1];
        endAntenna = cutSequence[max_index][0];
    }
    // 根据最强信号天线对在列表中的位置，微调搜索范围
    if (index[0] == max_index)  // 最强对是第一个：搜索范围向前扩展10度
    {
        startAngle = (startAntenna - 1) * perAngle - 10;
        endAngle = endAntenna * perAngle;
    }
    else if (index[size - 1] == max_index)  // 最强对是最后一个：搜索范围向后扩展10度
    {
        startAngle = (startAntenna - 2) * perAngle;
        endAngle = (endAntenna - 1) * perAngle + 10;
    }
    else  // 最强对在中间：搜索范围为所覆盖角度的中心区域
    {
        startAngle = (startAntenna - 1) * perAngle - perAngle / 2;
        endAngle = (endAntenna - 1) * perAngle + perAngle / 2;
    }
    // 处理跨越 0 度边界的情况：起始角度大于结束角度时，减去 360 使搜索范围变为 [start-360, end]
    if (startAngle > endAngle)
    {
        startAngle = startAngle - 360;
    }
}

/**
 * @brief 定向天线模式下的天线选择与搜索范围计算
 *
 * 算法目的：
 *   在定向天线系统中，由于天线具有方向性，信号往往只被其中几个天线接收到。
 *   此函数根据各天线的接收幅度，自动选择用于测向的天线子集，并计算搜索角度范围。
 *
 * 算法原理（逐步说明）：
 *   1. 对所有天线按幅度降序排序，找出信号最强的天线 max_index
 *   2. 以最强天线为中心，逐步向两侧扩展，选取相邻天线加入测向子集：
 *      - 每轮在选中的天线两侧各尝试扩展一个（优先扩展幅度更大的那一侧）
 *      - 持续扩展直到选中天线数达到 use_cut_num + 1
 *      - 该策略考虑了"饱和情况"：当天线饱和时，幅度最大的天线不一定正对信号源，
 *        通过选取两侧相邻天线来覆盖信号源可能的入射方向
 *   3. 根据选中的天线位置计算角度搜索范围：
 *      - 奇数个天线：以中间天线为中心，向两侧各延伸一个天线间距
 *      - 偶数个天线：以中间两个天线之间的位置为中心
 *   4. 搜索范围外扩 10 度作为安全余量
 *
 * @param A 各天线的接收幅度数组（线性值或相对值），A[i] 越大表示天线 i 接收到的信号越强
 * @param use_cut_num 期望选用的测向天线数（注意：实际选 use_cut_num + 1 个天线）
 * @param use_cut 输出的选中天线序号列表（从 1 开始编号）
 * @param startAngle 输出的角度搜索起始值（度）
 * @param endAngle 输出的角度搜索结束值（度）
 */
void ArithmeticDoa::getUseAntennaByADirect(const vector<double> A, const int use_cut_num, vector<int> &use_cut, int &startAngle, int &endAngle)
{
    // use_cut_num 用于测向的天线个数，给出的天线个数要+1
    // 理论上只需要找到最大值，然后再在两边选择几个天线即可
    // 但是，若存在饱和现象，则最大值对应的天线就不一定是信号源正对的天线
    if (use_cut_num < 2)
    {
        return;
    }
    int size = A.size();
    vector<int> index;
    index.clear();
    index.resize(size);
    for (int i = 0; i < size; i++)
    {
        index[i] = i;
    }
    // 按幅度从大到小排序，index 存储排序后的天线序号（0-based）
    std::sort(index.begin(), index.end(), [&](int i, int j)
              { return A[i] > A[j]; }); // 排序，从大到小
    int max_index = index[0];           // 幅度最大的天线序号
    int last_index = 0;
    int next_index = 0;
    vector<int> tp_index;
    tp_index.emplace_back(max_index); // 从最强天线开始构建选中列表
    int size2 = 1;
    int num = 0;
    Log("cal use ant:\n");
    // 迭代扩展直到选中足够多的天线或超过 20 次迭代（防止死循环）
    while (size2 < use_cut_num + 1 && num < 20)
    {
        // 计算当前选中天线范围左侧和右侧的第一个未选中天线
        last_index = (tp_index[0] - 1 + size) % size;              // 左侧扩展候选
        next_index = (tp_index[size2 - 1] + 1) % size;             // 右侧扩展候选
        // 在已排序的列表中查找，优先选择幅度更大的那一侧
        for (int i = 1; i < size; i++)
        {
            if (index[i] == last_index)  // 左侧天线幅度更大
            {
                tp_index.insert(tp_index.begin() + 0, index[i]); // 插入到列表头部
                break;
            }
            if (index[i] == next_index)  // 右侧天线幅度更大
            {
                tp_index.emplace_back(index[i]); // 追加到列表尾部
                break;
            }
        }

        size2 = tp_index.size();
        num++;
        for (int i = 0; i < size2; i++)
        {
            Log("%d,", tp_index[i]);
        }
        Log("\n");
    }
    // use_cut = tp_index;

    int mid = (int)use_cut_num / 2;
    int per = 360 / size; // 每个天线覆盖的角度宽度
    // 根据选中天线数量奇偶性计算搜索范围
    if ((use_cut_num + 1) % 2 == 1)  // 选中奇数个天线
    {
        // int midAnten = tp_index[mid];
        //  cout << "midAnten=" << midAnten << endl;
        startAngle = (tp_index[mid - 1]) * per;    // 中间左侧天线的起始角度
        endAngle = (tp_index[mid + 1]) * per;      // 中间右侧天线的结束角度
        endAngle = Round360(endAngle);
    }
    else  // 选中偶数个天线
    {
        int first = tp_index[mid];                 // 中间偏左的天线
        if (mid + 1 > use_cut_num)
        {
            mid = size2 - 2;
        }
        int sec = tp_index[mid + 1];               // 中间偏右的天线
        startAngle = first * per - per / 2;         // 两天线中间位置 - 半间距
        endAngle = sec * per + per / 2;             // 两天线中间位置 + 半间距
    }
    // 跨越 0 度边界处理
    if (startAngle > endAngle)
    {
        startAngle = startAngle - 360;
    }
    // 留出 10 度安全余量，防止边界误差
    startAngle = startAngle + 10;
    endAngle = endAngle - 10;
    use_cut.resize(size2);
    Log("direct ant use ant:{");
    for (int i = 0; i < (int)tp_index.size(); i++)
    {
        use_cut[i] = tp_index[i] + 1; // 转换为 1-based 天线编号
        Log("%d,  ", use_cut[i]);
    }
    Log("}");
    Log("  startAngle=%d,endAngle=%d\n", startAngle, endAngle);
}

// ==================== 以下为当前项目组(Spoofing)与Suppress组均未使用的函数，暂时注释保留 ====================
// /**
//  * @brief 从相位差数据还原各天线的绝对相位（按天线对顺序推导版本）
//  *
//  * 注意：假设天线对按顺序排列（如 [1,2], [2,3], [3,4], ...），便于顺序推导。
//  * 假设第一个天线对中的第一个天线的相位为 0（参考天线），然后按顺序推导其他天线的相位。
//  *
//  * 推导规则：
//  *   若 ant2 不是参考天线（first_antenna），则：
//  *     phase[ant2] = phase[ant1] - diff(ant1, ant2)
//  *   因为 diff(ant1, ant2) = phase(ant1) - phase(ant2)，所以 phase(ant2) = phase(ant1) - diff
//  *
//  * 潜在问题：
//  *   - 未处理切刀顺序最大序号大于天线个数的异常
//  *   - 未处理切刀顺序不是按天线序号连续的情况（如 [1 2], [3 4] 跳过了天线 2->3 的关系）
//  *
//  * @param cutSequence 天线对序列
//  * @param phaseDiff 相位差数组 [输入] 天线对间差值 / [输出] 各天线的绝对相位（弧度）
//  * @param antennNUm 天线总数
//  */
// void ArithmeticDoa::getAntennaPhase(vector<vector<int>> &cutSequence, vector<double> &phaseDiff, int antennNUm)
// { // 注意：异常未处理：切刀顺序最大序号大于天线个数；切刀顺序不是按天线序号的顺序来，例如【1 2】【3 4】
//     vector<double> tp_phs;
//     tp_phs.clear();
//     tp_phs.resize(antennNUm);
//     // 选取第一个天线对中的第一个天线作为参考天线，设其相位为 0
//     int first_antenna = cutSequence[0][0] - 1;
//     tp_phs[first_antenna] = 0.0;
//     int ant1 = 0;
//     int ant2 = 0;
//     for (int i = 0; i < (int)phaseDiff.size(); i++)
//     {
//         ant1 = cutSequence[i][0] - 1; // 天线对的第一个天线（0-based）
//         ant2 = cutSequence[i][1] - 1; // 天线对的第二个天线（0-based）
//         if (ant2 != first_antenna) // 参考天线本身不需要重新计算
//         {
//             // 从已知的 ant1 相位推导 ant2 的相位
//             // diff = phase(ant1) - phase(ant2)  =>  phase(ant2) = phase(ant1) - diff
//             tp_phs[ant2] = tp_phs[ant1] - phaseDiff[i];
//         }
//
//         Log("ant1=%d,ant2=%d,phs=%.2f\n", ant1 + 1, ant2 + 1, tp_phs[ant2] * 180 / m_PI);
//     }
//     phaseDiff.clear();
//     phaseDiff = tp_phs;
//     Log("get all per antenna phase sucess...\n");
// }
/**
 * @brief 从相位差数据还原指定天线的绝对相位（图连通性推导版本）
 *
 * 算法原理：
 *   将天线视为图中的节点，天线对之间的相位差视为有向边。
 *   以 use_ant[0] 为根节点（相位设为 0），使用广度优先搜索（BFS）的方式，
 *   逐步推导其余天线的绝对相位。
 *
 * 推导规则：
 *   - 若在 cutSequence 中找到 (target, known) 天线对，且已知 known 的相位：
 *     target_phase = known_phase + diff(target, known)
 *     因为 diff(target, known) = phase(target) - phase(known)
 *   - 若在 cutSequence 中找到 (known, target) 天线对：
 *     target_phase = known_phase - diff(known, target)
 *     因为 diff(known, target) = phase(known) - phase(target)
 *
 * 该版本适用于任意天线对排列（不要求连续），只要从参考天线出发能通过
 * 已知相位差关系到达目标天线即可。若某天线无法从参考天线推导到（图不连通），
 * 则返回 -1 表示失败。
 *
 * @param use_ant 需要求解相位的天线序号列表（从 1 开始编号），use_ant[0] 为参考天线
 * @param antennNUm 天线总数
 * @param cutSequence 天线对序列（所有可用的相位差关系）
 * @param phaseDiff 相位差数组 [输入] 各天线对间差值 / [输出] 各天线绝对相位（弧度）
 * @return 0 成功，所有天线相位均已推导出来；-1 失败，存在孤立天线
 */
int ArithmeticDoa::getAntennaPhase(const vector<int> use_ant, const int antennNUm, vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
{ // use_ant 用于测向的天线  cutSequence相位差对应的天线序号,该函数只用于相关干涉仪算法，所以每个相位差只涉及两个天线
    Log("use doa antenna:{");
    for (size_t i = 0; i < use_ant.size(); i++)
    {
        Log("%d,", use_ant[i]);
    }
    Log("}\n");
    int firstA = use_ant[0] - 1; // 参考天线（0-based），设其相位为 0
    // int antennNUm = (int)use_ant.size();
    vector<double> tp_phs;
    tp_phs.clear();
    tp_phs.resize(antennNUm);
    tp_phs[firstA] = 0; // 参考天线相位 = 0
    bool flg = false;
    int ant = 0;
    double tp_diff = 0.0;
    int tp_ant = 0;
    // 对于待求相位的每个天线（跳过参考天线 use_ant[0]）
    for (size_t j = 1; j < use_ant.size(); j++)
    {
        tp_ant = use_ant[j]; // 当前需要求解的天线编号
        flg = false;
        // 在已求解的天线中查找是否有连接关系
        for (size_t k = 0; k < j; k++)
        {
            ant = use_ant[k]; // 已求解相位的天线编号
            // 遍历所有天线对，寻找连接 tp_ant 和 ant 的相位差关系
            for (size_t i = 0; i < cutSequence.size(); i++)
            {
                // 情况 1：cutSequence 顺序为 (tp_ant, ant)，即 diff = phase(tp_ant) - phase(ant)
                if (tp_ant == cutSequence[i][0] && ant == cutSequence[i][1])
                {
                    flg = true;
                    tp_diff = tp_phs[ant - 1] + phaseDiff[i]; // phase(tp_ant) = phase(ant) + diff
                    break;
                }
                // 情况 2：cutSequence 顺序为 (ant, tp_ant)，即 diff = phase(ant) - phase(tp_ant)
                if (tp_ant == cutSequence[i][1] && ant == cutSequence[i][0])
                {
                    flg = true;
                    tp_diff = tp_phs[ant - 1] - phaseDiff[i]; // phase(tp_ant) = phase(ant) - diff
                    break;
                }
            }
        }

        if (flg)
        {
            tp_phs[tp_ant - 1] = tp_diff; // 记录推导出的相位
        }
        else
        {
            return -1; // 图不连通：当前天线无法从已知天线推导得到
        }
    }
    phaseDiff.clear();
    phaseDiff = tp_phs; // 返回各天线的绝对相位
    Log("per antenna phase:\n");
    for (int i = 0; i < phaseDiff.size(); i++)
    {
        double tp2 = phaseDiff[i] * 180 / m_PI;
        Log("---ant = %d,  phse=%.2f\n", i + 1, Round3600(tp2));
    }
    return 0;
}
/**
 * @brief 根据指定天线子集，生成所有天线对之间的相位差（C(N,2) 组合）
 *
 * 从各天线的绝对相位出发（phaseDiff 输入应为绝对相位而非相位差），
 * 对 use_cut 中指定的天线子集，生成全部两两组合的相位差：
 *   diff(ant_i, ant_j) = phase(ant_i) - phase(ant_j)
 * 组合数为 C(N,2) = N*(N-1)/2，其中 N = use_cut 的大小。
 *
 * @param use_cut 参与的天线序号列表（1-based）
 * @param cutSequence 输出的天线对序列（所有两两组合）
 * @param phaseDiff [输入] 各天线的绝对相位 / [输出] 天线对间的相位差
 */
void ArithmeticDoa::getUsePhaseDiffAll(const vector<int> use_cut, vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
{

    vector<double> result_phs;
    result_phs.clear();

    int size = (int)use_cut.size();
    int num = size * (size - 1) / 2; // 组合数 C(N,2)
    vectorResize(cutSequence, num, 2);
    result_phs.resize(num);
    int num1 = 0;
    int num2 = 0;
    num = 0;
    // 双重循环生成所有无重复的天线对 (i, j)，其中 i < j
    for (int i = 0; i < size - 1; i++)
    {
        num1 = use_cut[i];
        for (int j = i + 1; j < size; j++)
        {

            num2 = use_cut[j];
            cutSequence[num][0] = num1;        // 天线对中的第一个天线
            cutSequence[num][1] = num2;        // 天线对中的第二个天线
            result_phs[num] = phaseDiff[num1 - 1] - phaseDiff[num2 - 1]; // diff = phase(i) - phase(j)
            num++;
        }
    }
    phaseDiff.clear();
    phaseDiff = result_phs;
    for (int i = 0; i < phaseDiff.size(); i++)
    {
        double tp2 = phaseDiff[i] * 180 / m_PI;
        Log("---ant = %d a1=%d,a2=%d, phse=%.2f\n", i + 1, cutSequence[i][0], cutSequence[i][1], Round3600(tp2));
    }
}

// /**
//  * @brief 对所有天线，生成全部两两之间的相位差（全排列版本）
//  *
//  * 与 getUsePhaseDiffAll 的区别：本函数使用所有天线（默认从 1 到 N），
//  * phaseDiff 输入为各天线的绝对相位（按天线编号 1..N 排列），
//  * 输出为全部 C(N,2) 对天线之间的相位差。
//  *
//  * @param cutSequence 输出的天线对序列（全部 C(N,2) 组合）
//  * @param phaseDiff [输入] 所有天线的绝对相位 / [输出] 全部天线对间的相位差
//  */
// void ArithmeticDoa::getPhaseDiffAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
// {
//     vector<double> result_phs;
//     result_phs.clear();
//
//     int size = (int)phaseDiff.size(); // 天线总数
//     int num = size * (size - 1) / 2;  // 全部组合数 C(N,2)
//     vectorResize(cutSequence, num, 2);
//     result_phs.resize(num);
//     int num1 = 0;
//     int num2 = 0;
//     num = 0;
//     for (int i = 0; i < size - 1; i++)
//     {
//         num1 = i + 1; // 天线 i+1 的编号
//         for (int j = i + 1; j < size; j++)
//         {
//
//             num2 = j + 1; // 天线 j+1 的编号
//             cutSequence[num][0] = num1;
//             cutSequence[num][1] = num2;
//             result_phs[num] = phaseDiff[i] - phaseDiff[j]; // diff = phase(i) - phase(j)
//             num++;
//         }
//     }
//     phaseDiff.clear();
//     phaseDiff = result_phs;
//     Log("get all phase diff...\n ");
// }

// void ArithmeticDoa::getUsePhaseDiffAll(const vector<int> use_cut, vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
// {
//     vector<double> tp_phs_diff;
//     vector<vector<int>> tp_cutSequence;
//     int size = (int)use_cut.size();
//     int num = size * (size - 1) / 2;

//     vectorResize(tp_cutSequence, num, 2);
//     tp_phs_diff.resize(num);
//     int num1 = 0;
//     int num2 = 0;
//     bool flg1 = false;
//     bool flg2 = false;
//     num = 0;
//     for (int i = 0; i < (int)phaseDiff.size(); i++)
//     {
//         flg1 = false;
//         flg2 = false;
//         num1 = cutSequence[i][0];
//         num2 = cutSequence[i][1];
//         for (int j = 0; j < size; j++)
//         {
//             if (num1 == use_cut[j])
//             {
//                 flg1 = true;
//             }
//             if (num2 == use_cut[j])
//             {
//                 flg2 = true;
//             }
//         }
//         if (flg1 && flg2)
//         {
//             tp_cutSequence[num][0] = num1;
//             tp_cutSequence[num][1] = num2;
//             tp_phs_diff[num] = phaseDiff[i];
//             num++;
//         }
//     }
//     phaseDiff.clear();
//     phaseDiff = tp_phs_diff;
//     cutSequence.clear();
//     cutSequence = tp_cutSequence;
//     Log("get use doa phase diff:\n");
//     for (size_t i = 0; i < phaseDiff.size(); i++)
//     {
//         Log("%d,%d: phase diff= %.2f\n", cutSequence[i][0], cutSequence[i][1], phaseDiff[i]);
//     }
// }

/**
 * @brief 利用传递性原理，从已知相位差组合生成所有可能的相位差
 *
 * 算法原理（相位差传递性 -- 图的传递闭包）：
 *   将天线视为图中的节点，相位差视为有向边上的权重。
 *   利用以下传递规则，从已知的边推导出新的边：
 *
 *   规则 1 -- 路径连接 (A->B->C)：
 *     diff(A,C) = diff(A,B) + diff(B,C)
 *     对应：cutSequence[i][1] == cutSequence[j][0] (B==B)
 *
 *   规则 2 -- 反向路径连接 (C->B->A)：
 *     diff(C,A) = diff(C,B) + diff(B,A)
 *     对应：cutSequence[i][0] == cutSequence[j][1] (B==B)
 *
 *   规则 3 -- 同起点 (A->B, A->C)：
 *     diff(B,C) = diff(A,C) - diff(A,B)
 *     对应：cutSequence[i][0] == cutSequence[j][0] (A==A)
 *
 *   规则 4 -- 同终点 (B->A, C->A)：
 *     diff(B,C) = diff(B,A) - diff(C,A)
 *     对应：cutSequence[i][1] == cutSequence[j][1] (A==A)
 *
 * 算法采用迭代逼近策略：每次迭代扫描当前所有边对，尝试推导新边，
 * 若成功推导出新边则加入图中，继续下一轮迭代，直到无新边可推导或达到最大迭代次数。
 * 类比于 Floyd-Warshall 算法的思想，通过局部传递性逐步构建全连通图。
 *
 * 该函数增强了算法的鲁棒性：即使数据缺失了某些天线对的相位差，
 * 也能通过已知相位差推导出来，确保有足够的相位差信息用于测向。
 *
 * @param cutSequence [输入] 已知天线对序列 / [输出] 扩展后的天线对序列（包含推导出的新对）
 * @param phaseDiff [输入] 已知相位差数组 / [输出] 扩展后的相位差数组
 */
void ArithmeticDoa::setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff) // 根据天线序号，组合相位差
{
    // 使用嵌套 map 存储天线对有向图的邻接表：tp_antenna_map[ant1][ant2] = diff(ant1, ant2)
    map<int, map<int, double>> tp_antenna_map;
    tp_antenna_map.clear();
    map<int, double> tp1;
    // 初始化邻接表：将输入的已知天线对及其相位差加入图中
    for (int i = 0; i < (int)phaseDiff.size(); i++)
    {
        tp1.clear();
        int ant1 = cutSequence[i][0];
        if (tp_antenna_map.find(ant1) != tp_antenna_map.end())
        {
            tp1 = tp_antenna_map[ant1];
        }
        tp1[cutSequence[i][1]] = phaseDiff[i];
        tp_antenna_map[ant1] = tp1;
    }
    int count = 1; // 本轮新推导出的边数
    int size = 0;
    int start_index = 0;
    int tp_ant1 = 0;
    int tp_ant2 = 0;
    double tp2_diff = 0.0;
    // 迭代推导新边，直到无新边可推导或超过最大迭代次数（50 次防止异常情况死循环）
    while (0 != count && count < 50)
    {
        count = 0;
        size = (int)phaseDiff.size();
        // 双重循环：遍历所有已存在的边对 (i, j)，尝试推导新边
        for (int i = 0; i < size; i++)
        {
            if (start_index <= i)
            {
                start_index = i + 1;
            }

            for (int j = start_index; j < size; j++)
            {
                tp1.clear();
                // 规则 1：A->B 和 B->C => A->C
                if (cutSequence[i][1] == cutSequence[j][0])
                {
                    tp_ant1 = cutSequence[i][0];  // A
                    tp_ant2 = cutSequence[j][1];  // C
                    tp2_diff = phaseDiff[i] + phaseDiff[j]; // diff(A,C) = diff(A,B) + diff(B,C)
                }
                // 规则 2：B->C 和 A->B => C->A (实际上是 C->B + B->A = C->A 的另一个方向)
                if (cutSequence[i][0] == cutSequence[j][1])
                {
                    tp_ant1 = cutSequence[j][0];  // C
                    tp_ant2 = cutSequence[i][1];  // A
                    tp2_diff = phaseDiff[i] + phaseDiff[j]; // diff(C,A) = diff(C,B) + diff(B,A)
                }
                // 规则 3：A->B 和 A->C => B->C
                if (cutSequence[i][0] == cutSequence[j][0])
                {
                    tp_ant1 = cutSequence[i][1];  // B
                    tp_ant2 = cutSequence[j][1];  // C
                    tp2_diff = phaseDiff[j] - phaseDiff[i]; // diff(B,C) = diff(A,C) - diff(A,B)
                }
                // 规则 4：B->A 和 C->A => B->C
                if (cutSequence[i][1] == cutSequence[j][1])
                {
                    tp_ant1 = cutSequence[i][0];  // B
                    tp_ant2 = cutSequence[j][0];  // C
                    tp2_diff = phaseDiff[i] - phaseDiff[j]; // diff(B,C) = diff(B,A) - diff(C,A)
                }
                // 跳过自环边（起点和终点相同）
                if (tp_ant1 == tp_ant2)
                {
                    continue;
                }
                // 检查是否已经存在 (tp_ant2, tp_ant1) 的反向边（邻接表中有记录），若存在则跳过
                if (tp_antenna_map.find(tp_ant2) != tp_antenna_map.end())
                {
                    tp1 = tp_antenna_map[tp_ant2];
                    if (tp1.find(tp_ant1) != tp1.end())
                    {
                        continue;
                    }
                    tp1.clear();
                }
                // 检查是否已经存在 (tp_ant1, tp_ant2) 的正向边，若存在则跳过
                if (tp_antenna_map.find(tp_ant1) != tp_antenna_map.end())
                {
                    tp1 = tp_antenna_map[tp_ant1];
                    if (tp1.find(tp_ant2) != tp1.end())
                    {
                        continue;
                    }
                }

                // 新边不存在于邻接表中，添加新边
                tp1[tp_ant2] = tp2_diff;
                tp_antenna_map[tp_ant1] = tp1;
                vector<int> tp2;
                tp2.clear();
                tp2.resize(2);

                tp2[0] = tp_ant1;
                tp2[1] = tp_ant2;
                cutSequence.emplace_back(tp2);   // 添加新天线对
                phaseDiff.emplace_back(tp2_diff); // 添加对应的相位差
                ++count;
            }
        }
        start_index = size; // 下一轮扫描从上一轮结束位置开始，避免重复检查已检查过的对
    }
}


/**
 * @brief 从配置文件中读取频率-阵列半径映射数据
 *
 * 文件格式（每两行为一组）：
 *   第 1 行：频率区间，格式为 "起始MHz-结束MHz" 或 "起始MHz~结束MHz"
 *   第 2 行：对应的阵列半径（米）
 * 例如：
 *   100-200
 *   0.15
 *   200-400
 *   0.12
 * 表示 100~200MHz 频段使用 0.15m 阵列半径，200~400MHz 使用 0.12m 阵列半径。
 *
 * 不同频率对应不同阵列半径的原因：天线阵列通常在多个频段工作，
 * 各频段的天线互耦效应不同，导致等效阵列孔径（半径）有所变化。
 *
 * @param adr 配置文件路径
 * @param mR 输出的频率-半径映射数组（每个元素记录一个频段及其对应的半径）
 * @return 0 成功；-1 文件读取失败；-2 文件格式错误（频率行无法按 '-' 或 '~' 分割）
 */
int ArithmeticDoa::getRData(const string adr, vector<Rs> &mR){
    vector<Rs>().swap(mR);
    vector<string> tpVec;
    int n = readFile(adr, tpVec);

    if (0 != n){
        Log("***error:read r adr is flase-----%s\n", adr.c_str());
        return -1;
    }

    // 每次读取两行：第一行为频率区间，第二行为半径值
    for (int i = 0; i < tpVec.size(); i += 2){

        string &a = tpVec[i];
        vector<string> a2;

        // 尝试用 '-' 分割频率区间（格式：起始MHz-结束MHz）
        split(a2, a, '-');

        if (a2.size() == 1){
            a2.clear();
            // 尝试用 '~' 分割频率区间（格式：起始MHz~结束MHz）
            split(a2, a, '~');
        }

        if (a2.size() == 1){
            return -2; // 无法分割频率区间，格式错误
        }

        // 将 MHz 转换为 Hz（乘以 1e6）
        double starF1 = stod(a2[0]) * 1e6; // 起始频率（Hz）
        double endF1 = stod(a2[1]) * 1e6;  // 结束频率（Hz）

        Rs tmpR;
        tmpR.i_starF = starF1;
        tmpR.i_endF = endF1;
        tmpR.i_r = stod(tpVec[i + 1]); // 阵列半径（米）
        mR.emplace_back(tmpR);
    }

    tpVec.clear();
    vector<string>().swap(tpVec);
    Log("set fre and R is sucess...\n");
    return 0;
}

/**
 * @brief 利用相关干涉仪 + 虚拟阵元进行二次测向，解决相位模糊（"跳半周"）问题
 *
 * ===== 问题背景 =====
 * 当阵元间距 d > lambda/2 时，相位差可能超过 [-pi, pi] 的范围。
 * 由于实际测量只能得到 [-pi, pi] 范围内的相位（相位缠绕，Phase Wrapping），
 * 导致理论相位差和实测相位差的匹配出现"多峰"现象：在伪谱中出现多个峰值，
 * 难以判断哪个峰值对应真实来波方向。
 *
 * 例如：实际相位差为 200 度（超过 180 度），测量值会被折叠为 -160 度，
 * 从而与某个错误角度的理论值匹配，产生"镜像峰"。
 *
 * ===== 算法策略 =====
 * 1. 先用相关干涉仪粗测向，得到伪谱 diff 及其多个峰
 * 2. 对各候选峰，选取区分度最好的天线对（cos 值最小的，即理论相位差差异最大的）
 * 3. 用虚拟阵列扩展缩短等效基线，消除相位模糊
 * 4. 对比实际相位差与各候选角度下虚拟理论相位差的相关度，选出最佳匹配
 *
 * ===== 详细步骤 =====
 *
 * 步骤 1 -- 伪谱预处理（降噪）：
 *   将伪谱中小于 90 的值置零，过滤低相关噪声，仅保留显著的峰。
 *
 * 步骤 2 -- 寻峰：
 *   调用 findPeaks 在伪谱中寻找峰值。若只有 0 或 1 个峰，说明无模糊问题，
 *   直接返回（第一次测向结果即可信）。
 *
 * 步骤 3 -- 提取候选角度的理论相位差：
 *   取出前两个峰值角度 index[0] 和 index[1]（通常最强的两个候选），
 *   分别计算这两个角度下各天线对的理论相位差 tp_theory_diff1 和 tp_theory_diff2。
 *
 * 步骤 4 -- 选择鉴别基（Discriminative Baselines）：
 *   对每个天线对 j，计算两个候选角度下理论相位差的余弦值：
 *     cos_diff[j] = cos(tp_theory_diff1[j] - tp_theory_diff2[j])
 *   cos 值越小，说明该天线对在两个候选角度下的相位差差异越大，
 *   对该天线对而言两个角度更容易区分（鉴别力更强）。
 *   选取 cos 值最小的 num=3 个天线对作为"鉴别基"。
 *
 * 步骤 5 -- 虚拟阵列扩展：
 *   对选出的鉴别基天线对和对应的相位差，分别进行三次虚拟阵列扩展：
 *     - 用实际测量相位差进行虚拟扩展 (antnna, phase_diff)
 *     - 用候选角度1的理论相位差进行虚拟扩展 (antnna1, tp_theory_diff3)
 *     - 用候选角度2的理论相位差进行虚拟扩展 (antnna2, tp_theory_diff4)
 *   虚拟扩展改变了等效基线长度，使得原本模糊的相位差变得可区分。
 *
 * 步骤 6 -- 对比判定：
 *   计算实际虚拟相位差与两个候选角度下虚拟理论相位差的余弦相关度之和：
 *     sum1 = sum_i cos(phase_diff[i] - tp_theory_diff3[i])  // 候选角度1
 *     sum2 = sum_i cos(phase_diff[i] - tp_theory_diff4[i])  // 候选角度2
 *   取 sum 值更大的角度作为最终结果（相关度更高 = 匹配更好）。
 *
 * 步骤 7 -- 质量评估：
 *   基于最终选定的角度重新计算理论伪谱和测向质量。
 *
 * @param phaseTheory 理论相位矩阵
 * @param virMultiple 虚拟倍率（通常 < 1，用于缩短等效基线以消除模糊）
 * @param diff 第一次粗测向得到的伪谱数组（360 点），作为输入同时也会被修改（< 90 的值被清零）
 * @param data 实测数据（天线对和相位差）
 * @param angle 输出的最终测向角度（度）
 * @param quality 输出的测向质量（0-100）
 */
void ArithmeticDoa::calSecondDoaByVirInterf(const vector<vector<double>> phaseTheory, const double virMultiple, vector<double> diff, InterferInfo data, double &angle, double &quality) // 利用相关干涉仪+虚拟矩阵进行二次测向
{

    // ===== 步骤 1：伪谱预处理，滤除低相关噪声 =====
    for (int i = 0; i < (int)diff.size(); i++)
    {
        if (diff[i] < 90) // 相关度小于 90 的角度视为噪声
        {
            diff[i] = 0.0;
        }
    }
    // ===== 步骤 2：在伪谱中寻峰 =====
    vector<int> index;
    vector<double> vaules;
    findPeaks(diff, index, vaules); // 获取伪谱的峰值位置
    if (index.empty() || index.size() == 1) // 无峰或单峰，无需解模糊
    {
        return;
    }

    int size = data.i_Phase_Len;
    vector<double> cos_diff;
    vector<int> diff_index;
    cos_diff.resize(size);
    diff_index.resize(size);

    vector<double> tp_theory_diff1; // 候选角度 1 下各天线对的理论相位差
    vector<double> tp_theory_diff2; // 候选角度 2 下各天线对的理论相位差
    tp_theory_diff1.resize(size);
    tp_theory_diff2.resize(size);
    // ===== 步骤 3：计算两个候选角度下的理论相位差 =====
    // Log("angle1=%d,angle2=%d\n", index[0], index[1]);
    for (int j = 0; j < size; j++)
    {
        // 理论相位差 = 天线对中天线1的理论相位 - 天线对中天线2的理论相位
        tp_theory_diff1[j] = phaseTheory[index[0]][data.i_AntennaSq[j][0] - 1] - phaseTheory[index[0]][data.i_AntennaSq[j][1] - 1];
        tp_theory_diff2[j] = phaseTheory[index[1]][data.i_AntennaSq[j][0] - 1] - phaseTheory[index[1]][data.i_AntennaSq[j][1] - 1];
        // 计算两个候选角度下理论相位差的余弦值（cos 值越小，差异越大，区分度越好）
        cos_diff[j] = cos(tp_theory_diff1[j] - tp_theory_diff2[j]);
        // Log("cos_diff:i=%d,diff=%.2f\n", j, cos_diff[j]);
        diff_index[j] = j;
    }
    // ===== 步骤 4：选择鉴别力最强的天线对（鉴别基） =====
    // 按 cos_diff 从小到大排序，cos 值小的天线对鉴别力最强
    std::sort(diff_index.begin(), diff_index.end(), [&](int i, int j)
              { return cos_diff[i] < cos_diff[j]; });

    int num = 3; // 选择 3 个最具鉴别力的天线对
    //(int)size / 2;
    vector<vector<int>> antnna;
    antnna.resize(num);
    vector<double> phase_diff;
    phase_diff.resize(num);
    vector<double> tp_theory_diff3; // 鉴别基在候选角度 1 下的理论相位差
    vector<double> tp_theory_diff4; // 鉴别基在候选角度 2 下的理论相位差
    tp_theory_diff3.resize(num);
    tp_theory_diff4.resize(num);
    for (int i = 0; i < num; i++)
    {
        antnna[i].resize(2);
        antnna[i][0] = data.i_AntennaSq[diff_index[i]][0]; // 天线对中的天线1
        antnna[i][1] = data.i_AntennaSq[diff_index[i]][1]; // 天线对中的天线2
        phase_diff[i] = data.i_Phase_Diff[diff_index[i]];  // 对应的实测相位差
        tp_theory_diff3[i] = tp_theory_diff1[diff_index[i]]; // 候选角度1下的理论相位差
        tp_theory_diff4[i] = tp_theory_diff2[diff_index[i]]; // 候选角度2下的理论相位差
        //   Log("sort cos_diff:diff_index=%d,antanna1 = %d,antanna2=%d,phase_diff = %.2f\n", diff_index[i], antnna[i][0], antnna[i][1], phase_diff[i]);
    }
    // ===== 步骤 5：虚拟阵列扩展（三次并行扩展） =====
    vector<vector<int>> antnna1;
    vector<vector<int>> antnna2;
    antnna1 = antnna; // 用于候选角度 1 的虚拟扩展
    antnna2 = antnna; // 用于候选角度 2 的虚拟扩展
    getVirtual(virMultiple, antnna, phase_diff);      // 实测数据的虚拟扩展
    getVirtual(virMultiple, antnna1, tp_theory_diff3); // 候选角度 1 理论值的虚拟扩展
    getVirtual(virMultiple, antnna2, tp_theory_diff4); // 候选角度 2 理论值的虚拟扩展
    // ===== 步骤 6：对比相关度，确定最终角度 =====
    double sum1 = 0.0;
    double sum2 = 0.0;
    for (int i = 0; i < (int)antnna.size(); i++)
    {
        // 计算实测相位差与候选角度 1 虚拟理论相位差的余弦相关度
        sum1 = sum1 + cos(phase_diff[i] - tp_theory_diff3[i]);
        // 计算实测相位差与候选角度 2 虚拟理论相位差的余弦相关度
        sum2 = sum2 + cos(phase_diff[i] - tp_theory_diff4[i]);
        // Log("virtual:phase_diff = %.2f,tp_theory_diff3 = %.2f,tp_theory_diff4 = %.2f\n", phase_diff[i], tp_theory_diff3[i], tp_theory_diff4[i]);
    }
    // 选择相关度更高的角度
    if (sum1 < sum2) // 候选角度 2 匹配更好
    {
        angle = index[1];
        // ===== 步骤 7：用选定的角度重新计算质量 =====
        for (int j = 0; j < size; j++)
        {
            data.i_Phase_Diff[j] = tp_theory_diff2[j]; // 更新相位差为选定角度的理论值
        }
        vector<double> tp_theory_Doa_diff;
        calPseudoByInterfer(phaseTheory, data, tp_theory_Doa_diff); // 计算理论伪谱
        quality = getDoaMass(tp_theory_Doa_diff, diff);            // 计算测向质量
    }
}
/**
 * @brief 计算理论伪谱（相关干涉仪版本）-- 用于测向质量评估
 *
 * 伪谱定义：
 *   对于搜索范围内的每个角度 ang，计算该角度下各天线对的理论相位差与
 *   "已知最优匹配相位差"（存储在 data.i_Phase_Diff 中）的余弦相关度之和。
 *   diff[ang] = sum_i cos(theoryPhaseDiff_i(ang) - data.i_Phase_Diff[i])
 *
 * 与 calInterfer 的区别：
 *   - calInterfer 遍历所有角度找出最佳匹配，其过程产生"实际伪谱"
 *   - calPseudoByInterfer 使用已确定的最佳匹配相位差作为"理论值"，
 *     重新计算一遍伪谱，得到的称为"理论伪谱"
 *   - 这两个伪谱的相关系数即为测向质量（getDoaMass）
 *
 * 为什么要分别计算两个伪谱：
 *   实际伪谱反映的是"实测数据与各角度理论值的匹配程度"，
 *   理论伪谱反映的是"最佳角度理论值与各角度理论值的匹配程度"（理想情况）。
 *   两者越相似，说明实测数据越接近理论模型，测向结果越可靠。
 *
 * @param phaseTheory 理论相位矩阵
 * @param data 测向数据，其中 i_Phase_Diff 应已填充为最佳匹配角度下的理论相位差
 * @param diff 输出的理论伪谱数组（360 点），diff[ang] = 角度 ang 处的理论相关度累加值
 */
void ArithmeticDoa::calPseudoByInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, vector<double> &diff)
{
    int startAngle = (int)data.i_Start;
    int endAngle = (int)data.i_End;
    int size = data.i_Phase_Len;
    diff.clear();
    vector<double>().swap(diff);
    diff.resize(360);
    std::fill(diff.begin(), diff.end(), 0.0);
    int tp_ang = 0;
    // 在指定的搜索范围内遍历所有角度
    for (int ang = startAngle; ang < endAngle + 1; ang++)
    {
        double sumDiff = 0.0;
        tp_ang = Round360(ang); // 将角度归一化到 [0, 360)
        // 计算该角度下各天线对的余弦相关度
        for (int i = 0; i < size; i++)
        {
            int antn1 = data.i_AntennaSq[i][0];
            int antn2 = data.i_AntennaSq[i][1];
            // 该角度下天线对的理论相位差
            double theoryPhase = phaseTheory[tp_ang][antn1 - 1] - phaseTheory[tp_ang][antn2 - 1];
            // 余弦相关度：cos(0)=1 表示完美匹配，cos(pi)=-1 表示反相
            sumDiff = sumDiff + cos(theoryPhase - data.i_Phase_Diff[i]);
        }
        // 存储该角度的相关度累加值
        diff[tp_ang] = sumDiff;
    }
}

/**
 * @brief 相关干涉仪测向主算法 -- 通过相位差余弦相关匹配确定来波方向
 *
 * ===== 算法原理 =====
 * 相关干涉仪测向法是一种经典的无线电测向算法，其核心思想是：
 * 将实测的天线阵列相位差向量与预先计算的理论相位差向量进行相关匹配，
 * 相关度最高的角度即为来波方向的估计。
 *
 * ===== 匹配度量 =====
 * 使用余弦相关度作为匹配度量函数：
 *   correlation(ang) = sum_i cos( theory_phase_diff_i(ang) - actual_phase_diff_i )
 *
 * 使用余弦函数而非直接相减的原因：
 *   1. 相位具有 2*pi 周期性，直接相减无法区分相差 2*pi 整数倍的情况
 *   2. cos(delta) 对相位差 delta 是周期函数，天然处理了相位缠绕问题
 *   3. cos(0) = 1（完全匹配），cos(pi) = -1（完全反相），cos(pi/2) = 0（无相关）
 *   4. cos 函数在 [-pi/2, pi/2] 区间是单调递减的，具有良好的梯度特性
 *
 * ===== 算法步骤 =====
 *
 * 步骤 1 -- 遍历搜索：
 *   在数据指定的搜索范围 [startAngle, endAngle] 内，以 1 度为步长遍历所有候选角度。
 *   对每个候选角度 ang，计算该角度下各天线对的理论相位差。
 *
 * 步骤 2 -- 计算相关度：
 *   sumDiff = sum_i cos(theoryPhaseDiff_i(ang) - actualPhaseDiff_i)
 *   sumDiff 最大值为 N（天线对数），此时所有天线对的实测值与理论值完全一致。
 *   sumDiff 最小值为 -N，此时所有天线对完全反相。
 *
 * 步骤 3 -- 峰值搜索：
 *   在所有候选角度中，找到 sumDiff 最大的角度 ang_val 作为 DOA 估计结果。
 *
 * 步骤 4 -- 质量评估：
 *   基于最优角度的理论相位差，构造"理论伪谱"；基于实测值的遍历结果构造"实际伪谱"。
 *   计算两个伪谱的归一化互相关系数作为测向质量：
 *     quality = (corr + 1) / 2 * 100，其中 corr 为互相关系数。
 *   quality 反映最优角度的理想匹配曲线与实际匹配曲线的吻合程度。
 *
 * 步骤 5 -- 输出伪谱：
 *   diff2 输出加权伪谱：diff2[ang] = ((sumDiff/N + 1) / 2) * quality
 *   归一化到 [0, 1] 后再乘以质量值，用于后续的二次测向（解模糊）等处理。
 *
 * ===== 参数说明 =====
 * @param phaseTheory 理论相位矩阵，phaseTheory[ang][i] = 角度 ang（0-359度）处天线 i 的理论相位（弧度）
 *                    该矩阵通常由 calPhaseTheory 函数根据阵列参数预计算得到
 * @param data 实测数据，包含：
 *             - i_Start / i_End：角度搜索范围
 *             - i_Phase_Len：有效天线对数
 *             - i_AntennaSq：天线对序号（天线从 1 开始编号）
 *             - i_Phase_Diff：实测相位差（弧度）
 * @param angle 输出的测向角度（度），已归一化到 [0, 360) 范围
 * @param quality 输出的测向质量（0-100），反映结果可靠性
 * @param diff2 输出的加权伪谱数组（360 点），diff2[ang] = 归一化相关度 * quality
 */
void ArithmeticDoa::calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2) // 利用相关干涉仪测向
{
    diff2.resize(360);
    vector<double> diff; // 实际伪谱（用于质量评估）
    int startAngle = (int)data.i_Start;
    int endAngle = (int)data.i_End;
    int size = data.i_Phase_Len; // 天线对数
    double max_val = -9999.9;    // 初始化最大相关度为极小值
    int ang_val = 0;             // 记录最大相关度对应的角度
    diff.clear();
    vector<double>().swap(diff);
    diff.resize(360);
    int tp_ang = 0;
    InterferInfo tp_data = data;     // 副本，用于存储最佳匹配的理论相位差
    double tp_Phase_theory[200];     // 临时存储当前角度的理论相位差数组
    // ===== 步骤 1-3：遍历搜索 + 计算相关度 + 峰值搜索 =====
    for (int ang = startAngle; ang < endAngle + 1; ang++)
    {
        double sumDiff = 0.0;
        tp_ang = Round360(ang); // 角度归一化
        // 计算当前候选角度 ang 与实测值的余弦相关度
        for (int i = 0; i < size; i++)
        {
            int antn1 = data.i_AntennaSq[i][0]; // 天线对中第一个天线的编号（1-based）
            int antn2 = data.i_AntennaSq[i][1]; // 天线对中第二个天线的编号（1-based）
            // 当前角度下该天线对的理论相位差 = 天线1的理论相位 - 天线2的理论相位
            double theoryPhase = phaseTheory[tp_ang][antn1 - 1] - phaseTheory[tp_ang][antn2 - 1];
            tp_Phase_theory[i] = theoryPhase;

            // 余弦相关度累加：cos(理论相位差 - 实测相位差)
            // cos(0)=1 表示完美匹配，cos(pi)=-1 表示完全反相
            sumDiff = sumDiff + cos(theoryPhase - data.i_Phase_Diff[i]);
        }
        //  sumDiff = (sumDiff / size + 1) / 2 * 100;
        // sumDiff = sumDiff / size;
        // 更新最大相关度及其对应的角度
        if (sumDiff > max_val)
        {
            max_val = sumDiff;
            ang_val = tp_ang; // 记录最佳匹配角度
            // 保存最佳匹配角度下的各理论相位差，用于后续质量评估
            for (int j = 0; j < data.i_Phase_Len; j++)
            {
                tp_data.i_Phase_Diff[j] = tp_Phase_theory[j];
            }
        }
        diff[tp_ang] = sumDiff; // 实际伪谱：每个角度的相关度值
        // 输出归一化伪谱：将 sumDiff 映射到 [0, 1] 区间
        // (sumDiff/size + 1) / 2 将 [-1, 1] 的余弦均值映射到 [0, 1]
        diff2[tp_ang] = (sumDiff / size + 1) / 2;
    }

    // ===== 步骤 4：计算测向质量 =====
    angle = Round360(ang_val); // 确保角度在 [0, 360) 范围内
    vector<double> tp_theory_diff;
    // 以最佳匹配角度下的理论相位差作为"理论值"，重新计算一条理论伪谱
    calPseudoByInterfer(phaseTheory, tp_data, tp_theory_diff);
    // 计算理论伪谱与实际伪谱的相关系数，得到测向质量
    quality = getDoaMass(tp_theory_diff, diff, startAngle, endAngle); // 计算测向质量
    // ===== 步骤 5：将归一化伪谱乘以质量值，得到加权伪谱 =====
    for (int i = 0; i < 359; i++)
    {
        diff2[i] *= quality; // diff2[ang] = 归一化相关度 * 测向质量
    }
}


/**
 * @brief 计算均匀圆阵的理论相位（以阵列中心为参考点）
 *
 * ===== 物理模型 =====
 * 对于 N 元均匀圆阵（Uniform Circular Array, UCA）：
 *   - 天线均匀分布在半径为 r 的圆周上
 *   - 第 i 个天线（i 从 0 开始）的方位角为：theta_i = i * 2*pi / N
 *   - 以圆心为相位参考点，入射平面波来自角度 ang（度）
 *
 * ===== 相位公式推导 =====
 * 远场条件下，平面波到达天线 i 与到达圆心的波程差为：
 *   delta_d = r * cos(theta_i - alpha)
 *   其中 alpha = ang * pi / 180 是入射角的弧度值
 *
 * 对应的相位差为：
 *   phase_i = 2*pi * delta_d / lambda
 *           = 2*pi * f * r / c * cos(i * 2*pi/N - ang * pi/180)
 *
 * 其中：
 *   - f: 信号频率（Hz）
 *   - r: 阵列半径（m）
 *   - c: 光速 3e8（m/s），定义于 m_C
 *   - lambda = c/f: 波长（m）
 *
 * ===== 相位范围 =====
 * 由于 cos 值域为 [-1, 1]，相位绝对值最大为 2*pi*f*r/c = 2*pi*r/lambda。
 * 当 r > lambda/2 时，相位绝对值超过 pi，可能出现"跳半周"相位模糊。
 * 这就是为什么大孔径阵列需要虚拟阵列解模糊的原因。
 *
 * ===== 参数说明 =====
 * @param f 信号频率（Hz），用于计算波长
 * @param r 阵列半径（米），圆阵的物理半径
 * @param antnnaNum 天线阵元数量
 * @param theory 输出的理论相位矩阵：
 *               theory[ang][i] 表示来自角度 ang（0-359度）的信号
 *               到达天线 i（0-based）时相对于圆心的相位延迟（弧度）
 *               theory 的大小为 360 x antnnaNum
 */
void ArithmeticDoa::calPhaseTheory(const double f, const double r, const int antnnaNum, std::vector<std::vector<double>> &theory) // 计算理论相位差
{

    // 相邻天线之间的角度间隔（弧度）
    double perAngel = 2 * m_PI / antnnaNum;
    theory.resize(360);
    // 遍历所有 360 个角度（0-359 度，步长 1 度）
    for (int ang = 0; ang < 360; ang++)
    {
        theory[ang].resize(antnnaNum);
        // 计算每个天线在角度 ang 处的理论相位
        for (int i = 0; i < antnnaNum; i++)
        {
            // 天线 i 的角位置 = i * 2*pi/N
            // 入射波方向 = ang * pi/180
            // 相位 = 2*pi*f*r/c * cos(天线角位置 - 入射方向)
            double theoryPhase = 2 * m_PI * f * r / m_C * cos(i * perAngel - ang * m_PI / 180);
            theory[ang][i] = theoryPhase;
        }
    }
    // Log("get theory phase:f=%.2f, r=%.4f, antennaNum=%d\n ", f, r, antnnaNum);
}

/**
 * @brief 计算幅相法的理论伪谱（用于测向质量评估）
 *
 * 伪谱定义：
 *   对每个角度 ang，计算该角度下仿真阵列流型 simulateA[ang] 与
 *   最优匹配阵列流型 theory_A 的归一化内积（复数余弦相似度）：
 *     similarity = |sum(conj(sim_A[i]) * theory_A[i])| / (norm(sim_A) * norm(theory_A))
 *   similarity 取值范围 [0, 1]，1 表示完全匹配，0 表示完全不匹配。
 *
 * 归一化内积的物理意义：
 *   two 复数向量 A 和 B 的内积 conj(A).B 的模反映了两个向量在复数空间中的夹角，
 *   除以各自的模长后得到的值即为余弦相似度。
 *
 * 与相关干涉仪伪谱的类比：
 *   - 相关干涉仪：伪谱 = sum(cos(理论相位差 - 实际相位差))
 *   - 幅相法：伪谱 = 阵列流型的复数余弦相似度
 *   两者本质上都是衡量"理论值"与"实际值"相似度的方法，只是度量空间不同。
 *
 * @param simulateA 仿真阵列流型矩阵（360 x 天线数，复数）
 * @param theory_A 最优匹配角度下的阵列流型向量（复数），作为"理论值"
 * @param diff 输出的理论伪谱数组（360 点），diff[ang] = 归一化内积 * 100
 */
void ArithmeticDoa::calPseudoByAmpPhase(const vector<vector<complex<double>>> simulateA, vector<complex<double>> theory_A, vector<double> &diff)
{
    double tp_ActualNorm = getNorm(theory_A); // 理论向量的 L2 范数
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    diff.clear();
    diff.resize(360);
    vector<complex<double>> tp_A;
    // 遍历所有 360 个角度
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        tp_A = simulateA[ang];         // 当前角度的仿真阵列流型
        double tp_ANorm = getNorm(tp_A); // 当前角度阵列流型的 L2 范数
        // 计算共轭内积：sum(conj(sim_A[i]) * theory_A[i])
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * theory_A[i];
        }
        // 归一化内积的模（余弦相似度），映射到 [0, 100]
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm);
        diff[ang] = tp_diff * 100;
    }
}

/**
 * @brief 幅相法测向主算法（版本一：通过 InterferInfo 传入实测数据）
 *
 * ===== 算法原理 =====
 * 幅相法测向同时利用天线阵列的幅度和相位信息，通过复数阵列流型的内积匹配进行测向。
 * 相比于相关干涉仪法（仅使用相位），幅相法对非理想天线方向图有更好的适应性。
 *
 * ===== 算法步骤 =====
 *
 * 步骤 1 -- 构造实测阵列流型：
 *   从 InterferInfo 中提取幅度（i_Amp，线性值）和相位差（i_Phase_Diff，弧度），
 *   调用 getA 构造复数形式的实测阵列流型向量 actual：
 *     actual[k] = amp[k] * exp(j * phase_diff[k])
 *   注意：这里使用的是相位"差"而非绝对相位，因此阵列流型是"比值"形式。
 *
 * 步骤 2 -- 构造各角度的仿真阵列流型（比值形式）：
 *   对每个候选角度 ang，计算各天线对的复数比值：
 *     tp_A[k] = simulateA[ang][ant1-1] / simulateA[ang][ant2-1]
 *   使用比值而非绝对值的原因是：实测数据也是相位差（比值）形式，
 *   这样可以消除接收机通道间的不一致性。
 *
 * 步骤 3 -- 复数内积匹配：
 *   计算仿真阵列流型 tp_A 与实测阵列流型 actual 的归一化内积（余弦相似度）：
 *     similarity = |sum(conj(tp_A[i]) * actual[i])| / (norm(tp_A) * norm(actual))
 *   取 similarity 最大的角度作为 DOA 估计。
 *
 * 步骤 4 -- 质量评估：
 *   基于最优角度的仿真阵列流型，重新计算理论伪谱，并求其与实际伪谱的相关系数。
 *
 * ===== 与相关干涉仪方法的对比 =====
 *   - 相关干涉仪：correlation = sum(cos(理论相位差 - 实际相位差))
 *     等价于复数内积的实部（忽略幅度后）
 *   - 幅相法：correlation = |sum(conj(理论_A) * 实际_A)| / (norm * norm)
 *     同时考虑了幅度和相位，且使用复数内积的模，统一处理了相位差和幅度比
 *
 * @param simulateA 仿真阵列流型矩阵（360 x 天线数，复数），simulateA[ang][i] = 天线 i 在角度 ang 的复数响应
 * @param data 实测数据，包含幅度（i_Amp）、相位差（i_Phase_Diff）和天线对序号（i_AntennaSq）
 * @param angle 输出的测向角度（度，0-359）
 * @param quality 输出的测向质量（0-100）
 * @param diff 输出的归一化内积相似度曲线（360 点），diff[ang] = 相似度 * 100
 */
void ArithmeticDoa::calAmpPhase(vector<vector<complex<double>>> simulateA, const InterferInfo data, double &angle, double &quality, vector<double> &diff) // 幅相法测向
{
    int num = data.i_Phase_Len;
    vector<double> amp;
    amp.resize(num);
    vector<double> phase;
    phase.resize(num);
    // ===== 步骤 1：提取实测幅度和相位差，构造实测阵列流型 =====
    for (int i = 0; i < num; i++)
    {
        amp[i] = data.i_Amp[i];               // 线性幅度值
        phase[i] = data.i_Phase_Diff[i];      // 相位差（弧度）
    }
    vector<complex<double>> actual;
    getA(amp, phase, actual); // 组合为复数阵列流型：actual[k] = amp * exp(j*phase)
    double tp_ActualNorm = getNorm(actual); // 实测向量的 L2 范数

    // ===== 步骤 2-3：遍历搜索 + 复数内积匹配 =====
    double tp_max = -999.0;       // 最大相似度初始值
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    vector<complex<double>> tp_A;
    tp_A.clear();
    tp_A.resize(num);
    vector<complex<double>> tp_A2; // 存储最佳匹配角度的阵列流型
    tp_A2.clear();

    diff.clear();
    diff.resize(360);
    vector<vector<complex<double>>> tp_simulateA; // 存储所有角度的比值形式阵列流型，用于质量评估
    tp_simulateA.resize(360);
    // 遍历所有 360 个角度
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        // 对每个天线对 k，计算仿真阵列流型的比值
        // tp_A[k] = 仿真阵列中天线对第一个天线的响应 / 仿真阵列中天线对第二个天线的响应
        for (int k = 0; k < num; k++)
        {
            tp_A[k] = simulateA[ang][data.i_AntennaSq[k][0] - 1] / simulateA[ang][data.i_AntennaSq[k][1] - 1];
        }
        tp_simulateA[ang] = tp_A; // 保存当前角度的比值形式阵列流型

        double tp_ANorm = getNorm(tp_A); // 仿真向量的 L2 范数
        // 计算共轭内积并归一化
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * actual[i];
        }
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm); // 归一化内积的模
        // 更新最大相似度
        if (tp_max < tp_diff)
        {
            tp_max = tp_diff;
            angle = ang;      // 记录最佳匹配角度
            tp_A2.clear();
            tp_A2 = tp_A;     // 记录最佳匹配角度下的阵列流型
        }
        diff[ang] = tp_diff * 100; // 存储相似度曲线（百分比）
    }
    // ===== 步骤 4：计算测向质量 =====
    vector<double> tp_theory_diff;
    // 以最佳匹配角度下的阵列流型作为"理论值"，重新计算理论伪谱
    calPseudoByAmpPhase(tp_simulateA, tp_A2, tp_theory_diff);
    // 计算理论伪谱与实际伪谱的相关系数作为测向质量
    quality = getDoaMass(tp_theory_diff, diff, 0, 360);
    // quality = tp_max * 100; //(tp_max + 1) / 2 * 100;
}

/**
 * @brief 幅相法测向主算法（版本二：直接传入复数阵列流型向量）
 *
 * 与版本一的核心算法相同，区别在于：
 *   1. 输入直接是完整的复数阵列流型向量 actualA（已预处理为复数形式），
 *      不需要从 InterferInfo 中提取幅度和相位再组合
 *   2. 仿真阵列流型直接使用 simulateA[ang]（不需要计算天线对的比值）
 *   3. 角度输出取负值：angle = -ang（坐标系约定可能与版本一相反，取决于
 *      仿真数据的角度定义方式）
 *
 * ===== 适用场景 =====
 * 版本二适用于已预先将实测数据转换为复数阵列流型的场景，
 * 例如已经提取了各天线的绝对幅度和相位（而非相位差），
 * 并通过 getA 转换为复数形式。
 *
 * @param simulateA 仿真阵列流型矩阵（360 x 天线数，复数）
 * @param actualA 实测阵列流型向量（复数），各天线独立的复数响应
 * @param angle 输出的测向角度（度），注意此处取负值：angle = -ang
 * @param quality 输出的测向质量（0-100）
 * @param diff 输出的相似度曲线（360 点），diff[ang] = 相似度 * 100
 */
void ArithmeticDoa::calAmpPhase(const vector<vector<complex<double>>> simulateA, const vector<complex<double>> actualA, double& angle, double& quality, vector<double>& diff)
{
    int num = actualA.size();                              // 天线数量
    double tp_ActualNorm = getNorm(actualA);               // 实测向量的 L2 范数
    double tp_max = -999.0;                                // 最大相似度
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    diff.clear();
    diff.resize(360);
    vector<vector<complex<double>>> tp_simulateA;          // 保存所有角度的仿真阵列流型
    tp_simulateA.resize(360);

    vector<complex<double>> tp_A;
    tp_A.clear();
    tp_A.resize(num);
    vector<complex<double>> tp_A2;                         // 最佳匹配角度的阵列流型
    tp_A2.clear();
    // 遍历所有 360 个角度，计算相似度
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        tp_A = simulateA[ang];                             // 当前角度的仿真阵列流型
        tp_simulateA[ang] = simulateA[ang];                // 保存用于后续质量评估

        double tp_ANorm = getNorm(tp_A);                   // 仿真向量的 L2 范数
        // 计算共轭内积
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * actualA[i];
           // cout << conj(tp_A[i]) <<  endl;
           // cout << conj(tp_A[i]) * actualA[i] << endl;
        }
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm); // 归一化内积的模
        // 更新最大相似度及其对应角度
        if (tp_max < tp_diff)
        {
            tp_max = tp_diff;
            angle = Round3600(-ang);                       // 注意：取负角度（坐标系转换）
            tp_A2.clear();
            tp_A2 = tp_A;
        }
        diff[ang] = tp_diff * 100;                         // 相似度百分比

    }
    // 计算测向质量
    vector<double> tp_theory_diff;
    calPseudoByAmpPhase(tp_simulateA, tp_A2, tp_theory_diff);
    quality = getDoaMass(tp_theory_diff, diff, 0, 360);
}
/**
 * @brief 检查指定频率的幅度-相位仿真数据文件是否存在
 *
 * 仿真数据文件的命名规则为：
 *   - 幅度文件：{path}A-{频率Hz}.csv
 *   - 相位文件：{path}P-{频率Hz}.csv
 *
 * 此函数检查幅度文件是否存在（幅度和相位文件通常成对出现，
 * 通过检查幅度文件即可判断两者是否都存在）。
 *
 * @param path 仿真数据文件的目录路径
 * @param f 信号频率（Hz），用于构造文件名
 * @return true 文件存在并可打开，false 文件不存在
 */
bool ArithmeticDoa::existAmpPhsFile(const string path, const double f)
{
        bool tp_f = true;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(0); // 频率整数化显示
        ss << f;
        string path1 = ss.str();
        string csv = ".csv";
        string Amp_File_Path = "";

        Amp_File_Path.append(path);
        Amp_File_Path.append("A-");
        Amp_File_Path.append(path1);
        Amp_File_Path.append(csv);
        ifstream in(Amp_File_Path);
       // tp_f = in.is_open();
        return in.is_open();

}
/**
 * @brief 读取仿真数据并构建完整的复数阵列流型矩阵
 *
 * 处理流程：
 *   1. 查找最接近目标频率 f 的仿真数据文件：
 *      - 仿真数据通常以特定频率间隔存储（如每 10kHz 一组），
 *        不太可能恰好有目标频率的数据文件
 *      - 从目标频率开始，以 10kHz 为步长向外搜索，找到最近的存在文件
 *      - 最大搜索 10000 次，对应约 100MHz 范围
 *
 *   2. 读取找到的幅度数据（dB 值）和相位数据（度值）
 *
 *   3. 数据格式转换：
 *      - 幅度：dB 转线性值 -- linear = 10^(dB/20)
 *      - 相位：取反（适配坐标系约定，仿真数据的相位定义为入射方向）
 *
 *   4. 对每个角度（0-359 度），调用 getA 组合幅度和相位为复数阵列流型
 *
 * @param path 仿真数据文件目录路径
 * @param f 目标信号频率（Hz）
 * @param simulateA 输出的仿真阵列流型矩阵（360 x 天线数，复数）
 */
void ArithmeticDoa::calAmpPhaseSimulateA(const string path, const double f, vector<vector<complex<double>>> &simulateA) // 计算仿真阵列流型
{
    int num1 = 0;
    bool tp_bool = true;
    double tp_f1 = f;
    double tp_f = f;
    int diff = 10e3; // 频率搜索步长 10kHz
    // 搜索最接近目标频率的仿真数据文件
    while (num1<10000 ) // 最多搜索 10000 步
    {
        tp_f1 = f + diff*num1; // 向上搜索：f + 10kHz*num1
        tp_bool = existAmpPhsFile(path, tp_f1);
        if (tp_bool)
        {
            tp_f = tp_f1;
            break;
        }
        tp_f1 = f - diff * num1; // 向下搜索：f - 10kHz*num1
        tp_bool = existAmpPhsFile(path, tp_f1);
        if (tp_bool)
        {
            tp_f = tp_f1;
            break;
        }
        num1++;
    }
    simulateA.clear();
    // 读取幅度数据（dB 值）
    vector<vector<double>> AmpData;
    int flg_amp = getSimulateAmp(path, tp_f, AmpData);
    // 读取相位数据（度值）
    vector<vector<double>> PhsData;
    int flg_phase = getSimulatePhase(path, tp_f, PhsData);
    if (0 != flg_amp || 0 != flg_phase)
    {
        Log("*** error:simulate data is error!!!\n");
        return;
    }
    // 检查幅度和相位数据的维度是否一致
    if (PhsData.size() != AmpData.size() || PhsData[0].size() != AmpData[0].size())
    {
        Log("*** error:simulate data is error!!!\n");
        return;
    }
    int num = PhsData[0].size(); // 天线数量
    vector<double> tp_amp;
    vector<double> tp_phase;
    vector<complex<double>> tp_A;
    tp_amp.resize(num);
    tp_phase.resize(num);
    simulateA.resize(360);
    // 对每个角度，组合复数阵列流型
    for (int i = 0; i < (int)PhsData.size(); i++)
    {
        for (int j = 0; j < num; j++)
        {
            // 幅度转换：dB -> 线性值（10^(dB/20)）
            tp_amp[j] = pow(10, (AmpData[i][j] / 20));
            // tp_amp[j] = AmpData[i][j];
            // tp_phase[j] = PhsData[i][j] * PI / 180;
            // 相位取反：仿真数据中相位定义为入射方向，阵列流型中需要取反
            tp_phase[j] = -PhsData[i][j];
        }
        tp_A.clear();
        getA(tp_amp, tp_phase, tp_A); // 组合幅度和相位为复数
        simulateA[i] = tp_A;          // 存储当前角度的阵列流型
    }
}

/**
 * @brief 将幅度和相位组合为复数形式的阵列流型
 *
 * 阵列流型（Array Manifold）描述了天线阵列在不同来波方向下的复数响应。
 * 每个元素为：A[i] = amp[i] * exp(j * phase[i])
 *
 * 物理含义：A[i] 的模 = 天线 i 的增益（幅度响应），
 *          A[i] 的幅角 = 天线 i 的相位延迟（相位响应）。
 *
 * @param amp 幅度数组（线性值，非 dB）
 * @param phase 相位数组（弧度）
 * @param A 输出的复数阵列流型向量
 */
void ArithmeticDoa::getA(const vector<double> amp, const vector<double> phase, vector<complex<double>> &A) // 计算阵列流型
{
    int size = amp.size();
    A.clear();
    A.resize(size);
    for (int i = 0; i < size; i++)
    {
        // 利用欧拉公式构造复数：amp * (cos + j*sin) = amp * exp(j*phase)
        complex<double> tp(cos(phase[i]), sin(phase[i]));
        A[i] = amp[i] * tp;
    }
}

/**
 * @brief 生成虚拟阵元的一维编号（二维编码的哈希映射）
 *
 * 编码规则：virtualAntNum = ant1 * 10 + ant2
 * 含义：将天线对 (ant1, ant2) 编码为一个整型编号，用作虚拟理论相位矩阵的列索引。
 * 例如：天线对 (3, 5) -> 编号 35；天线对 (5, 3) -> 编号 53。
 * 注意此编码不是对称的，(3,5) 和 (5,3) 的编号不同，这是有意为之，
 * 因为 (A,B) 和 (B,A) 对应不同的虚拟阵元位置和相位关系。
 *
 * @param ant1 第一天线编号（从 1 开始）
 * @param ant2 第二天线编号（从 1 开始）
 * @return 虚拟阵元编号，用于线性化存储
 */
int ArithmeticDoa::getVirtualAntNum(const int ant1, const int ant2) // 生成虚拟阵元的编号
{
    return ant1 * 10 + ant2;
}

/**
 * @brief 从仿真数据文件中读取指定频率的相位数据
 *
 * 文件名格式：{path}P-{频率Hz}.csv
 * 文件内容：每行以角度（度）开头，后跟各天线的相位值（度），以逗号分隔。
 * 读取后自动将相位从度转换为弧度。
 *
 * @param path 仿真数据文件目录路径
 * @param f 信号频率（Hz），用于构造文件名
 * @param phase 输出的相位矩阵，phase[ang][i] = 角度 ang 处天线 i 的相位（弧度）
 * @return 0 成功，-1 文件读取或格式错误
 */
int ArithmeticDoa::getSimulatePhase(const string path, const double f, std::vector<std::vector<double>> &phase) // 获得阵列仿真数据的相位
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(0);
    ss << f;
    string path1 = ss.str();
    string csv = ".csv";
    //  string path = "/simulateData/";
    string Phase_File_Path = ""; // m_CurrentPath;
    Phase_File_Path.append(path);
    Phase_File_Path.append("P-");
    Phase_File_Path.append(path1);
    Phase_File_Path.append(csv);
    // vector<vector<double>> PhsData;
    // const char *phase_path = Phase_File_Path.c_str();
    phase.clear();
    int flg = getModeData(Phase_File_Path, phase); // 读取 CSV 文件
    if (0 != flg)
    {
        Log("***error:get simulate phase data  is error:%s\n", Phase_File_Path.c_str());
        cout << "***error:get simulate phase data is error!!!" << endl;
        phase.clear();
        return -1;
    }
    // 将相位从度转换为弧度
    for (int i = 0; i < (int)phase.size(); i++)
    {
        for (int j = 0; j < (int)phase[i].size(); j++)
        {
            phase[i][j] = phase[i][j] * m_PI / 180; // 度转弧度
        }
    }
    // phase = PhsData;
    return 0;
}

/**
 * @brief 从仿真数据文件中读取指定频率的幅度数据
 *
 * 文件名格式：{path}A-{频率Hz}.csv
 * 文件内容：每行以角度（度）开头，后跟各天线的幅度值（dB），以逗号分隔。
 * 注意：读取的幅度为 dB 值，需在调用方通过 10^(dB/20) 转换为线性值后使用。
 *
 * @param path 仿真数据文件目录路径
 * @param f 信号频率（Hz），用于构造文件名
 * @param amp 输出的幅度矩阵，amp[ang][i] = 角度 ang 处天线 i 的幅度（dB）
 * @return 0 成功，-1 文件读取或格式错误
 */
int ArithmeticDoa::getSimulateAmp(const string path, const double f, std::vector<std::vector<double>> &amp) // 获得阵列仿真数据的幅度
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(0);
    ss << f;
    string path1 = ss.str();
    string csv = ".csv";
    string Amp_File_Path = "";

    Amp_File_Path.append(path);
    Amp_File_Path.append("A-");
    Amp_File_Path.append(path1);
    Amp_File_Path.append(csv);

    int flg = getModeData(Amp_File_Path, amp); // 读取 CSV 文件
    if (flg != 0)
    {
        Log(" error:get simulate amp data  is error!!!\n");
        cout << "error:get simulate amp data  is error!!!" << endl;
        return -1;
    }
    return 0;
}

/**
 * @brief 从 CSV 文件中读取模板（仿真）数据
 *
 * 文件格式：每行一个角度，格式为 "角度, 值1, 值2, ..."
 * 例如 "0, -1.2, 0.5, 2.1" 表示在角度 0 度时，各天线/参数的值分别为 -1.2, 0.5, 2.1。
 *
 * 角度列用于确定该行数据对应的方向（支持任意角度，不要求按顺序排列），
 * 通过 Round360 归一化到 [0, 360) 范围后进行索引。
 *
 * @param path 模板数据文件的完整路径
 * @param data 输出的模板数据矩阵，data[ang][j] = 角度 ang 处的第 j 个参数值
 * @return 0 成功，非 0 文件读取失败
 */
int ArithmeticDoa::getModeData(const string path, vector<vector<double>> &data) // 获取模板数据
{
    vector<string> tpVec;
    tpVec.clear();
    int flg = 0;
    flg = readFile(path, tpVec); // 读取文件内容到字符串数组

    if (0 != flg)
    {
        Log("***error:get sigle channel error paranmeter adr is false!!!\n");
        cout << "***error:get sigle channel error paranmeter adr is false!!!" << endl;
        return flg;
    }
    int num = tpVec.size();
    data.clear();
    data.resize(360); // 预分配 360 个角度的空间
    for (int i = 0; i < num; i++)
    {
        string &a1 = tpVec[i];
        vector<string> a2;
        a2.clear();
        vector<string>().swap(a2);
        split(a2, a1, ','); // 按逗号分割行
        int ang = Round360(stoi(a2[0])); // 第一列为角度（度），归一化到 [0, 360)
        for (int j = 1; j < (int)a2.size(); j++)
        {
            data[ang].emplace_back(stod(a2[j])); // 后续列为数据值
        }
    }
    return 0;
}

/**
 * @brief 计算测向质量（区域版本）-- 基于两个伪谱的归一化互相关系数
 *
 * 测向质量计算公式：
 *   corr = sum(diffTheory[i] * diff[i]) / (sqrt(sum(diffTheory^2)) * sqrt(sum(diff^2)))
 *   quality = (corr + 1) / 2 * 100
 *
 * corr 为两个向量的归一化互相关系数（皮尔逊相关系数的简化形式，假设无 DC 偏移），
 * 取值范围 [-1, 1]。映射到 [0, 100] 后：
 *   - quality = 100：两个伪谱完全正相关（形状完全一致，最可靠）
 *   - quality = 50： 两个伪谱无相关性（随机匹配）
 *   - quality = 0：  两个伪谱完全负相关（形状完全相反，最不可靠）
 *
 * @param diffTheory 理论伪谱（最优匹配角度下的理论值构成的伪谱）
 * @param diff 实际伪谱（实测数据与各角度理论值匹配的伪谱）
 * @param starAngle 搜索起始角度（度）
 * @param endAngle 搜索结束角度（度）
 * @return 测向质量值（0-100）
 */
double ArithmeticDoa::getDoaMass(const vector<double> diffTheory, const vector<double> diff, int starAngle, int endAngle)
{
    double quality;
    int size = (int)diffTheory.size();
    double tp_sum = 0.0;        // sum(diffTheory * diff) -- 叉积和
    double tp_theory_sum = 0.0; // sum(diffTheory^2)   -- 理论伪谱的能量
    double tp_diff_sum = 0.0;   // sum(diff^2)         -- 实际伪谱的能量
    int tp_ang = 0;
    // 在指定的角度范围内累加
    for (int i = starAngle; i < endAngle + 1; i++)
    {
        tp_ang = Round360(i);
        tp_sum = tp_sum + diffTheory[tp_ang] * diff[tp_ang];
        tp_theory_sum = tp_theory_sum + diffTheory[tp_ang] * diffTheory[tp_ang];
        tp_diff_sum = tp_diff_sum + diff[tp_ang] * diff[tp_ang];
    }
    // 归一化因子 = sqrt(sum(x^2)) * sqrt(sum(y^2))
    double tp_mode_sum = sqrt(tp_theory_sum) * sqrt(tp_diff_sum);
    if (tp_mode_sum == 0) // 防止除零
    {
        quality = 0;
    }
    else
    {
        // corr = tp_sum / tp_mode_sum，取值范围 [-1, 1]
        // quality = (corr + 1) / 2 * 100，映射到 [0, 100]
        quality = (tp_sum / tp_mode_sum + 1) / 2 * 100;
    }

    return quality;
}

/**
 * @brief 计算测向质量（全局版本）-- 在全部角度范围内计算相关系数
 *
 * 与区域版本的算法完全相同，区别仅在于累加角度范围为 [0, 359]。
 *
 * @param diffTheory 理论伪谱（360 点）
 * @param diff 实际伪谱（360 点）
 * @return 测向质量值（0-100）
 */
double ArithmeticDoa::getDoaMass(const vector<double> diffTheory, const vector<double> diff) // 获得测向质量
{
    double quality;
    int size = (int)diffTheory.size();
    double tp_sum = 0.0;        // sum(diffTheory * diff)
    double tp_theory_sum = 0.0; // sum(diffTheory^2)
    double tp_diff_sum = 0.0;   // sum(diff^2)
    // 在全部 360 个角度范围内累加
    for (int i = 0; i < size; i++)
    {
        tp_sum = tp_sum + diffTheory[i] * diff[i];
        tp_theory_sum = tp_theory_sum + diffTheory[i] * diffTheory[i];
        tp_diff_sum = tp_diff_sum + diff[i] * diff[i];
    }
    // 计算归一化互相关系数
    double tp_mode_sum = sqrt(tp_theory_sum) * sqrt(tp_diff_sum);
    if (tp_mode_sum == 0) // 防止除零
    {
        quality = 0;
    }
    else
    {
        quality = (tp_sum / tp_mode_sum + 1) / 2 * 100; // 映射到 [0, 100]
    }

    return quality;
}


// =============================================================================
// == 原 SpoofingDoa.cpp —— 核心引擎实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: SpoofingDoa.cpp
// 功能描述: 欺骗干扰测向(DOA)主类的实现文件
// 系统角色: 本文件实现了欺骗测向系统的核心调度逻辑，包括:
//           1. 构造函数:初始化项目参数、频率表、检测阈值、理论相位差模板
//           2. setGNSSData:数据分发入口，根据dataLen区分检测/测向/校正模式
//           3. getAngleSpoofingDoa:输出最终测向结果，支持伪谱积分和外部筛选
//           4. setDataAngle:测向主流程(相位差计算→校正→检测→干涉仪/幅相法测向)
//           5. setDataAlarm:欺骗检测模式流程(相位差计算→按频点分类→三方法检测)
//           6. calAngle:干涉仪测向汇总(按频点逐星计算+跳半周检测与修复)
//           7. 跳半周检测与优化:检测相位跳变并使用全排列组合进行修复
// 配置说明: 各运行参数(阵列/切刀/阈值/流程开关)直接在 Init() 中内置, 不读取外部配置文件
// =============================================================================


// 欺骗测向算法对象构造函数
// 初始化流程: initProject(设置硬件参数) -> Init(内置运行参数+频率表+理论模板)
SpoofingDoa::SpoofingDoa(void){
    initProject();
    Init();
    // m_Phasediff_Threshold = m_Phasediff_Threshold / 360.0; // 将度转化为周
    // 部分频点的检测阈值单独设置，与 Python detection_lib.ORIGINAL_CONFIG_TEXT 规则一致
    // （2026-09 对齐：GLONASS G1/G2、BDS B1C 的颗数阈值已按 Python 修正）
    // 判定为严格大于（bestCount > m_Detection_Threshold），对应 Python 的 cnt > count_thr
    // 实参顺序固定为: (系统sys, 频点type, 卫星颗数阈值threshold, 相位差阈值phsThreshold)
    setThresholdDetectionDoa(0, 2, 4, -1);    // GPS L5:     Python Sys=0,Type=2  count=4(触发≥5)
    setThresholdDetectionDoa(1, 0, 2, 5.0);   // GLONASS G1: Python Sys=1,Type=0  count=2(触发≥3) phs=5.0
    setThresholdDetectionDoa(1, 1, 3, 5.0);   // GLONASS G2: Python Sys=1,Type=1  count=3(触发≥4) phs=5.0
    setThresholdDetectionDoa(3, 2, 3, 3.6);   // Galileo E1C: Python Sys=3,Type=2 count=3(触发≥4) phs=3.6
    setThresholdDetectionDoa(3, 12, 4, 3.6);  // Galileo E5a: Python Sys=3,Type=12 count=4(触发≥5) phs=3.6
    setThresholdDetectionDoa(3, 17, 4, 3.6);  // Galileo E5b: Python Sys=3,Type=17 count=4(触发≥5) phs=3.6
    setThresholdDetectionDoa(4, 17, 3, -1);   // BDS B2I:     Python Sys=4,Type=17 count=3(触发≥4)
    setThresholdDetectionDoa(4, 0, 3, -1);    // BDS B1I:     Python Sys=4,Type=0  count=3(触发≥4)
    setThresholdDetectionDoa(4, 2, 3, -1);    // BDS B3I:     Python Sys=4,Type=2  count=3(触发≥4)
    setThresholdDetectionDoa(4, 8, 2, -1);    // BDS B1C:     Python Sys=4,Type=8  count=2(触发≥3)（新增）
    setThresholdDetectionDoa(4, 19, 3, -1);   // BDS B2b:     Python Sys=4,Type=19 count=3(触发≥4)
    setThresholdDetectionDoa(4, 34, 3, 3.6);  // BDS B1X:     Python Sys=4,Type=34 count=3(触发≥4) phs=3.6
    // setTypeDetectionBySnr(1, 1, 0);
}


// 完整初始化函数(可重复调用)
// 初始化顺序:
// 1. 清空累积数据(伪谱积分、校正数据、历史记录)
// 2. 内置运行参数(原 DoaBSpoofingConfig.txt 内容, 直接内嵌于本函数)
// 3. 初始化GNSS频点频率映射表(initType)
// 4. 设置各频点阵列半径(setR)
// 5. 初始化检测阈值(initDetectionThreshold)
// 6. 根据算法类型初始化理论模板(理论相位差/仿真阵列流型)
void SpoofingDoa::Init(void){

    // 重置伪谱积分累积器(360个角度bin清零)
    m_Angle_Accumulate.clear();
    m_Angle_Accumulate.resize(360);
    std::fill(m_Angle_Accumulate.begin(), m_Angle_Accumulate.end(), 0.0);

    // 校正数据清零
    m_CorrectionData.clear();
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
    // 跳半周检测历史数据清零
    m_infoData180.clear();
    std::map<int, std::map<int, InterferInfo>>().swap(m_infoData180);

    // =========================================================================
    // 内置运行参数(不读取外部配置文件)
    // 以下取值与原始 DoaBSpoofingConfig.txt(2026-09 快照) 逐项一致，直接内嵌在代码中，
    // 消除运行时对配置文件的依赖；需要调整参数时改这里即可(括号内为原配置文件 key)。
    // 说明: initProject() 默认走 GN930U 分支(半径0.2/切刀{7,7}...)，此处按下方的
    //       GN902 配置覆盖，之后 setR()/initTheory() 按覆盖后的 m_omni_R 重建理论模板。
    // =========================================================================

    // ---- 日志: logAdr / logFlg ----
    m_LogFile = "./spoofingDoaLog_";
    PublicSpace::m_logFlg = 0;   // 0=关闭日志
    LogCreat(m_LogFile);

    // ---- 阵列物理参数: antennaNum / antennaType / r ----
    m_AntennaNum = 7;            // 天线阵元个数
    m_antnenaType = 0;           // 0=全向天线, 1=定向天线
    m_omni_R = 0.1865;           // 全向天线阵列半径(米) = GN902 硬件参数(与 Python ARRAY_RADIUS 一致)
    m_Radr.clear();              // 定向天线半径文件路径(全向天线不使用)

    // ---- 切刀顺序: cutThw ----
    // {1,1} = 自校准刀(天线对相同，用于通道校正，不参与测向)，
    // {1,2}~{1,7} = 六刀测向基线(对应 Python code 0/9/57/17/25/33/1)。
    m_cutSequence = { {1,1}, {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7} };

    // ---- 平滑/帧数: cutFrams / smoothFlg ----
    m_OneCut_Frams = 8;          // 一刀帧数(循环切刀检测每刀 8 帧, 与 dat 8 帧/刀一致)
    m_Smooth_Flag = 1;           // 1=多帧平滑(稳定性过滤+圆形均值)

    // ---- 测向刀数/最少相位差数: doaCutNum / doaMinCutNum ----
    m_Doa_Cut_Num = 6;
    m_Doa_Cut_min_Num = 6;
    if (m_Doa_Cut_min_Num >= (int)m_cutSequence.size()) {
        m_Doa_Cut_min_Num = (int)m_cutSequence.size() - 1;  // 自动限制不超过总刀数-1
    }

    // ---- 检测门限: angleThreshold / phaseDiffThreshold / detectionNum / snrThreshold ----
    m_Angle_Threshold = 3.0;     // 同一方向判定阈值(度)
    m_Phasediff_Threshold = 5.0; // 相位差相近判定阈值(度)
    m_Detection_Threshold_Num = 2; // 卫星颗数默认阈值(严格大于判定, 触发≥3)
    m_Snr_Threshold = 35.0;      // 载噪比门限(dB-Hz), 低于该值不参与处理
    m_Qulity_Threshold = 0.0;    // 测向质量门限

    // ---- 流程开关 ----
    m_Cyclic_Detection_Flag = 1; // cyclicDetectionFlag: 1=循环切刀检测(每刀聚类+跨刀连续确认+测向跟踪)
    m_Doa_Detection_Flag = 0;    // doaDetectionFlag: 0=测向中不额外做欺骗检测(循环流程覆盖)
    m_Doa_Arithmetic = 1;        // doaArithmetic: 1=相关干涉仪
    m_PseudoSpectrum_Flag = 0;   // pseudoSpectrumFlag: 0=不做伪谱积分
    m_Screen_detection_Flag = 0; // screendetectionFlag: 0=不按外部欺骗结果筛选
    m_Secondary_Doa_Flag = 0;    // secondaryDoaFlag: 0=进行二次确认测向
    m_Delete_Prn_Flag = 1;       // deletePrnFlag: 1=删除存在缺失刀数据的卫星
    m_getUseAntennaBySnr_Flag = 0; // getUseAntennaBySnrFlag: 0=不用载噪比定测向天线序号

    // ---- 虚拟阵列: virtualFlag / virtualMultiple ----
    m_Virtual_Flag = 0;          // 0=不使用虚拟阵列
    m_Virtual_Multiple = 0.94;   // 虚拟阵列扩展倍数(未使用)

    // ---- 跨刀连续确认: detectionRecoddsNum ----
    m_Detection_Recodds_Num = 2; // 连续多少刀判为欺骗才最终确认(对应 Python ALARM_CONSECUTIVE_P)

    // ---- 跳半周修复阈值: detection180QulityThreshold ----
    m_Detection180_Qulity_Threshold = 10.0;

    // ---- 数据保存: saveOriginalFlg / saveDataFlg / accumulateMultiplier ----
    m_Save_Original_Flg = 0;     // 0=不保存原始GNSS数据
    PublicSpace::m_save_data_Flg = 0; // 0=不保存数据
    m_Accumulate_multiplier = 0; // 伪谱积分衰减因子(未启用积分)

    // ---- 阵列仿真(仅 doaArithmetic=2/3 使用, 干涉仪+全向不依赖): simulateFile / SimulateFre ----
    m_Simulate_Data_file = "/simulateData/";
    m_All_Simulate_data_Fre = {1176e6, 1279e6, 1561e6, 1602e6};
    // 初始化GNSS频点频率映射表(m_F)
    initType();
    // 初始化循环切刀检测状态(连续报警计数与跟踪)
    resetCyclicDetection();
    // 设置各频点的阵列半径(m_R)
    setR(m_Radr);

    // 初始检测阈值设置(使用默认的卫星颗数阈值和相位差阈值)
    initDetectionThreshold(m_Detection_Threshold_Num, m_Phasediff_Threshold);

    // 默认所有频点不使用载噪比进行欺骗检测
    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        int intType = it->first;
        // -1 表示不使用载噪比检测欺骗信号
        m_Detection_snrThrehold[intType] = -1;
    }

    // 根据不同算法，初始化测向需要使用的模板
    // 注:三种算法互斥，仅初始化所选算法所需的模板数据
    if (1 == m_Doa_Arithmetic){
        initTheory();          // 干涉仪+理论相位:计算各频点的理论相位差模板
    }
    if (2 == m_Doa_Arithmetic){
        initSimulateA();       // 幅相法:加载仿真阵列流型
    }
    if (3 == m_Doa_Arithmetic){
        initTheoryBySimulatePhase(); // 干涉仪+阵列仿真相位:加载仿真相位差模板
    }
}

SpoofingDoa::~SpoofingDoa(void)
{
}


// =========================================================================
// 设置数据 - 系统主入口
// 根据dataLen区分三种运行模式:
//   dataLen=1: 欺骗检测模式(无校正) - 仅进行欺骗检测判断
//   dataLen=2: 欺骗检测模式(带校正) - 先计算校正数据再检测
//   dataLen>2: 完整测向模式 - 相位差计算→校正→检测→DOA计算
// 最后保存原始GNSS数据(可选)
// =========================================================================
void SpoofingDoa::setGNSSData(const GNSSData *data, int dataLen)
{
    // 检测数据
    // 若为检测数据，则设置为告警，并把结果写入到全局里面保存结果的数据中
    if (1 == dataLen) {
        PublicSpace::Log("star detection GNSSdata...\n");
        setDataAlarm(data[0]);
        m_Detection_Tag = 1;   // 标记为检测模式(不进行测向)
    }
    else if (2 == dataLen)
    {
        PublicSpace::Log("star detection GNSSdata & correction data...\n");
        setCorrectDetectionDataAlarm(data, dataLen);
        m_Detection_Tag = 1;   // 标记为检测模式
    }
    else
    {
        // 完整测向模式
        m_Detection_Tag = 0;   // 标记为测向模式
        int cutNum = (int)m_cutSequence.size();
        if (cutNum < 2)
        {
            PublicSpace::Log("error:cutSequence is mistake!!!\n");
            cout << "error:cutSequence is mistake!!!" << endl;
            return;
        }
        PublicSpace::Log("star Doa.........................\n");
        setDataAngle(data, dataLen);  // 执行完整测向流程
    }
    saveGNSSData(data, dataLen); // 保存原始数据(如果配置允许)
}

// =========================================================================
// 获取欺骗测向结果 - 主输出接口
// 流程: setSpoofingResult(汇总各频点测向结果) -> 外部结果筛选(可选)
//       -> 伪谱积分(可选) -> 日志输出
// =========================================================================
int SpoofingDoa::getAngleSpoofingDoa(SpoofingResult &result)
{
    // 汇总m_AngleResultData中的数据为SpoofingResult格式
    setSpoofingResult(result);
    // 根据外部欺骗检测结果筛选(如果启用了筛选功能)
    if (1 == m_Screen_detection_Flag && m_detectResult.size() > 0)
    {
        getSpoofingResultByDetection(result);
    }
    // 测向模式下输出非积分结果日志
    if (0 == m_Detection_Tag)
    {
        PublicSpace::Log("Not SpectrumDesity result:\n");
        LogSpoofingResult(result);
    }
    // 伪谱积分:累积多帧测向结果以提高稳定性
    if (1 == m_PseudoSpectrum_Flag && 0 == m_Detection_Tag)
    {
        setSpoofingResultPseudoSpectrumDesity(result);
    }

    PublicSpace::Log("Doa result:\n");
    LogSpoofingResult(result);
    return 0;
}

// =========================================================================
// 组装最终输出结果
// 将m_AngleResultData中每个频点的测向结果汇总为SpoofingResult
// 测向模式:直接输出每颗卫星的DOA，频点角度取各星DOA的圆形均值
// 检测模式:直接输出所有卫星的测向结果(角度可能为-1表示仅检测未测向)
// =========================================================================
void SpoofingDoa::setSpoofingResult(SpoofingResult &result)
{
    int count = 0;
    int typeInt = 0;
    double angle = 0.0;
    int prn = 0;
    // 遍历所有频点的测向结果
    for (auto it = m_AngleResultData.begin(); it != m_AngleResultData.end(); ++it)
    {
        typeInt = it->first;
        result.i_SatelliteAngle[count].i_Sys = typeInt / 100;   // 从编码恢复系统号
        result.i_SatelliteAngle[count].i_Type = typeInt % 100;  // 从编码恢复频点号
        result.i_SatelliteAngle[count].i_Alarm = 1;
        vector<AlarmData> tp = it->second;
        angle = -1;
        if (0 == m_Detection_Tag)
        {
            // 测向模式：直接输出每颗卫星的 DOA；频点角度取各星 DOA 的圆形均值
            // （原 calAlarmByAngle 角度聚类已按新流程移除，对应 Python circular_mean(doas)）
            vector<double> doas;
            doas.reserve(tp.size());
            for (unsigned int i = 0; i < tp.size(); i++)
            {
                doas.push_back((double)tp[i].i_Angle);
            }
            angle = tp.empty() ? -1.0 : circularMeanDeg(doas);
            if (angle < 0)
            {
                angle += 360.0;  // 归一化到 [0°, 360°)
            }
            result.i_SatelliteAngle[count].i_Angle = angle;
            result.i_SatelliteAngle[count].i_Count = (int)tp.size();
            for (unsigned int i = 0; i < tp.size(); i++)
            {
                prn = tp[i].i_Prn;
                result.i_SatelliteAngle[count].i_AlarmData[i] = tp[i];
                result.i_SatelliteAngle[count].i_AlarmData[i].i_Snr = m_Max_Snr[typeInt][prn];
            }
        }
        else
        {
            // 检测模式:无需角度聚类，直接输出所有结果
            result.i_SatelliteAngle[count].i_Angle = -1;
            result.i_SatelliteAngle[count].i_Count = (int)tp.size();
            for (unsigned int i = 0; i < tp.size(); i++)
            {
                prn = tp[i].i_Prn;
                result.i_SatelliteAngle[count].i_AlarmData[i] = tp[i];
                result.i_SatelliteAngle[count].i_AlarmData[i].i_Snr = m_Max_Snr[typeInt][prn];
            }
        }
        ++count;
    }
    result.i_Count = count; // 最终输出的频点个数
}


// =========================================================================
// 设置测向数据 - 完整DOA流程
// 这是测向模式的核心处理函数，完整流程:
// 1. 逐刀计算相位差(dataA)
// 2. 多帧数据平滑处理(可选)
// 3. GN930特殊处理:双板卡数据分离校正
// 4. 校正数据计算与相位校正
// 5. 欺骗信号检测与筛选(可选)
// 6. 相位差数据汇总为B格式(dataB)
// 7. 调用干涉仪或幅相法进行DOA计算
// =========================================================================
void SpoofingDoa::setDataAngle(const GNSSData *data, int dataLen)
{
    string nowT = getNowTime();

    PublicSpace::Log("cal phase diff cutNum:  %s \n", nowT.c_str());
    // 清空上一帧的伪谱值
    std::map<int, std::map<int, std::vector<double>>>().swap(m_Pseudo_Spectrum_Value);
    vector<vector<SatelliteDataPhaseDiffA>> dataA;
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);

    // 步骤1: 逐刀计算相位差
    for (int i = 0; i < dataLen; i++){
        PublicSpace::Log("cut: %d   PortOneNum: %d PortTwoNum: %d  \n", i + 1, data[i].i_PortOneNum, data[i].i_PortTwoNum);
        vector<SatelliteDataPhaseDiffA> tp;
        tp.clear();
        vector<SatelliteDataPhaseDiffA>().swap(tp);
        getSatelliteDataPhaseDiffA(data[i], tp);  // 计算两通道相位差
        dataA.emplace_back(tp);
        LogGNSSData(data[i], i + 1);              // 记录原始数据日志
    }

    // 日志输出B格式数据(调试用)
    if (0 != m_logFlg){
        vector<SatelliteDataPhaseDiffB> tpB;
        tpB.clear();
        getSatelliteDataPhaseDiffB(dataA, tpB);
        LogSatelliteDataPhaseDiffB(tpB);
    }

    // 步骤2: 多帧数据平滑处理
    if (m_OneCut_Frams > 1 && 1 == m_Smooth_Flag){
        nowT = getNowTime();
        PublicSpace::Log("get smooth data:  %s \n", nowT.c_str());
        // 多帧数据平滑,注意：计算相位的方式存在歧义，若启用该方法，则需注意验证
        getSmoothData(dataA);
    }

    // 若是有多帧数据，但不需要平滑，则取最后一帧数据
    if (m_OneCut_Frams > 1 && 0 == m_Smooth_Flag)
    {
        PublicSpace::Log("get end fram data:  %s \n", nowT.c_str());
        getEndFramData(dataA);
    }

    // 步骤3-4: 校正数据处理
    // GN930特殊处理:双板卡(两个独立接收通道)，需要分离两组校正数据
    if (m_Project_flg == GN930)
    {
        int cutNum = 4;
        vector<vector<SatelliteDataPhaseDiffA>> dataA1;
        dataA1.clear();
        vector<vector<SatelliteDataPhaseDiffA>> dataA2;
        dataA2.clear();
        dataA1.resize(cutNum);
        dataA2.resize(cutNum);
        vector<vector<int>> cutSequence_all;
        cutSequence_all = m_cutSequence;
        vector<vector<int>> cutSequence1;
        vector<vector<int>> cutSequence2;
        cutSequence1.resize(cutNum);
        cutSequence2.resize(cutNum);
        // 将数据和切刀顺序分为两组(对应两个板卡)
        for (int k = 0; k < cutNum; k++)
        {
            dataA1[k] = dataA[k];
            dataA2[k] = dataA[k + cutNum];
            cutSequence1[k] = m_cutSequence[k];
            cutSequence2[k] = m_cutSequence[k + cutNum];
        }
        nowT = getNowTime();
        // 处理第一个板卡的校正数据
        m_CorrectionData.clear(); // 校正数据清零,防止上一帧数据另一个板卡的校正数据有残留
        std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
        m_cutSequence.clear();
        vector<vector<int>>().swap(m_cutSequence);
        m_cutSequence = cutSequence1;
        PublicSpace::Log("get Correct Data 1:  %s \n", nowT.c_str());
        setCorrectionData(dataA1);    // 设置校正数据(从同天线功分通道计算)
        getCorrectedGnssData(dataA1); // 获得校正后的数据

        // 处理第二个板卡的校正数据
        m_CorrectionData.clear();
        std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
        m_cutSequence.clear();
        vector<vector<int>>().swap(m_cutSequence);
        m_cutSequence = cutSequence2;
        nowT = getNowTime();
        PublicSpace::Log("get Correct Data 2:  %s \n", nowT.c_str());
        setCorrectionData(dataA2);
        nowT = getNowTime();
        PublicSpace::Log("get Corrected Gnss Data:  %s \n", nowT.c_str());
        getCorrectedGnssData(dataA2);
        // 恢复完整切刀顺序
        m_cutSequence.clear();
        m_cutSequence = cutSequence_all;
        // 合并两组数据
        dataA.clear();
        vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
        dataA.resize(cutNum * 2);

        for (int k = 0; k < cutNum; k++)
        {
            dataA[k] = dataA1[k];
            dataA[k + cutNum] = dataA2[k];
        }

        cutSequence_all.clear();
        dataA1.clear();
        dataA2.clear();
        cutSequence1.clear();
        cutSequence2.clear();
    }
    else
    {
        // 非GN930:正常校正流程
        nowT = getNowTime();
        PublicSpace::Log("get Correct Data:  %s \n", nowT.c_str());
        setCorrectionData(dataA);    // 设置校正数据
        nowT = getNowTime();
        PublicSpace::Log("get Corrected Gnss Data:  %s \n", nowT.c_str());
        getCorrectedGnssData(dataA); // 获得校正后的数据
    }

    // 步骤5: 欺骗信号检测与筛选(可选)
    if (m_Cyclic_Detection_Flag != 0)
    {
        nowT = getNowTime();
        PublicSpace::Log("get Cyclic Detection Data:  %s \n", nowT.c_str());
        getCyclicDetectionData(dataA); // 循环切刀检测：每刀聚类 + 跨刀连续确认 + 跟踪
    }
    else if (m_Doa_Detection_Flag != 0)
    {
        nowT = getNowTime();
        PublicSpace::Log("get Spoofing Detection Data:  %s \n", nowT.c_str());
        getSpoofingDetectionData(dataA); // 对每刀数据进行欺骗检测，只保留为欺骗信号的数据
    }

    // 步骤6: 相位差数据汇总为B格式(dataB)
    vector<SatelliteDataPhaseDiffB> dataB;
    dataB.clear();
    getSatelliteDataPhaseDiffB(dataA, dataB);
    LogSatelliteDataPhaseDiffB(dataB);

    // 步骤6.5: 跨周期基线累积(循环检测)：本轮相位差合并进持久基线，缺刀位复用
    // 上一周期的相位差，凑齐 6 条基线做测向(对应 Python baselines 跨周期复用)。
    vector<SatelliteDataPhaseDiffB> doaDataB;
    doaDataB.clear();
    if (m_Cyclic_Detection_Flag != 0)
    {
        accumulateBaselines(dataB);      // 本轮相位差合并进持久基线
        getCrossCycleDataB(doaDataB);    // 取当前跟踪中(系统,频点)的累积基线做测向
    }
    else
    {
        doaDataB = dataB;                // 非循环检测：保持单轮测向原行为
    }

    // 步骤7: 根据算法类型调用对应的DOA计算
    if (1 == m_Doa_Arithmetic || 3 == m_Doa_Arithmetic){

        if (0 == m_Theory.size()){
            cout << "error:theory phase diff have not!!!!" << endl;
            return;
        }

        getResultInterferDoa(doaDataB);  // 相关干涉仪测向
    }

    if (2 == m_Doa_Arithmetic){

        if (0 == m_SimulateA.size()){
            cout << "error:SimulateA have not!!!!" << endl;
            return;
        }

        getResultAmpPhaseDoa(doaDataB);  // 幅相法测向
    }

    // 步骤8: 更新循环切刀跟踪状态中的最近一次 DOA（对应 Python tracking 的 last_doa）
    if (m_Cyclic_Detection_Flag != 0)
    {
        for (auto &tp : m_Tracking)
        {
            int typeInt = tp.first;
            TrackingInfo &t = tp.second;

            if (m_AngleResultData.find(typeInt) != m_AngleResultData.end() && !m_AngleResultData[typeInt].empty())
            {
                // 本轮仍报警：更新最近一次 DOA（取各星 DOA 圆形均值）
                vector<double> doas;
                double sumQ = 0.0;
                for (auto &ad : m_AngleResultData[typeInt])
                {
                    doas.push_back((double)ad.i_Angle);
                    sumQ += ad.i_Quality;
                }
                if (!doas.empty())
                {
                    double mean = circularMeanDeg(doas);
                    if (mean < 0)
                    {
                        mean += 360.0;  // 归一化到 [0°,360°)
                    }
                    t.doa_deg = mean;
                    t.quality = sumQ / doas.size();
                }
            }
            else if (m_ConsecutiveAlarm[typeInt] == 0 && t.doa_deg >= 0.0)
            {
                // 本轮已消失：把最近一次 DOA 作为该频点的报警结果保留在 m_AngleResultData
                vector<AlarmData> kept;
                for (int sid : t.cluster_sats)
                {
                    AlarmData ad;
                    ad.i_Prn = sid;
                    ad.i_Angle = Round360((int)t.doa_deg);
                    ad.i_Quality = t.quality;
                    ad.i_Snr = 0.0f;
                    if (m_Max_Snr.find(typeInt) != m_Max_Snr.end() && m_Max_Snr[typeInt].find(sid) != m_Max_Snr[typeInt].end())
                    {
                        ad.i_Snr = (float)m_Max_Snr[typeInt][sid];
                    }
                    kept.emplace_back(ad);
                }
                m_AngleResultData[typeInt] = kept;
            }
        }
    }
}


// =========================================================================
// 设置告警数据 - 欺骗检测模式(无校正)
// 流程: GNSSData -> 相位差计算 -> 按频点分类 -> 相位差法检测(getAlarm)
// 结果存储在 m_AngleResultData 中
// =========================================================================
void SpoofingDoa::setDataAlarm(const GNSSData data)
{
    string nowT = getNowTime();

    vector<SatelliteDataPhaseDiffA> dataA;
    getSatelliteDataPhaseDiffA(data, dataA);  // 计算相位差

    std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;

    //将设置的告警数据和id绑定(按频点分类)
    getSatelliteDataByType(dataA, dataT);
    PublicSpace::Log("set detection spoofing data:  %s  \n", nowT.c_str());

    LogSatelliteDataPhaseDiffType(dataT);
    m_AngleResultData.clear();
    getAlarm(dataT);  // 执行欺骗检测(相位差法+载噪比法)
}



// =========================================================================
// 带校正数据的欺骗检测模式
// 流程: GNSSData[0]用于计算校正数据 -> GNSSData[1]校正后用于欺骗检测
// 适用于需要消除通道差异的精确检测场景
// =========================================================================
void SpoofingDoa::setCorrectDetectionDataAlarm(const GNSSData *data, int dataLen)
{
    string nowT = getNowTime();
    vector<vector<SatelliteDataPhaseDiffA>> dataA;
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA.resize(dataLen);
    for (int i = 0; i < dataLen; i++)
    {
        vector<SatelliteDataPhaseDiffA> tp;
        tp.clear();
        vector<SatelliteDataPhaseDiffA>().swap(tp);
        getSatelliteDataPhaseDiffA(data[i], tp);
        dataA[i] = tp;
        LogGNSSData(data[i], i + 1);
    }
    // 处理校正数据(GNSSData[0] = 同天线自校准刀 = Python code=0)
    vector<vector<SatelliteDataPhaseDiffA>> calCuts;
    calCuts.emplace_back(dataA[0]);
    calCorrectionOffset(calCuts);  // 计算校正偏移(对应 Python compute_calibration)

    PublicSpace::Log("correction data:  %s \n", nowT.c_str());
    // 对第二帧数据(检测用数据)进行校正
    vector<vector<SatelliteDataPhaseDiffA>> dataA2;
    dataA2.resize(1);
    dataA2[0] = dataA[1];
    nowT = getNowTime();
    getCorrectedGnssData(dataA2);
    // 校正后的数据进行欺骗检测
    vector<SatelliteDataPhaseDiffA> dataA3;
    dataA3 = dataA2[0];
    std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
    getSatelliteDataByType(dataA3, dataT);
    PublicSpace::Log("set detection spoofing data:  %s \n", nowT.c_str());
    LogSatelliteDataPhaseDiffType(dataT);
    m_AngleResultData.clear();
    getAlarm(dataT);
}

// =========================================================================
// 检测是否存在跳半周现象
// 原理: 比较当前帧与历史帧的同一卫星、同一天线对的相位差
//       如果余弦值 < -0.95 (即角度差接近180度)，判定为跳半周
// 返回: 1=存在跳半周(并自动修复info中的相位差), 0=正常
// =========================================================================
int SpoofingDoa::getDetection180(int typeInt, int prn, InterferInfo &info)
{
    int flg = 0;
    std::map<int, InterferInfo> tp_prn_info;
    InterferInfo tp_info;
    if (m_infoData180.find(typeInt) == m_infoData180.end())
    {
        // 首次出现此频点，保存当前数据作为参考
        m_infoData180[typeInt][prn] = info;
    }
    else
    {
        tp_prn_info = m_infoData180[typeInt];
        if (tp_prn_info.find(prn) != tp_prn_info.end())
        {
            tp_info = tp_prn_info[prn];
            // 遍历所有天线对，寻找匹配对并检查相位差变化
            for (int i = 0; i < tp_info.i_Phase_Len; i++)
            {

                for (int j = 0; j < info.i_Phase_Len; j++)
                {
                    if (tp_info.i_AntennaSq[i][0] == info.i_AntennaSq[j][0] && tp_info.i_AntennaSq[i][1] == info.i_AntennaSq[j][1])
                    {
                        // 余弦值 <-0.95 说明两次相位差接近180度(跳半周)
                        if (cos(tp_info.i_Phase_Diff[i] - info.i_Phase_Diff[j]) < -0.95)
                        {
                            flg = 1;
                            info.i_Phase_Diff[j] += PI;  // 补偿半周修复
                        }
                        break;
                    }
                }
            }
        }
    }
    return flg;
}

// =========================================================================
// 干涉仪测向汇总 - 对每个频点的每颗卫星进行DOA计算
// 流程:
// 1. 遍历所有频点和卫星
// 2. 调用ArithmeticDoa::calInterfer进行相关干涉仪匹配
// 3. 检测并修复跳半周问题(getDetection180)
// 4. 可选:二次确认测向(虚拟干涉仪)
// 5. 质量过滤(低于m_Qulity_Threshold则丢弃)
// 6. 可选:保存伪谱值用于积分
// 7. 结果存入m_AngleResultData
// =========================================================================
void SpoofingDoa::calAngle(std::map<int, std::map<int, InterferInfo>> inferInfoData)
{
    m_AngleResultData.clear();
    int typeInt = 0;
    int prn = 0;
    map<int, InterferInfo> tp;
    InterferInfo tp_info;
    InterferInfo tp_info180;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    vector<vector<double>> phaseTheory;
    vector<double> pseudoValue;
    vector<double> pseudoValue180;
    map<int, vector<double>> tp_peseudo;

    for (auto it = inferInfoData.begin(); it != inferInfoData.end(); ++it)
    {
        typeInt = it->first;
        tp = it->second;
        phaseTheory.clear();
        phaseTheory = m_Theory[typeInt];  // 获取该频点的理论相位差模板
        tp_alarms.clear();
        tp_peseudo.clear();
        for (auto itt = tp.begin(); itt != tp.end(); ++itt)
        {
            pseudoValue.clear();
            pseudoValue180.clear();
            prn = itt->first;
            tp_info = itt->second;

            // 相关干涉仪匹配:计算实测相位差与理论模板的相关系数
            double angle;
            double quality;
            ArithmeticDoa::calInterfer(phaseTheory, tp_info, angle, quality, pseudoValue);
            PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,Fre=%.1f,R=%.4f,startAngle=%d,endAngle=%d,angle=%.2f,quality=%.2f\n",
                             typeInt / 100, typeInt % 100, prn, m_F[typeInt], m_R[typeInt],
                             tp_info.i_Start, tp_info.i_End, angle, quality);
            if (0 == m_Virtual_Flag)
            {
                PublicSpace::Log("antenna and phasediff:[\n");
                for (int i = 0; i < tp_info.i_Phase_Len; i++)
                {
                    PublicSpace::Log("   antenna1=%d,antenna2=%d,phasediff=%.2f\n",
                                     tp_info.i_AntennaSq[i][0], tp_info.i_AntennaSq[i][1], tp_info.i_Phase_Diff[i] * 180 / PI);
                }
                PublicSpace::Log("]\n");
            }

            // 跳半周检测与修复
            int detection_flg = 0;
            tp_info180 = tp_info;
            detection_flg = getDetection180(typeInt, prn, tp_info180); // 检测是否存在跳半周
            PublicSpace::Log("detection180=%d\n", detection_flg);
            if (1 == detection_flg)
            {
                double angle180;
                double quality180;
                // 使用修复后的相位差重新测向
                ArithmeticDoa::calInterfer(phaseTheory, tp_info180, angle180, quality180, pseudoValue180);
                int tp_start = Round360(tp_info.i_Start);
                int tp_end = Round360(tp_info.i_End);
                if (angle180 != tp_start && angle180 != tp_end)
                {
                    // 若跳半周纠正后的测向质量大于纠正前的阈值，或原结果在边界，则采用修复结果
                    if ((quality180 - quality) > m_Detection180_Qulity_Threshold || angle == tp_start || angle == tp_end)
                    {
                        quality = quality180;
                        angle = angle180;
                        pseudoValue.clear();
                        pseudoValue = pseudoValue180;
                        tp_info = tp_info180;
                    }
                }
                PublicSpace::Log("180:Sys=%d,Type=%d,Prn=%d,Fre=%.1f,R=%.4f,startAngle=%d,endAngle=%d,angle=%.2f,quality=%.2f\n",
                                 typeInt / 100, typeInt % 100, prn, m_F[typeInt], m_R[typeInt],
                                 tp_info.i_Start, tp_info.i_End, angle180, quality180);
                if (0 == m_Virtual_Flag)
                {
                    PublicSpace::Log("antenna and phasediff:[\n");
                    for (int i = 0; i < tp_info180.i_Phase_Len; i++)
                    {
                        PublicSpace::Log("   antenna1=%d,antenna2=%d,phasediff=%.2f\n",
                                         tp_info180.i_AntennaSq[i][0], tp_info180.i_AntennaSq[i][1], tp_info180.i_Phase_Diff[i] * 180 / PI);
                    }
                    PublicSpace::Log("]\n");
                }
            }
            // 保存当前数据用于下一帧的跳半周检测
            m_infoData180[typeInt][prn] = tp_info;

            // 二次确认测向(虚拟干涉仪)
            if (1 == m_Secondary_Doa_Flag)
            {

                ArithmeticDoa::calSecondDoaByVirInterf(phaseTheory, m_Virtual_Multiple, pseudoValue, tp_info, angle, quality);
            }

            // 质量过滤
            if (quality < m_Qulity_Threshold)
            {
                continue;  // 测向质量不达标，丢弃此结果
            }

            // 保存伪谱值用于多帧积分
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo[prn] = pseudoValue;
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
            }
            tp_alarm.i_Prn = prn;
            tp_alarm.i_Angle = (int)angle;
            tp_alarm.i_Quality = quality;
            tp_alarms.emplace_back(tp_alarm);
        }
        m_AngleResultData[typeInt] = tp_alarms;  // 保存该频点所有卫星的测向结果
    }
}

// =========================================================================
// 根据天线关系和相位差构建InterferInfo结构
// 这是从原始相位差数据到干涉仪测向输入的桥梁函数:
// 1. 验证数据有效性(载噪比>0，天线对有效)
// 2. 排除同天线自检刀(仅用于校正，不参与测向)
// 3. 构建天线对序列和相位差序列
// 4. 可选:应用虚拟阵列扩展
// 5. 记录该卫星的最大载噪比
// 返回: doaFlg=1表示数据有效可测向，0表示数据不足
// =========================================================================
void SpoofingDoa::calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg)
{
    doaFlg = 1;
    int diffLen = dataB.i_diffLen;
    int count = 0;
    map<int, map<int, double>> tp_antenna_map;
    map<int, double> tp1;
    float maxSnr = 99;

    vector<vector<int>> tp_antnna;
    vector<double> tp_diff;

    for (int j = 0; j < diffLen; j++)
    {
        // 跳过无效数据(载噪比接近0)
        if (dataB.i_Snr1[j] < 1e-6 || dataB.i_Snr2[j] < 1e-6)
        {
            continue;
        }
        // 跳过同天线自检刀(用于校正，不参与测向)
        if (m_cutSequence[j][0] == m_cutSequence[j][1])
        {
            continue;
        }

        if (dataB.i_Snr1[j] < maxSnr)
        {
            maxSnr = dataB.i_Snr1[j];
        }
        if (dataB.i_Snr2[j] < maxSnr)
        {
            maxSnr = dataB.i_Snr2[j];
        }
        tp_antnna.emplace_back(m_cutSequence[j]);  // 记录天线对
        tp_diff.emplace_back(dataB.i_phase_diff[j]); // 记录相位差
        tp1[m_cutSequence[j][1]] = dataB.i_phase_diff[j];
        tp_antenna_map[m_cutSequence[j][0]] = tp1;
        ++count;
    }
    // 有效天线对数量不足，无法测向
    if (count < m_Doa_Cut_min_Num)
    {
        doaFlg = 0;
        return;
    }
    // 设置天线对和相位差用于底层算法
    ArithmeticDoa::setUseAntennaAndPhaseAll(tp_antnna, tp_diff);
    // 虚拟阵列扩展
    if (1 == m_Virtual_Flag)
    {
        ArithmeticDoa::getVirtual(m_Virtual_Multiple, tp_antnna, tp_diff);
        ArithmeticDoa::setUseAntennaAndPhaseAll(tp_antnna, tp_diff);
    }
    int size = (int)tp_diff.size();
    if (size < m_Doa_Cut_min_Num)
    {
        doaFlg = 0;
        return;
    }
    // 记录最大载噪比
    int prn = dataB.i_Prn;
    int typeInt = TypeInt(dataB.i_Sys, dataB.i_Type);
    if (diffLen > 1)
    {
        maxSnr = dataB.i_Snr1[1];
    }
    m_Max_Snr[typeInt][prn] = maxSnr;
    // 构建InterferInfo输出
    for (int i = 0; i < size; i++)
    {
        info.i_Phase_Diff[i] = tp_diff[i] * 2 * PI;    // 相位差转换为弧度
        info.i_AntennaSq[i][0] = tp_antnna[i][0];       // 天线对-天线1
        info.i_AntennaSq[i][1] = tp_antnna[i][1];       // 天线对-天线2
    }
    info.i_Phase_Len = size;
}


// =========================================================================
// 根据map保存的数据,计算告警值
// 对每个频点依次进行:
// 1. 相位差法检测(calAlarmByPhaseDiff，载噪比≥35dB 作为数据质量门限)
// 2. 将检测结果合并到m_AngleResultData
// 注: 载噪比聚类(calAlarmBySnr)与角度聚类(calAlarmByAngle)已按新流程移除;
//     本函数为旧(非循环)检测路径，逐帧判定(单帧确认);循环检测路径的跨刀
//     连续确认由 getCyclicDetectionData 的 m_ConsecutiveAlarm 完成。
// =========================================================================
void SpoofingDoa::getAlarm(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT)
{
    int typeInt = 0;
    vector<AlarmData> tp_alarmData;
    vector<SatelliteDataPhaseDiffA> tpA;
    vector<SatelliteDataPhaseDiffA> tpA2;
    int alarm = 0;

    for (auto it = dataT.begin(); it != dataT.end(); ++it){
        typeInt = it->first;
        tpA.clear();
        tpA2.clear();
        tpA2 = it->second;

        // 欺骗检查 - 相位差法
        calAlarmByPhaseDiff(typeInt, tpA2, tpA, alarm);

        // 载噪比聚类检测已按新流程移除（原方法2: calAlarmBySnr），载噪比仅作为
        // 数据质量门限（≥35dB）在 calAlarmByPhaseDiff 中生效。

        if (0 == alarm){
            continue;
        }

        // 构建告警结果
        AlarmData tp;
        tp_alarmData.clear();
        vector<AlarmData>().swap(tp_alarmData);
        if (m_AngleResultData.find(typeInt) != m_AngleResultData.end())
        {
            tp_alarmData = m_AngleResultData[typeInt];
        }
        string nowT = getNowTime();
        PublicSpace::Log("%s \n get spoofing detection alarm info:\n", nowT.c_str());
        LogSatelliteDataPhaseDiffA(tpA);
        for (unsigned int i = 0; i < tpA.size(); i++)
        {
            int prn = tpA[i].i_Prn;
            bool flg = false;
            // 检查该卫星是否已存在于结果中(避免重复)
            for (unsigned int k = 0; k < tp_alarmData.size(); k++)
            {
                if (prn == tp_alarmData[k].i_Prn)
                {
                    flg = true;
                    break;
                }
            }
            if (flg)
            {
                continue;
            }
            tp.i_Prn = prn;
            tp.i_Angle = -1;     // 检测模式下角度未知
            tp.i_Quality = -1;   // 检测模式下质量未知
            tp.i_Snr = tpA[i].i_Snr1;
            m_Max_Snr[typeInt][tp.i_Prn] = tp.i_Snr;
            tp_alarmData.emplace_back(tp);
        }
        m_AngleResultData[typeInt] = tp_alarmData;
    }

}


// =========================================================================
// 多帧数据平滑处理
// 对每个切刀位置的多帧数据进行卫星相位差平滑
// 平滑方法: 排除差异大的帧，将剩余帧的相位转换为复数后取均值
// 注: 该方法用于减少噪声对相位差的影响，提高测向精度
// =========================================================================
void SpoofingDoa::getSmoothData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int cutNum = (int)m_cutSequence.size();
    vector<SatelliteDataPhaseDiffB> dataB;
    vector<vector<SatelliteDataPhaseDiffA>> oneCutData;
    oneCutData.resize(m_OneCut_Frams);
    vector<vector<SatelliteDataPhaseDiffA>> resultData;
    SatelliteDataPhaseDiffA tpA;
    vector<SatelliteDataPhaseDiffA> tpA2;
    for (int j = 0; j < cutNum; j++)
    {
        dataB.clear();
        oneCutData.clear();
        int index = 0;
        // 对每个切刀位置的多帧数据汇总
        for (int k = 0; k < m_OneCut_Frams; k++)
        {
            index = j * m_OneCut_Frams + k;
            oneCutData[k] = dataA[index];
        }
        // 汇总同一卫星在该刀各帧(秒)的相位差样本。按帧聚合时不套用
        // m_Delete_Prn_Flag 的「要求全部帧有效」过滤：某几帧缺失的卫星也保留，
        // 交给 calSmoothData 的覆盖判定 min(刀内帧数, MIN_STABLE_SAMPLES) 决定取舍，
        // 对应当前 Python build_vectors_and_detect / compute_calibration 的采样覆盖规则。
        // （原实现此处按 deletePrnFlag=1 会丢弃缺任一帧的卫星，比 Python 严苛太多）
        int savedDeleteFlag = m_Delete_Prn_Flag;
        m_Delete_Prn_Flag = 0;
        getSatelliteDataPhaseDiffB(oneCutData, dataB);  // 按卫星汇总
        m_Delete_Prn_Flag = savedDeleteFlag;
        // 校正刀与普通刀一样，只要求有效采样覆盖 min(刀内帧数, MIN_STABLE_SAMPLES) 帧，
        // 不再要求卫星从该刀第一帧(第一秒)就存在（Python compute_calibration 亦不要求）
        for (unsigned int i = 0; i < dataB.size(); i++)
        {
            calSmoothData(dataB[i], tpA);  // 对每颗卫星进行多帧平滑
            tpA2.emplace_back(tpA);
        }
        resultData.emplace_back(tpA2);
    }
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA = resultData;
}

// =========================================================================
// 对单颗卫星多帧相位差做稳定性过滤 + 跳半周处理 + 圆形均值
// 对应 Python detection_lib.py 的 check_stability / circular_mean：
//   1. 收集载噪比有效(两端口 > 1e-3)的相位差样本（单位: 周 -> 度）；
//   2. 均匀分布：有效采样数 >= min(总帧数, MIN_STABLE_SAMPLES)；
//   3. 360° 圆上最小覆盖弧 < STABILITY_RANGE_DEG(15°) -> 稳定，取圆形均值；
//   4. 否则折叠到 [0,180) 后跨度 < 15° -> 检测到跳半周（HALF_CYCLE_CORRECT=false
//      时按波动大处理，不使用，载噪比置 0 表示无效）；
//   5. 其余不稳定样本丢弃（载噪比置 0 表示无效）。
// 不再要求卫星从该刀第一帧(第一秒)就存在（Python compute_calibration 亦不要求）。
// 载噪比置 0 后，下游(校正 calCorrecteData / 检测 calAlarmByPhaseDiff / 测向
// calAngleUseAntenna)都会跳过该卫星。
// =========================================================================
void SpoofingDoa::calSmoothData(SatelliteDataPhaseDiffB dataB, SatelliteDataPhaseDiffA &dataA)
{
    vector<double> samples;  // 相位差样本(度)
    double sumSnr1 = 0.0;
    double sumSnr2 = 0.0;
    int length = dataB.i_diffLen;
    for (int i = 0; i < length; ++i)
    {
        if (dataB.i_Snr1[i] < 1e-3 || dataB.i_Snr2[i] < 1e-3)
        {
            continue;  // 载噪比无效，不使用该样本
        }
        samples.emplace_back(dataB.i_phase_diff[i] * 360.0);  // 周 -> 度
        sumSnr1 += dataB.i_Snr1[i];
        sumSnr2 += dataB.i_Snr2[i];
    }

    // 组装输出基本字段
    dataA.i_Sys = dataB.i_Sys;
    dataA.i_Type = dataB.i_Type;
    dataA.i_Prn = dataB.i_Prn;

    int n = (int)samples.size();
    if (n <= 0)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    // 均匀分布：至少覆盖 min(总帧数, MIN_STABLE_SAMPLES) 个有效采样
    int required = (length < MIN_STABLE_SAMPLES) ? length : MIN_STABLE_SAMPLES;
    if (n < required)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    // 稳定判定：360° 圆上最小覆盖弧 < STABILITY_RANGE_DEG(15°)
    if (circularSpanDeg(samples) < STABILITY_RANGE_DEG)
    {
        double smoothDeg = circularMeanDeg(samples);
        double smoothCycle = smoothDeg / 360.0;
        if (smoothCycle < 0)
        {
            smoothCycle += 1.0;  // 归一化到 [0,1) 周
        }
        dataA.i_phase_diff = smoothCycle;
        dataA.i_Snr1 = sumSnr1 / n;
        dataA.i_Snr2 = sumSnr2 / n;
        return;
    }

    // 跳半周检测：折叠到 [0,180) 后跨度 < STABILITY_RANGE_DEG(15°)。HALF_CYCLE_CORRECT=false 时
    // 跳半周按相位差波动大处理（不使用），载噪比置 0 表示无效。
    if (circularSpan180Deg(samples) < STABILITY_RANGE_DEG)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    // 普通波动大，不使用
    dataA.i_phase_diff = 0.0;
    dataA.i_Snr1 = 0.0;
    dataA.i_Snr2 = 0.0;
}

// =========================================================================
// 取最后一帧数据(不进行平滑时使用)
// 当配置为多帧但不平滑时，仅保留每个切刀位置的最后一帧
// =========================================================================
void SpoofingDoa::getEndFramData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    vector<vector<SatelliteDataPhaseDiffA>> resultDataA;
    int cutNum = (int)m_cutSequence.size();
    for (int j = 0; j < cutNum; j++)
    {
        int index = 0;
        index = (j + 1) * m_OneCut_Frams - 1;  // 取最后一帧索引
        resultDataA.emplace_back(dataA[index]);
    }

    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA = resultDataA;
}

// =========================================================================
// 设置各频点的阵列半径(m_R)
// 全向天线模式:所有频点使用统一半径(m_omni_R)
// 定向天线模式:根据频率从配置文件读取各频段的半径
// 定向天线的不同频率对应不同的有效阵列半径(天线方向图频率相关)
// =========================================================================
void SpoofingDoa::setR(const string adr){
    vector<Rs> m_Rs;
    m_Rs.clear();
    if (1 == m_antnenaType){
        ArithmeticDoa::getRData(adr, m_Rs);  // 读取定向天线频率-半径映射表
    }

    int typeInt = 0;
    double tpF = 0.0;

    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        typeInt = it->first;
        tpF = it->second;
        if (0 == m_antnenaType) // 全向天线
        {
            m_R[typeInt] = m_omni_R;  // 统一使用全向天线半径
        }
        else
        {
            // 定向天线:查找频率对应的可用半径
            for (unsigned int i = 0; i < m_Rs.size(); i++)
            {
                if (tpF >= m_Rs[i].i_starF && tpF <= m_Rs[i].i_endF)
                {
                    m_R[typeInt] = m_Rs[i].i_r;  // 使用该频段的半径
                    break;
                }
            }
        }
    }
}


// 设置告警门限
// @param sys 卫星系统(-1表示所有系统)
// @param type 卫星频点(-1表示所有频点)
// @param threshold 卫星颗数阈值
// @param phsThreshold 相位差阈值(单位:度)
// @note 当卫星系统和卫星频点均为-1时，表示对所有卫星系统和频率进行告警检测，否则对一个卫星系统和一个固定频率进行告警检测
void SpoofingDoa::setThresholdDetectionDoa(int sys, int type, int threshold, double phsThreshold)
{
    PublicSpace::Log("Sys=%i,Type=%i,coutThreshold=%i,phsThreshold=%.1f\n", sys, type, threshold, phsThreshold);
    if (-1 == sys && -1 == type)
    {
        // 对所有频点统一设置阈值
        initDetectionThreshold(threshold, phsThreshold);
        return;
    }

    // 对指定频点单独设置阈值
    if (-1 != threshold)
    {
        m_Detection_Threshold[TypeInt(sys, type)] = threshold;
    }
    if (phsThreshold > 0)
    {
        m_Detection_PhsThreshold[TypeInt(sys, type)] = phsThreshold / 360.0;  // 度转周
    }
}

// =========================================================================
// 设置是否使用载噪比进行欺骗检测
// 欺骗信号往往具有相同或相近的载噪比(同一干扰源发射)
// =========================================================================
void SpoofingDoa::setTypeDetectionBySnr(int thresholdNum, int sys, int type)
{
    if (-1 != thresholdNum)
    {
        m_Detection_snrThrehold[TypeInt(sys, type)] = thresholdNum;
    }
    PublicSpace::Log("join detection by snr:Sys=%i,Type=%i\n", sys, type);
}



// =========================================================================
// 清空循环切刀检测的连续报警计数与跟踪状态（对应 Python consecutive / tracking）
// =========================================================================
void SpoofingDoa::resetCyclicDetection(void)
{
    m_ConsecutiveAlarm.clear();
    m_Tracking.clear();
    m_Baselines.clear();   // 跨周期测向基线一并清空
}

// =========================================================================
// 设备 wrapper(GN902 等)覆盖循环切刀运行参数
// 供外部按“整轮(7 刀)流式喂帧”驱动引擎时使用；omniR>0 时按该半径重建
// m_R 与测向理论模板，确保与 Python ARRAY_RADIUS(=0.1865, GN902)一致。
// =========================================================================
void SpoofingDoa::configCyclicRuntime(bool cyclic, int oneCutFrams, bool smooth, double omniR)
{
    m_Cyclic_Detection_Flag = cyclic ? 1 : 0;
    m_Doa_Detection_Flag = 0;
    if (oneCutFrams > 0)
    {
        m_OneCut_Frams = oneCutFrams;
    }
    m_Smooth_Flag = smooth ? 1 : 0;

    if (omniR > 0 && 0 == m_antnenaType)
    {
        m_omni_R = omniR;
        m_R.clear();
        setR(m_Radr);           // 全向天线：m_R[typeInt] = m_omni_R
        m_Theory.clear();
        if (1 == m_Doa_Arithmetic)
        {
            initTheory();       // 按新半径重建 360° 理论相位模板
        }
    }
    PublicSpace::Log("configCyclicRuntime: cyclic=%d oneCutFrams=%d smooth=%d omniR=%.4f\n",
                     m_Cyclic_Detection_Flag, m_OneCut_Frams, m_Smooth_Flag, m_omni_R);
}

// =========================================================================
// 循环切刀欺骗检测（对应 Python detection_main.py / detection_lib.py 循环切刀主流程）
// 对每刀做相位差聚类检测，跨刀连续确认(连续 p=m_Detection_Recodds_Num 刀)，确认后进入
// 测向跟踪；只保留已确认欺骗的 (系统,频点) 卫星用于后续测向。
// 数据流: dataA 已按刀划分且完成校正，每刀 = 该刀稳定(校正后)相位差的卫星列表。
// =========================================================================
void SpoofingDoa::getCyclicDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int cutNum = (int)dataA.size();

    for (int j = 0; j < cutNum; ++j)
    {
        // 跳过同天线自校准刀(仅用于校正，不参与检测)
        if (j < (int)m_cutSequence.size() && m_cutSequence[j][0] == m_cutSequence[j][1])
        {
            continue;
        }

        std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
        getSatelliteDataByType(dataA[j], dataT);

        // 本刀各频点报警的卫星号集合
        std::map<int, std::set<int>> cutAlarms;
        for (auto &kv : dataT)
        {
            int typeInt = kv.first;
            vector<SatelliteDataPhaseDiffA> alarmSats;
            int alarm = 0;
            calAlarmByPhaseDiff(typeInt, kv.second, alarmSats, alarm);
            if (alarm)
            {
                std::set<int> sids;
                for (auto &s : alarmSats)
                {
                    sids.insert(s.i_Prn);
                }
                cutAlarms[typeInt] = sids;
            }
        }

        // 1. 本刀未报警的 (系统,频点) 中断连续
        for (auto &kv : m_ConsecutiveAlarm)
        {
            if (cutAlarms.find(kv.first) == cutAlarms.end())
            {
                kv.second = 0;
                // 连续报警被打断：仅当该 (系统,频点) 尚未确认(未进入跟踪)时才清空基线；
                // 已确认(跟踪中)的信号即使本刀聚类未达阈值(如严格阈值的 Galileo E1C
                // 某刀只聚集 3 颗)，也保留跨刀/跨周期累积的相位差(对应 Python 同修复)。
                if (m_Tracking.find(kv.first) == m_Tracking.end())
                {
                    m_Baselines.erase(kv.first);
                }
            }
        }
        // 2. 本刀报警的 (系统,频点) 连续 +1；达到 p 次确认进入跟踪
        for (auto &kv : cutAlarms)
        {
            int typeInt = kv.first;
            int c = m_ConsecutiveAlarm[typeInt] + 1;
            m_ConsecutiveAlarm[typeInt] = c;
            if (c >= m_Detection_Recodds_Num)
            {
                TrackingInfo &t = m_Tracking[typeInt];
                for (int sid : kv.second)
                {
                    t.cluster_sats.insert(sid);
                }
            }
        }
    }

    // 3. 只保留已确认(跟踪中)频点的【聚集卫星(cluster_sats)】数据，供后续相关干涉仪测向
    //    （对齐 Python run_doa_one：只对 tracking.info.cluster_sats 里的卫星做基线测向，
    //      而不是给该频点的全部卫星测向）
    for (int j = 0; j < cutNum; ++j)
    {
        vector<SatelliteDataPhaseDiffA> filtered;
        for (auto &sat : dataA[j])
        {
            int typeInt = TypeInt(sat.i_Sys, sat.i_Type);
            auto it = m_Tracking.find(typeInt);
            if (it != m_Tracking.end() &&
                it->second.cluster_sats.find(sat.i_Prn) != it->second.cluster_sats.end())
            {
                filtered.emplace_back(sat);
            }
        }
        dataA[j] = filtered;
    }
}

// =========================================================================
// 把本轮 dataB 合并进跨周期测向基线(相位差跨轮复用做测向)
// 对应 Python detection_main.py 的 baselines 累积：
//   - 每轮对同一 (系统,频点,卫星号)，仅覆盖本轮载噪比有效的刀位相位差；
//     本轮缺失(载噪比=0)的刀位保留上一周期的相位差，从而凑齐 6 条基线。
//   - 前提：校正偏移相邻轮基本稳定(与 Python 跨周期复用一致)。
// =========================================================================
void SpoofingDoa::accumulateBaselines(const std::vector<SatelliteDataPhaseDiffB> &dataB)
{
    for (const auto &b : dataB)
    {
        int typeInt = TypeInt(b.i_Sys, b.i_Type);
        int prn = b.i_Prn;
        auto &inner = m_Baselines[typeInt];
        auto itp = inner.find(prn);
        if (itp == inner.end())
        {
            inner[prn] = b;  // 新卫星：整体初始化
            continue;
        }
        SatelliteDataPhaseDiffB &acc = itp->second;
        acc.i_diffLen = b.i_diffLen;
        for (int j = 0; j < b.i_diffLen && j < 100; ++j)
        {
            if (b.i_Snr1[j] > 1e-6 && b.i_Snr2[j] > 1e-6)
            {
                acc.i_phase_diff[j] = b.i_phase_diff[j];
                acc.i_Snr1[j] = b.i_Snr1[j];
                acc.i_Snr2[j] = b.i_Snr2[j];
            }
        }
    }
}

// =========================================================================
// 取出当前跟踪中(系统,频点)的跨周期累积基线，作为测向输入
// 对应 Python run_doa_one：只对 tracking.cluster_sats 里的卫星做基线测向。
// =========================================================================
void SpoofingDoa::getCrossCycleDataB(std::vector<SatelliteDataPhaseDiffB> &doaDataB)
{
    doaDataB.clear();
    for (const auto &kv : m_Tracking)
    {
        int typeInt = kv.first;
        const TrackingInfo &t = kv.second;
        auto itt = m_Baselines.find(typeInt);
        if (itt == m_Baselines.end())
        {
            continue;
        }
        for (int prn : t.cluster_sats)
        {
            auto itp = itt->second.find(prn);
            if (itp != itt->second.end())
            {
                doaDataB.emplace_back(itp->second);
            }
        }
    }
}

// =========================================================================
// 对每刀数据进行欺骗检测，筛选出检测为欺骗的信号用于后续测向
// 两种模式:
//   mode=1: 每刀独立检测，各刀独立筛选
//   mode=2: 所有刀合并检测，筛选所有刀中共有的欺骗信号
// =========================================================================
void SpoofingDoa::getSpoofingDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    m_AngleResultData.clear();
    int dataLen = (int)dataA.size();
    int cutNum = (int)m_cutSequence.size();
    if (1 == m_Doa_Detection_Flag)
    {
        // 模式1: 每刀独立检测
        for (int i = 0; i < dataLen; i++)
        {
            std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
            getSatelliteDataByType(dataA[i], dataT);   // 按频点分类
            getAlarm(dataT);                            // 欺骗检测
            setSpoofingDetectionData(dataA[i]);         // 筛选数据
            m_AngleResultData.clear();
        }
    }
    if (2 == m_Doa_Detection_Flag)
    {
        // 模式2: 所有刀合并检测
        std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
        for (int i = 0; i < dataLen; i++)
        {
            // 跳过同天线自检刀(这些刀不包含测向信息)
            if (cutNum == dataLen)
            {
                if (m_cutSequence[i][0] == m_cutSequence[i][1])
                {
                    continue;
                }
            }
            dataT.clear();
            getSatelliteDataByType(dataA[i], dataT);
            getAlarm(dataT);
        }

        // 对所有刀的数据进行筛选
        for (int i = 0; i < dataLen; i++)
        {
            setSpoofingDetectionData(dataA[i]);
        }
    }
    m_AngleResultData.clear();
}

// =========================================================================
// 根据欺骗检测结果筛选卫星数据
// 仅保留在m_AngleResultData中标记为欺骗的卫星数据
// =========================================================================
void SpoofingDoa::setSpoofingDetectionData(std::vector<SatelliteDataPhaseDiffA> &spoofingData)
{
    vector<SatelliteDataPhaseDiffA> resultData;
    resultData.clear();
    int size = (int)spoofingData.size();
    int typeInt = 0;
    int prn = 0;
    SatelliteDataPhaseDiffA tpA;
    vector<AlarmData> tp_alarm;
    for (int i = 0; i < size; i++)
    {
        tpA = spoofingData[i];

        typeInt = TypeInt(tpA.i_Sys, tpA.i_Type);
        prn = tpA.i_Prn;
        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            continue;  // 该频点未检测到欺骗
        }
        else
        {
            tp_alarm = m_AngleResultData[typeInt];
            for (unsigned int j = 0; j < tp_alarm.size(); j++)
            {
                if (tp_alarm[j].i_Prn == prn)
                {
                    resultData.emplace_back(tpA);  // 保留标记为欺骗的卫星
                    break;
                }
            }
        }
    }
    spoofingData.clear();
    vector<SatelliteDataPhaseDiffA>().swap(spoofingData);
    spoofingData = resultData;
    vector<SatelliteDataPhaseDiffA>().swap(resultData);
}

// =========================================================================
// 全排列组合方式优化跳半周
// 原理: 对每刀相位差尝试±0.5周(半周)的补偿，枚举所有组合
//       对每种组合进行干涉仪测向，选取测向质量最高的那个组合
// 适用场景: 当载波相位存在整周模糊或半周跳变时
// 注意: 2^n种组合(n=相位差数量-1)，组合数随刀数指数增长
// =========================================================================
void SpoofingDoa::setPermutationOptimiz180(SatelliteDataPhaseDiffB dataB, const vector<vector<double>> phaseTheory, double &angle, double &qulity)
{

    int length = dataB.i_diffLen - 1;  // 排除参考刀(第0刀)
    int combinations = std::pow(2, length);  // 全排列组合数 = 2^n
    qulity = -99.0;
    SatelliteDataPhaseDiffB tpB;
    for (int i = 0; i < combinations; ++i)
    {
        tpB = dataB;
        // 根据二进制位决定每刀是否加半周
        for (int j = 1; j < length + 1; ++j)
        {
            double tp_phs_diff_original = dataB.i_phase_diff[j];
            if (i & (1 << (j - 1)))
            {
                tpB.i_phase_diff[j] = tp_phs_diff_original + 0.5;  // 加半周
            }
            else
            {
                tpB.i_phase_diff[j] = tp_phs_diff_original;  // 保持不变
            }
        }
        PublicSpace::Log("optimiz:num=%d\n", i);
        LogSatelliteDataPhaseDiffB(tpB);
        vector<double> pseudoValue;
        pseudoValue.clear();
        int doaFlg = 1;
        InterferInfo tp_info1;
        tp_info1.i_Start = 0;
        tp_info1.i_End = 359;
        tp_info1.i_Phase_Len = 0;

        double tp_angle1;
        double tp_quality1;
        calAngleUseAntenna(tpB, tp_info1, doaFlg);               // 构建InterferInfo
        ArithmeticDoa::calInterfer(phaseTheory, tp_info1, tp_angle1, tp_quality1, pseudoValue);  // 干涉仪测向
        PublicSpace::Log("Angle=%.1f,Quality=%.1f\n", tp_angle1, tp_quality1);
        // 质量达到最优立即返回
        if (tp_quality1 >= 99.9)
        {
            angle = tp_angle1;
            qulity = tp_quality1;
            return;
        }
        // 保留质量最高的组合
        if (tp_quality1 > qulity)
        {
            angle = tp_angle1;
            qulity = tp_quality1;
        }
    }
}

// =========================================================================
// 对所有卫星进行跳半周优化测向
// 对每颗卫星使用全排列组合方式尝试修复跳半周
// 结果存入m_AngleResultData
// =========================================================================
void SpoofingDoa::getOptimizResultDoa180(vector<SatelliteDataPhaseDiffB> dataB)
{
    int size = (int)dataB.size();
    int typeInt = 0;
    SatelliteDataPhaseDiffB tpB;
    vector<vector<double>> phaseTheory;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    for (int i = 0; i < size; i++)
    {
        tp_alarms.clear();
        tpB = dataB[i];
        typeInt = TypeInt(tpB.i_Sys, tpB.i_Type);
        phaseTheory.clear();
        phaseTheory = m_Theory[typeInt];
        double angle;
        double quality = 0.0;
        setPermutationOptimiz180(tpB, phaseTheory, angle, quality);  // 全排列优化

        tp_alarm.i_Prn = tpB.i_Prn;
        tp_alarm.i_Angle = (int)angle;
        tp_alarm.i_Quality = quality;

        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            tp_alarms.emplace_back(tp_alarm);
            m_AngleResultData[typeInt] = tp_alarms;
        }
        else
        {
            tp_alarms = m_AngleResultData[typeInt];
            tp_alarms.emplace_back(tp_alarm);
            m_AngleResultData[typeInt] = tp_alarms;
        }
    }
}

// =========================================================================
// 设置切刀顺序(外部接口调用)
// 接收一维数组表示的天线对序列，转换为二维向量
// @param len: 数组长度(天线序号个数，需为偶数)
// @param cutSq: 天线序号数组[天线1a, 天线1b, 天线2a, 天线2b, ...]
// =========================================================================
void SpoofingDoa::setCutSquence(int len, const int *cutSq)
{
    m_cutSequence.clear();
    vector<vector<int>>().swap(m_cutSequence);
    int size = len / 2;  // 天线对数量 = 总长度 / 2
    m_cutSequence.resize(size);
    int tp_index = 0;
    for (int i = 0; i < size; i++)
    {
        m_cutSequence[i].resize(2);
        tp_index = i * 2;
        if (cutSq[tp_index] < 1 || cutSq[tp_index + 1] < 1)
        {
            m_cutSequence.clear();  // 天线序号无效，清空
            return;
        }
        m_cutSequence[i][0] = cutSq[tp_index];      // 天线对-天线1
        m_cutSequence[i][1] = cutSq[tp_index + 1];  // 天线对-天线2
        PublicSpace::Log("set cutSequence:%d,m_cutSequence1 = %d,m_cutSequence2 = %d\n", i + 1, m_cutSequence[i][0], m_cutSequence[i][1]);
    }
}

// =========================================================================
// 设置外部欺骗检测结果(由其他检测模块传入)
// 用于多模块联合检测，交叉验证
// =========================================================================
void SpoofingDoa::setSpoofingDetecteResult(const vector<SingleDeceptiveResult> detectResult)
{
    m_detectResult.clear();
    vector<SingleDeceptiveResult>().swap(m_detectResult);
    if (detectResult.size() > 0)
    {
        m_detectResult = detectResult;
    }

    if (0 != m_logFlg)
    {
        PublicSpace::Log("detecion Spoofing result by other arithmetic:\n");
        for (int i = 0; i < detectResult.size(); i++)
        {
            PublicSpace::Log("Sys=%d,Type=%d,Prn={", detectResult[i].r_Sys, detectResult[i].r_Type);
            for (int j = 0; j < detectResult[i].prncount; j++)
            {
                PublicSpace::Log("%d,", detectResult[i].r_Prn[j]);
            }
            PublicSpace::Log("}\n");
        }
    }
}

// =========================================================================
// 根据外部欺骗检测结果筛选GNSS数据
// 仅保留外部检测结果中标记为欺骗的系统和频点的卫星数据
// =========================================================================
void SpoofingDoa::getDataByDetectionResult(const GNSSData data, GNSSData &resultData)
{

    GNSSData detetectData;
    int detect_num = (int)m_detectResult.size();
    int tp_result_num = 0;
    detetectData.i_PortOneNum = 0;
    detetectData.i_PortTwoNum = 0;
    for (int i = 0; i < detect_num; i++)
    {
        // 筛选端口1的数据
        for (int j = 0; j < data.i_PortOneNum; j++)
        {
            if (data.i_PortOne[j].i_Type == m_detectResult[i].r_Type && data.i_PortOne[j].i_Sys == m_detectResult[i].r_Sys)
            {
                tp_result_num = 0;
                for (int k1 = 0; k1 < m_detectResult[i].prncount; k1++)
                {
                    if (data.i_PortOne[j].i_Prn == m_detectResult[i].r_Prn[k1])
                    {
                        tp_result_num = detetectData.i_PortOneNum;
                        detetectData.i_PortOne[tp_result_num] = data.i_PortOne[j];

                        detetectData.i_PortOneNum = tp_result_num + 1;
                        break;
                    }
                }
            }
        }
        // 筛选端口2的数据
        for (int j = 0; j < data.i_PortTwoNum; j++)
        {
            tp_result_num = 0;
            if (data.i_PortTwo[j].i_Type == m_detectResult[i].r_Type && data.i_PortTwo[j].i_Sys == m_detectResult[i].r_Sys)
            {
                for (int k1 = 0; k1 < m_detectResult[i].prncount; k1++)
                {
                    if (data.i_PortTwo[j].i_Prn == m_detectResult[i].r_Prn[k1])
                    {
                        tp_result_num = detetectData.i_PortTwoNum;
                        detetectData.i_PortTwo[tp_result_num] = data.i_PortTwo[j];

                        detetectData.i_PortTwoNum = tp_result_num + 1;
                        break;
                    }
                }
            }
        }
    }
    resultData = detetectData;
}

// =========================================================================
// 根据外部欺骗检测结果筛选测向结果
// 将外部检测模块标记为欺骗但本模块未计算测向结果的频点补充到输出中
// 通过角度加权投票确定补充频点的到达角
// =========================================================================
void SpoofingDoa::getSpoofingResultByDetection(SpoofingResult &result)
{
    SpoofingResult tp_result = result;
    int detect_num = (int)m_detectResult.size();
    int count = result.i_Count;
    SatelliteAngle tp_Satellite;
    bool flg = false;
    int typeInt = 0;
    vector<AlarmData> tp_alarms;
    SingleDeceptiveResult tp_decepResult;

    AlarmData tp_result_alarm;

    for (int i = 0; i < detect_num; i++)
    {
        tp_decepResult = m_detectResult[i];
        int tp_sys = tp_decepResult.r_Sys;
        int tp_type = tp_decepResult.r_Type;
        flg = true;
        // 检查该频点是否已在测向结果中
        for (int j = 0; j < count; j++)
        {
            tp_Satellite = tp_result.i_SatelliteAngle[j];
            if (tp_Satellite.i_Sys == tp_sys && tp_Satellite.i_Type == tp_type)
            {
                flg = false;  // 已存在
                break;
            }
        }
        if (flg)
        {
            // 该频点不在测向结果中，需补充
            typeInt = TypeInt(tp_sys, tp_type);
            if (m_AngleResultData.find(typeInt) != m_AngleResultData.end())
            {
                SatelliteAngle tp_result_SateA;
                int SateASize = 0;
                vector<double> angles(360, 0.0);  // 角度加权投票累积器
                tp_alarms = m_AngleResultData[typeInt];
                int PrnNum = (int)tp_alarms.size();
                int detec_PrnNum = tp_decepResult.prncount;
                for (int k1 = 0; k1 < detec_PrnNum; k1++)
                {
                    for (int k2 = 0; k2 < PrnNum; k2++)
                    {
                        if (tp_alarms[k2].i_Prn == tp_decepResult.r_Prn[k1])
                        {
                            tp_alarms[k2].i_Snr = m_Max_Snr[typeInt][tp_alarms[k2].i_Prn];
                            tp_result_SateA.i_AlarmData[SateASize] = tp_alarms[k2];
                            SateASize = SateASize + 1;
                            // 角度加权投票:以角度为中心，±阈值范围内按距离和测向质量加权
                            int tp_angle = tp_alarms[k2].i_Angle;
                            double tp_qulity = tp_alarms[k2].i_Quality / 100;
                            for (int k3 = (0 - m_Angle_Threshold); k3 < m_Angle_Threshold; k3++)
                            {
                                int ang = tp_angle + k3;
                                int ang2 = Round360(ang);
                                angles[ang2] = angles[ang2] + (1 - fabs(k3) * 0.1) * tp_qulity;
                            }

                            break;
                        }
                    }
                }
                if (SateASize > 0)
                {
                    // 找到投票最高的角度
                    double maxAngl = -999.9;
                    int tp_angle = -1;
                    for (int k3 = 0; k3 < 360; k3++)
                    {
                        if (angles[k3] > maxAngl)
                        {
                            maxAngl = angles[k3];
                            tp_angle = k3;
                        }
                    }
                    int tp_count = result.i_Count;
                    tp_result_SateA.i_Sys = tp_sys;
                    tp_result_SateA.i_Type = tp_type;
                    tp_result_SateA.i_Alarm = 0;
                    tp_result_SateA.i_Count = SateASize;
                    tp_result_SateA.i_Angle = tp_angle;
                    result.i_Count = result.i_Count + 1;
                    result.i_SatelliteAngle[tp_count] = tp_result_SateA;
                }
            }
        }
    }
}

// =========================================================================
// 保存原始GNSS数据到二进制文件
// 用于离线回放和调试分析
// =========================================================================
void SpoofingDoa::saveGNSSData(const GNSSData *data, int dataLen)
{
    string fileName = "GNSSData_";
    fileName.append(to_string(dataLen));
    fileName.append(".dat");
    PublicSpace::saveArrayToBinary(fileName, data, dataLen);
}

// =============================================================================
// == 原 InitData.cpp —— 初始化与配置模块实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: InitData.cpp
// 功能描述: 初始化与配置模块
// 系统角色: 负责欺骗测向系统的所有初始化工作，包括:
//           1. 硬件平台参数初始化(天线数、切刀数、阵列半径等)
//           2. GNSS频点频率映射表初始化(m_F)
//           3. 欺骗检测阈值初始化
//           4. 检测历史记录初始化(滑动窗口队列)
//
// 支持设备: GN902, GN930U, GN930, GN560
// 支持星座: GPS, GLONASS, SBAS, Galileo, BDS, QZSS
// =============================================================================


// =========================================================================
// 初始化测向对象(根据项目号设置硬件参数)
// 不同项目的硬件配置差异:
//   GN902:   全向天线，7阵元，半径0.1865m，切刀 7+1刀
//   GN930:   全向天线，7阵元，半径0.1865m，切刀 7+1刀(带双板卡)
//   GN930U:  全向天线，7阵元，半径0.2000m，切刀 7刀
//   GN560:   定向天线，7阵元，半径0.1800m，切刀 7刀(环形连接)
// 关键参数:
//   m_AntennaNum:     天线阵元数
//   m_Doa_Cut_Num:    用于测向的刀数
//   m_Doa_Cut_min_Num: 最少有效相位差数量
//   m_omni_R:         全向天线阵列半径(米)
//   m_antnenaType:    天线类型(0全向/1定向)
//   m_cutSequence:    切刀顺序(RF开关切换序列)
// =========================================================================
void SpoofingDoa::initProject(void)
{

    switch (m_Project_flg)
    {
    case GN902:
        m_AntennaNum = 7;  // 阵列中的天线个数
        m_Doa_Cut_Num = 6; // 用于测向的刀数
        m_Doa_Arithmetic = 1;  // 使用相关干涉仪算法
        m_Doa_Cut_min_Num = 6; // 用于测向的相位差的最少数量
        m_omni_R = 0.1865;    // 全向天线阵列半径(米)
        m_antnenaType = 0;    // 全向天线
        m_cutSequence.clear();
        // 切刀顺序: {7,7}同天线功分(校正), {1,2}~{1,7}天线1与其他天线组成基线
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN930:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 6;
        m_omni_R = 0.1865;
        m_antnenaType = 0;
        m_cutSequence.clear();
        // GN930的切刀顺序: 中间多一个{7,7}校正刀(双板卡各需要一次校正)
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {7, 7}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN930U:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 6;
        m_omni_R = 0.2;      // 比GN902/GN930的半径(0.1865m)略大
        m_antnenaType = 0;
        m_cutSequence.clear();
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN560:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 3; // 定向天线最少3对即可测向(利用了方向性信息)
        m_omni_R = 0.18;
        m_antnenaType = 1;     // 定向天线
        m_cutSequence.clear();
        // 环形天线阵列的切刀顺序:相邻天线对形成环形基线
        // {1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,1}
        m_cutSequence = {{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 1}};

        break;
    default:
        break;
    }
}


// =========================================================================
// 设置频率索引(频点唯一编码)
// 编码规则: TypeInt = sys*100 + type
// 例如: GPS L1C/A: sys=0, type=0 -> TypeInt=0
//       BDS B1I:    sys=4, type=0 -> TypeInt=400
//       BDS B2a:    sys=4, type=12 -> TypeInt=412
// 此编码用于map的key，实现按频点的快速查找
// =========================================================================
int SpoofingDoa::TypeInt(int sys, int type)
{
    return sys * 100 + type;
}



// =========================================================================
// 初始化频率map(m_F)
// 功能: 建立所有支持GNSS频点到其中心频率(Hz)的映射
// 支持六大GNSS星座的所有主要民用和部分军用频点:
//
// GPS(系统号=0):  L1 C/A(1575.42MHz), L5(1176.45MHz), L2 P(1227.6MHz),
//                L1C(1575.42MHz), L2C(1227.6MHz)
// GLONASS(系统号=1): G1(1602.0MHz), G2(1246.0MHz), G3(1277.85MHz)
// SBAS(系统号=2): L1(1575.42MHz), L5(1176.45MHz)
// Galileo(系统号=3): E1B/E1C(1575.42MHz), E6C(1278.75MHz),
//                    E5a(1176.45MHz), E5b(1207.14MHz), AltBOC(1191.795MHz)
// BDS(系统号=4): B1I(1561.098MHz), B2I(1207.14MHz), B3I(1268.52MHz),
//                B1C(1575.42MHz), B2a(1176.45MHz), B2b(1207.14MHz)
// QZSS(系统号=5): L1(1575.42MHz), L5(1176.45MHz), L2C(1227.6MHz), L1C(1575.42MHz)
//
// 频率值用于理论相位差计算: phase_diff = 2*pi*r*sin(theta)*f/c
// =========================================================================
void SpoofingDoa::initType(void)
{
    /*0 = GPS
     1= GLONASS
     2 = SBAS
     3 = Galileo
     4 = BDS
     5 = QZSS
     6 = Reserved
     7 = Other
     */

    // ---------- GPS频点 ----------
    /*
    0 = L1 C/A
    2 = L5C
    5 = L2 P
    9 = L2 P codeless
    14 = L5 Q
    16=L1C
    17 = L2_C
    */

    m_F[TypeInt(0, 0)] = 1575.42e6;   // GPS L1 C/A
    m_F[TypeInt(0, 2)] = 1176.45e6;   // GPS L5
    m_F[TypeInt(0, 5)] = 1227.6e6;    // GPS L2 P
    m_F[TypeInt(0, 9)] = 1227.6e6;    // GPS L2 P codeless (同L2频率)
    m_F[TypeInt(0, 14)] = 1176.45e6;  // GPS L5 Q (同L5频率)
    m_F[TypeInt(0, 16)] = 1575.42e6;  // GPS L1C (同L1频率)
    m_F[TypeInt(0, 17)] = 1227.6e6;   // GPS L2C (同L2频率)

    // ---------- GLONASS频点 ----------
    /* 0 = L1 C/A  G1
    1 = L2 C/A  G2
    5 = L2 P  G2P
    6= G3 */
    m_F[TypeInt(1, 0)] = 1602.0e6;    // GLONASS G1 (FDMA中心频率)
    m_F[TypeInt(1, 1)] = 1246.0e6;    // GLONASS G2
    m_F[TypeInt(1, 5)] = 1246.0e6;    // GLONASS G2P (同G2频率)
    m_F[TypeInt(1, 6)] = 1277.85e6;   // GLONASS G3

    // ---------- SBAS频点 ----------
    /* 0 = L1 C/A
    6 = L5I
    */
    m_F[TypeInt(2, 0)] = 1575.42e6;   // SBAS L1
    m_F[TypeInt(2, 6)] = 1176.45e6;   // SBAS L5

    // ---------- Galileo频点 ----------
    /* 1= E1B
    2 = E1C
    7=E6C
    12 = E5a Q
    17 = E5b Q
    20 = AltBOC Q
    */
    m_F[TypeInt(3, 1)] = 1575.42e6;   // Galileo E1B
    m_F[TypeInt(3, 2)] = 1575.42e6;   // Galileo E1C (同E1频率)
    m_F[TypeInt(3, 7)] = 1278.75e6;   // Galileo E6C
    m_F[TypeInt(3, 12)] = 1176.45e6;  // Galileo E5a
    m_F[TypeInt(3, 17)] = 1207.14e6;  // Galileo E5b
    m_F[TypeInt(3, 20)] = 1191.795e6; // Galileo AltBOC

    // ---------- BDS频点 ----------
    /*
0 = B1 C/A (B1I)
    17 = B2 C/A (B2I)
    2 = B3 C/A (B3I)
    8 = B1C
    12 = B2a
    19 = B2b
    34=B1X(hg)
    49=B3X(hg)
    47 = B3I(hg)
    */
    m_F[TypeInt(4, 0)] = 1561.098e6;  // BDS B1I (B1频点)
    m_F[TypeInt(4, 17)] = 1207.14e6;  // BDS B2I
    m_F[TypeInt(4, 2)] = 1268.52e6;   // BDS B3I
    m_F[TypeInt(4, 8)] = 1575.42e6;   // BDS B1C (与GPS L1兼容)
    m_F[TypeInt(4, 12)] = 1176.45e6;  // BDS B2a (与GPS L5兼容)
    m_F[TypeInt(4, 19)] = 1207.14e6;  // BDS B2b

    /*hg - 军码频点*/
    m_F[TypeInt(4, 34)] = 1575.42e6;  // BDS B1X (军码)
    m_F[TypeInt(4, 49)] = 1268.52e6;  // BDS B3X (军码)
    m_F[TypeInt(4, 47)] = 1268.52e6;  // BDS B3I (军码)
    // ---------- QZSS频点 ----------
    /*0 = L1 C/A
    14 = L5Q
    17 = L2C
    16 = L1C
    */
    m_F[TypeInt(5, 0)] = 1575.42e6;   // QZSS L1
    m_F[TypeInt(5, 14)] = 1176.45e6;  // QZSS L5
    m_F[TypeInt(5, 17)] = 1227.6e6;   // QZSS L2C
    m_F[TypeInt(5, 16)] = 1575.42e6;  // QZSS L1C
}


// =========================================================================
// 初始化检测门限值(对所有频点统一设置)
// @param threshold: 卫星颗数阈值(>=此数量判定为欺骗)
// @param phsThreshold: 相位差门限值(度)(转换单位后: 周)
// 注: threshold需>=0, phsThreshold需>0
//     仅当threshold != -1 时才设置,仅当phsThreshold > 0 时才设置
// =========================================================================
void SpoofingDoa::initDetectionThreshold(int threshold, double phsThreshold){

    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        int intType = it->first;

        // 各个频点对应的卫星颗数阈值
        if (-1 != threshold){
            m_Detection_Threshold[intType] = threshold;
        }

        // 各个频点对应的相位差阈值(度 -> 周)
        if (phsThreshold > 0){
            m_Detection_PhsThreshold[intType] = phsThreshold / 360.0;
        }

    }

}

// =========================================================================
// 设置连续欺骗检测记录数(跨刀连续确认刀数)
// 取值范围: 1-10, 超出范围默认设为1
// 作用: 对应循环切刀检测中跨刀连续确认所需的连续刀数
//       (getCyclicDetectionData 中 m_ConsecutiveAlarm 累计阈值，
//        对应 Python detection_lib.ALARM_CONSECUTIVE_P 连续确认)
// =========================================================================
void SpoofingDoa::setDetectionRecordNum(int num)
{
    if (num < 1 || num > 10)
    {
        m_Detection_Recodds_Num = 1;
    }
    else
    {
        m_Detection_Recodds_Num = num;
    }
}

// =============================================================================
// == 原 PreparationData.cpp —— GNSS 数据预处理(相位差计算)实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: PreparationData.cpp
// 功能描述: GNSS数据预处理模块(相位差计算)
// 系统角色: 将原始GNSS观测数据转换为可用于DOA计算的相位差数据。
//           这是整个欺骗测向流程的第一步，负责:
//           1. 两通道卫星数据匹配(同卫星同频点配对)
//           2. 载波相位差计算(通道1相位 - 通道2相位)
//           3. 数据格式转换(A格式->B格式的汇总聚合)
//           4. 按频点分类组织数据
//
// 数据流转: GNSSData -> SatelliteDataPhaseDiffA(细粒度) -> SatelliteDataPhaseDiffB(粗粒度)
//   A格式: 每个卫星-切刀组合一条记录
//   B格式: 每个卫星一个对象，包含所有切刀的相位差数组
// =============================================================================
// 数据预处理
// 包括计算相位差


// =========================================================================
// 计算两个通道的相位差(A格式)
// 功能: 将单帧GNSS观测数据转换为SatelliteDataPhaseDiffA列表
// 处理步骤:
//   1. 遍历第一通道的所有卫星，对每颗卫星在第二通道中寻找匹配项
//   2. 匹配条件: 同系统(sys) + 同频点(type) + 同卫星号(prn)
//   3. 如果找到匹配且两通道载噪比均>=阈值，计算相位差并保存
//   4. 如果一通道有而二通道没有，保存为单通道数据(载噪比一方为0)
// 匹配过程中的检查:
//   - 频点有效性检查(频点编码是否在m_F中)
//   - 重复卫星检查(第一通道内同卫星号只保留第一条)
//   - 载噪比阈值过滤(低于m_Snr_Threshold的忽略)
// =========================================================================
void SpoofingDoa::getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA)
{
    dataA.clear();
    int size1 = data.i_PortOneNum;   // 第一通道卫星数
    int size2 = data.i_PortTwoNum;   // 第二通道卫星数
    int typInt = 0;
    bool flg_E = true;
    // data2_index: 记录第二通道中尚未匹配的卫星索引
    // 用于最后补充只在第二通道存在而第一通道不存在的卫星
    vector<int> data2_index;
    data2_index.clear();
    for (int j = 0; j < size2; ++j)
    {
        data2_index.emplace_back(j);
    }
    // 遍历第一通道的所有卫星
    for (int i = 0; i < size1; ++i)
    {
        SatelliteDataPhaseDiffA tp;
        flg_E = true;
        typInt = TypeInt(data.i_PortOne[i].i_Sys, data.i_PortOne[i].i_Type);
        // 频点有效性检查: 不支持的频点跳过
        if (m_F.find(typInt) == m_F.end())
        { // 若不存在系统或频点不进行处理
            continue;
        }

        // 重复卫星检查: 避免同PRN卫星在一帧中出现两次
        int find = 0;
        for (int j = 0; j < i; ++j)
        {
            if (data.i_PortOne[j].i_Prn == data.i_PortOne[i].i_Prn && data.i_PortOne[j].i_Sys == data.i_PortOne[i].i_Sys && data.i_PortOne[j].i_Type == data.i_PortOne[i].i_Type)
            {
                ++find; // 重复卫星
                break;
            }
        }
        if (find)
        {
            continue;  // 跳过重复卫星
        }
        // 在第二通道中寻找匹配的卫星
        for (int j = 0; j < size2; ++j)
        {
            typInt = TypeInt(data.i_PortTwo[j].i_Sys, data.i_PortTwo[j].i_Type);
            if (m_F.find(typInt) == m_F.end())
            { // 若不存在系统或频点不进行处理
                flg_E = false;
                continue;
            }
            // 匹配条件: 同系统 + 同频点 + 同卫星号
            if (data.i_PortOne[i].i_Prn == data.i_PortTwo[j].i_Prn && data.i_PortOne[i].i_Sys == data.i_PortTwo[j].i_Sys && data.i_PortOne[i].i_Type == data.i_PortTwo[j].i_Type)
            {
                flg_E = false;

                // 载噪比阈值过滤
                if (data.i_PortOne[i].i_Snr >= m_Snr_Threshold && data.i_PortTwo[j].i_Snr >= m_Snr_Threshold)
                {
                    // 从待匹配列表中移除(已匹配)
                    data2_index.erase(std::remove(data2_index.begin(), data2_index.end(), j), data2_index.end());

                    tp.i_Snr1 = data.i_PortOne[i].i_Snr;
                    tp.i_Snr2 = data.i_PortTwo[j].i_Snr;
                    tp.i_Prn = data.i_PortOne[i].i_Prn;
                    tp.i_Sys = data.i_PortOne[i].i_Sys;
                    tp.i_Type = data.i_PortOne[i].i_Type;

                    // 载波相位差计算: 通道1相位 - 通道2相位
                    double phs_tp = data.i_PortOne[i].i_Phase - data.i_PortTwo[j].i_Phase;

                    // 取小数部分，归一化到[0,1)周
                    double diff = phs_tp - (long long int)phs_tp;
                    if (diff < 0)
                    {
                        diff = diff + 1;  // 确保结果为非负
                    }
                    tp.i_phase_diff = diff;
                    dataA.emplace_back(tp);
                }
                break;
            }
        }
        if (flg_E)
        {
            // 第一通道有但第二通道没有: 保存为单通道数据
            tp.i_Snr1 = data.i_PortOne[i].i_Snr;
            tp.i_Snr2 = 0;           // 第二通道无数据
            tp.i_Prn = data.i_PortOne[i].i_Prn;
            tp.i_Sys = data.i_PortOne[i].i_Sys;
            tp.i_Type = data.i_PortOne[i].i_Type;
            tp.i_phase_diff = -1;    // 无效相位差
            dataA.emplace_back(tp);
        }
    }
    // 若某频点卫星，只有2通道有，而1通道不存在
    // 补充这些只在第二通道存在的卫星(单通道数据)
    for (int j = 0; j < (int)data2_index.size(); ++j)
    {
        typInt = TypeInt(data.i_PortTwo[j].i_Sys, data.i_PortTwo[j].i_Type);
        if (m_F.find(typInt) == m_F.end())
        { // 若不存在系统或频点不进行处理
            continue;
        }
        SatelliteDataPhaseDiffA tp;
        tp.i_Snr1 = 0;           // 第一通道无数据
        tp.i_Snr2 = data.i_PortTwo[data2_index[j]].i_Snr;
        tp.i_Prn = data.i_PortTwo[data2_index[j]].i_Prn;
        tp.i_Sys = data.i_PortTwo[data2_index[j]].i_Sys;
        tp.i_Type = data.i_PortTwo[data2_index[j]].i_Type;
        tp.i_phase_diff = -1;    // 无效相位差
        dataA.emplace_back(tp);
    }
}

// =========================================================================
// 将A格式数据汇总为B格式
// 功能: 将多帧(m_OneCut_Frams帧)的A格式数据按卫星聚合为B格式
//       A格式每条记录对应一颗星一个切刀位置
//       B格式每个对象对应一颗星所有切刀位置的汇总数组
//
// 聚合过程:
//   1. 遍历所有帧的所有卫星
//   2. 同一颗卫星(同系统+同频点+同PRN)的数据合并到一个B对象中
//   3. 载噪比和相位差按切刀序号(i)存入对应的数组索引
//   4. 如果启用了m_Delete_Prn_Flag，删除任何一刀数据缺失的卫星
// =========================================================================
void SpoofingDoa::getSatelliteDataPhaseDiffB(const vector<vector<SatelliteDataPhaseDiffA>> &dataA, vector<SatelliteDataPhaseDiffB> &dataB)
{
    vector<SatelliteDataPhaseDiffB> tp_dataB;
    tp_dataB.clear();
    int size = (int)dataA.size();
    SatelliteDataPhaseDiffA tpA;
    SatelliteDataPhaseDiffB tpB;
    for (int i = 0; i < size; i++)
    {
        int size2 = (int)dataA[i].size();
        for (int j = 0; j < size2; j++)
        {
            tpA = dataA[i][j];
            bool flg = true;
            int tpBsize = (int)tp_dataB.size();
            // 查找是否已有该卫星的B对象
            for (unsigned int k = 0; k < tpBsize; k++)
            {

                if (tpA.i_Sys == tp_dataB[k].i_Sys && tpA.i_Type == tp_dataB[k].i_Type && tpA.i_Prn == tp_dataB[k].i_Prn)
                {
                    // 找到已有对象，追加这个切刀位置的数据
                    tp_dataB[k].i_Snr1[i] = tpA.i_Snr1;
                    tp_dataB[k].i_Snr2[i] = tpA.i_Snr2;
                    tp_dataB[k].i_phase_diff[i] = tpA.i_phase_diff;
                    flg = false;
                    break;
                }
            }
            if (flg)
            {
                // 新卫星，创建新的B对象
                clearSatelliteDataPhaseDiffB(tpB);
                tpB.i_Sys = tpA.i_Sys;
                tpB.i_Type = tpA.i_Type;
                tpB.i_Prn = tpA.i_Prn;
                tpB.i_Snr1[i] = tpA.i_Snr1;
                tpB.i_Snr2[i] = tpA.i_Snr2;
                tpB.i_phase_diff[i] = tpA.i_phase_diff;
                tpB.i_diffLen = size;  // 记录总切刀数
                tp_dataB.emplace_back(tpB);
            }
        }
    }
    // 数据完整性过滤
    if (1 == m_Delete_Prn_Flag)
    {
        bool tp_flg = true;
        for (int i = 0; i < (int)tp_dataB.size(); i++)
        {
            tp_flg = true;
            tpB = tp_dataB[i];
            // 检查是否所有切刀位置都有有效数据
            for (int j = 0; j < tpB.i_diffLen; j++)
            {
                if (tpB.i_Snr1[j] < 1e-6 || tpB.i_Snr2[j] < 1e-6)
                {
                    tp_flg = false;  // 存在缺失刀
                    break;
                }
            }
            if (tp_flg)
            {
                dataB.emplace_back(tpB);  // 所有刀数据完整，保留
            }
        }
    }
    else
    {
        dataB = tp_dataB;  // 不做过滤，全部保留
    }
    tp_dataB.clear();
}


// =========================================================================
// 将告警数据和卫星系统、频率id绑定(按频点分类)
// 功能: 将A格式的卫星列表按频点编码(TypeInt)分组
// 输出: map<频点编码, vector<该频点的所有卫星相位差数据>>
// 用途: 欺骗检测需要按频点分别处理，因为不同频点的频率和阈值可能不同
// =========================================================================
void SpoofingDoa::getSatelliteDataByType(const std::vector<SatelliteDataPhaseDiffA> &dataA, std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT){
    int typeInt = 0;
    SatelliteDataPhaseDiffA tp;
    vector<SatelliteDataPhaseDiffA> tpVec;
    for (unsigned int i = 0; i < dataA.size(); i++){

        tp = dataA[i];
        typeInt = TypeInt(tp.i_Sys, tp.i_Type);

        if (dataT.find(typeInt) == dataT.end()){ // map中find()函数,表示没有找到(该频点首次出现)
            tpVec.clear();
            tpVec.push_back(tp);
            dataT[typeInt] = tpVec;
        }
        else{
            dataT[typeInt].push_back(tp);  // 追加到已有频点分组
        }

    }
}

// =========================================================================
// 清空SatelliteDataPhaseDiffB结构(重置所有字段为默认值)
// 为什么要清空: B对象在复用前需要清零，否则数组中的旧数据会污染新数据
// 注: 固定数组大小100，对应最多100个切刀位置
// =========================================================================
void SpoofingDoa::clearSatelliteDataPhaseDiffB(SatelliteDataPhaseDiffB &dataB)
{
    dataB.i_Prn = -1;
    dataB.i_Sys = -1;
    dataB.i_Type = -1;
    dataB.i_diffLen = 0;
    for (int i = 0; i < 100; i++)
    {
        dataB.i_Snr1[i] = 0.0;
        dataB.i_Snr2[i] = 0.0;
        dataB.i_phase_diff[i] = 0.0;
    }
}

// =============================================================================
// == 原 Corrected.cpp —— 通道相位校正实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Corrected.cpp
// 功能描述: 校正数据处理模块
// 系统角色: 本文件负责通道相位校正，消除不同接收通道之间的相位不一致性。
//           由于两个接收通道(端口0和端口1)的硬件特性不完全一致，会引入
//           固有的相位偏差(通道间相位误差)，需要通过校正消除。
//
// 校正原理: 在切刀顺序中，当天线对的两个天线相同时(如{1,1})，两个通道
//           接收的是同一天线的信号(通过功分器分配)，理论上相位差应为0。
//           实际测得的相位差即为通道间的固有相位误差。
//           将后续所有相位差测量值减去该校正值，即可消除通道误差。
//
// 关键功能:
//   1. setCorrectionData - 从同天线自校准刀({1,1}=code=0)收集校正数据
//   2. calCorrectionOffset - 计算校正偏移(对应 Python compute_calibration:
//      非 GLONASS 按频点圆形均值统一偏移, GLONASS 逐卫星偏移)
//   3. getCorrectedGnssData - 对所有相位差数据应用校正
//   4. calCorrecteData - 对单个卫星数据减去校正偏移(对应 Python offset_fn)
// =============================================================================
// 校正数据的相关操作

// =========================================================================
// 设置校正数据(主入口)
// 功能: 从同天线自校准刀({1,1} = Python code=0)提取校正信息，
//       计算校正偏移后对所有输入数据应用校正。
// =========================================================================
void SpoofingDoa::setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA)
{
    // 数据长度必须与切刀序列一致
    if (dataA.size() != m_cutSequence.size())
    {
        return;
    }

    // 收集同天线自校准刀的数据(天线对相同的刀，如 {1,1})
    vector<vector<SatelliteDataPhaseDiffA>> calCuts;
    for (unsigned int i = 0; i < dataA.size(); i++)
    {
        if (m_cutSequence[i][0] == m_cutSequence[i][1])
        {
            calCuts.emplace_back(dataA[i]);
        }
    }
    calCorrectionOffset(calCuts);
}

// =========================================================================
// 由同天线校正刀数据计算校正偏移 —— 对应 Python detection_lib.py compute_calibration
// 流程:
//   1. 数据筛选(对应 Python compute_calibration):
//      - 均匀分布:     有效采样数 >= min(校正刀内帧数, MIN_STABLE_SAMPLES)
//        (每刀多帧的稳定性过滤与覆盖判定已由上游 getSmoothData/calSmoothData 完成，
//         不满足的卫星载噪比已置 0，此处按 <1e-3 跳过；
//         不再要求信号从该刀第一帧(第一秒)就存在 —— Python 亦不要求);
//      - 相位差稳定:   最小覆盖弧 < STABILITY_RANGE_DEG(15°);
//   2. 偏移组合:
//      - 非 GLONASS: 每个(系统,频点)取各稳定卫星偏移的圆形均值, 存入 prn=-1;
//      - GLONASS(FDMA): 每颗卫星逐星偏移, 存入对应 prn;
//   3. 仅当本轮 {1,1} 校正刀能算出有效偏移时才重建 m_CorrectionData(与 Python 每周期
//      重算 cal_rx/cal_glo 一致); 无 {1,1} 刀或算不出时保留上一次校正, 沿用其校正。
// =========================================================================
void SpoofingDoa::calCorrectionOffset(const vector<vector<SatelliteDataPhaseDiffA>> &calCuts)
{
    // samples: typeInt -> prn -> [相位差(度), ...]
    map<int, map<int, vector<double>>> samples;  // 相位差样本(度)
    map<int, map<int, double>> sumSnr;           // 载噪比累加(仅用于日志)
    int nCalCuts = 0;

    for (unsigned int c = 0; c < calCuts.size(); ++c)
    {
        ++nCalCuts;
        for (const auto &sat : calCuts[c])
        {
            // 载噪比有效性检查(数据完整性判断，非质量门限)
            // 不满足「从该刀第一帧就存在」的卫星已在上游被置 0，此处自动跳过
            if (sat.i_Snr1 < 1e-3 || sat.i_Snr2 < 1e-3)
            {
                continue;
            }
            int typeInt = TypeInt(sat.i_Sys, sat.i_Type);
            int prn = sat.i_Prn;
            samples[typeInt][prn].emplace_back(sat.i_phase_diff * 360.0);  // 周->度
            sumSnr[typeInt][prn] += (sat.i_Snr1 + sat.i_Snr2) / 2.0;
        }
    }

    // 本轮没有 {1,1} 校正刀数据: 不重建, 沿用上一次校正继续校正
    if (nCalCuts <= 0)
    {
        return;
    }
    int required = (nCalCuts < MIN_STABLE_SAMPLES) ? nCalCuts : MIN_STABLE_SAMPLES;

    // 先写入临时容器, 仅当本轮 {1,1} 刀确实算出有效新校正时才整体替换 m_CorrectionData;
    // 否则(无有效星/全部不稳定)不清空不覆盖, 保留上一次校正值。
    map<int, map<int, SatelliteDataPhaseDiffA>> newCorrection;
    map<int, vector<double>> freqVals;  // 非 GLONASS: typeInt -> [稳定卫星偏移(度)...]

    for (auto &tkv : samples)
    {
        int typeInt = tkv.first;
        int sys = typeInt / 100;
        for (auto &skv : tkv.second)
        {
            int prn = skv.first;
            const vector<double> &samp = skv.second;

            // 1. 相位差稳定(最小覆盖弧 < STABILITY_RANGE_DEG=15°)
            if (circularSpanDeg(samp) >= STABILITY_RANGE_DEG)
            {
                continue;
            }
            // 2. 均匀分布(有效采样覆盖足够帧数)
            if ((int)samp.size() < required)
            {
                continue;
            }

            double meanDeg = circularMeanDeg(samp);
            double meanSnr = sumSnr[typeInt][prn] / samp.size();

            SatelliteDataPhaseDiffA tp;
            tp.i_Sys = sys;
            tp.i_Type = typeInt % 100;
            tp.i_Prn = prn;
            double cyc = fmod(meanDeg / 360.0, 1.0);
            if (cyc < 0)
            {
                cyc += 1.0;  // 归一化到 [0,1) 周
            }
            tp.i_phase_diff = cyc;
            tp.i_Snr1 = meanSnr;
            tp.i_Snr2 = meanSnr;

            if (sys == 1)
            {
                // GLONASS(FDMA): 逐卫星偏移
                newCorrection[typeInt][prn] = tp;
            }
            else
            {
                // 非 GLONASS: 收集用于频点级圆形均值
                freqVals[typeInt].emplace_back(meanDeg);
            }
        }
    }

    // 非 GLONASS: 每(系统,频点)统一偏移 = 稳定卫星偏移的圆形均值
    for (auto &fv : freqVals)
    {
        int typeInt = fv.first;
        double meanDeg = circularMeanDeg(fv.second);
        double cyc = fmod(meanDeg / 360.0, 1.0);
        if (cyc < 0)
        {
            cyc += 1.0;
        }
        SatelliteDataPhaseDiffA tp;
        tp.i_Sys = typeInt / 100;
        tp.i_Type = typeInt % 100;
        tp.i_Prn = -1;
        tp.i_phase_diff = cyc;
        tp.i_Snr1 = 0;
        tp.i_Snr2 = 0;
        newCorrection[typeInt][-1] = tp;
    }

    // {1,1} 校正刀没能算出任何稳定有效偏移: 保留上一次校正(不清空不覆盖), 沿用其校正。
    if (newCorrection.empty())
    {
        return;
    }

    // 整体替换为本轮新校正(与 Python 每周期重算 cal_rx/cal_glo 一致)。
    m_CorrectionData.swap(newCorrection);

    // 日志输出校正数据
    for (auto it = m_CorrectionData.begin(); it != m_CorrectionData.end(); ++it)
    {
        map<int, SatelliteDataPhaseDiffA> tpA;
        tpA = it->second;
        for (auto itt = tpA.begin(); itt != tpA.end(); ++itt)
        {
            SatelliteDataPhaseDiffA tp = itt->second;
            LogSatelliteDataPhaseDiffA(tp);
        }
    }
}

// =========================================================================
// 对所有输入数据进行相位校正
// 遍历所有切刀的所有卫星数据，分别调用calCorrecteData进行校正
// =========================================================================
void SpoofingDoa::getCorrectedGnssData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int size = (int)dataA.size();
    for (int i = 0; i < size; i++)
    {
        for (unsigned int j = 0; j < dataA[i].size(); j++)
        {
            calCorrecteData(dataA[i][j]);
        }
    }
}

// =========================================================================
// 对单个卫星数据进行相位差校正 —— 对应 Python offset_fn
// 校正策略(与 Python compute_calibration / offset_fn 语义一致):
//   - GLONASS(FDMA): 逐卫星偏移(该卫星在校正刀有偏移则减之, 否则偏移0不校正);
//   - 非 GLONASS:    统一频点偏移(prn=-1 综合值, 无则偏移0不校正);
//   无该频点校正数据时: 一律偏移 0（保持原值），对应 Python offset_fn 返回 0；
//   注: 原「测向模式无该频点校正则载噪比置 0 不进入测向」已移除，改为与 Python 一致
//       (仅当某 (系统,频点) 完全无稳定校正星时才按原相位差继续，Python 同样如此)。
// 公式: 校正后相位差 = 原始相位差 - 校正偏移
// =========================================================================
void SpoofingDoa::calCorrecteData(SatelliteDataPhaseDiffA &dataA)
{
    int typeInt = TypeInt(dataA.i_Sys, dataA.i_Type);
    int prn = dataA.i_Prn;

    // 载噪比无效
    if (dataA.i_Snr1 < 1e-3 || dataA.i_Snr2 < 1e-3)
    {
        return;
    }

    auto it = m_CorrectionData.find(typeInt);
    if (it == m_CorrectionData.end())
    {
        return;  // 不存在该频点校正数据：偏移0，保持原值（对齐 Python offset_fn）
    }

    const map<int, SatelliteDataPhaseDiffA> &tp_prnData = it->second;
    const SatelliteDataPhaseDiffA *off = nullptr;
    if (dataA.i_Sys == 1)
    {
        // GLONASS(FDMA): 逐卫星偏移; 无则该星偏移0不校正(对应 Python offset_fn 返回 0)
        auto ps = tp_prnData.find(prn);
        if (ps != tp_prnData.end())
        {
            off = &(ps->second);
        }
    }
    else
    {
        // 非 GLONASS: 统一频点偏移(prn=-1 综合值)
        auto pf = tp_prnData.find(-1);
        if (pf != tp_prnData.end())
        {
            off = &(pf->second);
        }
    }
    if (off != nullptr)
    {
        dataA.i_phase_diff = dataA.i_phase_diff - off->i_phase_diff;
    }
}

// =============================================================================
// == 原 Interf.cpp —— 相关干涉仪测向实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Interf.cpp
// 功能描述: 相关干涉仪测向模块
// 系统角色: 实现基于相关干涉仪(Correlative Interferometer)的DOA算法。
//           相关干涉仪是欺骗测向系统的主要DOA方法，其原理是:
//           1. 预先计算360个角度上的理论相位差(形成"指纹库")
//           2. 将实测相位差与每个角度的理论相位差进行相关匹配
//           3. 相关性最高的角度即为估计的到达角
//
// DOA算法选择(通过m_Doa_Arithmetic配置):
//   1=相关干涉仪(使用理论相位差模板) - 基于阵列几何参数计算
//   2=幅相法(使用仿真阵列流型) - 基于电磁仿真数据
//   3=相关干涉仪+阵列仿真数据(使用仿真相位差) - 基于仿真提取的相位差
//
// 关键功能:
//   1. getResultInterferDoa - 干涉仪测向调度(全向/定向天线分派)
//   2. initTheory - 初始化理论相位差模板(基于阵列几何)
//   3. initTheoryBySimulatePhase - 初始化仿真相位差模板(基于仿真数据)
// =============================================================================
// 相关干涉仪测向

// =========================================================================
// 相关干涉仪测向调度函数
// 根据天线类型调用对应的数据处理函数:
//   全向天线(m_antnenaType=0): 调用setInterferInfoDataOmni
//   定向天线(m_antnenaType=1): 调用setInterferInfoDataDirect
// 构建好InterferInfo数据后，统一调用calAngle进行测向计算
// =========================================================================
void SpoofingDoa::getResultInterferDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    string nowT = getNowTime();
    std::map<int, std::map<int, InterferInfo>> inferInfoData;
    if (0 == m_antnenaType)
    {
        PublicSpace::Log("set Interfer Info Data Omni   %s\n", nowT.c_str());
        setInterferInfoDataOmni(dataB, inferInfoData);    // 全向天线
    }
    else
    {
        nowT = getNowTime();
        PublicSpace::Log("set Interfer Info Data Direct   %s\n", nowT.c_str());
        setInterferInfoDataDirect(dataB, inferInfoData);  // 定向天线
    }
    nowT = getNowTime();
    PublicSpace::Log("cal angle    %s\n", nowT.c_str());
    calAngle(inferInfoData);  // 执行相关干涉仪角度计算
}


// =========================================================================
// 初始化理论相位差模板(基于阵列几何参数)
// 原理: 对于均匀圆阵(UCA), 天线i与天线j之间的理论相位差为:
//       phi = 2*pi*r/lambda * (sin(theta)*cos(alpha_i-phi0) - sin(theta)*cos(alpha_j-phi0))
//       其中: r=阵列半径, lambda=波长=c/f, theta=仰角, alpha=天线方位角
//
// 流程:
//   1. 遍历所有工作频点(m_F)
//   2. 获取该频点的阵列半径(m_R)
//   3. 调用ArithmeticDoa::calPhaseTheory计算360个角度的理论相位差
//   4. 可选: 如启用虚拟阵列(m_Virtual_Flag=1)，使用getVirtualTheory扩展
//   5. 结果存入m_Theory[typeInt]
// =========================================================================
void SpoofingDoa::initTheory(void){
    int typeInt = 0;
    double f = 0;
    double r = 0.0;

    vector<vector<double>> theory;
    vector<vector<double>> tp_theory;
    for (auto it = m_F.begin(); it != m_F.end(); ++it)
    {
        theory.clear();
        tp_theory.clear();
        typeInt = it->first;
        f = it->second;           // 频率(Hz)
        r = m_R[typeInt];         // 阵列半径(米)
        ArithmeticDoa::calPhaseTheory(f, r, m_AntennaNum, tp_theory); // 计算理论相位差

        theory = tp_theory;
        // 虚拟阵列扩展: 通过数学变换构造虚拟天线对，增加相位差数据量
        if (1 == m_Virtual_Flag)
        {
            ArithmeticDoa::getVirtualTheory(m_AntennaNum, tp_theory, m_Virtual_Multiple, theory);
        }

        m_Theory[typeInt] = theory; // 保存各个频点的理论相位差模板
    }
}


// =========================================================================
// 初始化基于阵列仿真相位差的理论模板
// 与initTheory的区别: 不使用几何公式计算理论相位，而是从电磁仿真数据文件中
// 读取预先仿真好的相位差数据。仿真数据比理论公式更精确，能反映实际天线耦合、
// 多径反射等非理想因素。
//
// 流程:
//   1. 从仿真数据文件加载各频率的相位差模板
//   2. 对于每个工作频点，在仿真数据中寻找频率最接近的模板
//   3. 可选: 如启用虚拟阵列，使用getVirtualTheory扩展
//   4. 如果没有任何仿真数据可用，自动切换到理论相位差模板(initTheory)
//   5. 结果存入m_Theory[typeInt]
// =========================================================================
void SpoofingDoa::initTheoryBySimulatePhase(void){
    int typeInt = 0;
    double f = 0;
    std::map<double, std::vector<std::vector<double>>> tp_theory2;
    vector<vector<double>> tp_theory;
    vector<vector<double>> tp_theory3;
    int num = m_All_Simulate_data_Fre.size();
    vector<double> erse_index;
    erse_index.clear();
    for (int i = 0; i < num; i++){

        tp_theory3.clear();
        tp_theory.clear();
        f = m_All_Simulate_data_Fre[i];
        int flg = 0;
        // 从仿真数据文件读取该频率的相位差模板
        flg = ArithmeticDoa::getSimulatePhase(m_Simulate_Data_file, f, tp_theory);

        if (-1 == flg){
            continue;  // 该频率无仿真数据
        }
        erse_index.emplace_back(f);
        tp_theory3 = tp_theory;

        // 虚拟阵列扩展
        if (1 == m_Virtual_Flag){
            ArithmeticDoa::getVirtualTheory(m_AntennaNum, tp_theory, m_Virtual_Multiple, tp_theory3);
        }

        tp_theory2[f] = tp_theory3;
    }
    // 如果没有任何仿真数据可用，自动切换到理论相位差模板
    if (0 == tp_theory2.size()){
        m_Doa_Arithmetic = 1; // 若没有阵列仿真数据，则自动转成使用相关干涉仪算法
        initTheory();
        return;
    }
    m_All_Simulate_data_Fre.clear();

    for (int i = 0; i < (int)erse_index.size(); i++){
        m_All_Simulate_data_Fre.emplace_back(erse_index[i]);
    }

    tp_theory.clear();
    double tp_f_min = 99999e8;
    double tp_f2 = 0;
    double tp_diff = 0.0;
    double f2 = 0.0;

    // 为每个工作频点匹配最接近的仿真频率
    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        typeInt = it->first;
        f = it->second;  // 工作频点实际频率
        tp_f_min = 99999e8;

        for (int i = 0; i < (int)m_All_Simulate_data_Fre.size(); i++){
            tp_f2 = m_All_Simulate_data_Fre[i];
            tp_diff = abs(tp_f2 - f);  // 频率差

            if (tp_f_min > tp_diff){
                tp_f_min = tp_diff;
                f2 = tp_f2;  // 记录最接近的仿真频率
            }

        }

        m_Theory[typeInt] = tp_theory2[f2]; // 保存各个频点的理论相位差模板(使用最接近频率的仿真数据)
    }

}

// =============================================================================
// == 原 AmpPhase.cpp —— 幅相法测向实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: AmpPhase.cpp
// 功能描述: 幅相法(Amplitude-Phase)测向模块
// 系统角色: 实现基于仿真阵列流型的幅相法DOA。与相关干涉仪不同，
//           幅相法不仅利用相位差信息，还利用幅度差(载噪比)信息进行测向。
//           仿真阵列流型(SimulateA)通过电磁仿真软件(如HFSS/CST)生成，
//           包含每个角度上的复数导向矢量(幅度和相位响应)。
//
// 核心算法: 将实测幅度和相位与仿真阵列流型进行匹配，计算相关系数，
//           相关性最高的角度即为估计的到达角。
//
// 关键功能:
//   1. getResultAmpPhaseDoa - 幅相法测向主函数(对各卫星逐星计算)
//   2. initSimulateA - 初始化仿真阵列流型模板
// =============================================================================
// 幅相法测向

// =========================================================================
// 幅相法测向主函数
// 算法流程(对每颗卫星):
//   1. 获取该频点的仿真阵列流型(m_SimulateA)
//   2. 构建幅度向量(ampsnr)和相位差向量(phase_diff)
//   3. 填充InterferInfo结构(包含幅度和相位差)
//   4. 调用ArithmeticDoa::calAmpPhase进行幅相法匹配
//   5. 记录匹配结果(角度、质量)到m_AngleResultData
//
// 幅度处理: 载噪比(SNR)通过10^(SNR/20)转换为线性幅度
// 相位处理: 相位差(周)乘以2π转换为弧度
// 参考通道(第0刀): 幅度取平均载噪比，相位差设为0
// =========================================================================
void SpoofingDoa::getResultAmpPhaseDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    // 清空上一轮的结果
    m_AngleResultData.clear();
    int size = dataB.size();
    int typeInt = 0;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    std::vector<std::vector<complex<double>>> tp_SimulateA;
    vector<double> diff;
    SatelliteDataPhaseDiffB tpB;
    int num1 = (int)m_cutSequence.size();
    vector<double> amp_snr;
    amp_snr.resize(num1);
    vector<double> phase_diff;
    phase_diff.resize(num1);
    double tmp1 = 0.0;
    double tmp2 = 0.0;
    InterferInfo data;
    data.i_Start = 0;
    data.i_End = 359;  // 全方向搜索
    map<int, vector<double>> tp_peseudo;

    // 设置天线对序列(与切刀顺序一致)
    for (int i = 0; i < num1; i++)
    {
        data.i_AntennaSq[i][0] = m_cutSequence[i][0];
        data.i_AntennaSq[i][1] = m_cutSequence[i][1];
    }
    data.i_Phase_Len = num1;

    // 对每颗卫星独立进行幅相法测向
    for (int i = 0; i < size; i++)
    {
        tp_alarms.clear();
        tpB = dataB[i];
        typeInt = TypeInt(tpB.i_Sys, tpB.i_Type);
        tp_SimulateA.clear();
        tp_SimulateA = m_SimulateA[typeInt];  // 获取该频点的仿真阵列流型
        double sum_snr = 0.0;
        int num2 = tpB.i_diffLen;
        int prn = dataB[i].i_Prn;
        tp_peseudo.clear();

        // 构建幅度向量和相位差向量(从第1刀开始，第0刀为参考)
        for (int j = 1; j < num2; j++)
        {
            sum_snr = sum_snr + tpB.i_Snr1[j];
            // 载噪比(dB)转换为线性幅度: 10^(SNR/20)
            amp_snr[j] = pow(10, (tpB.i_Snr2[j] / 20));
            // 相位差(周)转换为弧度: 相位差 * 2π
            phase_diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
            data.i_Amp[j] = pow(10, (tpB.i_Snr2[j] / 20));
            data.i_Phase_Diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
        }
        // 参考通道(第0刀): 相位差=0，幅度为平均载噪比
        tmp2 = sum_snr / (tpB.i_diffLen - 1);
        tmp1 = tmp2;
        amp_snr[0] = pow(10, (tmp1 / 20));
        phase_diff[0] = 0;
        data.i_Amp[0] = pow(10, (tmp1 / 20));
        data.i_Phase_Diff[0] = 0;

        double angle;
        double quality;
        diff.clear();

        // 调用底层幅相法匹配算法
        ArithmeticDoa::calAmpPhase(tp_SimulateA, data, angle, quality, diff);

        // 记录最大载噪比
        m_Max_Snr[typeInt][tpB.i_Prn] = tmp2;

        // 构建告警/测向结果
        tp_alarm.i_Prn = tpB.i_Prn;
        tp_alarm.i_Angle = (int)angle;
        tp_alarm.i_Quality = quality;

        // 按频点汇总结果到m_AngleResultData
        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            tp_alarms.emplace_back(tp_alarm);
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo[prn] = diff;  // 保存伪谱值
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
            }
        }
        else
        {
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo = m_Pseudo_Spectrum_Value[typeInt];
                tp_peseudo[prn] = diff;
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
            }
            tp_alarms = m_AngleResultData[typeInt];
            tp_alarms.emplace_back(tp_alarm);
        }

        m_AngleResultData[typeInt] = tp_alarms;
    }
}


// =========================================================================
// 初始化幅相法理论模版(仿真阵列流型)
// 流程:
//   1. 从仿真数据文件加载各频率的阵列流型数据
//   2. 对于每个工作的GNSS频点，在仿真数据中寻找频率最接近的模板
//      注: 仿真数据可能只有有限几个频率点，需要匹配最近的频率
//   3. 如果没有任何仿真数据可用，自动切换到相关干涉仪算法(m_Doa_Arithmetic=1)
// 结果: 存放在m_SimulateA中，按频点编码索引
// =========================================================================
void SpoofingDoa::initSimulateA(void){
    int typeInt = 0;
    double f = 0;
    std::map<double, std::vector<std::vector<complex<double>>>> tp_SimulateA;
    tp_SimulateA.clear();
    vector<vector<complex<double>>> simulateA;
    int num = m_All_Simulate_data_Fre.size();
    vector<double> erse_index;
    erse_index.clear();

    // 加载所有仿真频率的阵列流型
    for (int i = 0; i < num; i++){
        f = m_All_Simulate_data_Fre[i];
        simulateA.clear();
        ArithmeticDoa::calAmpPhaseSimulateA(m_Simulate_Data_file, f, simulateA);  // 从文件读取
        if (0 == simulateA.size()){
            continue;  // 该频率无仿真数据
        }
        erse_index.emplace_back(f);
        tp_SimulateA[f] = simulateA;
    }

    // 如果没有加载到任何仿真数据，自动切换到相关干涉仪算法
    if (0 == tp_SimulateA.size()){
        m_Doa_Arithmetic = 1; // 若没有阵列仿真数据，则自动转成使用相关干涉仪算法
        initTheory();
        return;
    }

    m_All_Simulate_data_Fre.clear();

    for (int i = 0; i < (int)erse_index.size(); i++){
        m_All_Simulate_data_Fre.emplace_back(erse_index[i]);
    }

    double tp_f_min = 999999e8;
    double tp_f2 = 0;
    double tp_diff = 0.0;
    double f2 = 0.0;

    // 为每个工作频点匹配最接近的仿真频率
    for (auto it = m_F.begin(); it != m_F.end(); ++it){

        typeInt = it->first;
        f = it->second;  // 工作频点实际频率
        tp_f_min = 999999e8;
        for (int i = 0; i < (int)m_All_Simulate_data_Fre.size(); i++){

            tp_f2 = m_All_Simulate_data_Fre[i];
            tp_diff = abs(tp_f2 - f);  // 频率差
            if (tp_f_min > tp_diff){
                tp_f_min = tp_diff;
                f2 = tp_f2;  // 记录最接近的仿真频率
            }

        }

        PublicSpace::Log("Fre = %.1f,f= %.1f\n", f, f2);
        m_SimulateA[typeInt] = tp_SimulateA[f2]; // 保存各个频点的仿真阵列流型(使用最接近频率的模板)
    }
}

// =============================================================================
// == 原 Omni.cpp —— 全向天线测向数据实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Omni.cpp
// 功能描述: 全向天线测向数据处理模块
// 系统角色: 处理全向天线(omni-directional antenna)模式下的干涉仪测向准备。
//           全向天线在所有方向上具有基本一致的增益，信号到达角范围为0-359度。
//           与定向天线不同，全向天线不需要根据信号强度限制搜索扇区。
//
// 关键功能:
//   setInterferInfoDataOmni - 构建全向天线模式下的干涉仪输入数据
//                             所有天线对参与测向，搜索范围为全向(0-359度)
// =============================================================================
// 全向天线测向的相关处理

// =========================================================================
// 全向天线: 构建干涉仪测向输入数据
// 与定向天线(Directed.cpp)的区别:
//   1. 全向天线搜索范围为0-359度(全向)，定向天线限制搜索扇区
//   2. 全向天线不需要根据载噪比选择天线扇区
//   3. 全向天线所有方向增益一致，不会因方向不同导致信号强度差异
// 流程:
//   1. 遍历所有卫星的相位差数据
//   2. 调用calAngleUseAntenna验证和构建InterferInfo结构
//   3. 设置搜索范围为全向(0-359度)
//   4. 将构建好的InterferInfo存入inferInfoData(map<频点编码, map<卫星号, InterferInfo>>)
// =========================================================================
void SpoofingDoa::setInterferInfoDataOmni(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData)
{
    int size = (int)dataB.size();
    int typInt = 0;
    int prn = 0;
    SatelliteDataPhaseDiffB tp1;
    int startAngle = 0;
    int endAngle = 359;  // 全向天线: 全方向搜索
    for (int i = 0; i < size; i++)
    {
        tp1 = dataB[i];
        typInt = TypeInt(tp1.i_Sys, tp1.i_Type);
        prn = tp1.i_Prn;

        InterferInfo tp2;
        int flg;
        // 根据天线关系和相位差构建干涉仪输入数据
        calAngleUseAntenna(tp1, tp2, flg);
        if (0 == flg)
        {
            continue;  // 有效天线对数量不足，跳过此卫星
        }
        // 设置搜索范围(全向天线无方向限制)
        tp2.i_Start = startAngle;
        tp2.i_End = endAngle;
        // 组织到按频点+卫星号索引的map中
        inferInfoData[typInt][prn] = tp2;
    }
    Log("set Interfer Info Data sucess....\n");
}

// =============================================================================
// == 原 Directed.cpp —— 定向天线测向数据实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Directed.cpp
// 功能描述: 定向天线测向数据处理模块
// 系统角色: 处理定向天线(directional antenna)模式下的干涉仪测向准备。
//           定向天线在不同方向具有不同增益，因此可以利用载噪比信息
//           缩小到达角搜索范围，提高测向效率和精度。
//
// 定向天线测向特点:
//   1. 每个天线阵元有独立的扇区覆盖范围(如7阵元各覆盖约51度)
//   2. 通过比较各天线的载噪比可确定信号大致的到达扇区
//   3. 搜索范围可缩小到特定扇区(startAngle ~ endAngle)，减少计算量
//   4. 与全向天线测向的主要区别在于搜索范围约束
//
// 关键功能:
//   1. setInterferInfoDataDirect - 定向天线模式的干涉仪数据构建
//   2. getUseAntennaBySnr - 利用载噪比选择测向用的天线和搜索扇区
// =============================================================================
// 定向天线的相关操作

// =========================================================================
// 定向天线: 构建干涉仪测向输入数据
// 与全向天线的区别: 需要根据天线方向特性确定搜索范围(startAngle, endAngle)
// 两种天线选择模式:
//   模式1 (m_getUseAntennaBySnr_Flag=1): 利用载噪比智能选择天线和扇区
//   模式2 (m_getUseAntennaBySnr_Flag=0): 按固定扇区划分
//                                       每个天线覆盖 360/m_AntennaNum 度的扇区
//                                       基于校正数据所在的天线确定主天线和扇区
// =========================================================================
void SpoofingDoa::setInterferInfoDataDirect(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData)
{
    int size = (int)dataB.size();
    int typInt = 0;
    int prn = 0;
    SatelliteDataPhaseDiffB tp1;
    int startAngle = 0;
    int endAngle = 359;
    int max_index = 0;
    vector<int> index;
    for (int i = 0; i < size; i++)
    {
        tp1 = dataB[i];
        typInt = TypeInt(tp1.i_Sys, tp1.i_Type);
        prn = tp1.i_Prn;

        if (1 == m_getUseAntennaBySnr_Flag)
        {
            // 模式1: 由载噪比确定用于测向的天线和搜索扇区
            getUseAntennaBySnr(tp1, startAngle, endAngle);
        }
        else
        {
            // 模式2: 按固定扇区划分(每个天线覆盖360/7≈51度)
            int per = (int)360 / m_AntennaNum;  // 每个天线的扇区角度

            int len = m_cutSequence.size();
            int doaNum = 0;
            vector<int> correction_index; // 校正数据的切刀序号(天线对相同)
            vector<int> doa_index;        // 测向数据的切刀序号(天线对不同)

            // 分离校正刀和测向刀
            for (int k1 = 0; k1 < len; k1++)
            {

                if (m_cutSequence[k1][0] == m_cutSequence[k1][1])
                {
                    // 同天线自检刀(用于校正数据)
                    correction_index.emplace_back(k1);
                    tp1.i_Snr1[k1] = 0.0;
                    tp1.i_Snr2[k1] = 0.0;
                }
                else
                {
                    // 不同天线对(用于测向数据)
                    doa_index.emplace_back(k1);
                    if (tp1.i_Snr1[k1] < 1e-6 || tp1.i_Snr2[k1] < 1e-6)
                    {
                        // 某个天线对数据无效时，相邻天线对也可能受影响
                        if (k1 == 2)
                        {
                            tp1.i_Snr1[1] = 0.0;
                            tp1.i_Snr2[1] = 0.0;
                        }
                        if (k1 == 3)
                        {
                            tp1.i_Snr1[4] = 0.0;
                            tp1.i_Snr2[4] = 0.0;
                        }
                        continue;
                    }

                    doaNum++;
                }
            }

            // 确定主天线(校正数据对应的天线)
            int mainAnten = m_cutSequence[correction_index[0]][0];
            // 计算扇区范围
            startAngle = (mainAnten - 2) * per;
            endAngle = (mainAnten)*per;
            // 比较首尾天线对的载噪比，确定信号来向在扇区的前半部还是后半部
            int doaNum2 = (int)doa_index.size();
            double tp_snr1 = tp1.i_Snr2[doa_index[0]];
            double tp_snr2 = tp1.i_Snr2[doa_index[doaNum2 - 1]];
            if (tp_snr1 < tp_snr2)
            {
                // 尾部载噪比更大，信号来自扇区后半部
                if (doaNum > m_Doa_Cut_Num)
                {
                    tp1.i_Snr1[doa_index[0]] = 0.0;
                    tp1.i_Snr2[doa_index[0]] = 0.0;
                }

                startAngle = startAngle + 30;
                endAngle = endAngle + 30;
            }
            else
            {
                // 头部载噪比更大，信号来自扇区前半部
                if (doaNum > m_Doa_Cut_Num)
                {
                    tp1.i_Snr1[doa_index[doaNum - 1]] = 0.0;
                    tp1.i_Snr2[doa_index[doaNum - 1]] = 0.0;
                }
                startAngle = startAngle - 30;
                endAngle = endAngle - 30;
            }
        }

        // 构建InterferInfo并进行搜索范围约束
        InterferInfo tp2;
        int flg;
        calAngleUseAntenna(tp1, tp2, flg);
        if (0 == flg)
        {
            continue;
        }
        // 设置定向天线的搜索范围约束
        tp2.i_Start = startAngle;
        tp2.i_End = endAngle;
        inferInfoData[typInt][prn] = tp2;
    }
}

// =========================================================================
// 利用载噪比确定用于测向的天线序号和搜索扇区
// 核心思想: 定向天线各阵元在不同方向具有不同的增益，
//           载噪比最高的天线说明信号来自该天线的覆盖方向。
// 算法流程:
//   1. 分离校正刀(天线对相同)和测向刀(天线对不同)
//   2. 计算各天线对两个通道的平均载噪比
//   3. 按载噪比从高到低排序
//   4. 从载噪比最高的天线开始，向两侧扩展连续的相邻天线
//   5. 直到收集到足够的连续天线(>=m_Doa_Cut_Num)
//   6. 调用ArithmeticDoa::calAngleSerchRange确定搜索扇区范围
// =========================================================================
void SpoofingDoa::getUseAntennaBySnr(SatelliteDataPhaseDiffB &dataB, int &startAngle, int &endAngle)
{
    int len = dataB.i_diffLen;
    if (len != (int)m_cutSequence.size())
    {
        cout << "error:data len don't cutSequenceNum!!!" << endl;
        return;
    }
    vector<double> tp_Snr1;
    tp_Snr1.resize(len);
    vector<double> tp_Snr2;
    tp_Snr2.resize(len);

    vector<double> tp_Snr;
    tp_Snr.resize(len);
    vector<int> index;

    vector<int> correction_index; // 校正数据的切刀序号(天线对相同)
    vector<int> doa_index;        // 测向数据的切刀序号(天线对不同)
    int doaNum = 0;
    for (int i = 0; i < len; i++)
    {

        if (m_cutSequence[i][0] == m_cutSequence[i][1])
        {
            correction_index.emplace_back(i);  // 校正刀
        }
        else
        {
            doa_index.emplace_back(i);  // 测向刀
            tp_Snr[doaNum] = (dataB.i_Snr1[i] + dataB.i_Snr2[i]) / 2;  // 两通道平均载噪比
            tp_Snr1[doaNum] = dataB.i_Snr1[i];
            tp_Snr2[doaNum] = dataB.i_Snr2[i];
            index.emplace_back(doaNum);
            doaNum++;
        }
        // 先清空所有天线对的载噪比，后续只恢复选中的
        dataB.i_Snr1[i] = 0.0;
        dataB.i_Snr2[i] = 0.0;
    }
    // 按载噪比从高到低排序(降序)
    std::sort(index.begin(), index.end(), [&](int i, int j)
              { return tp_Snr[i] > tp_Snr[j]; });

    // 从载噪比最高的天线(index[0])开始扩展
    int max_index = index[0];  // 载噪比最高的天线对索引
    int last_index = 0;
    int next_index = 0;
    vector<int> tp_index;
    tp_index.emplace_back(max_index);
    int size = 1;
    int flg = 1;
    int num = 0;

    // 检查天线对是否构成闭合环路(首尾天线对的编号连续)
    // 例如: {1,2},{2,3},{3,4},...,{7,1} 构成闭合环
    if (m_cutSequence[doa_index[0]][0] == m_cutSequence[doa_index[doaNum - 1]][1] || m_cutSequence[doa_index[0]][1] == m_cutSequence[doa_index[doaNum - 1]][0])
    {

        // 从最高载噪比的天线向两侧扩展，直到收集到足够的连续天线
        while (size < m_Doa_Cut_Num && flg > 0 && num < 20)
        {

            last_index = (tp_index[0] - 1 + doaNum) % doaNum;      // 向左侧扩展
            next_index = (tp_index[size - 1] + 1) % doaNum;        // 向右侧扩展
            flg = 0;
            for (int i = 1; i < doaNum; i++)
            {
                if (tp_Snr1[index[i]] < 1e-6 || tp_Snr2[index[i]] < 1e-6)
                {
                    break;  // 载噪比无效
                }
                if (index[i] == last_index)
                {
                    tp_index.insert(tp_index.begin() + 0, index[i]);  // 插入到左侧
                    flg = 1;
                    break;
                }
                if (index[i] == next_index)
                {
                    tp_index.emplace_back(index[i]);  // 追加到右侧
                    flg = 1;
                    break;
                }
            }
            size = (int)tp_index.size();
            num++;
        }
        // 重新组织选中的天线对数据
        vector<int>().swap(index);
        index.resize(size);
        double maxSnr2 = -99;
        for (int i = 0; i < size; i++)
        {
            int tp_index2 = doa_index[tp_index[i]];
            // 恢复选中的天线对的载噪比(其他保持为0)
            dataB.i_Snr1[tp_index2] = tp_Snr1[tp_index[i]];
            dataB.i_Snr2[tp_index2] = tp_Snr2[tp_index[i]];
            index[i] = tp_index2;
            if (maxSnr2 < tp_Snr[tp_index[i]])
            {
                maxSnr2 = tp_Snr[tp_index[i]];
                max_index = tp_index2;
            }
        }
        // 根据选中的天线确定搜索扇区范围
        ArithmeticDoa::calAngleSerchRange(index, m_cutSequence, m_AntennaNum, max_index, startAngle, endAngle);
    }
}

// =============================================================================
// == 原 Alarm.cpp —— 欺骗检测与告警实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Alarm.cpp
// 功能描述: 欺骗信号检测与告警模块
// 系统角色: 本文件实现了欺骗信号检测方法，是欺骗测向系统的检测前端:
//           1. 相位差法检测(calAlarmByPhaseDiff，载噪比≥35dB 作为数据质量门限)
// 检测原理: 欺骗信号由同一干扰源发射，因此多颗卫星的相位差相近(来自同一方向)
// =============================================================================

// =========================================================================
// 欺骗检测参数（对应 Python detection_lib.py）
// =========================================================================
const double SpoofingDoa::CNR_MIN_DB = 35.0;           // 载噪比质量门限：两端口都需 ≥35dB
const double SpoofingDoa::STABILITY_RANGE_DEG = 15.0;  // 稳定性阈值：最小覆盖弧 < 15° 判为稳定
                                                       // （对齐 Python detection_lib.STABILITY_RANGE_DEG=15，2026-09 版）
const int SpoofingDoa::MIN_STABLE_SAMPLES = 3;         // 每刀至少需要的有效采样帧数（对齐 Python MIN_STABLE_SAMPLES=3）

// =========================================================================
// 归一化角度到 [-180°, 180°)，对应 Python detection_lib.normalize_angle_180
// =========================================================================
double SpoofingDoa::normalizeAngle180(double deg)
{
    double r = fmod(deg + 180.0, 360.0);
    if (r < 0)
    {
        r += 360.0;
    }
    return r - 180.0;
}

// =========================================================================
// 角度圆形均值(度)，对应 Python circular_mean
// =========================================================================
double SpoofingDoa::circularMeanDeg(const std::vector<double> &degs)
{
    double s = 0.0;
    double c = 0.0;
    for (double d : degs)
    {
        double rad = d * PI / 180.0;
        s += sin(rad);
        c += cos(rad);
    }
    return atan2(s, c) * 180.0 / PI;
}

// =========================================================================
// 最小覆盖弧跨度(度)，对应 Python circular_span
// =========================================================================
double SpoofingDoa::circularSpanDeg(const std::vector<double> &degs)
{
    int n = (int)degs.size();
    if (n <= 1)
    {
        return 0.0;
    }
    vector<double> a;
    a.reserve(n);
    for (double d : degs)
    {
        a.push_back(normalizeAngle180(d));
    }
    sort(a.begin(), a.end());
    double maxGap = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double gap = fmod(a[(i + 1) % n] - a[i], 360.0);
        if (gap < 0)
        {
            gap += 360.0;
        }
        if (gap > maxGap)
        {
            maxGap = gap;
        }
    }
    return 360.0 - maxGap;
}

// =========================================================================
// 折叠到 [0°, 180°)，消除跳半周(±180°)歧义，对应 Python fold_half_cycle
// =========================================================================
double SpoofingDoa::foldHalfCycle(double deg)
{
    double r = fmod(normalizeAngle180(deg), 180.0);
    if (r < 0)
    {
        r += 180.0;
    }
    return r;
}

// =========================================================================
// 180° 半周圆上的最小覆盖弧跨度(度)，对应 Python circular_span_180
// =========================================================================
double SpoofingDoa::circularSpan180Deg(const std::vector<double> &degs)
{
    int n = (int)degs.size();
    if (n <= 1)
    {
        return 0.0;
    }
    vector<double> a;
    a.reserve(n);
    for (double d : degs)
    {
        double r = fmod(d, 180.0);
        if (r < 0)
        {
            r += 180.0;
        }
        a.push_back(r);
    }
    sort(a.begin(), a.end());
    double maxGap = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double gap = fmod(a[(i + 1) % n] - a[i], 180.0);
        if (gap < 0)
        {
            gap += 180.0;
        }
        if (gap > maxGap)
        {
            maxGap = gap;
        }
    }
    return 180.0 - maxGap;
}

// =========================================================================
// 利用相位差计算是否告警(相位差法欺骗检测)
// 核心思想: 真实卫星来自不同方向，相位差各不相同；
//           欺骗信号来自同一干扰源，相位差应当相近(差异在阈值内)
// 检测方法对应 Python detection_lib.py 的 cluster_satellites：
//   1. 数据筛选：两端口载噪比均 ≥ 35dB 的观测才参与检测（载噪比质量门限）；
//   2. 相位差由「周」转「度」并归一化到 [-180°, 180°)，消除 0/360 边界歧义；
//   3. 按相位差排序后，用滑动窗口找「跨度 < 相位差阈值」的最大卫星集合
//      （比原「以某星为参考数邻近星」更准确：聚类跨度直接受阈值约束）；
//   4. 最大聚类卫星数超过检测阈值(严格大于)则判为欺骗，与 Python 的 cnt > count_thr 一致。
// 输出:
//   alarmSatelliteData: 判定为欺骗的卫星相位差数据列表
//   alarm: 1=检测到欺骗, 0=未检测到
// =========================================================================
void SpoofingDoa::calAlarmByPhaseDiff(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm) // 利用相位差计算是否告警
{
    alarmSatelliteData.clear();
    alarm = 0;

    double phsThresholdDeg = m_Detection_PhsThreshold[typeInt] * 360.0;  // 相位差阈值：周 -> 度

    // 1. 数据筛选 + 相位差归一化：valid 存 (校正相位差[度], dataA 索引)
    vector<pair<double, int>> valid;
    for (unsigned int i = 0; i < dataA.size(); ++i)
    {
        if (dataA[i].i_Snr1 < CNR_MIN_DB || dataA[i].i_Snr2 < CNR_MIN_DB)
        {
            continue;  // 载噪比不达标，不使用该观测
        }
        double ph = normalizeAngle180(dataA[i].i_phase_diff * 360.0);  // [0,360) -> [-180,180)
        valid.emplace_back(ph, (int)i);
    }

    int n = (int)valid.size();
    if (n < 2)
    {
        return;  // 少于 2 颗有效卫星，无法形成聚集
    }

    // 2. 排序 + 复制一份(整体 +360°) 解决簇跨越 0/360 边界的问题
    sort(valid.begin(), valid.end());
    vector<pair<double, int>> ext;
    ext.reserve(2 * n);
    for (const auto &p : valid)
    {
        ext.emplace_back(p.first, p.second);
    }
    for (const auto &p : valid)
    {
        ext.emplace_back(p.first + 360.0, p.second);
    }

    // 3. 滑动窗口：窗口内相位差跨度 < 阈值，找最大聚类
    int bestCount = 0;
    vector<int> bestIds;
    int left = 0;
    for (int right = 0; right < (int)ext.size(); ++right)
    {
        while (ext[right].first - ext[left].first >= phsThresholdDeg)
        {
            ++left;
        }
        // 窗口最多包含 n 颗星，防止同一颗星(复制份)被重复计数
        while (right - left + 1 > n)
        {
            ++left;
        }
        set<int> ids;
        for (int k = left; k <= right; ++k)
        {
            ids.insert(ext[k].second);
        }
        if ((int)ids.size() > bestCount)
        {
            bestCount = (int)ids.size();
            bestIds.assign(ids.begin(), ids.end());
        }
    }

    // 4. 判定：最大聚类卫星数超过阈值（严格大于，与 Python 的 cnt > count_thr 一致）
    if (bestCount > m_Detection_Threshold[typeInt])
    {
        alarm = 1;
        for (int idx : bestIds)
        {
            alarmSatelliteData.emplace_back(dataA[idx]);
        }
    }
}


// =============================================================================
// == 原 SpectrumDesity.cpp —— 伪谱积分实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: SpectrumDesity.cpp
// 功能描述: 伪谱积分(Pseudo-spectrum Integration)模块
// 系统角色: 对多帧测向结果进行累积和加权，提高测向稳定性和精度。
//           伪谱积分是一种多帧融合技术，类似于图像处理中的帧累积。
//
// 工作原理:
//   1. 每帧测向得到每个角度(0-359度)的伪谱值(相关系数)
//   2. 将各卫星的伪谱按类型加权合并(欺骗信号获得更高权重)
//   3. 对合并后的伪谱进行多帧积分累积(m_Angle_Accumulate)
//   4. 每帧累积后乘以衰减因子(m_Accumulate_multiplier)，实现指数衰减
//   5. 累积器中的峰值角度即为最终的稳定测向结果
//
// 关键功能:
//   1. setSpoofingResultPseudoSpectrumDesity - 伪谱积分主函数
//   2. getPseudoSpectrumType - 按类型计算合并伪谱(加权)
//   3. getPseudoSpectrumDesity - 多星伪谱合成求角度
// =============================================================================
// 伪谱积分

// =========================================================================
// 伪谱积分主函数
// 流程:
//   1. 对每个频点的所有卫星伪谱进行加权合并(getPseudoSpectrumType)
//   2. 对各频点的合并伪谱再进行合成求角度(getPseudoSpectrumDesity)
//   3. 将角度结果累积到m_Angle_Accumulate积分器中
//   4. 在峰值角度附近施加邻域扩散(5度范围内的角度也获得递减的积分)
//   5. 选择累积值最高的角度作为最终结果
//   6. 所有角度累积值乘以衰减因子(m_Accumulate_multiplier)实现指数衰减
// 注: 最终结果会添加微小随机扰动(0.1-1.0度)，避免完全相同的输出
// =========================================================================
void SpoofingDoa::setSpoofingResultPseudoSpectrumDesity(SpoofingResult &result)
{
    int count = result.i_Count;
    SatelliteAngle tp_Satellite;
    AlarmData tp_alarm;
    int prnNum = 0;
    int typeInt = 0;
    int prn = 0;
    vector<vector<double>> pseudoS;       // 多星伪谱: [卫星数][360角度]
    vector<double> tp_pseudo;             // 合并后的伪谱(360维)
    vector<vector<double>> tp_pseudoS;    // 多频点合并伪谱: [频点数][360角度]
    vector<double> tp_pseudo2;
    int tp_Alarm_num = 0;
    for (int i = 0; i < count; i++)
    {

        tp_Satellite = result.i_SatelliteAngle[i];
        typeInt = TypeInt(tp_Satellite.i_Sys, tp_Satellite.i_Type);
        prnNum = tp_Satellite.i_Count;
        pseudoS.clear();
        // 收集该频点所有卫星的伪谱
        for (int j = 0; j < prnNum; j++)
        {
            tp_pseudo2.clear();
            prn = tp_Satellite.i_AlarmData[j].i_Prn;
            tp_pseudo2 = m_Pseudo_Spectrum_Value[typeInt][prn];  // 从保存的伪谱数据中取出
            pseudoS.emplace_back(tp_pseudo2);
        }
        tp_Alarm_num = tp_Satellite.i_Alarm;
        tp_pseudo.clear();
        PublicSpace::Log("Sys=%d,Type=%d\n", tp_Satellite.i_Sys, tp_Satellite.i_Type);
        getPseudoSpectrumType(pseudoS, tp_pseudo, tp_Alarm_num);  // 加权合并伪谱
        tp_pseudoS.emplace_back(tp_pseudo);
    }

    if (count > 0)
    {
        double tp_angle = 0;
        getPseudoSpectrumDesity(tp_pseudoS, tp_angle);  // 多频点合成求角度
        int tp_angle_diff = 0;
        int tp_angle_diff2 = 0;

        // 伪谱累积: 主角度获得权重50
        m_Angle_Accumulate[tp_angle] += 50;
        double angle2 = tp_angle;
        double max_accu = m_Angle_Accumulate[tp_angle];
        // 邻域扩散: 主角度±5度范围内获得递减的积分权重
        for (int i = 1; i < 6; i++)
        {
            tp_angle_diff = Round360((tp_angle - i));
            tp_angle_diff2 = Round360((tp_angle + i));
            m_Angle_Accumulate[tp_angle_diff] += (50 - i);   // 权重递减
            m_Angle_Accumulate[tp_angle_diff2] += (50 - i);
            if (m_Angle_Accumulate[tp_angle_diff] > max_accu)
            {
                angle2 = tp_angle_diff;
                max_accu = m_Angle_Accumulate[tp_angle_diff];
            }
            if (m_Angle_Accumulate[tp_angle_diff2] > max_accu)
            {
                angle2 = tp_angle_diff2;
                max_accu = m_Angle_Accumulate[tp_angle_diff2];
            }
        }

        // 添加微小随机扰动(0.1-1.0度)，避免完全相同的输出
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(1, 10);
        for (int i = 0; i < count; i++)
        {
            int rand = dis(gen);
            result.i_SatelliteAngle[i].i_Angle = angle2 + rand / 10.0;  // 添加0.1-1.0度随机扰动
            result.i_SatelliteAngle[i].i_Alarm = 1;
        }
    }
    // 日志输出累积值，并应用衰减因子
    PublicSpace::Log("Accumulate:\n");
    for (int i = 0; i < 360; i++)
    {
        PublicSpace::Log("%.2f,", m_Angle_Accumulate[i]);
        // 指数衰减: 每帧累积值乘以衰减因子(0-1之间)
        // 衰减因子为0则不清零(保留历史)，因子<1则历史影响逐渐减小
        m_Angle_Accumulate[i] *= m_Accumulate_multiplier;
    }
    PublicSpace::Log("\n");
}

// =========================================================================
// 多频点伪谱合成求角度
// 功能: 将多个频点的合并伪谱叠加(各角度值相加)
//       找到叠加后伪谱值最大的角度作为估计的到达角
// =========================================================================
void SpoofingDoa::getPseudoSpectrumDesity(vector<vector<double>> diff, double &angle)
{
    int size = (int)diff.size();  // 频点数
    double max_diff = -9999.0;
    int max_angle = 0;
    PublicSpace::Log("diff sum all:\n");
    double tp_sum = 0.0;
    for (int k1 = 0; k1 < 360; k1++)
    {
        tp_sum = 0.0;
        // 将所有频点的伪谱在该角度上的值相加
        for (int j = 0; j < size; j++)
        {

            tp_sum = tp_sum + diff[j][k1];
        }
        if (tp_sum > max_diff)
        {
            max_diff = tp_sum;
            max_angle = k1;  // 峰值角度
        }
        PublicSpace::Log("%.4f,", tp_sum);
    }
    PublicSpace::Log("\n");
    angle = max_angle;
}

// =========================================================================
// 按类型计算合并伪谱(加权)
// 功能: 对同一频点的多颗卫星的伪谱进行合并
// 加权策略:
//   - Alarm=1(欺骗信号): 将合并后的伪谱乘以3(提高权重)
//     因为欺骗信号被认为来自同一方向，其伪谱应当一致
//   - Alarm=0(非欺骗): 保持原权重
// 输出: meanDiff[360] 该频点的合并伪谱(每个角度的均值/加权值)
// =========================================================================
void SpoofingDoa::getPseudoSpectrumType(const vector<vector<double>> diff, vector<double> &meanDiff, int Alarm)
{

    meanDiff.clear();
    meanDiff.resize(360);
    int size = (int)diff.size();  // 该频点的卫星数
    vector<double> sum_diff;
    sum_diff.clear();
    sum_diff.resize(360);
    for (int k1 = 0; k1 < 360; k1++)
    {
        sum_diff[k1] = 0.0;
    }
    // 各角度伪谱求和
    for (int j = 0; j < size; j++)
    {
        for (int k1 = 0; k1 < 360; k1++)
        {
            sum_diff[k1] = sum_diff[k1] + diff[j][k1];
            PublicSpace::Log("%.4f,", diff[j][k1]);
        }
        PublicSpace::Log("\n");
    }

    // 计算均值并应用加权
    for (int k1 = 0; k1 < 360; k1++)
    {

        meanDiff[k1] = sum_diff[k1] / size;  // 均值归一化
        if (1 == Alarm)
        {
            meanDiff[k1] = meanDiff[k1] * 3;  // 欺骗信号的伪谱权重乘以3
        }
        PublicSpace::Log("%.4f,", meanDiff[k1]);
    }
    PublicSpace::Log("\n");
}

// =============================================================================
// == 原 WriteLog.cpp —— 日志输出实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: WriteLog.cpp
// 功能描述: 日志输出模块
// 系统角色: 提供各类数据结构的格式化日志输出函数，用于:
//           1. 调试和问题定位
//           2. 数据流程跟踪
//           3. 测向结果记录和分析
// 日志内容: 测向结果(SpoofingResult)、原始GNSS数据、相位差A/B格式数据
// 注: 日志输出通过PublicSpace::Log()函数实现，最终写入日志文件
// =============================================================================
// 编写日志的相关函数

// =========================================================================
// 输出欺骗测向结果日志
// 格式: 时间戳 -> 频点数 -> 每个频点的系统、频点、卫星数、到达角
//       -> 每颗卫星的PRN、载噪比、测向角度、测向质量
// =========================================================================
void SpoofingDoa::LogSpoofingResult(const SpoofingResult result)
{ // 输出检测结果或测向结果
    string nowT = getNowTime();
    PublicSpace::Log(" %s   Spoofing num = %d\n", nowT.c_str(), result.i_Count);
    for (int i = 0; i < result.i_Count; i++)
    {
        PublicSpace::Log("Sys=%d,Type=%d,Count=%d,Angle=%.2f\n",
                         result.i_SatelliteAngle[i].i_Sys, result.i_SatelliteAngle[i].i_Type, result.i_SatelliteAngle[i].i_Count, result.i_SatelliteAngle[i].i_Angle);
        PublicSpace::Log("{\n");
        for (int j = 0; j < result.i_SatelliteAngle[i].i_Count; j++)
        {
            PublicSpace::Log("Prn=%d,Snr=%.1f,Angle=%d,Quality=%.2f;\n",
                             result.i_SatelliteAngle[i].i_AlarmData[j].i_Prn, result.i_SatelliteAngle[i].i_AlarmData[j].i_Snr, result.i_SatelliteAngle[i].i_AlarmData[j].i_Angle, result.i_SatelliteAngle[i].i_AlarmData[j].i_Quality);
        }
        PublicSpace::Log("}\n");
    }
}

// =========================================================================
// 输出原始GNSS观测数据日志
// 包含两个通道(端口0和端口1)的所有卫星信息:
//   卫星系统、频点、PRN、伪距、载噪比、载波相位、多普勒
// =========================================================================
void SpoofingDoa::LogGNSSData(const GNSSData data, int n)
{
    SatelliteData tp;
    // 原始数据(仅在启用原始数据保存时输出)
    if (0 != m_Save_Original_Flg)
    {
        // 端口0(第一通道)
        PublicSpace::Log("%d-1,i_PortOneNum:%d\n", n, data.i_PortOneNum);
        for (int i = 0; i < data.i_PortOneNum; i++)
        {
            tp = data.i_PortOne[i];
            PublicSpace::Log("%d-1,Sys=%d,Type=%d,Prn=%d,Psr=%.5f,Snr=%.1f,Phase=%.5f,Dop=%.5f\n", n, tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Psr, tp.i_Snr, tp.i_Phase, tp.i_Dop);
        }
        // 端口1(第二通道)
        PublicSpace::Log("%d-2,i_PortTwoNum:%d\n", n, data.i_PortTwoNum);
        for (int i = 0; i < data.i_PortTwoNum; i++)
        {
            tp = data.i_PortTwo[i];
            PublicSpace::Log("%d-2,Sys=%d,Type=%d,Prn=%d,Psr=%.5f,Snr=%.1f,Phase=%.5f,Dop=%.5f\n", n, tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Psr, tp.i_Snr, tp.i_Phase, tp.i_Dop);
        }
    }
}

// =========================================================================
// 输出单颗卫星的B格式相位差数据日志
// 格式: 卫星信息(系统/频点/PRN) -> snr1数组 -> snr2数组 -> 相位差数组(度)
// 注: 日志中相位差已转换为度(原始数据为周，乘以360)
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffB(const SatelliteDataPhaseDiffB tp)
{
    PublicSpace::Log("Sys=%d,Type=%d,Prn=%d\n",
                     tp.i_Sys, tp.i_Type, tp.i_Prn);
    // 第一通道载噪比
    PublicSpace::Log("       snr1=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_Snr1[j]);
    }
    PublicSpace::Log("]\n");
    // 第二通道载噪比
    PublicSpace::Log("       snr2=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_Snr2[j]);
    }
    PublicSpace::Log("]\n");
    // 载波相位差(度)
    PublicSpace::Log("  phasediff=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_phase_diff[j] * 360);
    }
    PublicSpace::Log("]\n");
}

// =========================================================================
// 输出多颗卫星的B格式相位差数据日志(批量)
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffB(const vector<SatelliteDataPhaseDiffB> &dataB)
{
    SatelliteDataPhaseDiffB tp;

    for (unsigned int i = 0; i < dataB.size(); i++)
    {
        tp = dataB[i];
        PublicSpace::Log("Sys=%d,Type=%d,Prn=%d\n",
                         tp.i_Sys, tp.i_Type, tp.i_Prn);
        PublicSpace::Log("       snr1=[");
        for (int j = 0; j < tp.i_diffLen; j++)
        {
            PublicSpace::Log("%.2f,", tp.i_Snr1[j]);
        }
        PublicSpace::Log("]\n");
        PublicSpace::Log("       snr2=[");
        for (int j = 0; j < tp.i_diffLen; j++)
        {
            PublicSpace::Log("%.2f,", tp.i_Snr2[j]);
        }
        PublicSpace::Log("]\n");
        PublicSpace::Log("  phasediff=[");
        for (int j = 0; j < tp.i_diffLen; j++)
        {
            PublicSpace::Log("%.2f,", tp.i_phase_diff[j] * 360);
        }
        PublicSpace::Log("]\n");
    }
}

// =========================================================================
// 输出多颗卫星的A格式相位差数据日志(批量)
// A格式: 每颗卫星一行，包含系统/频点/PRN/两个SNR/相位差(度)
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffA(const vector<SatelliteDataPhaseDiffA> &dataA)
{

    for (unsigned int i = 0; i < dataA.size(); i++)
    {
        SatelliteDataPhaseDiffA tp = dataA[i];
        PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,snr1=%.2f,snr2=%.2f,phasediff=%.2f\n",
                         tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Snr1, tp.i_Snr2, tp.i_phase_diff * 360);
    }
}

// =========================================================================
// 输出单颗卫星的A格式相位差数据日志
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffA(const SatelliteDataPhaseDiffA dataA)
{

    SatelliteDataPhaseDiffA tp = dataA;
    PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,snr1=%.2f,snr2=%.2f,phasediff=%.2f\n",
                     tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Snr1, tp.i_Snr2, tp.i_phase_diff * 360);
}

// =========================================================================
// 输出按频点分类的相位差数据日志
// 按频点分组输出该频点下所有卫星的A格式相位差数据
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffType(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT)
{
    std::vector<SatelliteDataPhaseDiffA> tp_dataA;
    for (auto it = dataT.begin(); it != dataT.end(); it++)
    {
        tp_dataA.clear();
        tp_dataA = it->second;
        LogSatelliteDataPhaseDiffA(tp_dataA);
    }
    Log("\n");
}

// =============================================================================
// == 原 GN902.cpp —— 接口层实现(GN902 类) =================================================
// =============================================================================

#include <vector>
#include <map>

using namespace std;

// =============================================================================
// 每个 GN902 实例的内部状态(引擎 + 整轮帧缓冲)。
// 以对象地址索引存放在文件内容器中，避免改动 GN902.h 中已有的类定义/成员。
//   eng       : SpoofingDoa 引擎。构造时即 initProject()+Init()(运行参数内置在引擎代码中，
//               不读取外部配置文件)，并按 Python 流程阈值初始化各频点)。引擎内部持跨轮(跨周期)状态:
//               m_ConsecutiveAlarm / m_Tracking / m_Baselines，在本类多轮调用间持续累积。
//   buf/cuts  : 当前轮逐帧送来的 GNSSData 与对应 cutIdx(通道2天线号 1..7)。
//   calFrames : 已储存的校正刀(cutIdx=1 / 天线对{1,1})数据，每轮出现校正刀则整体刷新。
//   result    : 最近一轮喂入引擎后取出的测向结果(供 GetResult 返回)。
// =============================================================================
namespace {
struct GN902State
{
    SpoofingDoa *eng;                // 欺骗检测+测向引擎(循环切刀)
    std::vector<GNSSData> buf;       // 当前轮待处理帧(原始到达顺序)
    std::vector<int> cuts;           // 每帧对应 cutIdx
    std::vector<GNSSData> calFrames; // 储存的校正刀数据
    SpoofingResult result;           // 最近一轮测向结果

    GN902State() : eng(0) { clearResult(); }

    ~GN902State()
    {
        if (eng)
        {
            delete eng;
            eng = 0;
        }
    }

    void clearResult()
    {
        result.i_Count = 0;
        for (int i = 0; i < 24; ++i)
        {
            result.i_SatelliteAngle[i].i_Sys = 0;
            result.i_SatelliteAngle[i].i_Type = 0;
            result.i_SatelliteAngle[i].i_Alarm = 0;
            result.i_SatelliteAngle[i].i_Angle = -1.0;
            result.i_SatelliteAngle[i].i_Count = 0;
        }
    }
};

// 实例容器: 对象地址 -> 状态(不改动 GN902.h 的既有类定义/成员)
std::map<const GN902 *, GN902State *> g_gn902State;
} // namespace

// GN902 实例容器(声明于 interface.h)，供 Create_GN902/Release_GN902 等 C 接口使用
std::vector<GN902 *> GN902Container;

GN902::GN902(){
    GN902State *st = new GN902State();
    // SpoofingDoa 构造即 initProject()+Init()：运行参数已内置在引擎 Init() 中(不读配置文件)，
    // 并按 Python 流程对齐各频点阈值。
    st->eng = new SpoofingDoa();
    if (st->eng != 0)
    {
        // GN902 运行参数：循环切刀检测开、全向半径 0.1865m(重建理论模板)、
        // 暂不平滑(整轮喂帧时按每刀实际帧数自适应)。
        st->eng->configCyclicRuntime(true, 0, false, 0.1865);
        // 切刀顺序 = {1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7}
        // (7 组天线对 = 校正刀 + 六刀测向，对齐 Python code 0/9/57/17/25/33/1)
        const int cutSeq[14] = {1, 1, 1, 2, 1, 3, 1, 4, 1, 5, 1, 6, 1, 7};
        st->eng->setCutSquence(14, cutSeq);
    }
    g_gn902State[this] = st;
}

GN902::~GN902(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it != g_gn902State.end())
    {
        delete it->second; // 内部释放引擎
        g_gn902State.erase(it);
    }
}


// 设置阈值检测参数
// @param phsDiffThreshold 位相差阈值
// @param satelliteCountThreshold 卫星数阈值
// @param cutCountThreshold 通道2对应天线阈值
// @param sysEnum 系统类型
// @param typeEnum 类型
void GN902::SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    SpoofingDoa *eng = it->second->eng;
    // 参数顺序: (系统sysEnum, 频点typeEnum, 卫星数阈值satelliteCountThreshold, 相位差阈值phsDiffThreshold)
    eng->setThresholdDetectionDoa(sysEnum, typeEnum, (int)satelliteCountThreshold, phsDiffThreshold);
    // cutCountThreshold = 连续确认刀数 p(对应 ALARM_CONSECUTIVE_P)，>0 时生效
    if (cutCountThreshold > 0)
    {
        eng->setDetectionRecordNum((int)cutCountThreshold);
    }
    return;
}

// 设置数据
// @param data 数据指针
// @param cutIdx 通道二天线索引
// @param endFlag 结束标志
void GN902::SetData(const GNSSData* data, int cutIdx, int endFlag){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        return;
    }
    GN902State *st = it->second;
    // 帧空或 cutIdx 非法(1=校正刀, 2..7=六测向刀)则忽略该帧，不打断整轮累积
    if (data == 0 || cutIdx < 1 || cutIdx > 7)
    {
        return;
    }
    // 非最后一帧数据，保存数据
    if (!endFlag){
        // 保存数据
        st->buf.push_back(*data);
        st->cuts.push_back(cutIdx);
        return;
    }
    // 最后一帧数据，先保存后整轮处理：
    // Detect() 组批喂入引擎(更新告警flag/跟踪)，Doa() 判断是否测向并取出结果
    else{
        st->buf.push_back(*data);
        st->cuts.push_back(cutIdx);
        Detect();
        Doa();
    }
    return;
}

// 获取结果
// @param result 结果结构体
void GN902::GetResult(SpoofingResult& result){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        result.i_Count = 0;
        return;
    }
    // 取最近一轮(喂入引擎后)的测向结果
    result = it->second->result;
    return;
}

// 检测
// 整轮组批并喂入引擎，完成循环切刀欺骗检测与跟踪(跨轮状态在引擎内连续累积)。
void GN902::Detect(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    GN902State *st = it->second;

    // 取出本轮缓冲(swap 后引用安全)
    std::vector<GNSSData> frameData;
    std::vector<int> frameCuts;
    frameData.swap(st->buf);
    frameCuts.swap(st->cuts);

    // 拆帧：
    //   cutIdx=1  -> 校正刀(code=0 / 天线对1-1)，不参与测向，先存为校正数据；
    //   cutIdx=2..7 -> 六刀测向刀(对应 code 9,57,17,25,33,1 / {1,2}..{1,7})。
    std::vector<GNSSData> calNew;
    std::vector<std::vector<int> > detByCut(6); // index = cutIdx-2 (0..5)
    for (size_t i = 0; i < frameCuts.size(); ++i)
    {
        int c = frameCuts[i];
        if (c == 1)
        {
            calNew.push_back(frameData[i]);
        }
        else
        {
            detByCut[c - 2].push_back((int)i); // c 已保证 2..7
        }
    }

    // 本轮出现了校正刀数据 -> 刷新储存的校正数据(替换旧校正，取本轮为准)
    if (!calNew.empty())
    {
        st->calFrames = calNew;
    }
    if (st->calFrames.empty())
    {
        return; // 尚无校正数据则丢弃本轮(跨轮连续/跟踪状态不变)
    }

    // 测向轮需六刀齐全；缺刀则丢弃本轮
    for (int r = 0; r < 6; ++r)
    {
        if (detByCut[r].empty())
        {
            return;
        }
    }

    // 六测向刀每刀帧数是否一致
    int C = (int)detByCut[0].size();
    bool detUniform = true;
    for (int r = 1; r < 6; ++r)
    {
        if ((int)detByCut[r].size() != C)
        {
            detUniform = false;
            break;
        }
    }

    // 校正行(引擎 row0)：六刀帧数统一为 C 且校正帧数 >= C 时，取校正帧末尾 C 帧
    // (最稳定尾段)以便整体平滑；否则使用全部校正帧(将落入“每刀取末帧”路径)。
    std::vector<GNSSData> calRow;
    if (detUniform && (int)st->calFrames.size() >= C)
    {
        calRow.assign(st->calFrames.end() - C, st->calFrames.end());
    }
    else
    {
        calRow = st->calFrames;
    }

    bool uniform = detUniform && ((int)calRow.size() == C) && C > 0;
    int oneCutFrams = uniform ? C : 1;
    bool smooth = uniform;

    // 按引擎行序组批：row0 = 校正(cutIdx1)，row1..6 = 六刀测向(cutIdx2..7)
    std::vector<GNSSData> batch;
    if (uniform)
    {
        batch.reserve((size_t)7 * C);
        for (int k = 0; k < C; ++k)
        {
            batch.push_back(calRow[k]);
        }
        for (int r = 0; r < 6; ++r)
        {
            for (int k = 0; k < C; ++k)
            {
                batch.push_back(frameData[detByCut[r][k]]);
            }
        }
    }
    else
    {
        // 帧数不一致(或校正帧不足) -> 每行取末尾(最稳定)一帧
        batch.reserve(7);
        batch.push_back(calRow.back());
        for (int r = 0; r < 6; ++r)
        {
            batch.push_back(frameData[detByCut[r].back()]);
        }
    }

    // 运行一轮：引擎 row0 校正刀用于通道校正，六刀测向刀做检测+测向；
    // 跨轮 m_ConsecutiveAlarm / m_Tracking / m_Baselines 在引擎内持续累积，缺刀位
    // 用上一周期相位差补齐六条基线。
    st->eng->configCyclicRuntime(true, oneCutFrams, smooth, 0.0);
    st->eng->setGNSSData(batch.data(), (int)batch.size());
    return;
}

// 测向
// 从引擎取出本轮测向结果，存入内部 result 供 GetResult 返回。
void GN902::Doa(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    GN902State *st = it->second;
    st->clearResult();
    st->eng->getAngleSpoofingDoa(st->result);
    return;
}
