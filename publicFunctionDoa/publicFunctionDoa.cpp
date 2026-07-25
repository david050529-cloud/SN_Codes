/**
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
#include "publicFunctionDoa.h"
using namespace std;
namespace PublicSpace
{
    // ==================== 命名空间全局变量定义 ====================

    /**
     * @brief 当前可执行文件所在目录路径，默认为当前目录 "."
     * @note 程序启动时通过 getCurrentExecutablePath() 获取实际路径并更新
     */
    string m_CurrentPath = ".";
    // extern string m_CurrentPath;

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

    /**
     * @brief 多项式最小二乘拟合
     *
     * 算法流程：
     *   1. 构造 Vandermonde 矩阵 A: A(i,j) = x(i)^j (j从0到n)
     *   2. 使用 BDCSVD (Bidiagonal Divide and Conquer SVD) 分解求解最小二乘问题 A * coeffs = y
     *   3. BDCSVD 方法在大规模数据下具有较好的数值稳定性和计算效率
     *
     * @param x 自变量数据点(Eigen列向量)
     * @param y 因变量数据点(Eigen列向量)
     * @param n 拟合多项式最高次数
     * @param coeffs 输出参数，多项式系数[常数项, 一次项, ..., n次项]，长度为n+1
     */
    void polyfit(const VectorXd &x, const VectorXd &y, const int n, VectorXd &coeffs)
    {
        int numData = x.size();
        MatrixXd A(numData, n + 1);

        // 构造Vandermonde矩阵
        for (int i = 0; i < numData; ++i)
        {
            A(i, 0) = 1;
            for (int j = 1; j <= n; ++j)
            {
                A(i, j) = A(i, j - 1) * x(i);
            }
        }
        // 使用最小二乘法求解
        coeffs = A.bdcSvd(ComputeThinU | ComputeThinV).solve(y);
    }

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
    /**
     * @brief 将二维复数vector调整为指定维度(cols x rows)，所有元素初始化为 (1.0 + 0.0i)
     *
     * @param data 输入/输出参数，要调整的二维复数vector，调用后旧数据被清空
     * @param cols 目标列数（外层vector的大小）
     * @param rows 目标行数（内层vector的大小）
     */
    void VectorResizeToOne(std::vector<std::vector<std::complex<double>>> &data, int cols, int rows)
    {
        // 清空容器
        data.clear();

        data.resize(cols, std::vector<std::complex<double>>(rows, std::complex<double>(1.0, 0.0)));
    }
    /**
     * @brief 获取当前可执行文件所在的目录路径（跨平台实现）
     *
     * Windows: 使用 GetModuleHandle(NULL) + GetModuleFileName + PathRemoveFileSpec
     *          需要链接 Shlwapi.lib (PathRemoveFileSpec函数)
     * Linux:   使用 dladdr + dirname 获取动态库加载地址并提取目录路径
     *
     * @return 可执行文件所在目录的绝对路径，失败时返回 "."
     */
    std::string getCurrentExecutablePath(void)
    { // 根据不同的操作系统采用不同的方法来获取当前可执行文件的目录路径
#ifdef _WIN32
        char path[MAX_PATH];
        HMODULE hModule = GetModuleHandle(NULL);
        if (hModule != NULL)
        {
            GetModuleFileName(hModule,path, sizeof(path));
            PathRemoveFileSpec(path); // 移除文件名，只保留目录
            return path;
        }
#else
        Dl_info dl_info;
        if (dladdr(reinterpret_cast<void *>(getCurrentExecutablePath), &dl_info))
        {
            char *dir_path = strdup(dl_info.dli_fname);
            std::string directory = dirname(dir_path);
            free(dir_path); // 释放分配的内存
            dir_path = NULL;
            return directory;
        }
#endif
        return ".";
    }
   
}
