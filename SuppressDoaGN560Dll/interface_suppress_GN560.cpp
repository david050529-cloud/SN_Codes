/************************************************************************/
/* 文件名称: interface_suppress_GN560.cpp
/* 文件描述: GN560项目压制干扰测向DLL接口函数的实现
/*
/* 功能概述:
/*   本文件实现了SuppressDoaGN560动态链接库的所有导出接口函数。
/*   该DLL专为GN560硬件平台设计，封装了单通道压制干扰测向的完整流程。
/*
/* GN560项目的测向特点:
/*   1. 硬件平台: GN560接收机，采用单通道时分复用架构
/*   2. 天线阵列: 7阵元均匀圆阵（全向天线），半径默认0.186m
/*   3. 切刀方式: 相邻天线对切换 {{1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,1}}
/*   4. 移相次数: 单通道4次移相 (0度/90度/180度/270度)
/*   5. 测向算法: 默认使用相关干涉仪(m_Doa_Arithmetic=1)，
/*      也可配置为幅相法、联合对角化多信号测向等
/*   6. IQ积分: 默认5次幅度积分，用于平滑单通道移相带来的幅度抖动
/*   7. 数据转换: 外部传入double型IQ数据，内部转为short型处理
/*
/* 与基础版SuppressDoa接口的关键实现差异:
/*   1. createSuppressDoa: 固定startPointNum=0, endPointNum=pointNum，
/*      与GN560单通道数据格式匹配
/*   2. initSuppressDoa: 额外加载单通道移相误差系数文件并设置采样带宽，
/*      这是GN560单通道测向误差校正的关键步骤
/*   3. setDataSuppressDoa: 接收double*类型的IQ数据并转换为short*，
/*      同时设置startLen动态调整有效数据起始位置
/*   4. getAngleSuppressDoa: 固定取第一个信号的结果作为单信号输出
/*   5. getMuiltAngleSuppressDoa: 支持多信号联合测向结果输出
/*   6. 无线程锁: GN560版本未使用互斥锁，适用于单线程调用场景
/*
/* 调用流程示例:
/*   int id = createSuppressDoa(1024);           // 创建对象，1024采样点
/*   initSuppressDoa(id, 100e6, "./coeff.csv", 20000000); // 初始化，100MHz
/*   setDataSuppressDoa(id, iqData, len, 100);   // 设置数据，跳过前100点
/*   double angle, quality;
/*   getAngleSuppressDoa(id, angle, quality);     // 获取单信号测向结果
/*   releaseSuppressDoa(id);                      // 释放对象
/*
/* 创建时间: 2024年
/* 修改记录:
/************************************************************************/

#include "interface_suppress_GN560.h"
using namespace std;

// =========================================================================
// 平台相关的版本号定义
// Windows和Linux平台分别编译，版本号中标注平台和编译类型
// =========================================================================
#ifdef _WIN32
// char LogFile[] = "D:\\doa_B.log";
char Version[] = "V1.0.10.20251028_X86_WIN64_debug";
#else
// char LogFile[] = "./doa_B.log";
char Version[] = "V1.0.10.20251028_X86_LIN64_debug";
#endif

/************************************************************************/
/* 函数名称: GetALGVersion
/* 功能描述: 获取算法DLL的版本号字符串
/* 参数说明: 无
/* 返回值:
/*   char* - 指向静态版本字符串的指针
/*           格式: "V<主版本>.<次版本>.<修订>.<日期>_X86_<平台>_<编译类型>"
/*           示例: "V1.0.10.20251028_X86_WIN64_debug"
/* 说明:
/*   版本号中的日期(20251028)表示该版本的构建日期。
/*   返回的字符串指针指向静态内存区域，调用方无需释放。
/*   可用于运行时版本校验、日志记录和问题追溯。
/************************************************************************/
char *GetALGVersion()
{
    return Version;
}

/************************************************************************/
/* 函数名称: createSuppressDoa
/* 功能描述: 创建GN560压制干扰测向对象并返回对象ID
/* 创建时间: 2023/12/22
/* 输入参数:
/*   int pointNum - 采样点数，即每次测向使用的IQ数据点数
/*                  此参数决定了FFT长度和数据处理缓冲区大小
/*                  注意：GN560创建时固定startPointNum=0, endPointNum=pointNum
/*                  与基础版不同，不需要调用方指定起止位置
/* 返回值:
/*   int - 对象ID（句柄），用于后续所有函数调用
/*         每次调用返回递增的唯一ID，从0开始
/* 实现说明:
/*   1. 调用getLogCont(m_num)初始化当前对象的日志上下文
/*   2. 创建SuppressDoa对象，底层使用GN560项目配置：
/*      - 项目标志: m_Project_flg = GN560
/*      - 天线类型: 全向天线(m_antnenaType=0)
/*      - 天线个数: 7阵元均匀圆阵(m_AntennaNum=7)
/*      - 通道数:   3通道(m_channelNum=3)
/*      - 默认算法: 相关干涉仪(m_Doa_Arithmetic=1)
/*   3. 将对象指针存入全局容器SuppressDoaObjectContainer
/*   4. 计数器m_num自增，为下一个对象准备唯一ID
/************************************************************************/
int createSuppressDoa(int pointNum)
{
    getLogCont(m_num);
    SuppressDoa *target = new SuppressDoa(pointNum, 0, pointNum);
    SuppressDoaObjectContainer[m_num++] = target;
    int id = m_num - 1;
    return id;
}

/************************************************************************/
/* 函数名称: initSuppressDoa
/* 功能描述: 初始化GN560压制干扰测向对象
/* 创建时间: 2023/12/22
/* 输入参数:
/*   int id       - 对象ID，由createSuppressDoa返回
/*   double f     - 测向中心频率（Hz）
/*                  用于计算理论相位差和选择对应的阵列孔径
/*   const char *adr - 单通道移相误差系数文件路径（CSV格式）
/*                     文件中包含每个切刀通道对在四次移相(0/90/180/270度)
/*                     状态下的幅度和相位误差系数
/*                     格式: channel1, channel2, shiftPhs[4], amChang[4]
/*   int bandwidth - 采样带宽（Hz）
/*                   用于计算m_bandwidth_coefficient功率系数，
/*                   影响测向质量的权重调整
/* 返回值:
/*    0  - 初始化成功
/*   -1  - 对象为空（传入的ID在容器中不存在）
/*   其他 - 内部初始化函数返回的错误码
/* 实现说明:
/*   1. 空指针检查：确保ID对应的对象存在
/*   2. 设置日志上下文：关联当前对象的日志
/*   3. 调用initSuppressDoa(f)：初始化频率相关参数
/*      - 加载天线阵列配置文件(如果配置为定向天线)
/*      - 初始化理论相位差数据
/*      - 对于GN560(全向天线)，根据频率查找对应的圆阵半径m_R
/*   4. 设置采样带宽：用于计算功率系数，影响后续IQ幅度积分和测向质量
/*   5. 加载单通道移相误差系数：GN560的关键初始化步骤
/*      - 读取CSV文件中的误差参数，存储到m_shift_phase_arg[10]
/*      - 这些参数用于calPhaseBy4shiftPhase校正四次移相引入的误差
/************************************************************************/
int initSuppressDoa(int id, double f, const char *adr, int bandwidth)
{
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid SuppressDoa* parameter");
        return -1;
    }
    getLogCont(id);
    SuppressDoa *target = SuppressDoaObjectContainer[id];
    string str(adr);
    int flg = target->initSuppressDoa(f);               // 初始化频率及理论相位
    int flg1 = target->setBandwidth(bandwidth);          // 设置采样带宽，计算功率系数
    if (flg == 0)
    {
        int flg2 = target->setSignaleChannelCoefficeicentadr(adr); // 加载单通道移相误差系数
        return flg2;
    }
    return flg;
}

/************************************************************************/
/* 函数名称: setDataSuppressDoa
/* 功能描述: 设置GN560测向用的IQ原始数据
/* 创建时间: 2023/12/22
/* 输入参数:
/*   int id         - 对象ID
/*   double *data   - IQ数据数组（double类型）
/*                    GN560接口使用double接收数据，适应外部调用方的数据格式
/*                    数据为交替排列的I/Q分量: I0,Q0,I1,Q1,I2,Q2,...
/*   int length     - data数组的总长度（元素个数，非采样点数）
/*                    如果每个采样点有I和Q两个值，则采样点数 = length / 2
/*   int startLen   - 有效数据的起始采样点偏移量
/*                    用于跳过数据开头可能存在的拖尾/不稳定采样段
/*                    值为-1时表示使用默认起始位置
/*   double briefAngle - 粗测向角度（度），默认值400
/*                       当前版本内部强制设为400（不使用粗测向引导）
/*                       预留参数，未来可用于缩小测向搜索范围、提高精度
/* 返回值:
/*    0  - 数据设置成功
/*   -1  - 对象为空（无效ID）
/*   其他 - 起始点设置或数据处理的错误码
/* 实现说明:
/*   1. 空指针检查
/*   2. 内部强制briefAngle=400（等效于不使用粗测向）
/*      GN560当前版本不使用粗测向引导，所有角度均从全范围(0-360度)搜索
/*   3. 设置有效数据起始位置(startLen)和结束位置(-1表示使用全部数据)
/*   4. 数据类型转换：double* -> short*
/*      - 在堆上分配short数组
/*      - 逐元素将double截断转换为short（丢失小数精度，但IQ数据本身为整数）
/*      - 转换完成后的short数据传入底层SuppressDoa::setData
/*   5. 释放临时short数组，防止内存泄漏
/************************************************************************/
int setDataSuppressDoa(int id, double *data, int length, int startLen, double briefAngle)
{
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid SuppressDoa* parameter");
        return -1;
    }
    getLogCont(id);

    // GN560当前版本不使用粗测向引导，强制设为400度（超出有效角度范围，表示不启用）
    briefAngle = 400;

    SuppressDoa *target = SuppressDoaObjectContainer[id];
    int n1 = target->setStartEndPointNum(startLen, -1); // 设置有效数据区间（-1表示使用全部数据到末尾）
    if (n1 == 0)
    {
        // 数据类型转换: double -> short，以适应底层算法的数据类型要求
        short *datashort = new short[length];
        for (int i = 0; i < length; i++)
        {
            datashort[i] = data[i]; // 隐式截断转换，丢弃小数部分
        }
        int n = target->setData(datashort, length); // 将转换后的数据传入测向算法核心

        // 释放临时缓冲区，防止内存泄漏
        if (datashort != nullptr)
        {
            delete[] datashort;
            datashort = nullptr;
        }

        return n;
    }
    else
    {
        return n1;
    }
}

/************************************************************************/
/* 函数名称: getAngleSuppressDoa
/* 功能描述: 获取单信号源的压制干扰测向结果（角度和质量）
/* 创建时间: 2023/12/22
/* 输入参数:
/*   int id         - 对象ID
/* 输出参数:
/*   double &angle  - 测向角度（度），取值范围0~360
/*                    返回第一个信号源的来波方向
/*   double &quality - 测向质量，取值范围0~100
/*                     数值越高说明测向结果越可靠
/*                     综合评估了信号强度、相位一致性等因素
/* 返回值:
/*    0  - 测向成功，angle和quality有效
/*   -1  - 对象为空（无效ID）
/*   其他 - 底层getAngleSuppressDoa的错误码
/* 实现说明:
/*   1. 内部固定signalNum=1，即仅测向一个信号源
/*   2. 调用底层getAngleSuppressDoa(signalNum, tp_angle, tp_quality, amp)，
/*      该函数依据对象配置的算法(m_Doa_Arithmetic)执行：
/*      - 相关干涉仪(算法1): 计算实测相位差，与理论相位差模板匹配，找最小误差角度
/*      - 幅相法(算法2): 利用幅度和相位联合估计来波方向
/*      - 联合对角化(算法3): 分离多个信号后逐一测向
/*   3. 从结果数组中提取第一个信号的角度和质量
/*   4. 对于GN560单通道：测向基于4次移相计算的相位差，经移相误差系数校正后，
/*      进行相关干涉仪匹配得到最终角度
/************************************************************************/
int getAngleSuppressDoa(int id, double& angle, double& quality)
{
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid SuppressDoa* parameter");
        return -1;
    }
    getLogCont(id);
    SuppressDoa* target = SuppressDoaObjectContainer[id];

    // 固定为单信号测向模式
    int signalNUm = 1;
    double amp[10];          // 信号幅度数组（暂不启用，占位）
    double tp_angle[10];     // 临时角度结果缓冲（最多支持10个信号）
    double tp_quality[10];   // 临时质量结果缓冲

    int n = target->getAngleSuppressDoa(signalNUm, tp_angle, tp_quality, amp);
    if (n == 0)
    {
        // 从结果数组中取出第一个信号的角度和质量
        angle = tp_angle[0];
        quality = tp_quality[0];
    }
    return n;
}

/************************************************************************/
/* 函数名称: getMuiltAngleSuppressDoa
/* 功能描述: 获取多信号源的压制干扰测向结果
/* 输入参数:
/*   int id        - 对象ID
/* 输入/输出参数:
/*   int &sigNum   - 信号源个数（输入时指定期望个数，输出时返回实际个数）
/* 输出参数:
/*   double* angle  - 测向角度数组（度），调用方需预先分配>=sigNum的空间
/*                    各元素分别对应各信号源的来波方向
/*   double* quality - 测向质量数组（0~100），调用方需预先分配>=sigNum的空间
/*                     各元素分别对应各信号源的测向质量
/* 返回值:
/*    0  - 测向成功
/*   -1  - 对象为空（无效ID）
/*   其他 - 底层测向算法的错误码
/* 实现说明:
/*   1. 适用于存在多个干扰源/信号源的复杂电磁环境
/*   2. 内部调用底层getAngleSuppressDoa(sigNum, tp_angle, tp_quality, amp)，
/*      底层算法流程：
/*      a. 获取单通道IQ数据经移相后计算的相位差和幅度信息
/*      b. (若sigNum>1) 使用联合对角化算法(JADE)进行信号分离：
/*         - getRAll: 构建多个伪协方差矩阵
/*         - getJointA: 联合对角化估计混合矩阵（阵列流型）
/*         - 用估计的阵列流型进行相关干涉仪匹配
/*      c. 对每个分离出的信号分别测向
/*   3. 将结果从临时缓冲区复制到调用方提供的数组中
/*   4. 注意：调用前需通过setSignalNum或init时设置正确的信号个数，
/*      否则可能得到错误的测向结果
/************************************************************************/
int getMuiltAngleSuppressDoa(int id, int &sigNum, double* angle, double* quality)
{
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid SuppressDoa* parameter");
        return -1;
    }
    getLogCont(id);
    SuppressDoa* target = SuppressDoaObjectContainer[id];

    double amp[10];          // 信号幅度数组（暂不启用，占位）
    double tp_angle[10];     // 临时角度结果缓冲
    double tp_quality[10];   // 临时质量结果缓冲

    int n = target->getAngleSuppressDoa(sigNum, tp_angle, tp_quality, amp);
    if (n == 0)
    {
        // 将每个信号的角度和质量从临时缓冲复制到调用方提供的数组
        for (int i = 0; i < sigNum; i++)
        {
            angle[i] = tp_angle[i];
            quality[i] = tp_quality[i];
        }
    }
    return n;
}

/************************************************************************/
/* 函数名称: releaseSuppressDoa
/* 功能描述: 释放压制干扰测向对象，回收所有相关资源
/* 创建时间:
/* 输入参数:
/*   int id - 对象ID，由createSuppressDoa返回
/* 返回值:
/*    0  - 释放成功
/*   -1  - 对象为空（ID无效或已被释放）
/* 实现说明:
/*   1. 空指针检查：防止重复释放
/*   2. 关闭日志文件：LogClose()关闭当前对象的日志输出
/*   3. 从全局容器中移除该ID的记录
/*   4. 调用delete销毁SuppressDoa对象，触发其析构函数：
/*      - 释放动态分配的缓冲区
/*      - 清理积分数据队列(m_Am_Density)
/*      - 释放Eigen矩阵内存
/*      - 清理其他动态资源
/*   5. 注意：释放后该ID即变为无效，后续调用任何函数使用该ID都将返回-1
/************************************************************************/
int releaseSuppressDoa(int id)
{
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid SuppressDoa* parameter");
        return -1;
    }
    getLogCont(id);
    LogClose();                                     // 关闭日志文件，确保数据写入磁盘
    SuppressDoa *target = SuppressDoaObjectContainer[id];
    //  target->saveDataClose();                    // 已注释：数据保存功能关闭
    SuppressDoaObjectContainer.erase(id);            // 从容器中移除
    delete target;                                   // 释放对象内存
    return 0;
}

/************************************************************************/
/* 函数名称: setAmDataSuppressDoa
/* 功能描述: 设置单通道幅度数据（当前版本暂未启用）
/* 输入参数:
/*   int id        - 对象ID
/*   double *AmData - 幅度数据数组
/*   int length    - 幅度数据长度
/* 返回值:
/*    0  - 当前版本固定返回0（功能空实现）
/*   -1  - 对象为空（无效ID）
/* 实现说明:
/*   该函数原设计用于外部传入已计算的幅度数据，跳过内部幅度计算步骤。
/*   当前版本该功能未启用，内部直接返回0。
/*   内部调用target->setSigleChannelDoaDataAm已被注释。
/*   保留此接口是为了前向兼容，未来可能在需要外部幅度数据注入时启用。
/************************************************************************/
int setAmDataSuppressDoa(int id, double *AmData, int length)
{
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid SuppressDoa* parameter");
        return -1;
    }
    getLogCont(id);
    SuppressDoa *target = SuppressDoaObjectContainer[id];
    // return target->setSigleChannelDoaDataAm(AmData, length); // 功能未启用
    return 0;
}

/************************************************************************/
/* 函数名称: setSignalNum
/* 功能描述: 设置测向的信号源个数
/* 输入参数:
/*   int id        - 对象ID
/*   int signalNum - 信号源个数，必须大于0
/*                   单信号测向设为1；多信号测向设为实际信号个数
/* 返回值:
/*    0  - 设置成功
/*   -1  - 对象为空（无效ID）
/*   -2  - 参数不合法（signalNum <= 0）
/* 实现说明:
/*   该参数决定了测向时使用单信号算法还是多信号分离算法：
/*   - signalNum=1: 使用标准相关干涉仪或幅相法，直接测向
/*   - signalNum>1: 使用联合对角化(JADE)算法分离信号后再分别测向
/*   对于GN560单通道多信号场景，内部通过getMuiltSignalBySigleChannel
/*   实现多信号分离，利用不同的伪协方差矩阵进行联合对角化估计。
/*   注意：signalNum设置过大会引入伪峰，建议根据实际电磁环境合理设置。
/************************************************************************/
int setSignalNum(int id, int signalNum)
{
    if (signalNum <= 0)
    {
        return -2;
    }
    getLogCont(id);
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SuppressDoa* target = SuppressDoaObjectContainer[id];

    target->setSignalNum(signalNum);
    return 0;
}
