// =============================================================================
// 文件名: interface.cpp
// 功能描述: DLL外部C接口的实现
//
//   每个导出函数遵循统一的调用模式:
//     1. 参数合法性检查 (返回值 -2 表示参数错误)
//     2. 通过ID从容纳器中获取目标对象 (返回值 -1 表示对象不存在)
//     3. 调用对象的对应方法
//     4. 返回操作结果 (0 表示成功)
//
//   返回值约定:
//     0:  成功
//     -1: 对象ID无效(容器中不存在)
//     -2: 参数错误(指针为空/长度<=0/数值不合理)
//
//   日志机制: getLogCont(id) 提取日志上下文，使日志能按对象ID定向输出
//   线程安全: 依赖上层调用者保证同一ID的操作串行化
// =============================================================================
#include "interface.h"
#ifdef _WIN32
string  systems = "WIN64";
#else
string  systems = "LIN64";
#endif

// =============================================================================
// GetALGVersion: 获取算法库版本号
// 返回值: 格式为 "V1.0.0.YYYYMMDD_X86_WIN64" 或 "V1.0.0.YYYYMMDD_X86_LIN64"
// 注意: 调用者需要负责释放返回的char*内存(delete[])
// =============================================================================
char* GetALGVersion()
{
    string tmpVersion = "V1.0.0.20251010_X86_";
    tmpVersion += systems;
    const char* tmpVersion2 = tmpVersion.c_str();
    char* Version = new char[100];
    strncpy(Version, tmpVersion2, strlen(tmpVersion2) + 1);
    return Version;
}

// 压制干扰测向

// =============================================================================
// createSuppressDoa: 创建测向对象
// 参数:
//   id: (输出)新对象的唯一标识符，后续所有操作通过此ID访问
//   pointNum: FFT采样点数
//   startPointNum: 有效起始采样点(用于跳过滤波器拖尾)
//   endPointNum: 有效结束采样点
// 返回值: 0=成功, -2=参数错误
// 内部: 创建SuppressDoa对象存入全局容器，ID自增
// =============================================================================
int createSuppressDoa(int &id, int pointNum, int startPointNum, int endPointNum)
{
    if (pointNum<0|| startPointNum<0|| endPointNum<0)
    {//参数设置不合理
        return -2;
    }
    id = m_num; // 分配ID
    getLogCont(id); // 设置日志上下文
    SuppressDoa *target = new SuppressDoa(pointNum, startPointNum, endPointNum);
    SuppressDoaObjectContainer[m_num] = target; // 存入容器
    m_num++; // ID自增
    return 0;
}

// =============================================================================
// initSuppressDoa: 初始化测向对象
// 设置工作频率f，触发理论参数计算(相位差库/阵列流型)
// 返回值: 0=成功, -1=对象不存在, -2=频率参数错误
// =============================================================================
int initSuppressDoa(int id, double f)
{
    if (f <= 0)
    {//参数设置不合理
        return -2;
    }
    getLogCont(id);
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SuppressDoa *target = SuppressDoaObjectContainer[id];
    return target->initSuppressDoa(f);
}

// =============================================================================
// setCorrectionDataSuppressDoa: 设置通道校正数据
// crrectionData: short数组, IQ交替排列的数据
// length: 数据长度(IQ对的数量)
// 功能: 计算各通道相对于参考通道(通道0)的幅相校正系数
// =============================================================================
int setCorrectionDataSuppressDoa(int id, short *crrectionData, int length)
{
    if (crrectionData == nullptr || length <= 0)
    {
        return -2;
    }
    getLogCont(id);
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SuppressDoa *target = SuppressDoaObjectContainer[id];

    return target->setCorrectionData(crrectionData, length);
}

// =============================================================================
// setDataSuppressDoa: 传输测向IQ数据
// data: short数组, 按[切刀][通道/移相][IQ交替]顺序排列
// length: 数据长度(IQ对的数量)
// 数据会自动除以校正系数进行通道补偿
// =============================================================================
int setDataSuppressDoa(int id, short *data, int length)
{
    if (data == nullptr || length <= 0)
    {
        return -2;
    }
    getLogCont(id);
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SuppressDoa *target = SuppressDoaObjectContainer[id];

    target->setData(data, length);
    return 0;
}

// =============================================================================
// setSignalNum: 设置同频信号个数
// 单信号(1): 使用相关干涉仪直接测向
// 多信号(>1): 使用联合对角化盲分离后分别测向
// =============================================================================
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
    SuppressDoa *target = SuppressDoaObjectContainer[id];

    target->setSignalNum(signalNum);
    return 0;
}

// =============================================================================
// getAngleSuppressDoa: 执行测向计算并获取结果
// 参数(输出):
//   signalNum: 实际检测到的信号个数
//   angles: 各信号来波方向(度, 0-360范围, 精确到0.1度)
//   qualities: 各信号的测向置信度(0-1之间)
//   amplitudes: 各信号的幅度信息(dB, 暂未启用)
// 注: qualities已根据信号强度进行动态加权修正
// =============================================================================
int getAngleSuppressDoa(int id, int &signalNum, double *angles, double *qualities, double *amplitudes)
{
    getLogCont(id);
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SuppressDoa *target = SuppressDoaObjectContainer[id];
    target->getAngleSuppressDoa(signalNum, angles, qualities, amplitudes);
    return 0;
}

// =============================================================================
// getSpectrumSupressDoa: 获取频谱数据 (暂未实现)
// =============================================================================
int getSpectrumSupressDoa(int id, int &signalNum, short *spectrumData, int &length)
{
    getLogCont(id);
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SuppressDoa *target = SuppressDoaObjectContainer[id];
    target->getSpectrumSupressDoa(signalNum, spectrumData, length);
    return 0;
}

// =============================================================================
// releaseSuppressDoa: 释放测向对象
// 从容纳器中移除并delete对象，释放所有相关资源
// 注意: 释放后该ID对应的对象不再可用
// =============================================================================
int releaseSuppressDoa(int id)
{
    getLogCont(id);
    if (SuppressDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    LogClose(); // 关闭日志
    SuppressDoa *target = SuppressDoaObjectContainer[id];
    SuppressDoaObjectContainer.erase(id); // 从容器移除
    delete target; // 释放内存
    return 0;
}
