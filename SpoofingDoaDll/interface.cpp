// =============================================================================
// 文件名: interface.cpp
// 功能描述: DLL导出接口的实现文件
// 系统角色: 实现DLL的C语言导出接口，负责:
//           1. 管理SpoofingDoa对象生命周期(创建、初始化、释放)
//           2. 对象容器管理(支持多实例并发运行)
//           3. 线程安全(预留互斥锁机制)
//           4. 将外部调用转发到对应的SpoofingDoa对象方法
// 接口标准: C语言调用约定，确保跨语言(如C/C++/Python等)兼容调用
// =============================================================================
#include "interface.h"

// using namespace PublicSpace;


// =========================================================================
// 创建测向对象
// 功能: 在全局容器中创建一个新的SpoofingDoa实例，并分配唯一ID
// 实现: 采用递增ID分配策略，确保ID不重复(从0开始递增)
//       SpoofingDoa构造函数会自动完成初始化(读取配置、频率表等)
// =========================================================================
int createSpoofingDoa(int &id){

    // 查找下一个可用的ID(从m_num开始向上查找空闲位置)
    while (1){
        if (SpoofingDoaObjectContainer.find(m_num) == SpoofingDoaObjectContainer.end()){
            break;  // 找到可用的ID
        }
        else{
            m_num++;  // 该ID已被占用，继续寻找
        }
    }

    // g_lock.lock();
    id = m_num;

    // 创建日志上下文(用于多实例日志隔离)
    getLogCont(id);

    // 创建SpoofingDoa对象(构造函数中完成初始化)
    SpoofingDoa *target = new SpoofingDoa();
    SpoofingDoaObjectContainer[m_num] = target;  // 存入全局容器
    m_num++;
    //  g_lock.unlock();
    return 0;
}


// =========================================================================
// 初始化测向对象(预留接口)
// 当前未实现实际功能，仅做参数合法性检查
// 注: 实际初始化已在构造函数中完成
// =========================================================================
int initSpoofingDoa(int id, const char *adr){
    getLogCont(id);
    // std::lock_guard<std::mutex> lock(g_lock);

    if (SpoofingDoaObjectContainer[id] == NULL){
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }

    SpoofingDoa *target = SpoofingDoaObjectContainer[id];

    // std::lock_guard<std::mutex> lock(*(target->mu()));
    // int a = target->Init(adr);
    // target->setConfigTxtAdr(adr);
    return 0;
}


// =========================================================================
// 设置告警门限
// 功能: 为指定频点或所有频点设置欺骗检测的卫星颗数阈值和相位差阈值
// 原理: 欺骗信号来自同一干扰源，多颗卫星的相位差应相近
//       threshold: 同方向卫星数达到此值即判定为欺骗
//       phsThreshold: 相位差在此范围内认为"相近"
// =========================================================================
int setThresholdDetectionDoa(int id, int countThreshold, double phsThreshold, int sys, int type){
    getLogCont(id);
    // std::lock_guard<std::mutex> lock(g_lock);

    if (SpoofingDoaObjectContainer[id] == NULL){
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }

    SpoofingDoa *target = SpoofingDoaObjectContainer[id];

    // std::lock_guard<std::mutex> lock(*(target->getMutext()));
    target->setThresholdDetectionDoa(countThreshold, phsThreshold, sys, type);
    return 0;
}


// =========================================================================
// 设置GNSS观测数据 - 主要数据输入接口
// 功能: 驱动欺骗检测或测向流程，根据dataLen区分运行模式
//       dataLen=1: 仅欺骗检测(单帧相位差检测)
//       dataLen=2: 带校正的欺骗检测(先校正再检测)
//       dataLen>2: 完整测向模式(相位差计算→校正→检测→DOA)
// =========================================================================
int setSpoofingDoa(int id, const GNSSData *data, int length){
    getLogCont(id);
    // std::lock_guard<std::mutex> lock(g_lock);

    if (SpoofingDoaObjectContainer[id] == NULL){
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }

    SpoofingDoa *target = SpoofingDoaObjectContainer[id];

    // std::lock_guard<std::mutex> lock(*(target->getMutext()))
    target->setGNSSData(data, length);
    return 0;
}

// =========================================================================
// 获取欺骗测向结果 - 主要数据输出接口
// 功能: 返回处理后的测向结果，包括每个频点的到达角和卫星列表
// =========================================================================
int getAngleSpoofingDoa(int id, SpoofingResult &result)
{
    getLogCont(id);
    // std::lock_guard<std::mutex> lock(g_lock);
    if (SpoofingDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SpoofingDoa *target = SpoofingDoaObjectContainer[id];

    // std::lock_guard<std::mutex> lock(*(target->getMutext()));
    int a = target->getAngleSpoofingDoa(result);
    return a;
}

// =========================================================================
// 设置罗盘基准角度(预留接口)
// 用于外部罗盘/惯导提供的参考方向校准
// 当前未实现
// =========================================================================
int setCompassDoa(int id, double result)
{
    getLogCont(id);
    // 预留接口，当前不执行任何操作
    // if (SpoofingDoaObjectContainer[id] == NULL)
    // {
    //     printf("invalid DeceiveDoa* parameter");
    //     return -1;
    // }
    // SpoofingDoa *target = SpoofingDoaObjectContainer[id];

    return 0;
}

// =========================================================================
// 设置连续欺骗检测记录数
// 功能: 配置滑动窗口长度，需连续N帧均检测为欺骗才最终判定
// 作用: 降低瞬时干扰或噪声导致的虚警率
// =========================================================================
int setDetectionRecordNum(int id, int num)
{
    getLogCont(id);
    // std::lock_guard<std::mutex> lock(g_lock);
    if (SpoofingDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }
    SpoofingDoa *target = SpoofingDoaObjectContainer[id];
    target->setDetectionRecordNum(num);
    return 0;
}

// =========================================================================
// 释放测向对象
// 功能: 销毁指定的SpoofingDoa实例，释放所有相关资源
// 操作: 1. 关闭日志 2. 从全局容器移除 3. delete对象 4. 递减计数
// =========================================================================
int releaseSpoofingDoa(int id)
{
    getLogCont(id);
    // std::lock_guard<std::mutex> lock(g_lock);
    if (SpoofingDoaObjectContainer[id] == NULL)
    {
        printf("invalid DeceiveDoa* parameter");
        return -1;
    }

    SpoofingDoa *target = SpoofingDoaObjectContainer[id];
    LogClose();                          // 关闭日志文件
    SpoofingDoaObjectContainer.erase(id); // 从容器中移除
    delete target;                        // 释放内存
    target = NULL;                        // 置空指针
    m_num--;                              // 计数递减
    // std::lock_guard<std::mutex> lock(g_lock);
    // std::lock_guard<std::mutex> lock(*(target->getMutext()));
    return 0;
}
