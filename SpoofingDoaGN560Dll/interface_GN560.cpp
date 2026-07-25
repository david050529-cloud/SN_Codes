// =============================================================================
// 文件名: interface_GN560.cpp
// 描  述: GN560欺骗信号测向DLL接口函数的实现
// 功  能: 实现 interface_GN560.h 中声明的 6 个导出函数，提供完整的测向引擎
//         生命周期管理。本文件充当 DLL 与外部调用者之间的桥梁，负责：
//         1. 实例生命周期管理（创建、初始化、释放）
//         2. 数据格式转换（外部 RangData 转内部 GNSSData）
//         3. 结果传递（内部 SpoofingResult 转外部 DJDFResult）
//         4. 欺骗检测结果注入和切刀顺序管理
//
// 架构设计:
//   外部调用者 <--> interface_GN560.cpp (DLL接口层)
//                     |
//                     v
//              SpoofingDoaGN560 (GN560专用类)
//                     |
//                     v
//              SpoofingDoa (基类，核心算法)
//                     |
//                     v
//              ArithmeticDoa (算法基类)
//
// 数据流:
//   输入: RangData (外部格式) --> GNSSData (内部格式) --> 测向算法 --> 测向结果
//   输出: SpoofingResult (内部格式) --> DJDFResult (外部格式)
//
// 多实例支持:
//   通过全局 map<int, SpoofingDoaGN560*> 管理多个测向引擎实例，
//   每个实例有唯一ID，支持同时处理多个接收机通道的数据。
// =============================================================================

#include "interface_GN560.h"
using namespace std;

// =============================================================================
// 函数名: createDeceiveDoa
// 描  述: 创建测向引擎实例并返回实例ID
// 实现细节:
//   1. getLogCont(m_num) - 记录日志上下文，便于多实例日志区分
//   2. new SpoofingDoaGN560() - 在堆上创建新对象
//   3. SpoofingDoaObjectContainer[m_num] = target - 存入全局容器
//   4. m_num++ - 自增计数器，为下一次创建准备新的唯一ID
//   5. return (m_num - 1) - 返回当前实例的ID
// 返回值: 新实例的唯一ID (>=0)
// 注  意: 创建后必须调用 initDeceiveDoa() 完成初始化才能使用
// =============================================================================
int createDeceiveDoa(void)
{
    getLogCont(m_num);                              // 设置当前日志上下文（用于多实例日志区分）
    SpoofingDoaGN560 *target = new SpoofingDoaGN560();  // 在堆上创建 GN560 测向引擎对象
    SpoofingDoaObjectContainer[m_num] = target;     // 将对象指针存入全局容器
    m_num++;                                        // 实例计数器自增，确保下一个实例有唯一ID
    return (m_num - 1);                             // 返回当前实例的ID（自增前的值）
}

// =============================================================================
// 函数名: initDeceiveDoa
// 描  述: 初始化指定的测向引擎实例
// 参  数: id  - 实例ID
//         adr - 配置文件路径（传入参数，但当前实现使用硬编码路径 "/Radius.txt"）
// 返回值: 0 表示成功，-1 表示失败（实例ID无效）
// 实现细节:
//   1. 检查实例ID是否有效（容器中是否存在对应对象）
//   2. 硬编码配置文件路径为 "/Radius.txt"（注意：传入的 adr 参数未被使用）
//   3. 调用 target->Init(adr2) 执行 GN560 的完整初始化流程
// 注  意: 当前实现中 Init 的参数是硬编码的 "/Radius.txt"，传入的 adr 参数被忽略。
//         这可能是一个临时方案或遗留设计，实际部署时应注意配置文件路径是否正确。
// =============================================================================
int initDeceiveDoa(int id, const char *adr)
{
    getLogCont(id);                                 // 设置当前日志上下文
    if (SpoofingDoaObjectContainer[id] == NULL)     // 检查实例是否存在
    {
        printf("invalid DeceiveDoa* parameter");    // 输出错误信息到控制台
        return -1;                                  // 返回失败码
    }
    char adr2[50] = "/Radius.txt";                  // 硬编码的配置文件路径（包含各频点阵列半径信息）
    SpoofingDoaGN560 *target = SpoofingDoaObjectContainer[id];  // 获取实例指针
    target->Init(adr2);                             // 执行 GN560 初始化（设置参数、读取配置、初始化理论相位差等）
    return 0;                                       // 返回成功码
}

// =============================================================================
// 函数名: setDataDeceiveDoa
// 描  述: 将原始观测数据转换为内部格式并送入测向引擎
// 参  数: id       - 实例ID
//         rangdata - 原始观测数据数组（RangData 格式，来自外部接收机）
//         len      - 数据数组长度（帧数，rangdata 中的元素个数）
// 返回值: 0 表示成功，-1 表示失败（实例ID无效）
// 实现细节:
//   1. 检查实例ID是否有效
//   2. 动态分配 GNSSData 数组（内部格式）
//   3. 逐帧逐通道拷贝数据：RangData -> GNSSData
//      - 拷贝 PortOne (第一通道) 数据：系统、频点、多普勒、相位、PRN、伪距、载噪比
//      - 拷贝 PortTwo (第二通道) 数据：同上
//   4. 调用 target->setGNSSData(data, len) 将数据送入基类处理
// 注  意:
//   - 使用 new 分配内存但未显式释放，setGNSSData 内部应负责管理内存
//   - 被注释掉的 lock_guard 表明原先考虑了多线程安全，当前版本未加锁
//   - RangData 和 GNSSData 的结构定义几乎相同，此处的逐字段拷贝是为了
//     解耦外部数据结构与内部数据结构
// =============================================================================
int setDataDeceiveDoa(int id, RangData *rangdata, int len)
{
    getLogCont(id);                                 // 设置当前日志上下文
    if (SpoofingDoaObjectContainer[id] == NULL)     // 检查实例是否存在
    {
        printf("invalid DeceiveDoa* parameter");    // 输出错误信息
        return -1;                                  // 返回失败码
    }
    SpoofingDoaGN560 *target = SpoofingDoaObjectContainer[id];  // 获取实例指针
    // std::lock_guard<std::mutex> lock(*(target->getMutext())); // 多线程互斥锁（当前注释未启用）

    // 动态分配内部数据格式数组（与 rangdata 长度一致）
    GNSSData *data = new GNSSData[len];
    for (int i = 0; i < len; i++)                   // 遍历每一帧数据
    {
        int PortOneNum = rangdata[i].i_PortOneNum;  // 第一通道卫星数量
        int PortTwoNum = rangdata[i].i_PortTwoNum;  // 第二通道卫星数量
        data[i].i_PortOneNum = PortOneNum;          // 拷贝第一通道卫星个数
        data[i].i_PortTwoNum = PortTwoNum;          // 拷贝第二通道卫星个数

        // ---- 拷贝第一通道 (Port One) 的卫星数据 ----
        for (int j = 0; j < PortOneNum; j++)
        {
            data[i].i_PortOne[j].i_Sys   = rangdata[i].i_PortOne[j].i_Sys;   // 卫星系统 (GPS/BDS/GLONASS等)
            data[i].i_PortOne[j].i_Type  = rangdata[i].i_PortOne[j].i_Type;  // 卫星频点 (L1/B1等)
            data[i].i_PortOne[j].i_Dop   = rangdata[i].i_PortOne[j].i_Dop;   // 载波多普勒 (Hz)
            data[i].i_PortOne[j].i_Phase = rangdata[i].i_PortOne[j].i_Phase; // 载波相位 (周)
            data[i].i_PortOne[j].i_Prn   = rangdata[i].i_PortOne[j].i_Prn;   // 卫星PRN号
            data[i].i_PortOne[j].i_Psr   = rangdata[i].i_PortOne[j].i_Psr;   // 伪距 (m)
            data[i].i_PortOne[j].i_Snr   = rangdata[i].i_PortOne[j].i_Snr;   // 载噪比 (dB-Hz)
        }

        // ---- 拷贝第二通道 (Port Two) 的卫星数据 ----
        for (int j = 0; j < PortTwoNum; j++)
        {
            data[i].i_PortTwo[j].i_Sys   = rangdata[i].i_PortTwo[j].i_Sys;   // 卫星系统
            data[i].i_PortTwo[j].i_Type  = rangdata[i].i_PortTwo[j].i_Type;  // 卫星频点
            data[i].i_PortTwo[j].i_Dop   = rangdata[i].i_PortTwo[j].i_Dop;   // 载波多普勒
            data[i].i_PortTwo[j].i_Phase = rangdata[i].i_PortTwo[j].i_Phase; // 载波相位
            data[i].i_PortTwo[j].i_Prn   = rangdata[i].i_PortTwo[j].i_Prn;   // 卫星PRN号
            data[i].i_PortTwo[j].i_Psr   = rangdata[i].i_PortTwo[j].i_Psr;   // 伪距
            data[i].i_PortTwo[j].i_Snr   = rangdata[i].i_PortTwo[j].i_Snr;   // 载噪比
        }
    }
    target->setGNSSData(data, len);     // 将转换后的数据送入基类进行信号处理和测向计算

    return 0;                           // 返回成功码
}

// =============================================================================
// 函数名: getAngleDeceiveDoa
// 描  述: 获取测向引擎计算出的欺骗信号来波方向角度
// 参  数: id     - 实例ID
//         result - 输出参数（引用），存储测向结果
// 返回值: 测向结果中的有效频点个数 (>=0)，-1 表示失败
// 实现细节:
//   1. 检查实例ID是否有效
//   2. 调用 target->getAngleSpoofingDoa(tp_result) 获取内部格式的测向结果
//   3. 将内部格式 SpoofingResult 转换为外部格式 DJDFResult
//      - 拷贝结果个数 (i_Count)
//      - 拷贝每个频点的角度信息：系统、频点、告警标志、角度、卫星数量
//      - 拷贝每个频点下各卫星的详细告警数据：PRN、载噪比、角度、测向质量
//   4. 返回有效频点个数
// 注  意:
//   - DJDFResult 与 SpoofingResult 的结构基本对应，但 DJDFResult 是 DLL 导出接口
//     使用的数据结构，而 SpoofingResult 是内部算法使用的数据结构，二者需要转换
//   - i_Alarm 字段：0=非欺骗信号，1=欺骗信号
//   - i_Quality 字段：测向质量因子，值越大表示测向结果可信度越高
// =============================================================================
int getAngleDeceiveDoa(int id, DJDFResult &result)
{
    getLogCont(id);                                 // 设置当前日志上下文
    if (SpoofingDoaObjectContainer[id] == NULL)     // 检查实例是否存在
    {
        printf("invalid DeceiveDoa* parameter");    // 输出错误信息
        return -1;                                  // 返回失败码
    }
    SpoofingDoaGN560 *target = SpoofingDoaObjectContainer[id];  // 获取实例指针
    SpoofingResult tp_result;                       // 临时内部格式测向结果
    int n = target->getAngleSpoofingDoa(tp_result); // 调用基类方法获取测向结果

    // ---- 将内部格式 SpoofingResult 转换为外部格式 DJDFResult ----
    result.i_Count = tp_result.i_Count;             // 拷贝有效频点个数
    for (int i = 0; i < result.i_Count; i++)        // 遍历每个频点/系统结果
    {
        // result.i_SatelliteAngle[i] = tp_result.i_SatelliteAngle[i]; // 已注释的直接赋值方式
        result.i_SatelliteAngle[i].i_Alarm = tp_result.i_SatelliteAngle[i].i_Alarm;   // 告警标志：0=非欺骗 1=欺骗
        result.i_SatelliteAngle[i].i_Angle = tp_result.i_SatelliteAngle[i].i_Angle;   // 该频点的测向角度(度)
        result.i_SatelliteAngle[i].i_Count = tp_result.i_SatelliteAngle[i].i_Count;   // 该频点被判定为欺骗的卫星个数
        result.i_SatelliteAngle[i].i_Sys   = tp_result.i_SatelliteAngle[i].i_Sys;     // 卫星系统 (1=GPS, 5=BDS等)
        result.i_SatelliteAngle[i].i_Type  = tp_result.i_SatelliteAngle[i].i_Type;    // 卫星频点

        // ---- 拷贝该频点下每个被判定为欺骗的卫星的详细告警数据 ----
        for (int j = 0; j < tp_result.i_SatelliteAngle[i].i_Count; j++)
        {
            result.i_SatelliteAngle[i].i_AlarmData[j].i_Prn     = tp_result.i_SatelliteAngle[i].i_AlarmData[j].i_Prn;     // 欺骗卫星PRN号
            result.i_SatelliteAngle[i].i_AlarmData[j].i_Snr     = tp_result.i_SatelliteAngle[i].i_AlarmData[j].i_Snr;     // 该卫星的载噪比
            result.i_SatelliteAngle[i].i_AlarmData[j].i_Angle   = tp_result.i_SatelliteAngle[i].i_AlarmData[j].i_Angle;   // 该卫星的测向角度
            result.i_SatelliteAngle[i].i_AlarmData[j].i_Quality = tp_result.i_SatelliteAngle[i].i_AlarmData[j].i_Quality; // 该卫星的测向质量
        }
    }
    return n;                                       // 返回测向结果的频点个数
}

// =============================================================================
// 函数名: setDeceptiveResult
// 描  述: 将外部欺骗检测模块的检测结果注入测向引擎
// 参  数: id   - 实例ID
//         data - 欺骗检测结果（常量引用）
// 返回值: 0 表示成功，-1 表示失败（实例ID无效），-2 表示未进行切刀操作
// 实现细节:
//   1. 检查实例ID是否有效
//   2. 检查 cutSequencenum 是否为 0：
//      - 若为 0，表示外部未进行天线切刀操作，无法进行定向天线欺骗测向
//      - 此时输出警告 "cutSequencenum = 0 !!!" 并返回 -2
//   3. 调用 target->setCutSquence() 设置天线切刀顺序
//      - cutSequencenum: 切刀次数
//      - cutSequence: 每刀对应的天线对编码数组
//   4. 将 DeceptiveResult 中的欺骗检测结果转换为 vector<SingleDeceptiveResult>
//      - 若欺骗数量 > 0，遍历填充 vector
//      - 若欺骗数量 <= 0，输出日志 "spoofing gnss have not!!!" (无欺骗信号)
//   5. 调用 target->setSpoofingDetecteResult() 将欺骗检测结果注入引擎
// 注  意:
//   - 此函数应在 setDataDeceiveDoa 之前调用，使引擎提前知道哪些卫星是欺骗信号
//   - cutSequencenum 通常为 8（GN560 定向天线标准切刀模式）
//   - cutSequence 中的天线对编码用于定向天线测向时选择对应的天线对相位差数据
//   - 引擎根据这些欺骗检测结果，结合 m_Screen_detection_Flag 决定是否筛选数据
// =============================================================================
int setDeceptiveResult(int id, const DeceptiveResult data)
{
    getLogCont(id);                                 // 设置当前日志上下文
    if (SpoofingDoaObjectContainer[id] == NULL)     // 检查实例是否存在
    {
        printf("invalid DeceiveDoa* parameter");    // 输出错误信息
        return -1;                                  // 返回失败码
    }
    SpoofingDoaGN560 *target = SpoofingDoaObjectContainer[id];  // 获取实例指针

    // ---- 检查是否进行了天线切刀操作 ----
    if (data.cutSequencenum == 0)                   // 未进行切刀（单天线或未切换模式）
    {
        cout << "cutSequencenum = 0 !!!" << endl;   // 输出警告信息
        return -2;                                  // 返回"未进行切刀"错误码
    }

    // ---- 设置天线切刀顺序 ----
    // cutSequencenum: 切刀次数（如 8 表示 8 次切刀）
    // cutSequence: 每次切刀对应的天线对编码，定向天线通过这些天线对进行相位差测量
    target->setCutSquence(data.cutSequencenum, data.cutSequence);

    // ---- 转换欺骗检测结果格式 ----
    vector<SingleDeceptiveResult> deceptive;
    deceptive.clear();
    vector<SingleDeceptiveResult>().swap(deceptive); // 强制释放 vector 内存

    int deceipNum = data.strnum;                    // 被判定为欺骗的频点个数
    if (deceipNum > 0)                              // 存在欺骗信号
    {
        deceptive.resize(deceipNum);                // 调整 vector 大小
        for (int i = 0; i < deceipNum; i++)         // 遍历每个欺骗频点
        {
            deceptive[i] = data.allresult[i];       // 拷贝欺骗检测详细结果（系统/频点/角度/卫星号列表）
        }
    }
    else                                            // 无欺骗信号
    {
        Log("spoofing gnss have not!!!");           // 记录日志：未检测到欺骗信号
    }

    // ---- 将欺骗检测结果注入测向引擎 ----
    // 引擎将在后续测向过程中根据此结果筛选数据（当 m_Screen_detection_Flag=1 时）
    target->setSpoofingDetecteResult(deceptive);
    return 0;                                       // 返回成功码
}

// =============================================================================
// 函数名: releaseDeceiveDoa
// 描  述: 释放指定的测向引擎实例，回收所有内存和系统资源
// 参  数: id - 实例ID
// 返回值: 0 表示成功，-1 表示失败（实例ID无效或已被释放）
// 实现细节:
//   1. 检查实例ID是否有效
//   2. 从全局容器中移除该实例条目
//   3. 重置实例计数器 m_num = 0
//   4. 调用 delete 释放对象内存（自动调用析构函数链：SpoofingDoaGN560 -> SpoofingDoa -> ArithmeticDoa）
// 注  意:
//   - m_num = 0 这是一个有问题的设计：它假设每次只有一个实例在用，
//     如果有多个实例同时存在，释放一个实例后新的 ID 会与已有的冲突。
//     在多实例场景中应避免使用此重置逻辑。
//   - 释放后该 ID 将失效，调用者不应再使用此 ID 调用其他接口函数。
// =============================================================================
int releaseDeceiveDoa(int id)
{
    if (SpoofingDoaObjectContainer[id] == NULL)     // 检查实例是否存在（或已被释放）
    {
        printf("invalid DeceiveDoa* parameter");    // 输出错误信息
        return -1;                                  // 返回失败码
    }
    SpoofingDoaGN560 *target = SpoofingDoaObjectContainer[id];  // 获取实例指针
    SpoofingDoaObjectContainer.erase(id);           // 从全局容器中移除该实例（不释放内存，仅移除引用）
    m_num = 0;                                      // 重置实例计数器（注意：多实例场景下这是有问题的）
    delete target;                                  // 释放对象内存，自动调用析构函数链
    return 0;                                       // 返回成功码
}
