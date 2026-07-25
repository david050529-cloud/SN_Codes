// =============================================================================
// 文件名: SpoofingDoaGN560.h
// 描  述: GN560设备欺骗信号测向类的头文件
// 功  能: 定义 SpoofingDoaGN560 类，继承自 SpoofingDoa 基类。
//         该类专用于 GN560 硬件平台，与基类的默认配置存在以下关键差异：
//         - 天线类型: 定向天线 (m_antnenaType = 1)，基类默认为全向天线 (0)
//         - 天线数量: 7 个阵元 (m_AntennaNum = 7)
//         - 测向算法: 相关干涉仪 (m_Doa_Arithmetic = 1)
//         - 启用伪谱积分 (m_PseudoSpectrum_Flag = 1)
//         - 启用欺骗检测筛选 (m_Screen_detection_Flag = 1)
//         - 专门的检测阈值调用 setThresholdDetectionDoa()
//         这些参数在 Init() 方法中完成配置。
// =============================================================================

#include "../SpoofingDoaDll/SpoofingDoa.h"
#pragma once
#include <math.h>
#include <stdarg.h>
#include <vector>
#include <queue>
#include <map>
#include <mutex>
#include <complex>
#include <algorithm>
#include <cstring>
#include <random>
using namespace std;

// =============================================================================
// 类  名: SpoofingDoaGN560
// 继承自: SpoofingDoa (公有继承)
// 描  述: GN560 平台的欺骗信号测向处理类。
//         与基类 SpoofingDoa 的主要区别在于 Init() 方法中设置了 GN560 特有的参数，
//         包括定向天线模式、7阵元阵列、相关干涉仪算法、伪谱积分等。
//         该类自身不添加新的成员变量或方法，所有核心算法均继承自基类。
// 用  法: SpoofingDoaGN560 obj;
//         obj.Init("/Radius.txt");  // 传入配置文件路径进行初始化
// =============================================================================
class SpoofingDoaGN560 : public SpoofingDoa
{
    public:

    // 默认构造函数，不执行任何特殊操作，初始化逻辑在 Init() 中完成
    SpoofingDoaGN560(void){};

    // 析构函数，基类 SpoofingDoa 的析构函数会自动被调用
    ~SpoofingDoaGN560(void){};

    // -------------------------------------------------------------------------
    // 方法名: Init
    // 参  数: adr - 配置文件路径（通常为 "/Radius.txt"），用于读取阵列半径等配置
    // 返回值: 无
    // 描  述: GN560 平台的初始化函数。重写基类方法，完成以下工作：
    //         1. 保存配置文件路径
    //         2. 设置 GN560 特有参数：天线数、天线类型、算法选择、阈值等
    //         3. 调用基类 SpoofingDoa::Init() 完成通用初始化
    //         4. 调用 setThresholdDetectionDoa() 为各频点设置欺骗检测阈值
    // 注  意: 此方法必须在对象创建后、任何其他操作前调用
    // -------------------------------------------------------------------------
    void Init(const char *adr);

};