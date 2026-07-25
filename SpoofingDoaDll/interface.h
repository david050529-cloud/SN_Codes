// =============================================================================
// 文件名: interface.h
// 功能描述: DLL导出接口头文件
// 系统角色: 本文件定义了欺骗测向DLL的所有导出函数(C语言接口)，
//           供外部程序(如主控软件)通过动态链接库方式调用。
//           接口设计遵循C语言调用约定，跨平台兼容Windows和Linux。
// 主要接口:
//   createSpoofingDoa      - 创建欺骗测向算法对象(支持多实例)
//   initSpoofingDoa        - 初始化测向对象
//   setThresholdDetectionDoa - 设置欺骗检测阈值
//   setSpoofingDoa         - 输入GNSS观测数据
//   getAngleSpoofingDoa    - 获取测向结果
//   setCompassDoa          - 设置罗盘数据(预留)
//   setDetectionRecordNum  - 设置连续检测记录数
//   releaseSpoofingDoa     - 释放测向对象资源
// =============================================================================
#pragma once
#include "SpoofingDoa.h"

// 版本信息:根据编译平台自动选择Windows或Linux版本标识
#ifdef _WIN32
char Version[] = "V1.0.0.20250710_X86_WIN64_debug";
#else
char Version[] = "V1.0.0.20250710_X86_LIN64_debug";
#endif

// DLL导出宏定义:Windows使用__declspec(dllexport)，Linux使用extern "C"
#if defined(_MSC_VER) || defined(_WIN32) || defined(_WIN64)
#ifdef EXTERN_C
#undef EXTERN_C
#endif
#define EXTERN_C extern "C" __declspec(dllexport)
#else
    #define EXTERN_C extern "C"
#endif

using namespace PublicSpace;
using namespace std;

// 全局测向算法对象容器:支持创建多个独立的测向实例
// key=对象唯一ID, value=SpoofingDoa对象指针
map<int, SpoofingDoa *> SpoofingDoaObjectContainer;

// 测向算法对象数量计数器(用于分配唯一ID)
int m_num = 0;

// =========================================================================
// DLL导出接口函数声明
// =========================================================================

// 获取算法版本号字符串(如 "V1.0.0.20250710_X86_WIN64_debug")
EXTERN_C char* GetALGVersion(void) { return Version;  };

// 创建一个欺骗测向对象
// @param id [out] 返回创建的对象唯一标识ID
// @return 0-成功, 其他-失败
EXTERN_C int createSpoofingDoa(int &id);

// 初始化测向对象(预留接口,当前未使用)
// @param id 测向对象ID
// @param adr 配置文件路径
EXTERN_C int initSpoofingDoa(int id, const char *adr);

// 设置欺骗检测阈值
// @param id 测向对象ID
// @param countThreshold 卫星颗数阈值(同方向卫星数>=此值判定为欺骗)
// @param phsThreshold 相位差阈值(度)
// @param sys 卫星系统(-1表示所有系统)
// @param type 卫星频点(-1表示所有频点)
EXTERN_C int setThresholdDetectionDoa(int id, int countThreshold, double phsThreshold, int sys, int type);

// 设置GNSS观测数据(驱动测向或检测流程)
// @param id 测向对象ID
// @param data GNSS原始观测数据
// @param length 数据条数(1=欺骗检测, 2=带校正检测, >2=完整测向)
EXTERN_C int setSpoofingDoa(int id, const GNSSData *data, int length);

// 获取欺骗测向结果
// @param id 测向对象ID
// @param result [out] 测向结果(包含各频点到达角和卫星列表)
EXTERN_C int getAngleSpoofingDoa(int id, SpoofingResult &result);

// 设置罗盘基准角度(预留接口,用于校准参考方向)
// @param id 测向对象ID
// @param result 罗盘读数(度)
EXTERN_C int setCompassDoa(int id, double result);

// 设置连续欺骗检测记录数(需连续N帧检测到欺骗才判定)
// @param id 测向对象ID
// @param num 连续帧数
EXTERN_C int setDetectionRecordNum(int id, int num);

// 释放测向对象(销毁对象并释放资源)
// @param id 需释放的测向对象ID
EXTERN_C int releaseSpoofingDoa(int id);
