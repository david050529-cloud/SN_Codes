// =============================================================================
// 文件名: interface.h
// 功能描述: 抑制式测向DLL的外部C接口声明 (DLL导出接口)
//
//   本文件定义了SuppressDoa测向DLL的所有外部C语言接口函数。
//   通过这些接口，外部调用者(如C#/Python/LabVIEW等)可以:
//     1. 创建测向对象 (createSuppressDoa)
//     2. 初始化测向参数 (initSuppressDoa)
//     3. 设置校正数据 (setCorrectionDataSuppressDoa)
//     4. 传输测向IQ数据 (setDataSuppressDoa)
//     5. 设置信号个数 (setSignalNum)
//     6. 获取测向结果角度 (getAngleSuppressDoa)
//     7. 释放测向对象 (releaseSuppressDoa)
//
//   设计模式: 工厂模式 + 对象容器管理
//     - SuppressDoaObjectContainer: map<int, SuppressDoa*> 存储所有对象实例
//     - id: 每次创建返回唯一ID，后续所有操作通过ID访问对应对象
//     - g_lock: 互斥锁，保证多线程安全(m_num自增、容器操作)
//     - m_num: 自增计数器，作为新建对象的唯一标识ID
//
//   平台兼容: _MSC_VER/WIN32/WIN64 使用 __declspec(dllexport) 导出
//             Linux/其他 使用 extern "C" 导出
// =============================================================================
#include "SuppressDoa.h"
#include "../publicFunctionDoa/publicFunctionDoa.h"

#pragma once
#if defined(_MSC_VER) || defined(_WIN32) || defined(_WIN64)
#ifdef EXTERN_C
#undef EXTERN_C
#endif
#define EXTERN_C extern "C" __declspec(dllexport)
#else
#define EXTERN_C extern "C"
#endif
/*
#ifdef _WIN32
char Version[] = "V1.0.0.20250619_X86_WIN64_debug";
#else
char Version[] = "V1.0.0.20250619_X86_WIN64_debug";
#endif*/

// 对象容器: 存储所有创建的SuppressDoa对象，key=id, value=对象指针
map<int, SuppressDoa *> SuppressDoaObjectContainer;
mutex g_lock;      // 全局互斥锁，线程安全
int m_num = 0;     // 对象ID计数器(自增)
// 压制干扰测向

// 获取算法库版本号字符串
EXTERN_C char *GetALGVersion();

// 创建测向对象: 返回对象的唯一ID
// id: (输出)新建对象的唯一标识符
// pointNum: FFT采样点数
// startPointNum/endPointNum: 有效采样点的起止位置
EXTERN_C int createSuppressDoa(int &id, int pointNum, int startPointNum, int endPointNum);

// 初始化测向: 设置工作频率并完成理论参数计算
// f: 工作频率(Hz)
EXTERN_C int initSuppressDoa(int id, double f);

// 设置通道校正数据: 计算各通道相对参考通道的校正系数
EXTERN_C int setCorrectionDataSuppressDoa(int id, short *crrectionData, int length); // 设置校正数据

// 设置测向数据: 传输IQ数据到测向对象
EXTERN_C int setDataSuppressDoa(int id, short *data, int length);                    // 传输测向数据

// 设置同频信号个数: 多信号场景需要调用
EXTERN_C int setSignalNum(int id, int signalNum);

// 获取测向结果: 返回信号数、角度、置信度、幅度
// signalNum: (输入/输出)信号个数
// angles: 各信号角度(度, 0-360)
// qualities: 各信号置信度(0-1)
// amplitudes: 各信号幅度
EXTERN_C int getAngleSuppressDoa(int id, int &signalNum, double *angles, double *qualities, double *amplitudes);

// 获取频谱数据(暂未实现)
EXTERN_C int getSpectrumSupressDoa(int id, int signalNum, short *spectrumData, int &length);

// 释放测向对象: 从容器中删除并释放内存
EXTERN_C int releaseSuppressDoa(int id);

