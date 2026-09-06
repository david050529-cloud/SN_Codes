// =============================================================================
// 文件名: interface_GN902.h
// 功能描述: GN902 欺骗检测/测向 DLL 接口声明（对应 接口.md 中的 GN902 接口）
// 系统角色: GN902 主控按「逐帧流式 + 切刀标识 + 结束标志」把两端口卫星观测
//           (GNSSData) 送入 DLL。校正刀(code=0 / 天线对1-1 / cutIdx=1)数据单独
//           存为校正数据且不参与测向；测向轮由六刀(cutIdx=2..7)组成，本层缓冲六刀、
//           endFlag=1 触发一次「校正 + 六刀检测测向」。
//
// 约定（未在 接口.md 文档化处，见 interface_GN902.cpp 顶部说明）：
//   1. Init_GN902 的卫星数/相位差阈值数组按固定「系统×频点」表顺序提供(长度>=12)，
//      未列频点用 DLL 默认阈值(卫星数2、相位差5°)；
//   2. cutCountThreshold[0] = 连续确认刀数 p(对应 ALARM_CONSECUTIVE_P，建议 2)；
//   3. SetData_GN902: 每帧一个 GNSSData + cutIdx(通道2对应天线号1..7)；
//      cutIdx=1 的帧被储存为校正数据(每轮出现则刷新)；
//      endFlag=1 表示六刀测向轮数据结束，触发一次「校正+六刀」完整循环切刀
//      「检测+测向」，跨轮连续确认/跟踪状态在引擎内持续累积。
// =============================================================================
#pragma once
#include "SpoofingDoa.h"

#if defined(_MSC_VER) || defined(_WIN32) || defined(_WIN64)
#define GN902_API extern "C" __declspec(dllexport)
#else
#define GN902_API extern "C"
#endif

// 版本号（避免与 interface.h 的 Version 全局名冲突）
extern char GN902Version[];

// 创建 GN902 欺骗测向对象
// @param id [out] 对象唯一 ID
// @return 0=成功, 1=失败
GN902_API int Create_GN902(int &id);

// 初始化 GN902 对象（配置运行参数 + 阈值）
// @param id                    算法 id
// @param phsDiffThreshold      相位差阈值数组(度)，顺序见表 3
// @param satelliteCountThreshold 卫星数量阈值数组(颗，严格大于触发)，顺序同上
// @param cutCountThreshold     切刀次数阈值数组(连续确认刀数 p)
// @return 0=成功, 1=错误 id
GN902_API int Init_GN902(int id, double *phsDiffThreshold, double *satelliteCountThreshold,
                         double *cutCountThreshold);

// 向 GN902 对象送入一帧卫星观测数据(流式)
// @param id      算法 id
// @param data    单帧两通道 GNSS 观测
// @param cutIdx  通道2 对应天线号(1..7；1 为校正刀 code=0/天线对1-1，数据存为校正；
//               2..7 为六刀测向刀 {1,2}..{1,7})
// @param endFlag 六刀测向轮是否结束：0=否，1=是(触发一次「校正+六刀」检测+测向)
// @return 0=成功, 1=错误 id
GN902_API int SetData_GN902(int id, const GNSSData *data, int cutIdx, int endFlag);

// 获取 GN902 对象测向结果（单角，度 0..359）
// @param id     算法 id
// @param angle  [out] 欺骗信号来向角；无有效结果时置 -1
// @return 0=成功, 1=错误 id
GN902_API int GetResult_GN902(int id, double &angle);

// 释放 GN902 对象
// @param id 算法 id
// @return 0=成功, 1=错误 id
GN902_API int Release_GN902(int id);

// 获取算法版本
GN902_API char *GetALGVersionGN902(void);
