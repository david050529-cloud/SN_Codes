#ifndef GN902_INTERFACE_H__
#define GN902_INTERFACE_H__

#include "GN902.h"
#include <vector>

#ifdef _WIN32
    #define GN902_EXTERN_C extern "C" __declspec(dllexport)
#else
    #define GN902_EXTERN_C extern "C"
#endif

// 与 interface.h 中同名，保持一致
extern std::vector<GN902*> GN902Container;

GN902_EXTERN_C int Create_GN902(int &id);

GN902_EXTERN_C int SetThresholdDetection_GN902(
    int id,
    double phsDiffThreshold,
    double satelliteCountThreshold,
    double cutCountThreshold,
    int sysEnum,
    int typeEnum);

// 一秒传入一次数据；cutIdx_1 通道1天线，cutIdx_2 通道2天线
// 天线对 {8,9} 或 {9,8} 为固定基线采集模式（仅采集相位差，不测向）
GN902_EXTERN_C int SetData_GN902(int id, const GNSSData* data,
                                 int cutIdx_1, int cutIdx_2);

GN902_EXTERN_C int GetResult_GN902(int id, SpoofingResult& result);

GN902_EXTERN_C int Release_GN902(int id);

#endif