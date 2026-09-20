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

GN902_EXTERN_C char* GetALGVersion();
GN902_EXTERN_C int Create_GN902(int &id);

GN902_EXTERN_C int SetThresholdDetection_GN902(
    int id,
    int satelliteCountThreshold,
    double phsDiffThreshold,
    int sysEnum,
    int typeEnum);

// 一秒传入一次数据；cutIdx_1 通道1天线，cutIdx_2 通道2天线
//   - 校正刀: (7,7)  同一根天线接两个端口，测通道固有相差；每轮必须传一次
//   - 六测向刀: (1,2)..(1,7)  参考天线(通道1)固定为天线 1，通道2 依次切到 2..7
//   - 天线对 {8,9} 或 {9,8} 为固定基线采集模式（仅采集相位差，不测向）
// 注意: 校正刀若不按 (7,7) 传入，该帧会被整帧丢弃，轮边界不触发，
//       Detect/Doa 一次都不执行（表现为完全无结果）。
GN902_EXTERN_C int SetData_GN902(int id, const GNSSData* data,
                                 int cutIdx_1, int cutIdx_2);

GN902_EXTERN_C int GetResult_GN902(int id, SpoofingResult& result);

// ABI 自检: 返回本库编译时各跨边界结构体的尺寸指纹(见 GN902.h 的 GN902AbiSignature)。
// 宿主按自己的头文件算 GN902AbiSignature() 与本函数比对, 不相等即说明
// "头文件与库不是同一次构建", 必须重新编译库再运行 —— 否则 GetResult_GN902
// 会按库的结构体尺寸写宿主的栈对象, 触发 *** stack smashing detected ***。
GN902_EXTERN_C unsigned int GetAbiSignature_GN902();

GN902_EXTERN_C int Release_GN902(int id);

// 连续切刀计数阈值: >0 时生效, 对应连续报警确认次数
GN902_EXTERN_C int SetCutnumThreshold_GN902(int id, int thresholdCount);

#endif