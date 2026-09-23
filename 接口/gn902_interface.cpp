// gn902_interface.cpp
#include "gn902_interface.h"
#include <cstdio>
#include <new>
#include <exception>
#include <iostream>

// #define DOA_DEBUG
#ifdef _WIN32
 char Version[] = "V1.1.0";
#else
 char Version[] = "V1.1.0";
#endif

using namespace std;

// GN902Container 的定义在 GN902.cpp 中，这里只 extern 声明（已在头文件中声明）
// 不要重复定义！

// 获取版本号
char* GetALGVersion(){
	return Version;
}

// ABI 自检: 返回本库编译时各跨边界结构体的尺寸指纹。
// 宿主用 GN902AbiSignature()(见 GN902.h) 与本函数比对, 不相等即说明
// 头文件与库不是同一次构建(典型: 改了结构体但加载的还是旧库), 必须重编。
// 此时若继续运行, GetResult_GN902 会按库的结构体尺寸写宿主的栈对象,
// 触发 *** stack smashing detected *** (或静默的字段错位)。
unsigned int GetAbiSignature_GN902(){
	return GN902AbiSignature();
}

// 创建GN902对象
int Create_GN902(int & id){
    try{
        GN902* p = new (nothrow) GN902();
        if(!p){
            return 1;
        }
        for(id = 0; id < (int)GN902Container.size(); ++id){
            if(!GN902Container[id]){
                GN902Container[id] = p;
                break;
            }
        }
        if(id == (int)GN902Container.size()){
            GN902Container.push_back(p);
        }
    }
    catch(const std::exception& e){
        std::cout << "error:: create GN902 failed" << endl;
        std::cerr << e.what() << '\n';
        return -1;
    }
    return 0;
}

// 设置阈值
int SetThresholdDetection_GN902(int id,
                                int satelliteCountThreshold,
                                double phsDiffThreshold,
                                int sysEnum,
                                int typeEnum){
    if(id < 0 || id >= (int)GN902Container.size() || !GN902Container[id]){
        return 1;
    }
    GN902* p = GN902Container[id];
    p->SetThresholdDetection(phsDiffThreshold, satelliteCountThreshold,
                             sysEnum, typeEnum);
    return 0;
}

// 设置连续切刀数(连续报警确认次数)
int SetCutnumThreshold_GN902(int id, int thresholdCount){
    if(id < 0 || id >= (int)GN902Container.size() || !GN902Container[id]){
        return 1;
    }
    GN902* p = GN902Container[id];
    p->SetCutnumThreshold(thresholdCount);
    return 0;
}

// 喂数据
int SetData_GN902(int id, const GNSSData* data, int cutIdx_1, int cutIdx_2){
    if(id < 0 || id >= (int)GN902Container.size() || !GN902Container[id]){
        return 1;
    }
    GN902* p = GN902Container[id];
    p->SetData(data, cutIdx_1, cutIdx_2);
    return 0;
}

// 取结果
int GetResult_GN902(int id, SpoofingResult& result){
    if(id < 0 || id >= (int)GN902Container.size() || !GN902Container[id]){
        return 1;
    }
    GN902* p = GN902Container[id];
    p->GetResult(result);
    return 0;
}

// 释放
int Release_GN902(int id){
    if(id < 0 || id >= (int)GN902Container.size() || !GN902Container[id]){
        return 1;
    }
    delete GN902Container[id];
    GN902Container[id] = nullptr;
    return 0;
}