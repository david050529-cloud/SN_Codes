// gn902_interface.cpp
#include "gn902_interface.h"
#include <cstdio>
#include <new>
#include <exception>
#include <iostream>

// #define DOA_DEBUG
#ifdef _WIN32
 char Version[] = "V1.6.9.1";
#else
 char Version[] = "V1.6.9.1";
#endif

using namespace std;

// GN902Container 的定义在 GN902.cpp 中，这里只 extern 声明（已在头文件中声明）
// 不要重复定义！

// 获取版本号 
char* GetALGVersion(){
	return Version;
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
                                double phsDiffThreshold,
                                double satelliteCountThreshold,
                                double cutCountThreshold,
                                int sysEnum,
                                int typeEnum){
    if(id < 0 || id >= (int)GN902Container.size() || !GN902Container[id]){
        return 1;
    }
    GN902* p = GN902Container[id];
    p->SetThresholdDetection(phsDiffThreshold, satelliteCountThreshold,
                             cutCountThreshold, sysEnum, typeEnum);
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