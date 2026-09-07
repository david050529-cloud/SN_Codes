#include "GN902.h"

GN902::GN902(){

}

GN902::~GN902(){
    
}


// 设置阈值检测参数
// @param phsDiffThreshold 位相差阈值
// @param satelliteCountThreshold 卫星数阈值
// @param cutCountThreshold 通道2对应天线阈值
// @param sysEnum 系统类型
// @param typeEnum 类型
void GN902::SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum){
    return;
}

// 设置数据
// @param data 数据指针
// @param cutIdx 通道二天线索引
// @param endFlag 结束标志
void GN902::SetData(const GNSSData* data, int cutIdx, int endFlag){
    // 非最后一帧数据，保存数据
    if (!endFlag){
        // 保存数据
        ;
    }
    // 最后一阵数据，更新是否报警，判断是否需要测向，如需，测向
    else{
        // 数据处理;
        ;
        // 更新告警flag
        ;
        // 是否告警
        ;
        // 是否需要测向
        ;
    }
    return;
}

// 获取结果
// @param result 结果结构体
void GN902::GetResult(SpoofingResult& result){
    // 取结果
    return;
}

// 检测
void GN902::Detect(){
    return;
}

// 测向
void GN902::Doa(){
    return;
}
