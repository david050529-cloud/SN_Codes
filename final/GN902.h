#pragma once

struct SatelliteData
{
    int i_Prn;      // 卫星号（PRN）
    int i_Sys;      // 卫星系统编码
    int i_Type;     // 频点编码
    double i_Psr;   // 伪距（米）
    float i_Snr;    // 载噪比（dB-Hz）
    double i_Phase; // 载波相位（周）
    double i_Dop;   // 多普勒频移（Hz）
};

struct GNSSData
{
    int i_PortOneNum;             // Port1卫星条数
    SatelliteData i_PortOne[500]; // Port1所有卫星数据
    int i_PortTwoNum;             // Port2卫星条数
    SatelliteData i_PortTwo[500]; // Port2所有卫星数据
};

struct AlarmData
{
    // 卫星号（PRN），取值范围与卫星系统 i_Sys 对应：
    //   GPS(0)      : 1~32
    //   GLONASS(1)  : 1~24（扩展号 65~85）
    //   SBAS(2)     : 120~158
    //   Galileo(3)  : 1~36
    //   BDS(4)      : 1~63（BDS-2: 1~16, BDS-3: 17~63）
    //   QZSS(5)     : 193~202
    int i_Prn;
    float i_Snr;      // 载噪比
    int i_Angle;      // 测向角度
    double i_Quality; // 测向质量（0-100）
};


struct SatelliteAngle
{
    int i_Sys;       // 卫星系统
    int i_Type;      // 卫星频点
    int i_Alarm;     // 报警标识：0=正常，1=欺骗
    double i_Angle;  // 欺骗信号来向角度（度）
    int i_Count;     // 被欺骗卫星数
    AlarmData i_AlarmData[32]; // 最多32颗报警卫星详情
};

struct SpoofingResult
{
    int i_Count;                          // 报警频点数
    SatelliteAngle i_SatelliteAngle[24];  // 最多24个频点的结果
};


class GN902
{
public:
    GN902();
    ~GN902();
    void SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum);
    void SetData(const GNSSData* data, int cutIdx, int endFlag);
    void GetResult(SpoofingResult& result);
private:
    void Detect();
    void Doa();
};
