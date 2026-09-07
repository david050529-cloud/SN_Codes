#ifndef INTERFACE_H__
#define INTERFACE_H__
#include "Doa.h"
#include "MicroDoa.h"
#include <vector>
#include "WW.h"
#include "AS.h"
#include "BaseDoa.h"
#include "OmniDoa.h"
#include "DireDoa.h"
#include "SuppressingDoa.h"
#include "GN902.h"
// #include "MulPath.h"
    
#ifdef _WIN32
    #define EXTERN_C extern "C" __declspec(dllexport)
#else
    #define EXTERN_C extern "C"
#endif

extern std::vector<Doa*> DoaContainer;
extern std::vector<MicroDoa*> MicroDoaContainer; 
extern std::vector<WW*> WWContainer;
extern std::vector<AS*> ASContainer;
extern std::vector<OmniDoa*> OmniDoaContainer;
extern std::vector<DireDoa*> DireDoaContainer;
extern std::vector<SuppressingDoa*> SuppressingDoaContainer;
extern std::vector<GN902*> GN902Container;

// 全向天线
EXTERN_C char* GetALGVersion();
EXTERN_C int Create_Doa(int &id,int channelNum,int antennaNum,int pointNum);
EXTERN_C int Init_Doa(int id,double f,double r);
EXTERN_C int SetThw_Doa(int id,int cutNum,const int *thw,int length);
EXTERN_C int SetByCut_Doa(int id);
EXTERN_C int SetByChannel_Doa(int id);
EXTERN_C int SetBW_Doa(int id,double BW);
EXTERN_C int SetUsePointNum_Doa(int id, int startPointNum, int endPointNum);
EXTERN_C int SetFrequencyBasedOn_Doa(int id);
EXTERN_C int SetFrequencyBasedOff_Doa(int id);
EXTERN_C int SetCorrection_Doa(int id,const double *correction,int length);
EXTERN_C int SetData_Doa(int id,const double* data,int length);
EXTERN_C int Release_Doa(int id);
EXTERN_C int Interferometer_Doa(int id,double &angle,double &quality);
EXTERN_C int GetLevel_Doa(int id, double &level);
EXTERN_C int EstSigNum_Doa(int id, int &sigNum);
EXTERN_C int MUSIC_Doa(int id, int sigNum, double *angles, double *amplitudes);
EXTERN_C int DML_Doa(int id, int sigNum, double *angles, double *amplitudes);
EXTERN_C int IDML_Doa(int id, int sigNum, double *angles, double *amplitudes, int *coherency, int &resultSigNum);
EXTERN_C int DBF_Doa(int id,int sigNum,double *angles,double *quality,double *amplitudes,std::complex<double> *coeff,int &coeffNum);
EXTERN_C int AI_Doa(int id,int sigNum,double *angles,double *quality,double *amplitudes);  // 需要更新
// 老版本使用时域IQ自行转换为频率IQ测向算法, 不再更新
EXTERN_C int FFT_Doa(int id, int sigNum, double *angles, double *quality, double *amplitudes);
EXTERN_C int SetFFTMode_Doa(int id, int FFTMode);
EXTERN_C int SetFFTTreshold_Doa(int id, double FFTThreshold);
EXTERN_C int SetFFTTresholdAdjust_Doa(int id, double FFTThresholdAdjust);
EXTERN_C int SetCorrectionFFT_Doa(int id, const double *correction, int length);
EXTERN_C int SetDataFFT_Doa(int id, const double *data, int length);
EXTERN_C int InterferometerFFT_Doa(int id, int &angleNum, double *angles, double *qualitys, int *frequencies);
EXTERN_C int SetTimeFactor_Doa(int id,double time,double BW);
EXTERN_C int SetUpdateDensity_Doa(int id,double newData,double &result);
EXTERN_C int SetUpdateDensityMult_Doa(int id,const double *newDatas,double *results,int dataNum);
EXTERN_C int SetClearDensity_Doa(int id);
EXTERN_C int SignalCombine_Doa(int id, int& angleNum, double* angles, double* qualitys, int* frequencies);
EXTERN_C int SetUseDensitiesFFT_Doa(int id, int mag);
EXTERN_C int SetUpdateDensitiesFFT_Doa(int id, int angleNum, double* angles, int* frequencies);
EXTERN_C int setClearDensitiesFFT_Doa(int id);
EXTERN_C int SetThrowPointNum_Doa(int id, int startCut, int endCut); 
EXTERN_C int Coherent_Doa(int id, int sigNum, double* angles);
EXTERN_C int SetUseAccumulation_Doa(int id, bool use);
EXTERN_C int GetSpecArr_Doa(int id, double* specArr);
// 设置FFT测向线程数量，由于设备性能较差，弃用
EXTERN_C int SetThreadNum_Doa(int id, int threadNum);
EXTERN_C int SetAverageTime_Doa(int id, double at);
EXTERN_C int SetFFTAverageNum_Doa(int id, int & at);


// 微波比幅
EXTERN_C int Create_MicroDoa(int &id,int antennaNum,int pointNum,double f);
EXTERN_C int Init_MicroDoa(int id,int type,const char *modelFile);
EXTERN_C int SetUsePointNum_MicroDoa(int id, int startPointNum, int endPointNum);
EXTERN_C int SetData_MicroDoa(int id,const double *data,int length);
EXTERN_C int GetAngle_MicroDoa(int id,double &angle);
EXTERN_C int SetFFTThreshold_MicroDoa(int id, double FFTThreshold);
EXTERN_C int SetDataFFT_MicroDoa(int id, const double *data, int length);
EXTERN_C int GetAngleFTT_MicroDoa(int id, int &angleNum, double *angles);
EXTERN_C int Release_MicroDoa(int id);
EXTERN_C int SetTimeFactor_MicroDoa(int id, double time, double BW);
EXTERN_C int SetUpdateDensity_MicroDoa(int id, double newData, double& result);
EXTERN_C int SetClearDensity_MicroDoa(int id);
EXTERN_C int SetGetAmpMode_MicroDoa(int id, int typy);


// 520S沃森瓦特函数（不再维护）
EXTERN_C int Create_WW(int& id, int channelNum, int antennaNum, int pointNum);
EXTERN_C int Init_WW(int id, double f, double r);
EXTERN_C int SetCorrection_WW(int id, const double* correction, int length);
EXTERN_C int SetData_WW(int id, const double* data, int length);
EXTERN_C int GetAngle_WW(int id, double& angle);
EXTERN_C int SetThresholdFFT_WW(int id, double threshold);
EXTERN_C int SetCorrectionFFT_WW(int id, const double* correction, int length);
EXTERN_C int SetDataFFT_WW(int id, const double* data, int length);
EXTERN_C int GetAngleFFT_WW(int id, int& angleNum, double* angles, int* frequencies);
EXTERN_C int Release_WW(int id);
EXTERN_C int SetByCut_WW(int id);
EXTERN_C int SetByChannel_WW(int id);


// 8阵元水平天线相关(幅相法)
EXTERN_C int Create_AS(int& id, int channelNum, int antNum, int pointNum);
EXTERN_C int Init_AS(int id, int type, double f, double r);
EXTERN_C int SetThw_AS(int id,int cutNum,const int* thw, int length);
EXTERN_C int SetThrowPointNum_AS(int id, int startCutNum, int endCutNum);
EXTERN_C int SetModel_AS(int id, const char* fileDir);
EXTERN_C int SetCorrection_AS(int id, const double* correctionData, int length);
EXTERN_C int SetData_AS(int id, const double* data, int length);
EXTERN_C int AmplitutdePhaseAlg_AS(int id, double& angle, double& quality);
EXTERN_C int AI_AS(int id,int sigNum, double* angles, double* qualities, double* amplitudes, int* newCut, int* newCutAntCodes, int &antCode);
EXTERN_C int DBF_AS(int id,int sigNum, std::complex<double> *coeff, bool &flag);
EXTERN_C int SetAverageTime_AS(int id, double at);
EXTERN_C int SetDataFFT_AS(int id, const double* data, int length);
EXTERN_C int SetAmplitudeDataFFT_AS(int id, const double* data, int length);
EXTERN_C int SetThresholdFFT_AS(int id, double threshold);
EXTERN_C int SetCorrectionFFT_AS(int id, const double* correction, int length);
EXTERN_C int GetAngleFFT_AS(int id, int& angleNum, double* angles, int* frequencies);
EXTERN_C int Release_AS(int id);
EXTERN_C int EstSigNum_AS(int id, int &sigNum);
EXTERN_C int Coherent_AS(int id, int sigNum, double *angles, double *amplitudes);

// 二期新增
EXTERN_C int SetDbfFlag_Doa(int id, int tag);
EXTERN_C int SetDbfFlag_AS(int id, int tag);
EXTERN_C int AntiMultipath_Doa(int id, double &angle, double &quality);
EXTERN_C int AntiMultipath_AS(int id, double &angle, double &quality);
EXTERN_C int SetFastAIMode_Doa(int id);
EXTERN_C int SetFastAIMode_AS(int id);
EXTERN_C int GetCorrectionCoeffFFT_AS(int id, short* corCoeff);
EXTERN_C int SetErrorAntennaIndex_Doa(int id, int antNum, int* errorIdx);
EXTERN_C int SetErrorAntennaIndex_AS(int id, int antNum, int* errorIdx);
EXTERN_C int GetPhaseDifferenceFFT_Doa(int id, double* phaseDiff);
EXTERN_C int SetElevationVec_Doa(int id, double* elevations, int eleLen);
EXTERN_C int SetElevationVec_AS(int id, double* elevations, int eleLen);

// 卫导版本
// SP 压制
// SF 欺骗
EXTERN_C int Create_GN902(int &id);
EXTERN_C int SetThresholdDetection_GN902(int id, double phsDiffThreshold, double satelliteCountThreshold, 
            double cutCountThreshold, int sysEnum, int typeEnum);
EXTERN_C int SetData_GN902(int id, const GNSSData* data, int cutIdx, int endFlag);
EXTERN_C int GetResult_GN902(int id, double& angle);
EXTERN_C int Release_GN902(int id);
#endif
