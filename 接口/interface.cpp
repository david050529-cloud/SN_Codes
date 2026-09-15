#include "interface.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include <chrono>
#include <mutex>
#include <exception>

// #define DOA_DEBUG
#ifdef _WIN32
 char LogFile[] = "D:\\doa.log";
 char Version[] = "V1.6.9.1";
#else
 char LogFile[] = "./doa.log";
 char Version[] = "V1.6.9.1";
#endif
using namespace std;

mutex fileWriteMutex;

namespace {
	const int BY_CUT_MODE = 0;
	const int BY_CHANNEL_MODE = 1;
	unsigned int LogCount = 0;
	const unsigned int MAX_LOG_COUNT = 10240000;
	const int FFT_AMP_MODE = 1;
	const int IQ_AMP_MODE = 0;
}

std::vector<Doa*> DoaContainer;
std::vector<MicroDoa*> MicroDoaContainer	;
std::vector<WW*> WWContainer;
std::vector<AS*> ASContainer;
std::vector<OmniDoa*> OmniDoaContainer;
std::vector<DireDoa*> DireDoaContainer;
std::vector<SuppressingDoa*> SuppressingDoaContainer;

void LogCreate(){
	#ifdef DOA_DEBUG
		LogCount = 0;
		FILE *fp = fopen(LogFile, "w");
		fprintf(fp, "Doa LogFile create, version=%s\n", Version);
		fclose(fp);
	#endif
}

void Log(const char *format, ...){
	#ifdef DOA_DEBUG
		lock_guard<mutex> lock(fileWriteMutex);
		FILE *fp = fopen(LogFile, "a+");

		va_list args;
		va_start(args, format);
		vfprintf(fp, format, args);
		va_end(args);

		fclose(fp);
	#endif 
}


// 接口 - 获取算法版本号
// ********************************************************************************
// 调用后返回算法版本号
// ********************************************************************************
char* GetALGVersion(){
	return Version;
}

// 接口 - 创建Doa对象
// ********************************************************************************
// id - 对象编号
// channelNum - 通道数
// antennaNum - 天线数
// pointNum - 采样点数
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int Create_Doa(int &id,int channelNum,int antennaNum,int pointNum){
	try{
		OmniDoa* p  = new (nothrow) OmniDoa(channelNum, antennaNum, pointNum);
		if(!p){
			return 1;
		}
		for(id = 0; id < OmniDoaContainer.size();++id){
			if(!OmniDoaContainer[id]){
				OmniDoaContainer[id] = p;
				break;
			}
		}
		if(id == OmniDoaContainer.size()){
			OmniDoaContainer.push_back(p);
		}
	}
	catch(const std::exception& e)
	{
		std::cout << "error:: create Doa(创建Doa对象失败)" << endl;
		std::cerr << e.what() << '\n';
		return -1;
	}
	
	try{
		if (!id) {
			LogCreate();
		}
	}
	catch(const std::exception& e){
		std::cout << "error:: create Log(创建日志文件失败)" << endl;
		std::cerr << e.what() << '\n';
		return -2;
	}
	
    Log("Create_Doa success, id=%d,channelNum=%d, antennaNum=%d, pointNum=%d\n",id, channelNum, antennaNum, pointNum);

    return 0;
}

// 接口 - 初始化Doa对象
// ********************************************************************************
// id - 对象编号
// f - 通道数
// r - 天线数
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int Init_Doa(int id,double f,double r)
{
    Log("Init_Doa begin, id=%d, f=%f, r=%f\n",id,f,r);

    if(id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("Init_Doa failed\n");
        return 1;
    }

    OmniDoa *p = OmniDoaContainer[id];
    p->Init(f, r);
    Log("Init_Doa success,id=%d\n",id);
    return 0;
}

// 接口 - 设置切刀顺序
// ********************************************************************************
// id - 对象编号
// cutNum - 切刀数
// thw - 切刀数组，一维数组
// length - 切刀数组长度
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int SetThw_Doa(int id,int cutNum,const int *thw,int length)
{
    Log("SetThw_Doa begin, id=%d, cutNum=%d, length=%d, thw=[ ",id,cutNum,length);
    for(int i=0;i<length-1;++i){
        Log("%d, ",thw[i]);
    }
    Log("%d ]\n",thw[length-1]);


    if(id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetThw_Doa failed\n");
        return 1;
    }

    OmniDoa *p = OmniDoaContainer[id];
    vector<int> thwVec(thw, thw+length);
    p->SetThw(cutNum, thwVec);

    Log("SetThw_Doa success,id=%d\n",id);

    return 0;
}

// 接口 - 设置数据按刀优先传入
// ********************************************************************************
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int SetByCut_Doa(int id)
{
    Log("SetByCut_Doa begin, id=%d\n",id);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetByCut_Doa failed\n");
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
	p->SetDataMode(BY_CUT_MODE);

    Log("SetByCut_Doa success, id=%d\n",id);

    return 0;
}

// 接口 - 设置数据按通道优先传入
// ********************************************************************************
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int SetByChannel_Doa(int id)
{
    Log("SetByChannel_Doa begin, id=%d\n",id);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetByChannel_Doa failed\n");
        return 1;
    }

    OmniDoa *p=OmniDoaContainer[id];
	p->SetDataMode(BY_CHANNEL_MODE);

    Log("SetByChannel_Doa success, id=%d\n",id);

    return 0;
}


// 接口 - 设置测向点数
// @param
// id - 对象编号
// startPointNum - 起始测向点
// endPointNum - 结束测向点
// 正确调用后返回0，错误根据不同情况返回不同非零值
int SetUsePointNum_Doa(int id, int startPointNum, int endPointNum)
{
	Log("SetUsePointNum_Doa begin, id=%d, startPointNum=%d, endPointNum=%d\n", id, startPointNum, endPointNum);

	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("SetUsePointNum_Doa failed\n");
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	p->SetUsePointNum(startPointNum, endPointNum);

	Log("SetUsePointNum_Doa success,id=%d\n", id);

	return 0;
}


// 接口 - 设置测向使用点
// @param
// id - 测向对象id
// startCut - 开始抛点数量
// endCut - 结束抛点数量
// @note
// 该功能于SetUsePointNum_Doa接口类似，区别在与本接口输入参数为抛点数量
int SetThrowPointNum_Doa(int id, int startCut, int endCut)
{
    Log("SetThrowPointNum_Doa begin, id=%d, startCut=%d, endCut=%d\n", id, startCut, endCut);

    if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    p->SetUsePoint(startCut, endCut);

    Log("SetThrowPointNum_Doa success, id=%d, startCut=%d, endCut=%d\n", id, startCut, endCut);
    return 0;
}

// 接口 - 设置测向带宽
// ********************************************************************************
// id - 对象编号
// BW - 测向带宽
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int SetBW_Doa(int id,double BW)
{
    Log("SetBW_Doa begin, id=%d, BW=%f\n",id,BW);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetBW_Doa failed\n");
        return 1;
    }

    OmniDoa *p = OmniDoaContainer[id];
    p->SetBW(BW);

    Log("SetBW_Doa success,id=%d\n",id);

    return 0;
}


// 接口 - 设置基于频率域数据计算协方差矩阵
// @param
// id - 对象编号
// 正确调用后返回0，错误根据不同情况返回不同非零值
int SetFrequencyBasedOn_Doa(int id)
{
    Log("SetFrequencyBasedOn_Doa begin,id=%d\n",id);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetFrequencyBasedOn_Doa failed\n");
        return 1;
    }

    OmniDoa *p = OmniDoaContainer[id];
    p->SetFrequencyBased(true);

    Log("SetFrequencyBasedOn_Doa success,id=%d\n",id);

    return 0;
}

// 接口 - 设置不基于频率域数据计算协方差矩阵
// ********************************************************************************
// id - 对象编号
// ********************************************************************************
int SetFrequencyBasedOff_Doa(int id)
{
    Log("SetFrequencyBasedOff_Doa begin,id=%d\n",id);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetFrequencyBasedOff_Doa failed\n");
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    p->SetFrequencyBased(false);

    Log("SetFrequencyBasedOff_Doa success,id=%d\n",id);

    return 0;
}

// 接口 - 计算校正系数
// ********************************************************************************
// id - 对象id
// correction - 校正所需的数据
// length - 校正数据长度
// 正确完成会返回0，同时会设置积分稳定起始时间，错误会返回1
// ********************************************************************************
int SetCorrection_Doa(int id, const double* correction, int length)
{
    Log("SetCorrection_Doa begin, id = %d, length = %d\n",id,length);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetCorrection_Doa failed\n");
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    vector<double> correctionVec(correction,correction+length);

    p->SetCorrection(correctionVec);
    Log("SetCorrection_Doa success,id = %d\n",id);
    return 0;
}

// 接口 - 设置数据
// ********************************************************************************
// id - 对象id
// data - 传入的数据
// length - 传入数据长度
// 正确完成会返回0，错误会返回1
// ********************************************************************************
int SetData_Doa(int id, const double* data,int length)
{
    Log("SetData_Doa begin, id = %d, length = %d\n",id, length);
	
    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetData_Doa failed\n");
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    vector<double> dataVec(data, data + length);
    p->SetData(dataVec);
	
    Log("SetData_Doa success, id = %d\n",id);
    return 0;
}


// 接口 - 释放测向对象
// ********************************************************************************
// id - 对象id
// 正确完成会返回0，错误会返回1
// ********************************************************************************
int Release_Doa(int id)
{
    Log("Release_Doa begin,id=%d\n",id);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("Release_Doa failed\n");
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    delete p;
    OmniDoaContainer[id] = nullptr;

    Log("Release_Doa success,id=%d\n",id);
    return 0;
}

// 接口 - 常规干涉仪算法
// ********************************************************************************
// id - 对象id
// angle - 存储测向角度
// quality - 存储测向质量
// 正确完成会返回0，错误会返回1
// ********************************************************************************
int Interferometer_Doa(int id, double &angle,double &quality)
{
    Log("Interferometer_Doa begin, id = %d\n",id);

    if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("Interferometer_Doa failed\n");
        return 1;
    }

    OmniDoa *p = OmniDoaContainer[id];
    p->Interferometer(angle, quality);

    Log("Interferometer_Doa success, id = %d, angle = %f, quality = %f\n",id, angle, quality);

    return 0;
}


// 计算电平
// @param id 测向对象id
// @param level 用于接收函数返回的电平
// @return 0 成功 1 失败
int GetLevel_Doa(int id, double &level)
{
	Log("GetLevel_Doa begin,id=%d\n", id);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("GetLevel_Doa failed\n");
		return 1;
	}

	OmniDoa *p = OmniDoaContainer[id];
	p->GetLevel(level);

	Log("GetLevel_Doa success, id=%d, level=%f\n", id, level);

	return 0;
}

// 接口 - 信号个数估计
// ********************************************************************************
// id - 对象id
// sigNum - 用于接收函数返回的信号个数
// 正确完成会返回0，错误会返回1
// ********************************************************************************
int EstSigNum_Doa(int id, int &sigNum)
{
	Log("EstSigNum_Doa begin, id=%d\n", id);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("EstSigNum_Doa failed\n");
		return 1;
	}

	OmniDoa *p = OmniDoaContainer[id];
	p->EstSigNum(sigNum);

    if (sigNum == 0) {
        Log("EstSigNum_Doa success BUT zero, set to 1 for running, id=%d, sigNum=%d\n", id, sigNum);
		sigNum = 1;
        return 0;
    }

	Log("EstSigNum_Doa success, id=%d, sigNum=%d\n", id, sigNum);

	return 0;
}


// 接口 - 信号个数估计
// ********************************************************************************
// id - 对象id
// sigNum - 用于接收函数返回的信号个数
// 正确完成会返回0，错误会返回1
// ********************************************************************************
int EstSigNum_AS(int id, int &sigNum)
{
	Log("EstSigNum_AS begin, id=%d\n", id);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		Log("EstSigNum_AS failed\n");
		return 1;
	}

	DireDoa *p = DireDoaContainer[id];
	p->EstSigNum(sigNum);

    if (sigNum == 0) {
        Log("EstSigNum_AS success BUT zero, set to 1 for running, id=%d, sigNum=%d\n", id, sigNum);
		sigNum = 1;
        return 0;
    }

	Log("EstSigNum_AS success, id=%d, sigNum=%d\n", id, sigNum);

	return 0;
}


// 接口 - 定向天线相干信号测向算法
// @param id 测向对象id
// @param sigNum 信号个数
// @param angles 返回计算角度
// @param amplitudes 返回信号幅度
// @return 0 成功 其他 失败
int Coherent_AS(int id, int sigNum, double *angles, double *amplitudes) { 
	Log("Coherent_AS begin, id=%d\n", id);

	if (id<0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		return 1;
	}

	DireDoa *p = DireDoaContainer[id];
	vector<double> anglesVec;
	vector<double> amplitudesVec;

	p->SetSignalNum(sigNum);
	p->DML(sigNum, anglesVec, amplitudesVec);
	for (int i = 0; i<sigNum; ++i) {
		angles[i] = anglesVec[i];
		amplitudes[i] = amplitudesVec[i];
	}

	Log("sigNum=%d\n", sigNum);
	for (int i = 0; i<sigNum; ++i) {
		Log("signal%d: angle=%f, amplitude=%f\n", i + 1, angles[i], amplitudes[i]);
	}
	Log("Coherent_AS success, id=%d\n", id);

	return 0;
}

// 接口 - MuSic算法
// ********************************************************************************
// id - 对象编号
// sigNum - 信号个数
// angles - 返回计算角度
// amplitudes - 返回信号幅度
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int MUSIC_Doa(int id, int sigNum, double *angles, double *amplitudes)
{
	Log("MUSIC_Doa begin, id=%d, sigNum=%d\n", id, sigNum);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	vector<double> anglesVec;
	vector<double> amplitudesVec;
		
	p->SetSignalNum(sigNum);
	p->MUSIC(sigNum, anglesVec, amplitudesVec);
	for (int i = 0; i<sigNum; ++i) {
		angles[i] = anglesVec[i];
		amplitudes[i] = amplitudesVec[i];
	}

	Log("sigNum=%d\n", sigNum);
	for (int i = 0; i<sigNum; ++i) {
		Log("signal%d: angle=%f, amplitude=%f\n", i + 1, angles[i], amplitudes[i]);
	}
	Log("MUSIC_Doa success, id=%d\n", id);

	return 0;
}

// 接口 - DML算法
// ********************************************************************************
// id - 对象编号
// sigNum - 信号个数
// angles - 返回计算角度
// amplitudes - 返回信号幅度
// 正确调用后返回0，错误根据不同情况返回不同非零值
// ********************************************************************************
int DML_Doa(int id, int sigNum, double *angles, double *amplitudes)
{
	Log("DML_Doa begin, id=%d\n", id);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}

	OmniDoa *p = OmniDoaContainer[id];
	vector<double> anglesVec;
	vector<double> amplitudesVec;

	p->SetSignalNum(sigNum);
	p->DML(sigNum, anglesVec, amplitudesVec);
	for (int i = 0; i<sigNum; ++i) {
		angles[i] = anglesVec[i];
		amplitudes[i] = amplitudesVec[i];
	}

	Log("sigNum=%d\n", sigNum);
	for (int i = 0; i<sigNum; ++i) {
		Log("signal%d: angle=%f, amplitude=%f\n", i + 1, angles[i], amplitudes[i]);
	}
	Log("DML_Doa success, id=%d\n", id);

	return 0;
}


// 接口 - IDML算法
// @param
// id - 对象编号
// sigNum - 信号个数
// angles - 返回计算角度
// amplitudes - 返回信号幅度
// coherency - 返回相干组
// resultSigNum - 返回结果信号个数
// @note
// 该接仅在老三九设备中使用
int IDML_Doa(int id, int sigNum, double *angles, double *amplitudes, int *coherency, int &resultSigNum)
{
	Log("IDML_Doa begin, id=%d, sigNum=%d\n", id, sigNum);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}

	OmniDoa *p = OmniDoaContainer[id];
	vector<double> anglesVec;
	vector<double> amplitudesVec;
	vector<int> coherencyVec;
	p->SetSignalNum(sigNum);
	p->IDML(sigNum, anglesVec, amplitudesVec, coherencyVec);
	resultSigNum = anglesVec.size();
	for (int i = 0; i<resultSigNum; ++i) {
		angles[i] = anglesVec[i];
		amplitudes[i] = amplitudesVec[i];
		coherency[i] = coherencyVec[i];
	}

	Log("resultSigNum=%d\n", resultSigNum);
	for (int i = 0; i<resultSigNum; ++i) {
		Log("signal%d: angle=%f, amplitudes=%f, coherency=%d\n", i, angles[i], amplitudes[i], coherency[i]);
	}
	Log("IDML_Doa success, id=%d\n", id);

	return 0;
}

// 接口 - DBF算法
// @param
// id - 测向对象id
// sigNum - 信号个数
// angles - 存储最终测向结果
// quality - 存储测向质量结果
// amplitudes - 存储信号幅度强度
// coeff - 存储解混系数
// coeffNum - 存储解混系数个数
// @note
// 计算成功返回0，失败根据不同情况返回非零数组
int DBF_Doa(int id,int sigNum,double *angles,double *quality,double *amplitudes,complex<double> *coeff,int &coeffNum)
{
    Log("DBF_Doa begin,id=%d, sigNum=%d\n",id,sigNum);

    if(id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
		Log("DBF_Doa failed\n");
        return 1;
    }

    OmniDoa *p = OmniDoaContainer[id];
    vector<double> anglesVec;
    vector<double> qualityVec;
    vector<double> amplitudesVec;
    vector<complex<double>>  coeffVec;

	p->SetSignalNum(sigNum);
    p->DBF(sigNum, anglesVec, qualityVec, amplitudesVec, coeffVec);

	// 对结果进行排序
	vector<int> indexArr(sigNum);
	for (size_t i = 0; i < sigNum; i++){
		int largeThan = 0;
		for (size_t j = 0; j < sigNum; j++){
			if (anglesVec[i] > anglesVec[j]){
				largeThan++;
			}
		}
		indexArr[i] = largeThan;
	}
    for(int i = 0; i < sigNum; i++){
        angles[indexArr[i]] = anglesVec[i];
        quality[indexArr[i]] = qualityVec[i];
        amplitudes[indexArr[i]] = amplitudesVec[i];
    }

	// 拷贝解混系数
    coeffNum=coeffVec.size();
    for(int i=0;i<coeffVec.size();++i){
        coeff[i]=coeffVec[i];
    }

    Log("DBF_Doa success,id=%d, coeffNum=%d, coeffData: \n",id,coeffNum);
    for(int i=0;i<coeffNum;++i){
        Log("%f + 1i * %f,\n",coeff[i].real(),coeff[i].imag());
    }
    Log("angles, quality, amplitude:\n");
    for(int i=0;i<sigNum;++i){
        Log("sigNum=%d, angle=%f, quality=%f, amplitude=%f\n",i+1,angles[i], quality[i], amplitudes[i]);
    }

    return 0;
}


// 接口 - AI测向算法
// ********************************************************************************
// id - 测向对象id
// sigNum - 信号个数
// angles - 存储最终测向结果
// quality - 存储测向质量结果
// amplitudes - 存储信号幅度强度
// 测向质量仅在不开启积分功能（积分时间设置为0）的条件下启用
// ********************************************************************************
int AI_Doa(int id,int sigNum,double *angles,double *quality, double *amplitudes)
{
    Log("AI_Doa begin,id=%d, sigNum=%d\n",id,sigNum);

    if(id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
		Log("AI_Doa failed\n");
        return 1;
    }

    if (sigNum == 0) {
        Log("AI_Doa failed, No Signal.\n");
        return 0;
    }

    OmniDoa* p = OmniDoaContainer[id];
    vector<double> anglesVec(sigNum);
    vector<double> qualityVec(sigNum);
    vector<double> amplitudesVec(sigNum);

	p->SetSignalNum(sigNum);

    if(sigNum==1){
		// p->MUSIC(1, anglesVec, amplitudesVec);
		p->Interferometer(anglesVec[0], qualityVec[0]);
        angles[0] = anglesVec[0];
		amplitudes[0] = amplitudesVec[0];
		quality[0] = 1.0;
        // amplitudes[0]=1;
    }else{
		// 若通道数比信号个数少，或快速模式,或双通道设备，调用联合对角化
		// if (sigNum > p->GetChannelNum() || p->GetFastAIMode() || p->GetChannelNum() == 2){
		if (p->GetFastAIMode() || p->GetChannelNum() == 2){
        	p->AI(sigNum,anglesVec,qualityVec,amplitudesVec);
			Log("SigNum > ChannelNum Or FastAIMode, use AI.\n");
		}
		else{
			bool flag = p->GetAIFlag();
			Log("AIFlag=%d\n", flag);
			if (!flag){
				p->SetAlgorithmTag(sigNum);
				Log("SetAlgorithmTag=%d\n", p->GetAlgorithmTag());
			}
			int algorithmTag = p->GetAlgorithmTag();
			Log("algorithmTag=%d\n", algorithmTag);
			if (algorithmTag == 0){
				Log("AI Mode\n");
				p->AI(sigNum, anglesVec, qualityVec, amplitudesVec);
			}
			else if (algorithmTag == 1)	{
				Log("CMA Mode\n");
				p->CMA(sigNum, anglesVec, qualityVec, amplitudesVec);
			}
			else{
				Log("AI_Doa failed, unknown algorithm tag=%d\n", algorithmTag);
				return 1;
			}
		}
		vector<int> indexArr(sigNum);
		for (size_t i = 0; i < sigNum; i++){
			int largeThan = 0;
			for (size_t j = 0; j < sigNum; j++){
				if (anglesVec[i] > anglesVec[j]){
					largeThan++;
				}
			}
			indexArr[i] = largeThan;
		}

        for(int i = 0; i < sigNum; i++){
            angles[indexArr[i]] = anglesVec[i];
            quality[indexArr[i]] = qualityVec[i];
            amplitudes[indexArr[i]] = amplitudesVec[i];
        }
    }
        
    Log("AI_Doa success,id=%d, result:\n",id);
    for(int i=0;i<sigNum;++i){
        Log("signal%d : angle=%f, quality=%f, amplitude=%f\n",i,angles[i], quality[i], amplitudes[i]);
    }
	
    return 0;
}


// 接口 - FFT测向算法
// @param
// id - 测向对象id
// sigNum - 信号个数
// angles - 存储最终测向结果
// quality - 存储测向质量结果
// amplitudes - 存储信号幅度强度
// @note
// 该接口输入的数据为时域IQ数据，需自行转换为频域IQ数据后使用FFT测向，暂时没看到应用场景，使用Doa类，不再更新
int FFT_Doa(int id, int sigNum, double *angles, double *quality, double *amplitudes)
{
	// Log("FFT_Doa begin,id=%d, sigNum=%d\n", id, sigNum);

	// if (id<0 || id >= DoaContainer.size() || !DoaContainer[id]) {
	// 	Log("FFT_Doa failed\n");
	// 	return 1;
	// }

	// Doa *p = DoaContainer[id];
	// vector<double> anglesVec;
	// vector<double> qualityVec;
	// vector<double> amplitudesVec;
	// p->FFT(sigNum, anglesVec, qualityVec, amplitudesVec);
	// for (int i = 0; i<sigNum; ++i) {
	// 	angles[i] = anglesVec[i];
	// 	quality[i] = qualityVec[i];
	// 	amplitudes[i] = amplitudesVec[i];
	// }

	// Log("FFT_Doa success,id=%d, result:\n", id);
	// for (int i = 0; i<sigNum - 1; ++i) {
	// 	Log("signal%d : angle=%f, quality=%f, amplitude=%f\n", i, angles[i], quality[i], amplitudes[i]);
	// }

	return 0;
}



// 接口 - 设置FFT测向模式
// @param
// id - 测向对象id
// FFTMode - FFT测向模式，0为常规模式，1为快速模式，2为赖老师提供自动模式，3为自己写的信号归集模式
int SetFFTMode_Doa(int id, int FFTMode)
{
	Log("SetFFTMode_Doa begin, id=%d, FFTMode=%d\n", id, FFTMode);

	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("SetFFTMode_Doa failed\n");
		return 1;
	}

	OmniDoa *p = OmniDoaContainer[id];
	p->SetFFTMode(FFTMode);

	Log("SetFFTMode_Doa success,id=%d\n", id);

	return 0;
}


// 接口 - 设置FFT测向门限
// @param
// id - 测向对象id
// FFTThreshold - FFT测向门限值
int SetFFTTreshold_Doa(int id, double FFTThreshold)
{
	Log("SetFFTTreshold_Doa begin, id=%d, FFTThreshold=%f\n", id, FFTThreshold);

	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("SetFFTTreshold_Doa failed\n");
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	p->SetFFTThreshold(FFTThreshold);

	Log("SetFFTTreshold_Doa success,id=%d\n", id);

	return 0;
}


// 接口 - 设置FFT测向门限调整值
// @param
// id - 测向对象id
// FFTThresholdAdjust - FFT测向门限调整值
int SetFFTTresholdAdjust_Doa(int id, double FFTThresholdAdjust)
{
	Log("SetFFTTresholdAdjust_Doa begin,id=%d, FFTThresholdAdjust=%f\n", id, FFTThresholdAdjust);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("SetFFTTresholdAdjust_Doa failed\n");
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	p->SetFFTThresholdAdjust(FFTThresholdAdjust);

	Log("SetFFTTresholdAdjust_Doa success,id=%d\n", id);

	return 0;
}

// 接口 - 设置FFT测向校正系数
// @param
// id - 测向对象id
// correction - 校正系数数组
// length - 校正系数数组长度
// @note
// 该接口用于设置FFT测向的校正系数，通常在FFT测向前需要先计算校正系数
// 正确完成会返回0，错误会返回1, length需注意实部虚部 * 2
int SetCorrectionFFT_Doa(int id, const double *correction, int length)
{
	Log("SetCorrectionFFT_Doa begin,id=%d, length=%d\n", id, length);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("SetCorrectionFFT_Doa failed\n");
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	vector<double> correctionVec(correction, correction + length);
	p->SetCorrectionFFT(correctionVec);

	Log("SetCorrectionFFT_Doa success,id=%d\n", id);
	
	return 0;
}

// 设置FFT测向数据
// @param
// id - 测向对象id
// data - 传入的FFT数据
// length - 传入数据长度
// @note
// length需注意实部虚部 * 2
int SetDataFFT_Doa(int id, const double *data, int length)
{
	Log("SetDataFFT_Doa begin,id=%d, length=%d\n", id, length);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("SetDataFFT_Doa failed\n");
		return 1;
	}

	OmniDoa *p = OmniDoaContainer[id];
	vector<double> dataVec(data, data + length);
	auto beginTime = std::chrono::high_resolution_clock::now();
	p->SetDataFFT(dataVec);
	auto endTime = std::chrono::high_resolution_clock::now();
	std::chrono::duration<double> dur = endTime - beginTime;
	Log("SetData Cost %.4f s\n", dur.count());

	Log("SetDataFFT_Doa success,id=%d\n", id);
	return 0;
}

// 接口 - 干涉仪FFT测向
// @param
// id - 测向对象id
// angleNum - 返回测向角度个数
// angles - 存储测向角度
// qualitys - 存储测向质量
// frequencies - 存储测向频率
int InterferometerFFT_Doa(int id, int &angleNum, double *angles, double *qualitys, int *frequencies)
{
	Log("InterferometerFFT_Doa begin,id=%d\n", id);

	if (id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		Log("InterferometerFFT_Doa failed\n");
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	vector<double> anglesVec;
	vector<double> qualitysVec;
	vector<int> frequenciesVec;

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	// 标记不考虑俯仰角
	p->SetSignalNum(-1);
	p->InterferometerFFT(anglesVec, qualitysVec, frequenciesVec);
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	std::chrono::duration<double> time_span = std::chrono::duration_cast<std::chrono::duration<double>>(end - begin);
	//Log("InterferometerFFT_Doa time cost: %f seconds\n", time_span.count());
	// std::cout << "InterferometerFFT_Doa time cost: " << time_span.count() << " seconds" << std::endl;
	angleNum = anglesVec.size();

	if (!anglesVec.empty() && anglesVec[0] == -999){
		angleNum = -999;
	}

	for (int i = 0; i < angleNum; ++i) {
		angles[i] = anglesVec[i];
		qualitys[i] = qualitysVec[i];
		frequencies[i] = frequenciesVec[i];
	}

	Log("angleNum=%d\n", angleNum);
	for (int i = 0; i<angleNum; ++i) {
		Log("signal%d: angle=%f, quality=%f, frequency=%d\n", i, angles[i], qualitys[i], frequencies[i]);
	}

	Log("InterferometerFFT_Doa success,id=%d\n", id);
	return 0;
}



// 创建微波测向对象函数
/*****************************
* id：对象id号
* antennaNum：阵元数量
* pointNum：采样点数
* f：中心频率
* 创建成果返回0，创建失败返回1
*****************************/
int Create_MicroDoa(int &id,int antennaNum,int pointNum,double f)
{
    LogCreate();
    Log("Create_MicroDoa begin,antennaNum=%d, pointNum=%d, f=%f\n",antennaNum,pointNum,f);

    MicroDoa *p=new (nothrow) MicroDoa(antennaNum,pointNum,f);
    if(!p){
		Log("Create_MicroDoa failed\n");
        return 1;
    }

    for(id=0;id<MicroDoaContainer.size();++id){
        if(!MicroDoaContainer[id]){
            MicroDoaContainer[id]=p;
            break;
        }
    }
    if(id==MicroDoaContainer.size()){
        MicroDoaContainer.push_back(p);
    }

    Log("Create_MicroDoa success, id=%d\n",id);
    return 0;
}

/*****************************
*�������ܣ���ʼ��΢���ȷ������㷨����
*���룺idΪ�����㷨����ID��typeΪ΢���ȷ�����ƴ����ʽ������0Ϊȫ���߱ȷ���1Ϊ1-2ƴ��
*      2Ϊ2-3ƴ��3Ϊ1-3ƴ��4Ϊ1-4ƴ,-1Ϊģ��Աȷ�����-2Ϊģ��Ա���ȫ���߱ȷ����
	   ������modelFileΪ����ģ���ļ����·����
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int Init_MicroDoa(int id,int type,const char *modelFile)
{
    Log("Init_MicroDoa begin,id=%d,type=%d, modelFile=%s\n",id,type,modelFile);

    if(id<0 || id>=MicroDoaContainer.size() || !MicroDoaContainer[id]){
		Log("Init_MicroDoa failed\n");
        return 1;
    }

    MicroDoa *p=MicroDoaContainer[id];
    p->Init(type,modelFile);

    Log("Init_MicroDoa success, id=%d\n",id);

    return 0;
}

/*****************************
*�������ܣ��˺�����������ʹ�õĵ�������
*���룺idΪ�����㷨����ID��startPointNumΪ��ʼ������endPointNumΪ����������
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*˵���������������ã�ϵͳĬ���������ĵ���ΪpointNum�㡣
*      ����Ҫ������ͷ�ͽ�β�����ɵ㣬�����������������
*****************************/
int SetUsePointNum_MicroDoa(int id, int startPointNum, int endPointNum)
{
	Log("SetUsePointNum_MicroDoa begin,id=%d,startPointNum=%d, emdPointNum=%d\n", id, startPointNum, endPointNum);

	if (id<0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		Log("SetUsePointNum_MicroDoa failed\n");
		return 1;
	}

	MicroDoa *p = MicroDoaContainer[id];
	p->SetUsePointNum(startPointNum, endPointNum);

	Log("SetUsePointNum_MicroDoa success, id=%d\n", id);

	return 0;
}

/*****************************
*�������ܣ�������㷨���󴫵ݱȷ��������ݡ�
*���룺idΪ�����㷨����ID��dataΪָ��ȷ��������������ָ�룻
*      lengthΪ�ȷ�������������ĳ��ȡ�
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int SetData_MicroDoa(int id,const double *data,int length)
{
    Log("SetData_MicroDoa begin, id=%d, dataLength=%d\n",id,length);


    if(id<0 || id>=MicroDoaContainer.size() || !MicroDoaContainer[id]){
		Log("SetData_MicroDoa failed\n");
        return 1;
    }

    MicroDoa *p=MicroDoaContainer[id];
    vector<double> dataVec(data,data+length);
    p->SetData(dataVec);

    Log("SetData_MicroDoa success, id=%d\n",id);
    return 0;
}

/*****************************
*�������ܣ���ȡ΢���ȷ�����ʾ��ȡ�
*���룺idΪ�����㷨����ID��angleΪ�������ͣ�����ִ�гɹ���д���ź�ʾ��ȡ�
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int GetAngle_MicroDoa(int id,double &angle)
{
    Log("GetAngle_MicroDoa begin,id=%d\n",id);

    if(id<0 || id>=MicroDoaContainer.size() || !MicroDoaContainer[id]){
		Log("GetAngle_MicroDoa failed\n");
        return 1;
    }

    MicroDoa *p=MicroDoaContainer[id];
    p->GetAngle(angle);

    Log("GetAngle_MicroDoa success, id=%d, angle=%f\n",id,angle);
    return 0;
}

int SetFFTThreshold_MicroDoa(int id, double FFTThreshold)
{
	Log("SetFFTThreshold_MicroDoa begin,id=%d, FFTThreshold=%f\n", id,FFTThreshold);

	if (id<0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		Log("SetFFTThreshold_MicroDoa failed\n");
		return 1;
	}

	MicroDoa *p = MicroDoaContainer[id];
	p->SetFFTThreshold(FFTThreshold);

	Log("SetFFTThreshold_MicroDoa success, id=%d\n", id);
	return 0;
}

int SetDataFFT_MicroDoa(int id, const double *data, int length)
{
	Log("SetDataFFTT_MicroDoa begin,id=%d, dataLength=%d\n", id, length);

	if (id<0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		Log("SetDataFFTT_MicroDoa failed\n");
		return 1;
	}

	MicroDoa *p = MicroDoaContainer[id];
	vector<double> dataVec(data, data + length);
	p->SetDataFFT(dataVec);

	Log("SetDataFFTT_MicroDoa success, id=%d\n", id);
	return 0;
}

int GetAngleFTT_MicroDoa(int id, int &angleNum, double *angles)
{
	Log("GetAngleFTT_MicroDoa begin,id=%d\n", id);

	if (id<0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		Log("GetAngleFTT_MicroDoa failed\n");
		return 1;
	}

	MicroDoa *p = MicroDoaContainer[id];
	vector<double> anglesVec;
	p->GetAngleFFT(anglesVec);
	angleNum = anglesVec.size();
	for (int i = 0; i < angleNum; ++i) {
		angles[i] = anglesVec[i];
	}

	Log("angleNum=%d\n", angleNum);
	for (int i = 0; i<angleNum; ++i) {
		Log("signal%d: angle=%f\n", i+1, angles[i]);
	}
	Log("GetAngleFTT_MicroDoa success, id=%d\n", id);

	return 0;
}

/*****************************
*�������ܣ��˺�����������΢���ȷ������㷨����
*���룺idΪ�����㷨����ID��
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int Release_MicroDoa(int id)
{
    Log("Release_MicroDoa begin,id=%d\n",id);

    if(id<0 || id>=MicroDoaContainer.size() || !MicroDoaContainer[id]){
        return 1;
    }

    MicroDoa *p=MicroDoaContainer[id];
    delete p;
    MicroDoaContainer[id]=nullptr;

    Log("Release_MicroDoa success,id=%d\n",id);
    return 0;
}

// 接口 - 设置积分、稳定器衰减系数
// @param
// id - 测向对象id
// time - 积分时间系数
int SetTimeFactor_Doa(int id,double time,double BW)
{
    Log("SetTimeFactor_Doa begin, id=%d, time=%f, BW=%f\n",id,time,BW);

    if(id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        return 1;
    }

    OmniDoa* p=OmniDoaContainer[id];
    p->SetTimeFactor(time,BW);

    Log("SetTimeFactor_Doa success,id=%d\n",id);
    return 0;
}

// 接口 - 更新累加器概率密度函数
// @param
// id - 测向对象id
// newData - 新的测向角度
// result - 累加器记录的最优测向角度
int SetUpdateDensity_Doa(int id,double newData,double &result)
{
    Log("SetUpdateDensity_Doa begin, id=%d,newDada=%f\n", id, newData);

    if(id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    p->SetUpdataDensity(newData,result);

    Log("SetUpdateDensity_Doa success, id=%d, result=%f\n",id,result);
    return 0;
}

// 多角度累测向累加器更新
// @param
// id - 测向对象id
// newDatas - 新的测向角度数组
// results - 累加器记录的最优测向角度数组
// dataNum - 新的测向角度数组长度
int SetUpdateDensityMult_Doa(int id,const double *newDatas,double *results,int dataNum)
{
    Log("SetUpdateDensityMult_Doa begin, id=%d, dataNum=%d, newDatas: [",id,dataNum);
    for(int i=0;i<dataNum;++i){
        Log("%f, ",newDatas[i]);
    }
    Log("]\n");
    

    if(id<0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        return 1;
    }

    OmniDoa* p=OmniDoaContainer[id];
    vector<double> newDatasVec(newDatas,newDatas+dataNum);
    vector<double> resultsVec;
    p->SetUpdataDensity(newDatasVec,resultsVec);
    for(int i=0;i<resultsVec.size();++i){
        results[i]=resultsVec[i];
    }

    Log("results: [");
    for(int i=0;i<dataNum;++i){
        Log("%f, ",results[i]);
    }
    Log("]\n");
    Log("SetUpdateDensityMult_Doa success, id=%d\n",id);
    return 0;
}

// 接口 - 清除累加器概率密度函数
// @param
// id - 测向对象id
int SetClearDensity_Doa(int id)
{
    Log("SetClearDensity_Doa begin,id=%d\n",id);

    if(id<0 || id>=OmniDoaContainer.size() || !OmniDoaContainer[id]){
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    p->SetClearDensity();

    Log("SetClearDensity_Doa success,id=%d\n",id);
    return 0;
}

// 接口 - FFT测向信号归集
// ********************************************************************************
// id - 对象id
// 正确完成会返回0，错误会返回1
// ********************************************************************************
int SignalCombine_Doa(int id, int& angleNum, double* angles, double* qualitys, int* frequencies)
{
    Log("SignalCombine_Doa begin, id=%d, angleNum=%d\n", id, angleNum);

    float angleRes = 5;
    int freqRes = 3;

    if (angleNum == 0 || angleNum == 1)
        return 0;
    int newAngleNum = 1;
    double curAngle = angles[0];
    int curFrequency = frequencies[0];
    for (int i = 1; i < angleNum; i++) 
    {
        if (abs(angles[i] - curAngle) <= angleRes && abs(frequencies[i] - curFrequency) <= freqRes)
        {
            continue;
        }
        else
        {
            angles[newAngleNum] = angles[i];
            frequencies[newAngleNum] = frequencies[i];
            qualitys[newAngleNum] = qualitys[i];

            newAngleNum++;
            curAngle = angles[i];
            curFrequency = frequencies[i];
        }
    }
    angleNum = newAngleNum;

    Log("SignalCombine_Doa success, id=%d, angleNum=%d\n", id, angleNum);
	return 0;
}


// 接口 - 更新FFT测向累加器
// @param
// id - 测向对象id
// angleNum - 测向角度个数
// angles - 测向角度数组
// frequencies - 测向角度对应的频率索引
int SetUpdateDensitiesFFT_Doa(int id, int angleNum, double* angles, int* frequencies)
{
    Log("SetUpdateDensityFFT_Doa begin, id=%d\n", id);

    if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];

    p->SetUpdateDensitiesFFT(angleNum, angles, frequencies);


    Log("SetUpdateDensityFFT_Doa success, id=%d\n", id);
    return 0;
}

// 接口 - 设置FFT测向是否累加器
// @param
// id - 测向对象id
// mag - 累加器角度放大倍数
// @note
// 调用该接口则开启FFT测向累加器功能，mag为累加器角度放大倍数
int SetUseDensitiesFFT_Doa(int id, int mag)
{
    Log("SetUseDensitiesFFT_Doa begin, id=%d\n", id);

    if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
        return 1;
    }
    OmniDoa* p = OmniDoaContainer[id];

    p->InitFFTDensities(mag);

    Log("SetUseDensitiesFFT_Doa success, id=%d\n", id);
    return 0;
}

// 接口 - 清除FFT测向累加器
// @param
// id - 测向对象id
int setClearDensitiesFFT_Doa(int id)
{
    Log("setClearDensitiesFFT_Doa begin,id=%d\n", id);

    if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
        return 1;
    }

    OmniDoa* p = OmniDoaContainer[id];
    p->SetClearDensitiesFFT();

    Log("setClearDensitiesFFT_Doa success,id=%d\n", id);
    return 0;
}


// ��ɭ������������
int Create_WW(int& id, int channelNum, int antennaNum, int pointNum)
{
	LogCreate();
	Log("Create_WW begin,channelNum=%d, antennaNum=%d, pointNum=%d\n", channelNum, antennaNum, pointNum);

	WW* p = new (nothrow) WW(channelNum, antennaNum, pointNum);
	if (!p) {
		Log("Create_WW failed\n");
		return 1;
	}

	for (id = 0; id < WWContainer.size(); ++id) {
		if (!WWContainer[id]) {
			WWContainer[id] = p;
			break;
		}
	}
	if (id == WWContainer.size()) {
		WWContainer.push_back(p);
	}

	Log("Create_WW success, id=%d\n", id);
	return 0;
}

int Init_WW(int id, double f, double r)
{
	Log("Init_WW begin, id=%d, f=%f, r=%f\n", id, f, r);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("Init_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	p->Init(f, r);

	Log("Init_WW success,id=%d\n", id);

	return 0;
}

int SetCorrection_WW(int id, const double* correction, int length)
{
	Log("SetCorrection_WW begin, id=%d, length=%d\n", id, length);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("SetCorrection_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	vector<double> correctionVec(correction, correction + length);
	p->SetCorrection(correctionVec);

	Log("SetCorrection_WW success,id=%d\n", id);

	return 0;
}

int SetData_WW(int id, const double* data, int length)
{
	Log("SetData_WW begin, id=%d, length=%d\n", id, length);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("SetData_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	vector<double> dataVec(data, data + length);
	p->SetData(dataVec);

	Log("SetData_WW success,id=%d\n", id);

	return 0;
}

int GetAngle_WW(int id, double& angle)
{
	Log("GetAngle_WW begin, id=%d\n", id);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("GetAngle_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	p->GetAngle(angle);

	Log("GetAngle_WW success,id=%d, angle=%f\n", id, angle);

	return 0;
}

int SetThresholdFFT_WW(int id, double threshold)
{
	Log("SetThresholdFFT_WW begin, id=%d, threshold=%f\n", id, threshold);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("SetThresholdFFT_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	p->SetThreshold(threshold);

	Log("SetThresholdFFT_WW success,id=%d\n", id);

	return 0;
}

int SetCorrectionFFT_WW(int id, const double* correction, int length)
{
	Log("SetCorrectionFFT_WW begin, id=%d, length=%d\n", id, length);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("SetCorrectionFFT_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	vector<double> correctionVec(correction, correction + length);
	p->SetCorrectionFFT(correctionVec);

	Log("SetCorrectionFFT_WW success,id=%d\n", id);

	return 0;
}

int SetDataFFT_WW(int id, const double* data, int length)
{
	Log("SetDataFFT_WW begin, id=%d, length=%d\n", id, length);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("SetDataFFT_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	vector<double> dataVec(data, data + length);
	p->SetDataFFT(dataVec);

	Log("SetDataFFT_WW success,id=%d\n", id);

	return 0;
}

int GetAngleFFT_WW(int id, int& angleNum, double* angles, int* frequencies)
{
	Log("GetAngleFFT_WW begin, id=%d\n", id);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("GetAngleFFT_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	vector<double> anglesVec;
	vector<int> frequenciesVec;
	p->GetAngleFFT(anglesVec, frequenciesVec);
	angleNum = anglesVec.size();
	for (int i = 0; i < angleNum; ++i) {
		angles[i] = anglesVec[i];
		frequencies[i] = frequenciesVec[i];
	}

	Log("GetAngleFFT_WW success,id=%d, angleNum=%d, angles and frequencies:\n", id, angleNum);
	for (int i = 0; i < angleNum; ++i) {
		Log("angle%d : %f, %d\n", i + 1, angles[i], frequencies[i]);
	}

	return 0;
}

int Release_WW(int id)
{
	Log("Release_WW begin,id=%d\n", id);

	if (id < 0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		Log("Release_WW failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	delete p;
	WWContainer[id] = nullptr;

	Log("Release_WW success,id=%d\n", id);

	return 0;
}

int SetByCut_WW(int id)
{
	Log("SetByCut_Doa begin, id=%d\n", id);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("SetByCut_Doa failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	p->SetDataMode(BY_CUT_MODE);

	Log("SetByCut_Doa success, id=%d\n", id);

	return 0;
}

/*****************************
*�������ܣ��˺��������������ݰ�ͨ���档��
*���룺idΪ�����㷨����ID��
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int SetByChannel_WW(int id)
{
	Log("SetByChannel_Doa begin, id=%d\n", id);

	if (id < 0 || id >= WWContainer.size() || !WWContainer[id]) {
		Log("SetByChannel_Doa failed\n");
		return 1;
	}

	WW* p = WWContainer[id];
	p->SetDataMode(BY_CHANNEL_MODE);

	Log("SetByChannel_Doa success, id=%d\n", id);

	return 0;
}

// 接口 - 相干测向
// @param
// id - 测向对象id
// sigNum - 信号数量
// angles - 存储测向角度数组
int Coherent_Doa(int id, int sigNum, double* angles) {
	Log("Coherent_Doa begin, id=%d\n", id);

	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}

	OmniDoa *p = OmniDoaContainer[id];
	vector<double> anglesVecDML, anglesVecNAUU, amplitudesVec;
	p->SetSignalNum(sigNum);

	// p->Coherent(sigNum, anglesVecNAUU);
	p->DML(sigNum, anglesVecDML, amplitudesVec);

	for (int i = 0; i < sigNum; ++i) {
		angles[i] = anglesVecDML[i];
	}

	Log("sigNum=%d\n", sigNum);
	for (int i = 0; i < sigNum; ++i) {
		Log("signal%d: angle=%f, amplitude=%f\n", i + 1, angles[i], amplitudesVec[i]);
	}

	//Log("DML Result\n");
	//for (int i = 0; i < sigNum; ++i) {
	//	Log("signal%d: angle=%f\n", i + 1, anglesVecDML[i]);
	//}
	//Log("NUAA Result\n");
	//for (int i = 0; i < sigNum; ++i) {
	//	Log("signal%d: angle=%f\n", i + 1, anglesVecNAUU[i]);

	Log("Coherent_Doa success, id=%d\n", id);

	return 0;
}

// 接口 - 设置启用积分累积功能
// @param
// id - 测向对象id
// use - 是否启用积分累积功能
// @note
// 调用该接口则开启积分累积功能，use为true表示启用，false表示不启用
// 正确完成会返回0，错误会返回1
int SetUseAccumulation_Doa(int id, bool use) {
	Log("Set UseAccumulation begin, id=%d\n", id);

	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	p->SetUseAccumulation(use);

	Log("Set UseAccumulation success, id=%d\n", id);

	return 0;
}


// 接口 - 获取测向对象的伪谱数组
// @param
// id - 测向对象id
// specArr - 存储伪谱数组
int GetSpecArr_Doa(int id, double* specArr){
	Log("Get SpecArr begin, id=%d\n", id);

	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	vector<double> specVec;
	p->GetSpectrum(specVec);

	for (size_t i = 0; i < specVec.size(); i++) {
		specArr[i] = specVec[i];
		// cout << specArr[i] << endl;
	}

	Log("Get SpecArr success, id=%d\n", id);

	return 0;
}


// �ȷ��ȶ��������ӿ�
// �ȶ�������
/*****************************
*�������ܣ������ȶ���������
*���룺idΪ�����㷨����ID��timeΪ�ۼ�ʱ�䳣������λΪs����BWΪ�����������λΪHz����
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int SetTimeFactor_MicroDoa(int id, double time, double BW)
{
	Log("SetTimeFactor_MicroDoa begin, id=%d, time=%f, BW=%f\n", id, time, BW);

	if (id < 0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		return 1;
	}

	MicroDoa* p = MicroDoaContainer[id];
	p->SetTimeFactor(time, BW);

	Log("SetTimeFactor_MicroDoa success,id=%d\n", id);
	return 0;
}

/*****************************
*�������ܣ����õ�ʾ���ͳ�ƣ�
*���룺idΪ�����㷨����ID��newDataΪ��������ĽǶȣ�resultΪ�������ͣ�
*      ����ִ�гɹ���д���ȶ���ĽǶȡ�
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int SetUpdateDensity_MicroDoa(int id, double newData, double& result)
{
	Log("SetUpdateDensity_MicroDoa begin, id=%d,newDada=%f\n", id, newData);

	if (id < 0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		return 1;
	}

	MicroDoa* p = MicroDoaContainer[id];
	p->SetUpdataDensity(newData, result);

	Log("SetUpdateDensity_MicroDoa success, id=%d, result=%f\n", id, result);
	return 0;
}

/*****************************
*�������ܣ����ϵͳ�ĸ����ܶȼ��䣻
*���룺idΪ�����㷨����ID��
*���������ִ�гɹ�ʱ����0���������ʧ��������ط�0ֵ��
*****************************/
int SetClearDensity_MicroDoa(int id)
{
	Log("SetClearDensity_MicroDoa begin,id=%d\n", id);

	if (id < 0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		return 1;
	}

	MicroDoa* p = MicroDoaContainer[id];
	p->SetClearDensity();

	Log("SetClearDensity_MicroDoa success,id=%d\n", id);
	return 0;
}


// 接口 - 设置测向线程数
// @param
// id - 测向对象id
// threadNum - 线程数
// @note
// 测试多线程可能导致设备内存占满，弃用
int SetThreadNum_Doa(int id, int threadNum) {
	Log("SetThreadNum_Doa begin,id=%d, threadNum=%d\n", id, threadNum);
	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}
	OmniDoa* p = OmniDoaContainer[id];
	p->SetThreadNum(threadNum);
	Log("SetThreadNum_Doa success,id=%d\n", id);
	return 0;
}



int SetGetAmpMode_MicroDoa(int id, int type)
{
	Log("SetGetAmpMode_MicroDoa begin,id=%d\n", id);

	if (id < 0 || id >= MicroDoaContainer.size() || !MicroDoaContainer[id]) {
		return 1;
	}

	MicroDoa* p = MicroDoaContainer[id];

	p->setGetAmpModel(type);

	Log("SetGetAmpMode_MicroDoa success,id=%d\n", id);
	return 0;
}

// 接口 - 设置积分、稳定时间
// @param
// id - 对象id
// at - 积分、稳定时间，单位为秒
// @note
// 积分时间小于等于0则不开启积分功能，正确完成会返回0，错误会返回1
int SetAverageTime_Doa(int id, double at){
	Log("SetAverageTime_Doa begin, id = %d, AverageTime = %fs\n", id, at);

	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];

	p->SetAverageTime(at);

	Log("SetAverageTime_Doa success, id = %d, AverageTime = %fs\n", id, at > 0 ? at : 0);
	return 0;
}

int SetFFTAverageNum_Doa(int id, int & at){
	Log("SetFFTAverageNum_Doa begin, id = %d, AverageNum = %d\n", id, at);
	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]) {
		return 1;
	}
	OmniDoa* p = OmniDoaContainer[id];
	p->SetFFTAverageNum(at);
	Log("SetFFTAverageNum_Doa success, id = %d, AverageNum = %d\n", id, at);
	return 0;
}


// 创建水平天线测向对象
// id : 编号
// channelNum : 通道个数
// antNUm : 天线个数
// pointNum : 采样点数
// 创建成果返回0, 其余情况返回非零值
int Create_AS(int &id, int channelNum, int antNum, int pointNum){
	try{
		LogCreate();
		Log("Create_AS begin, id=%d, channelNum=%d, antNum=%d, pointNum=%d\n", id, channelNum, antNum, pointNum);

		DireDoa* p = new(nothrow) DireDoa(channelNum, antNum, pointNum);
		if (!p){
			Log("Create_AS Failed.\n");
			return 1;
		}

		for (id = 0; id < DireDoaContainer.size(); id++){
			if (!DireDoaContainer[id]){
				DireDoaContainer[id] = p;
				break;
			}
		}

		if (id == DireDoaContainer.size()){
			DireDoaContainer.push_back(p);
		}

		Log("Create_AS success, id = %d, channelNum = %d, antNum = %d, pointNum = %d\n", id, channelNum, antNum, pointNum);
		return 0;
	}
	catch(const std::exception& e){
		std::cerr << e.what() << '\n';
		return -1;
	}
}


// 初始化水平天线测向对象
// id : 编号 
// type : 天线类型 - 0:垂直-频谱 1:水平-频谱 2:垂直-研发 3:水平-研发
// f : 频率（Hz）
// r : 孔径（半径, m）
int Init_AS(int id, int type, double f, double r){
	Log("Init_AS begin, id=%d, f=%f, r=%f, type=%d\n",id, f, r, type);

    if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("Init_AS failed\n");
        return 1;
    }

    DireDoa* p = DireDoaContainer[id];
    p -> Init(type, f, r);

    Log("Init_AS success,id=%d\n",id);

    return 0;
}

// 设置切刀方式
// id : 编号 
// cutNum : 切刀数
// thw : 切刀方式
int SetThw_AS(int id, int cutNum, const int* thw, int length){
	Log("SetThw_AS begin, id=%d, cutNum=%d, length=%d, thw=[\n",id, cutNum, length);
	int channelNum = length / cutNum;
    for(int i = 0; i < length - 1; ++i){
        Log("%d, ",thw[i]);
		if (i % channelNum == 2 && i != 0){
			Log("\n");
		}
    }
    Log("%d\n]\n",thw[length-1]);


    if(id <0  || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("SetThw_AS failed\n");
        return 1;
    }

    DireDoa* p = DireDoaContainer[id];
    vector<int> thwVec(thw, thw + length);
    p->SetThw(cutNum, thwVec);

    Log("SetThw_AS success,id=%d\n",id);
	return 0;
}

// 设置抛点数量
// id : 编号
// startCutNum : 每刀数据前端抛点数量
// endCutNum : 每刀数据后端抛点数量
int SetThrowPointNum_AS(int id, int startCutNum, int endCutNum){
    Log("SetThrowPointNum_AS begin, id=%d, startCutNum=%d, endCutNum=%d\n", id, startCutNum, endCutNum);
    
	if(id <0  || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("SetUsedPointNum_AS failed\n");
        return 1;
    }

	DireDoa* p = DireDoaContainer[id];
	p->SetUsePoint(startCutNum, endCutNum);
	// p->SetUsedPointNum();
    
	Log("SetThrowPointNum_AS success,id=%d\n",id);
	return 0;
}

// 导出模型数据
// id : 编号
// fileDir : 模型所在文件夹
int SetModel_AS(int id, const char* fileDir){
	Log("SetModel_AS begin, id=%d\n", id);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("SetModel_AS failed\n");
        return 1;
    }

	DireDoa* p = DireDoaContainer[id];
	int ret = p->SetModel(fileDir);
	if (ret != 0){
		return ret;
	}
    
	Log("SetModel_AS success,id=%d\n",id);
	return 0;
}

// 计算校正系数
// @param id : 编号
// @param correctionData : 校正数据
// @param length : 数据长度
int SetCorrection_AS(int id, const double * correctionData, int length){
    Log("SetCorrection_AS begin, id=%d, length=%d\n", id, length);

    if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("SetCorrection_AS failed\n");
        return 1;
    }
    DireDoa* p = DireDoaContainer[id];
    vector<double> correctionVec(correctionData, correctionData + length);
    p->SetCorrection(correctionVec); 

    Log("SetCorrection_AS success,id = %d\n",id);
    return 0;
}

// 校正数据
// id : 编号
// data : 测向数据
// length : 数据长度
int SetData_AS(int id, const double * data, int length){
    Log("SetData_AS begin, id = %d, length = %d\n",id, length);
	
    if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("SetData_AS failed\n");
        return 1;
    }

    DireDoa* p = DireDoaContainer[id];
    vector<double> dataVec(data, data + length);
    p->SetData(dataVec);
    Log("SetData_AS success,id=%d\n",id);
    return 0;
}

// 幅相法测向算法
// id : 编号
// angle : 测向角度
// quality : 测向质量
int AmplitutdePhaseAlg_AS(int id, double &angle, double &quality){
	Log("AmplitutdePhaseAlg_AS begin, id=%d\n", id);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
		Log("AmplitutdePhaseAlg_AS failed, id=%d\n", id);
		return -1;
	}

	DireDoa* p = DireDoaContainer[id];
	p->AmplitudePhaseAlg(angle, quality);
	
 	Log("AmplitutdePhaseAlg_AS success, id = %d, angle = %f, quality = %f\n",id, angle, quality);
	return 0;
}

// AI测向算法
// @param id : 编号
// @param sigNum : 信号个数
// @param angles : 测向角度
// @param qualities : 测向质量
// @param newCutMode : 切刀方式
int AI_AS(int id, int sigNum, double * angles, double * qualities, double *amplitudes, \
	int *newCut, int* newCutAntCodes, int & antCode){

	Log("AI_AS begin, id=%d, sigNum=%d\n",id,sigNum);

	// -----------------------相干信号临时测试---------------------------
	// Coherent_AS(id, sigNum, angles, amplitudes);
	// return 0;
	// -----------------------相干信号临时测试---------------------------

    if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
		Log("AI_AS failed\n");
        return 1;
    }

    if (sigNum == 0) {
        Log("AI_AS failed, No Signal.\n");
        return -1;
    }
    DireDoa* p = DireDoaContainer[id];

    vector<double> anglesVec;
    vector<double> qualityVec;
    vector<double> amplitudesVec;
	vector<int> fixedAnt;
	vector<int> antCodes;

	antCode = 0;
	p->SetSignalNum(sigNum);

    if(sigNum==1){
		double angle;
		double quality;
		p->AmplitudePhaseAlg(angle, quality);
		if (p->GetDbfFlag() == 1){
			p->GetAntCode(fixedAnt, antCodes, antCode);
		}
        angles[0] = angle;
		qualities[0] = quality;
        amplitudes[0]=1.0;
    }else{
		// p->GetChannelNum();
		// p->GetDbfFlag();
		// if (sigNum > p->GetChannelNum() || p->GetFastAIMode()){
		if (p->GetDbfFlag() == 1 || p->GetFastAIMode() || p->GetChannelNum() <= 2){
		// if (1){
			// p->AI(sigNum,anglesVec,qualityVec,amplitudesVec);
			p->AI(sigNum, anglesVec, qualityVec, amplitudesVec, fixedAnt, antCodes, antCode);
			Log("SigNum > ChannelNum Or DbfFlag Or FastAIMode, use AI.\n");
		}
		else{
			bool flag = p->GetAIFlag();
			if (!flag){
				p->SetAlgorithmTag(sigNum);
			}
			int algorithmTag = p->GetAlgorithmTag();
			if (algorithmTag == 0){
				Log("AI Mode\n");
				// p->AI(sigNum, anglesVec, qualityVec, amplitudesVec);
				p->AI(sigNum, anglesVec, qualityVec, amplitudesVec, fixedAnt, antCodes, antCode);
			}
			else if (algorithmTag == 1)	{
				Log("CMA Mode\n");
				p->CMA(sigNum, anglesVec, qualityVec, amplitudesVec);
			}
			else{
				Log("AI_Doa failed, unknown algorithm tag=%d\n", algorithmTag);
				return 1;
			}
		}
		// 按照0 - 360排序
		vector<int> indexArr(sigNum);
		for (size_t i = 0; i < sigNum; i++){
			int largeThan = 0;
			for (size_t j = 0; j < sigNum; j++){
				if (anglesVec[i] > anglesVec[j]){
					largeThan++;
				}
			}
			indexArr[i] = largeThan;
		}
        for(int i = 0; i < sigNum; i++){
            angles[indexArr[i]] = anglesVec[i];
            qualities[indexArr[i]] = qualityVec[i];
            amplitudes[indexArr[i]] = amplitudesVec[i];
        }
    }

	// 如果调用的空域滤波算法，则需要将天线代码和固定天线信息传递给新切刀
	if (p->GetDbfFlag() == 1){
		for (int i = 0; i < fixedAnt.size(); i++){
			newCut[i] = fixedAnt[i];
		}
		for (int i = 0; i < antCodes.size(); i++){
			newCutAntCodes[i] = antCodes[i];
		}
	}

        
    Log("AI_AS success,id=%d, result:\n",id);
    for(int i = 0;i < sigNum; ++i){
        Log("signal%d : angle=%f, quality=%f, amplitude=%f\n",i,angles[i], qualities[i], amplitudes[i]);
    }
	
    return 0;
}

// 空域滤波算法返回解混系数
// id : 编号
// sigNum : 信号个数
// coeff : 解混系数
int DBF_AS(int id, int sigNum, std::complex<double>* coeff, bool &flag){

	Log("DBF_AS begin,id=%d, sigNum=%d\n",id, sigNum);

    if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
		Log("DBF_AS failed\n");
        return 1;
    }

    DireDoa *p = DireDoaContainer[id];

    vector<complex<double>>  coeffVec;

	p->SetSignalNum(sigNum);
    p->DBFAllChannel(sigNum, coeffVec, flag);

    int coeffNum = coeffVec.size();

    for(int i = 0; i < coeffVec.size(); i++){
        coeff[i]=coeffVec[i];
    }

    Log("DBF_Doa success,id=%d, coeffNum=%d, coeffData: \n",id,coeffNum);
    for(int i = 0; i < coeffNum; ++i){
        Log("%f + 1i * %f,\n",coeff[i].real(),coeff[i].imag());
    }

    return 0;
}

int SetAverageTime_AS(int id, double at){

	Log("SetAverageTime_AS begin,id=%d, AverageTime=%f\n", id, at);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		return 1;
	}

	DireDoa* p = DireDoaContainer[id];

	p->SetAverageTime(at);

	Log("SetAverageTime_AS success,id=%d, AverageTime=%f\n", id, at);
	return 0;
}


// 设置FFT数据	
// id : 编号
// data : 输入数据
// length : 数据长度
// 正常运行后，返回0，其他根据具体情况返回非0值
int SetDataFFT_AS(int id, const double * data, int length){
	Log("SetDataFFT_AS begin, id=%d, length=%d\n", id, length);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		return 1;
	}

	DireDoa* p = DireDoaContainer[id];
	vector<double> dataVec(data, data + length);
	p->SetDataFFT(dataVec);

	Log("SetDataFFT_AS success,id=%d\n", id);
	return 0;
}


// 设置FFT频谱数据
// id : 编号
// data : 输入数据
// length : 数据长度
// 正常运行后，返回0，其他根据具体情况返回非0值
int SetAmplitudeDataFFT_AS(int id, const double * data, int length){
	Log("SetAmplitudeDataFFT_AS begin, id=%d, length=%d\n", id, length);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		return 1;
	}

	DireDoa* p = DireDoaContainer[id];
	vector<double> dataVec(data, data + length);
	p->SetAmplitudeDataFFT(dataVec);

	Log("SetAmplitudeDataFFT_AS success,id=%d\n", id);
	return 0;
}


// 设置FFT门限
// id : 编号
// threshold : 门限
// 正常运行后，返回0，其他根据具体情况返回非0值
int SetThresholdFFT_AS(int id, double threshold){
	Log("SetThresholdFFT_AS begin, id=%d, threshold=%f\n", id, threshold);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		return 1;
	}

	DireDoa* p = DireDoaContainer[id];
	// p->SetThresholdFFT(threshold);
	p->SetFFTThreshold(threshold);

	Log("SetThresholdFFT_AS success,id=%d\n", id);
	return 0;
}


// 设置FFT校正数据
// id : 编号
// correction : 校正数据
// length : 数据长度
int SetCorrectionFFT_AS(int id, const double *correction, int length){
	Log("SetCorrectionFFT_AS begin, id=%d, length=%d\n", id, length);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		return 1;
	}

	DireDoa* p = DireDoaContainer[id];
	vector<double> correctionVec(correction, correction + length);
	p->SetCorrectionFFT(correctionVec);

	Log("SetCorrectionFFT_AS success,id=%d\n", id);
	return 0;
}


// 获取FFT测向角度
// id : 编号
// angleNum : 角度个数
// angles : 角度
// frequencies : 频率
int GetAngleFFT_AS(int id, int &angleNum, double* angles, int* frequencies)
{
	Log("GetAngleFFT_AS begin, id=%d\n", id);

	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]) {
		return 1;
	}

	DireDoa* p = DireDoaContainer[id];
	double qualities[4096];
	std::vector<double> anglesVec;
	std::vector<int> frequenciesVec;
	std::vector<double> qualitiesVec;
	p->GetAngleFFT(angleNum, anglesVec, frequenciesVec, qualitiesVec);

	for(int i = 0; i < angleNum; ++i){
		angles[i] = anglesVec[i];
		frequencies[i] = frequenciesVec[i];
		qualities[i] = qualitiesVec[i];
	}
	for (int i = 0; i < angleNum; ++i){
		Log("angle%d : %f, frequency%d : %d, quality%d : %f\n", i, angles[i], i, frequencies[i], i, qualities[i]);
	}
	Log("GetAngleFFT_AS success,id=%d\n", id);
	return 0;
}

// 释放对象
// id : 编号
int Release_AS(int id){
    Log("Release_AS begin,id=%d\n",id);

    if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("Release_AS failed\n");
        return 1;
    }

    DireDoa* p = DireDoaContainer[id];
    delete p;
    DireDoaContainer[id] = nullptr;

    Log("Release_AS success,id=%d\n",id);
    return 0;
}


// 设置是否调用空域滤波算法
int SetDbfFlag_Doa(int id, int tag){
	Log("SetDbfTag_Doa begin, id=%d\n", id);
	if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
		Log("Release_AS failed\n");
		return 1;
	}

	OmniDoa* p = OmniDoaContainer[id];
	p->SetDbfFlag(tag);

	Log("SetDbfTag_Doa success, id=%d\n", id);
	return 0;
}


// 设置是否调用空域滤波算法
// @param ID 算法对象编号
// @param tag 是否调用空域滤波算法，0表示不掉用，1表示掉用，默认值为1
int SetDbfFlag_AS(int id, int tag){
	Log("SetDbf_AS begin, id=%d\n", id);
	if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
		Log("Release_AS failed\n");
		return 1;
	}

	DireDoa* p = DireDoaContainer[id];
	p->SetDbfFlag(tag);

	Log("SetDbf_AS success, id=%d, tag=%d\n", id, tag);
	return 0;
}

int AntiMultipath_Doa(int id, double &angle, double &quality)
{
	Log("AntiMultipath_Doa begin, id=%d\n", id);
	if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("AntiMultipath_Doa failed\n");
        return 1;
    }

	OmniDoa* p = OmniDoaContainer[id];
	p->AntiMultipath(angle, quality);
	Log("AntiMultipath_Doa success, id=%d, angle=%f, quality=%f\n", id, angle, quality);

	return 0;
}

int AntiMultipath_AS(int id, double & angle, double & quality){
	Log("AntiMultipath_AS begin, id=%d\n", id);
	if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("AntiMultipath_AS failed\n");
        return 1;
    }

	DireDoa* p = DireDoaContainer[id];
	p->AntiMultipath(angle, quality);
	Log("AntiMultipath_Doa success, id=%d, angle=%f, quality=%f\n", id, angle, quality);

	return 0;
}

int SetFastAIMode_Doa(int id){
	Log("SetFastAIMode_Doa begin, id=%d\n", id);
	if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetFastAIMode_Doa failed\n");
        return 1;
    }

	OmniDoa* p = OmniDoaContainer[id];
	p->SetFastAIMode(true);
	return 0;
}

int SetFastAIMode_AS(int id){
	Log("SetFastAIMode_AS begin, id=%d\n", id);
	if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("SetFastAIMode_AS failed\n");
        return 1;
    }

	DireDoa* p = DireDoaContainer[id];
	p->SetFastAIMode(true);
	return 0;
}

int GetCorrectionCoeffFFT_AS(int id, short* corCoeff){
	Log("GetCorrectionCoeffFFT_AS begin, id=%d\n", id);
	if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("GetCorrectionCoeffFFT_AS failed\n");
        return 1;
    }

	DireDoa* p = DireDoaContainer[id];
	std::vector<std::complex<double>> corCoeffVec;
	p->GetCorrectionCoeff(corCoeffVec);
	for (int i = 0; i < corCoeffVec.size(); i++){
		corCoeff[2 * i] = static_cast<short>(corCoeffVec[i].real() * 30000);
		corCoeff[2 * i + 1] = static_cast<short>(corCoeffVec[i].imag() * 30000);
	}
	Log("GetCorrectionCoeffFFT_AS success, id=%d\n", id);
	return 0;
}


// 设置错误天线索引
// @param id 算法id
// @param antNum 错误天线索引数组的长度
// @param errorIdx 错误天线索引数组
// @return 正确运行返回0，错误id返回1
int SetErrorAntennaIndex_Doa(int id, int antNum, int * errorIdx){
	Log("SetErrorAntennaIndex_Doa begin, id=%d\n", id);
	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("SetErrorAntennaIndex_Doa failed\n");
        return 1;
    }
	OmniDoa* p = OmniDoaContainer[id];
	p->SetErrorAntIndices(std::vector<int>(errorIdx, errorIdx + antNum));
	Log("SetErrorAntennaIndex_Doa success, id=%d\n", id);
	return 0;
}

int SetErrorAntennaIndex_AS(int id, int antNum, int * errorIdx){
	Log("SetErrorAntennaIndex_AS begin, id=%d\n", id);
	if (id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("SetErrorAntennaIndex_AS failed\n");
        return 1;
    }
	DireDoa* p = DireDoaContainer[id];
	p->SetErrorAntIndices(std::vector<int>(errorIdx, errorIdx + antNum));
	Log("SetErrorAntennaIndex_AS success, id=%d\n", id);
	return 0;
}

int GetPhaseDifferenceFFT_Doa(int id, double * phaseDiff){
	Log("GetPhaseDifferenceFFT_Doa begin, id=%d\n", id);
	if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
        Log("GetPhaseDifferenceFFT_Doa failed\n");
        return 1;
    }

	OmniDoa* p = OmniDoaContainer[id];
	std::vector<double> phaseDiffVec;
	p->GetPhaseDifferenceFFT(phaseDiffVec);
	for (int i = 0; i < phaseDiffVec.size(); i++){
		phaseDiff[i] = -1 * phaseDiffVec[i];
	}	
	Log("GetPhaseDifferenceFFT_Doa success, id=%d\n", id);
	return 0;
}


// 设置俯仰角数组
// @param id 算法id
// @param elevation 俯仰角数组
// @param eleLen 俯仰角数组的长度
// @return 正确运行返回0，错误id返回1
int SetElevationVec_Doa(int id, double *elevations, int eleLen){
	Log("Set Elevation Angles Vector Begin, id=%d\n", id);
	if (id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
		Log("Set Elevation Angles Vector Failed. WRONG id\n");
		return 1;
	}

	Log("Elevation Angles:\n");
	vector<double> eleVec;
	for (int i = 0; i < eleLen; i++){
		Log("%f\t", elevations[i]);
		eleVec.push_back(elevations[i]);
	}

	OmniDoa *p = OmniDoaContainer[id];
	p->SetElevationVec(eleVec);

	Log("Set Elevation Angles Vector Success, id=%d\n", id);	
    return 0;
}

int SetElevationVec_AS(int id, double *elevations, int eleLen){
    return 0;
}

// 创建GN902对象
int Create_GN902(int & id){
	try{
		GN902* p  = new (nothrow) GN902();
		if(!p){
			return 1;
		}
		for(id = 0; id < GN902Container.size();++id){
			if(!GN902Container[id]){
				GN902Container[id] = p;
				break;
			}
		}
		if(id == GN902Container.size()){
			GN902Container.push_back(p);
		}
	}
	catch(const std::exception& e)
	{
		std::cout << "error:: create GN902 failed" << endl;
		std::cerr << e.what() << '\n';
		return -1;
	}
	
	try{
		if (!id) {
			LogCreate();
		}
	}
	catch(const std::exception& e){
		std::cout << "error:: create Log failed" << endl;
		std::cerr << e.what() << '\n';
		return -2;
	}
	
    Log("Create_GN902 success, id=%d\n",id);

    return 0;
}

// 初始化GN902对象
// @param id 算法id
// @param phsDiffThreshold 位相差阈值
// @param satelliteCountThreshold 卫星数量阈值
// @param cutCountThreshold 切刀次数阈值
// @param sysEnum 系统类型
// @param typeEnum 频点类型
// @return 正确运行返回0，错误id返回1
int SetThresholdDetection_GN902(int id, double phsDiffThreshold, double satelliteCountThreshold, 
            double cutCountThreshold, int sysEnum, int typeEnum){
	if(id < 0 || id >= GN902Container.size() || !GN902Container[id]){
        Log("SetThresholdDetection_GN902 failed\n");
        return 1;
    }
	GN902* p = GN902Container[id];
	p->SetThresholdDetection(phsDiffThreshold, satelliteCountThreshold, cutCountThreshold, sysEnum, typeEnum);
	Log("SetThresholdDetection_GN902 success, id=%d\n", id);
	return 0;
}

// 设置GN902虚拟阵元测向参数
// @param id 算法id
// @param secondaryDoa 是否启用虚拟干涉仪二次测向(解相位模糊, 1=是 0=否)
// @param virtualExpand 是否启用虚拟阵列扩展(扩大等效孔径, 1=是 0=否)
// @param virMultiple 虚拟倍率(<1 缩短基线解模糊, >1 扩大孔径)
// @return 正确运行返回0，错误id返回1
int SetVirtualDoa_GN902(int id, int secondaryDoa, int virtualExpand, double virMultiple){
	if(id < 0 || id >= GN902Container.size() || !GN902Container[id]){
        Log("SetVirtualDoa_GN902 failed\n");
        return 1;
    }
	GN902* p = GN902Container[id];
	p->SetVirtualDoa(secondaryDoa != 0, virtualExpand != 0, virMultiple);
	Log("SetVirtualDoa_GN902 success, id=%d\n", id);
	return 0;
}


// 设置GN902对象数据
// @param id 算法id
// @param data 数据数组
// @param cutIdx 通道2对应天线
// @param endFlag 结束标志（是否为最后一帧数据，0为不是，1为是）
// @return 正确运行返回0，错误id返回1
int SetData_GN902(int id, const GNSSData* data, int cutIdx_1, int cutIdx_2){
	if(id < 0 || id >= GN902Container.size() || !GN902Container[id]){
        Log("SetData_GN902 failed\n");
        return 1;
    }
	GN902* p = GN902Container[id];
	p->SetData(data, cutIdx_1, cutIdx_2);
	Log("SetData_GN902 success, id=%d\n", id);
	return 0;
}


// 获取GN902对象结果
// @param id 算法id
// @param result 结果结构体
// @param result 结果结构体
// @return 正确运行返回0，错误id返回1
int GetResult_GN902(int id, SpoofingResult& result){
	if(id < 0 || id >= GN902Container.size() || !GN902Container[id]){
        Log("GetResult_GN902 failed\n");
        return 1;
    }
	GN902* p = GN902Container[id];
	p->GetResult(result);
	Log("GetResult_GN902 success, id=%d\n", id);
	return 0;
}

// 释放GN902对象
// @param id 算法id
// @return 正确运行返回0，错误id返回1
int Release_GN902(int id){
	return 0;
}



// 创建压制测向算法对象
// @param id - 算法id
// @param pointNum - 点数
// @param startPointNum - 起始点数
// @param endPointNum - 结束点数
// @note 起始点数和结束点数默认值为0，创建成功后会为id赋值
int Create_SP(int &id, int pointNum, int startPointNum=0, int endPointNum=0){
	try{
		SuppressingDoa* p  = new (nothrow) SuppressingDoa(pointNum, startPointNum, endPointNum);
		if(!p){
			return 1;
		}
		for(id = 0; id < SuppressingDoaContainer.size();++id){
			if(!SuppressingDoaContainer[id]){
				SuppressingDoaContainer[id] = p;
				break;
			}
		}
		if(id == SuppressingDoaContainer.size()){
			SuppressingDoaContainer.push_back(p);
		}
	}
	catch(const std::exception& e)
	{
		std::cout << "error:: Create_SP(创建SP对象失败)" << endl;
		std::cerr << e.what() << '\n';
		return -1;
	}
	
	try{
		if (!id) {
			LogCreate();
		}
	}
	catch(const std::exception& e){
		std::cout << "error:: create Log(创建日志文件失败)" << endl;
		std::cerr << e.what() << '\n';
		return -2;
	}
	
    Log("Create_SP success, id=%d,pointNum=%d, startPointNum=%d, endPointNum=%d\n", 
		id, pointNum, startPointNum, endPointNum);

    return 0;
}

// int SetAlgorithmTag_Doa(int id, int tag)
// {
// 	Log("SetAlgorithmTag_Doa begin, id=%d\n", id);
// 	if(id < 0 || id >= OmniDoaContainer.size() || !OmniDoaContainer[id]){
//         Log("Release_AS failed\n");
//         return 1;
//     }

//     OmniDoa* p = OmniDoaContainer[id];
// 	p->SetAlgorithmTag(tag);

// 	Log("SetAlgorithmTag_Doa success, id=%d\n", id);
// 	return 0;
// }


int SetAlgorithmTag_Doa(int id, int tag)
{
	Log("SetAlgorithmTag_Doa begin, id=%d\n", id);
	if(id < 0 || id >= DireDoaContainer.size() || !DireDoaContainer[id]){
        Log("Release_AS failed\n");
        return 1;
    }

    DireDoa* p = DireDoaContainer[id];
	p->SetAlgorithmTag(tag);

	Log("SetAlgorithmTag_Doa success, id=%d\n", id);
	return 0;
}

int SetErrorAnt_Doa(int id, int * antNum, int * errorIdx)
{
return 0;
}

int SetErrorAnt_AS(int id, int * antNum, int * errorIdx)
{
return 0;
}