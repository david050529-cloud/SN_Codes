#include "interface.h"
#include <iostream>
#include <fstream>
#include <sstream>
#include <time.h>
#include <chrono>
#include <random>
#include "json.hpp"
#include "gn902_dat_parser.h"

// #include <arm_neon.h>

using namespace std;
using json = nlohmann::json;

int main12(json jsonData);
int main13(json jsonData);
int main19(json jsonData);

int main22(json jsonData);
int main23(json jsonData);
int main29(json jsonData);

int main62(json jsonData);
int main63(json jsonData);
int main69(json jsonData);

int main72(json jsonData);
int main73(json jsonData);
int main79(json jsonData);

int main82(json jsonData);
int main83(json jsonData);
int main89(json jsonData);

int main98(json jsonData);
int main99(json jsonData);

int main101(json jsonData);
int main102(json jsonData);
int main103(json jsonData);
int main104(json jsonData);
int main105(json jsonData);

int main111(json jsonData);
int main112(json jsonData);
int main113(json jsonData);
int main114(json jsonData);

int main139(json jsonData);

int main141(json jsonData);

int main991(json jsonData);

int main902(json jsonData);

int main(){
	std::ifstream f("usedConfig.json");
	if (!f.is_open()){
		cout << "Error: Failed to open config.json file." << endl;
		return 1;
	}
	json jsonData;
	f >> jsonData;
	int method = jsonData["method"];
	int type = jsonData["method type"];
	// 测向方法（Method）与对应类型（Type）定义表：
	// 注：恒模分离包含在多信号测向中
	// | Method ID | Method 名称                | 支持的 Type 列表                                     |
	// |----------|---------------------------|-----------------------------------------------------|
	// | 1        | 全向天线干涉仪            | 双通道（2）, 三通道（3）, 九通道（9）                       |
	// | 2        | 全向天线Music            | 双通道（2）, 三通道（3）, 九通道（9）                       |
	// | 3        | 全向天线DML              | 双通道（2）, 三通道（3）, 九通道（9）                       |
	// | 4        | 全向天线IDM              | 双通道（2）, 三通道（3）, 九通道（9）                       | 
	// | 5        | WW方法                   | （不维护）                                              |
	// | 6        | 全向天线空域滤波           | 双通道（2）, 三通道（3）, 九通道（9）                      |
	// | 7        | 全向天线AI               | 双通道（2）, 三通道（3）, 九通道（9）                       |                     
	// | 8        | 全向天线FFT              | 双通道（2）, 三通道（3）, 九通道（9）                       ｜                 
	// | 9        | 比幅测向                 | 八阵元（8）, 九阵元（9）                                  |                     
	// | 10       | 定向天线三通道            | 12刀幅相（1）, 4刀幅相（2）, AI（3）, 空域滤波（4）, FFT（5） |   
	// | 11		  | 定向天线双通道			  | 8-26.5G幅相法(1)， I型平台(2)， II型平台（3）, 39二期（4）   |
	// | 12       | 抗多径测向	             | 三通道全向(1)                                            |
	// | 13       | 相干		            | 九通道(9)                                            |
	// | 14       | 定向天线九通道            | 空域滤波(1)                                            |
	// | 99       | 辅助功能                 | 信号估计（1）                                            |      
	// |----------|---------------------------|-----------------------------------------------------|           

	// 九通道采样率与采样点数 
	// ｜———————————————————————————————————————————————————————————————|
	// ｜   采样率（Hz）   ｜ 采样点数 ｜   采样时间（s）   ｜  抗1Hz拍频帧数 ｜
	// ｜     1000        ｜  2048    ｜   2.0480         ｜       1       ｜
	// ｜     1250        ｜  2048    ｜   1.6384         ｜       1       ｜
	// ｜     1500        ｜  2048    ｜   1.3653         ｜       1       ｜
	// ｜     2000        ｜  2048    ｜   1.0240         ｜       1       ｜
	// ｜     2500        ｜  2048    ｜   0.8192         ｜       2       ｜
	// ｜     3125        ｜  2048    ｜   0.6554         ｜       2       ｜
	// ｜     4000        ｜  2048    ｜   0.5120         ｜       2       ｜
	// ｜     5000        ｜  2048    ｜   0.4096         ｜       3       ｜
	// ｜     6250        ｜  2048    ｜   0.3277         ｜       4       ｜
	// ｜    10000        ｜  2048    ｜   0.2048         ｜       5       ｜
	// ｜    12500        ｜  2048    ｜   0.1638         ｜       7       ｜
	// ｜    15000        ｜  2048    ｜   0.1365         ｜       8       ｜
	// ｜    25000        ｜  2048    ｜   0.0819         ｜       13      ｜
	// ｜    50000        ｜  2048    ｜   0.0410         ｜       25      ｜
	// ｜    80000        ｜  2048    ｜   0.0256         ｜       40      ｜
	// ｜   100000        ｜  2048    ｜   0.0205         ｜       49      ｜
	// ｜   125000        ｜  2048    ｜   0.0164         ｜       62      ｜
	// ｜   200000        ｜  2048    ｜   0.0102         ｜       98      ｜
	// ｜   250000        ｜  2048    ｜   0.0082         ｜       123     ｜
	// ｜   500000        ｜  2048    ｜   0.0041         ｜       245     ｜
	// ｜   800000        ｜  2048    ｜   0.0026         ｜       391     ｜
	// ｜  1000000        ｜  2048    ｜   0.0020         ｜       489     ｜
	// ｜  1280000        ｜  2048    ｜   0.0016         ｜       625     ｜
	// ｜  1500000        ｜  2048    ｜   0.0014         ｜       733     ｜
	// ｜  2000000        ｜  2048    ｜   0.0010         ｜       977     ｜
	// ｜  5000000        ｜  2048    ｜   0.00041        ｜       2442    ｜
	// ｜  8000000        ｜  2048    ｜   0.00026        ｜       3907    ｜
	// ｜ 10000000        ｜  2048    ｜   0.00020        ｜       4883    ｜
	// ｜ 20000000        ｜  2048    ｜   0.00010        ｜       9766    ｜
	// ｜ 40000000        ｜  2048    ｜   0.00005        ｜       19532   ｜
	// ｜ 80000000        ｜  2048    ｜   0.00003        ｜       39063   ｜
	// ｜—————————————————————————————————————————————————————————————————|

	// 三通道（12刀）采样率与采样点数
	// ｜———————————————————————————————————————————————————————————————|
	// ｜  采样率（Hz）    ｜ 采样点数  ｜  采样时间（s）   ｜ 抗1Hz拍频帧数 ｜
	// ｜  1000          ｜  128     ｜  1.536         ｜      1       ｜
	// ｜  1250          ｜  128     ｜  1.2288        ｜      1       ｜
	// ｜  1500          ｜  128     ｜  1.024         ｜      1       ｜
	// ｜  2000          ｜  128     ｜  0.768         ｜      2       ｜
	// ｜  2500          ｜  128     ｜  0.6144        ｜      2       ｜
	// ｜  3125          ｜  256     ｜  0.98304       ｜      2       ｜
	// ｜  4000          ｜  256     ｜  0.768         ｜      2       ｜
	// ｜  5000          ｜  256     ｜  0.6144        ｜      2       ｜
	// ｜  6250          ｜  512     ｜  0.98304       ｜      2       ｜
	// ｜  10000         ｜  512     ｜  0.6144        ｜      2       ｜
	// ｜  12500         ｜  512     ｜  0.49152       ｜      3       ｜
	// ｜  15000         ｜  1024    ｜  0.8192        ｜      2       ｜
	// ｜  25000         ｜  1024    ｜  0.49152       ｜      3       ｜
	// ｜  50000         ｜  1024    ｜  0.24576       ｜      5       ｜
	// ｜  80000         ｜  2048    ｜  0.3072        ｜      4       ｜
	// ｜  100000        ｜  2048    ｜  0.24576       ｜      5       ｜
	// ｜  125000        ｜  2048    ｜  0.196608      ｜      6       ｜
	// ｜  200000        ｜  2048    ｜  0.12288       ｜      9       ｜
	// ｜  250000        ｜  2048    ｜  0.098304      ｜      11      ｜
	// ｜  500000        ｜  2048    ｜  0.049152      ｜      21      ｜
	// ｜  800000        ｜  2048    ｜  0.03072       ｜      33      ｜
	// ｜  1000000       ｜  2048    ｜  0.024576      ｜      41      ｜
	// ｜  1280000       ｜  2048    ｜  0.0192        ｜      53      ｜
	// ｜  1500000       ｜  2048    ｜  0.016384      ｜      62      ｜
	// ｜  2000000       ｜  2048    ｜  0.012288      ｜      82      ｜
	// ｜  5000000       ｜  2048    ｜  0.0049152     ｜      204     ｜
	// ｜  8000000       ｜  2048    ｜  0.003072      ｜      326     ｜
	// ｜  10000000      ｜  2048    ｜  0.0024576     ｜      407     ｜
	// ｜  20000000      ｜  2048    ｜  0.0012288     ｜      814     ｜
	// ｜  40000000      ｜  2048    ｜  0.0006144     ｜      1628    ｜
	// ｜  80000000      ｜  2048    ｜  0.0003072     ｜      3256    ｜
	// ｜————————————————————————————————————————————————————————————————|


	// 切刀方式汇总
	// ————————————————————-｜
	// 三通道 12刀 			 ｜
	// cut1    ｜  1, 4, 7 	｜
	// cut2    ｜  2, 5, 8 	｜
	// cut3	   ｜  3, 6, 9 	｜
	// cut4    ｜  1, 5, 9 	｜
	// cut5    ｜  2, 6, 7 	｜
	// cut6    ｜  3, 4, 8 	｜
	// cut7    ｜  1, 6, 8 	｜
	// cut8    ｜  2, 4, 9 	｜
	// cut9    ｜  3, 5, 7 	｜
	// cut10   ｜  1, 2, 3 	｜
	// cut11   ｜  4, 5, 6 	｜
	// cut12   ｜  7, 8, 9 	｜
	// ————————————————————-｜
	// 三通道 7刀 			 ｜
	// cut1    ｜  1, 2, 7 	｜
	// cut2    ｜  3, 2, 7 	｜
	// cut3	   ｜  4, 2, 7 	｜
	// cut4    ｜  2, 5, 7 	｜
	// cut5    ｜  2, 6, 7 	｜
	// cut6    ｜  2, 8, 7 	｜
	// cut7    ｜  7, 2, 9 	｜
	// ————————————————————-｜

	// 天线类型包含： 
	// 0:垂直-福建天线产线 
	// 1:水平-福建天线产线 
	// 2:垂直-福建天线研发 
	// 3:水平-福建天线研发 
	// 4:8-26.5G测向 
	// 5-I型平台 
	// 6-II型平台，垂直 
	// 7-II型平台，水平

	// 孔径
	// 固定站
	// 移动站

	// 频段划分

	switch (method) {
case 1:
	switch (type) {
	case 2:
		main12(jsonData);
		break;
	case 3:
		main13(jsonData);
		break;
	case 9:
		main19(jsonData);
		break;
	default:
		break;
	}
	break;
case 2:
	switch (type) {
	case 2:
		main22(jsonData);
		break;
	case 3:
		main23(jsonData);
		break;
	case 9:
		main29(jsonData);
		break;
	default:
		break;
	}
	break;
case 6:
	switch (type) {
	case 2:
		main62(jsonData);
		break;
	case 3:
		main63(jsonData);
		break;
	case 9:
		main69(jsonData);
		break;
	default:
		break;
	}
	break;
case 7:
	switch (type) {
	case 2:
		main72(jsonData);
		break;
	case 3:
		main73(jsonData);
		break;
	case 9:
		main79(jsonData);
		break;
	default:
		break;
	}
	break;
case 8:
	switch (type) {
	case 2:
		main82(jsonData);
		break;
	case 3:
		main83(jsonData);
		break;
	case 9:
		main89(jsonData);
		break;
	default:
		break;
	}
	break;
case 9:
	switch (type) {
	case 8:
		main98(jsonData);
		break;
	case 9:
		main99(jsonData);
		break;
	default:
		break;
	}
	break;
case 10:
	switch (type) {
	case 1:
		main101(jsonData);
		break;
	case 2:
		main102(jsonData);
		break;
	case 3:
		main103(jsonData);
		break;
	case 4:
		main104(jsonData);
		break;
	case 5:
		main105(jsonData);
		break;
	default:
		break;
	}
	break;
case 11:
	switch (type) {
	case 1:
		main111(jsonData);
		break;
	case 2:
		main112(jsonData);
		break;
	case 3:
		main113(jsonData);
		break;
	case 4:
		main114(jsonData);
		break;
	default:
		break;
	}
	break;
case 13:
	switch (type)
	{
	case 9:
		main139(jsonData);
		break;
	default:
		break;
	}
	break;
case 14:
	switch (type){
		case 1:
			main141(jsonData);
			break;
		default:
			break;
	}
	break;
case 90:
	main902(jsonData);
	break;
case 99:
	switch (type) {
	case 1:
		main991(jsonData);
		break;
	default:
		break;
	}
	break;
default:
	break;
}
}

// 双通道干涉仪测向	
int main12(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double BW = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	double at = jsonData["at"];		// 积分时间
	int countNum = jsonData["calculationTime"];		// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	// string dirName = "./data/ch3/IQ/Dire/data0925_3个信号幅度分辨率/三个幅度相近/";
	int dataType = jsonData["dataType"];
	string dirName = jsonData["dirName"];
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];
	string modelDir = jsonData["modelDir"];

	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * countNum);
	
	// 以下，_0, _1 data数据读取
	if (dataType == 0){
		FILE* fps[channelNum];
		string fileName;
		for (int i = 0; i < channelNum; ++i) {
			fileName = dirName + "fft" + to_string(i + 1) + ".dat";
			fps[i] = fopen(fileName.c_str(), "rb");
		}

		vector<double> correctionVec;
		int start = 0;
		short tp = 0;
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					correction[start++] = tp;
				}
			}
		}
		vector<double> dataVec;
		start = 0;
		for (int n = 0; n < countNum; ++n) {
			for (int i = 0; i < cutNum; ++i) {
				for (int j = 0; j < channelNum; ++j) {
					for (int k = 0; k < pointNum * 2; ++k) {
						fread(&tp, sizeof(short), 1, fps[j]);
						data[start++] = tp;
					}
				}
			}
		}

		for (int i = 0; i < channelNum; ++i) {
			fclose(fps[i]);
		}
	}
	else{
		FILE* corFp = fopen(corDirName.c_str(), "rb");
		FILE* doaFp = fopen(doaDirName.c_str(), "rb");
		string fileName;

		vector<double> correctionVec;
		short tp = 0;
		for (int i = 0; i < length; ++i) {
			// fread(&tp, sizeof(short), 1, corFp);
			tp = 1;
			correction[i] = tp;
		}

		vector<double> dataVec;
		for (int i = 0; i < length * countNum; ++i) {
			// fread(&tp, sizeof(short), 1, doaFp);
			tp = 1;
			data[i] = tp;
		}

		fclose(corFp);
		fclose(doaFp);
	}	

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;

	// 创建测向对象
	if (ret= Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	ret = SetThrowPointNum_Doa(id, beginCutNum, endCutNum);
	if (ret != 0) {
		return ret;
	}
		
	// 设置数据保存模式刀方式
	// int byChannel = jsonData["byChannel"];
	// if (byChannel == 0) {
	// 	ret = SetByChannel_Doa(id);
	// }
	// else{
	// 	ret = SetByCut_Doa(id);
	// }
	if (ret != 0) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, BW)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 测向
	for (int count = 0; count < countNum; ++count) {
		if (ret = SetData_Doa(id, data + count * length, length)) {
			return ret;
		}

		if (ret = Interferometer_Doa(id, angle, quality)) {
			return ret;
		}

		cout << "angle=" << angle << ", quality=" << quality << endl;
	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 三通道干涉仪测向 
int main13(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double BW = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	double at = jsonData["at"];		// 积分时间
	int countNum = jsonData["calculationTime"];		// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	// string dirName = "./data/ch3/IQ/Omni/data251202/003/";
	int dataType = jsonData["dataType"];
	string dirName = jsonData["dirName"];
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];
	string modelDir = jsonData["modelDir"];

	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * countNum);
	
	// 0 : fftx.dat , 1 : _0, _1 data数据读取
	if (dataType == 0){
		FILE* fps[channelNum];
		string fileName;
		for (int i = 0; i < channelNum; ++i) {
			fileName = dirName + "fft" + to_string(i + 1) + ".dat";
			fps[i] = fopen(fileName.c_str(), "rb");
		}

		vector<double> correctionVec;
		int start = 0;
		short tp = 0;
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					correction[start++] = tp;
				}
			}
		}
		vector<double> dataVec;
		start = 0;
		for (int n = 0; n < countNum; ++n) {
			for (int i = 0; i < cutNum; ++i) {
				for (int j = 0; j < channelNum; ++j) {
					for (int k = 0; k < pointNum * 2; ++k) {
						fread(&tp, sizeof(short), 1, fps[j]);
						data[start++] = tp;
					}
				}
			}
		}

		for (int i = 0; i < channelNum; ++i) {
			fclose(fps[i]);
		}
	}
	else{
		FILE* corFp = fopen(corDirName.c_str(), "rb");
		FILE* doaFp = fopen(doaDirName.c_str(), "rb");
		string fileName;

		vector<double> correctionVec;
		short tp = 0;
		for (int i = 0; i < length; ++i) {
			fread(&tp, sizeof(short), 1, corFp);
			correction[i] = tp;
		}

		vector<double> dataVec;
		for (int i = 0; i < length * countNum; ++i) {
			fread(&tp, sizeof(short), 1, doaFp);
			data[i] = tp;
		}

		fclose(corFp);
		fclose(doaFp);
	}	

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置抛点数量
	// if (ret = SetUsePointNum_Doa(id, beginCutNum, pointNum - endCutNum)) {
	// 	return ret;
	// }
	SetThrowPointNum_Doa(id, beginCutNum, endCutNum);

	// 设置数据保存模式刀方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, BW)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 测向
	for (int count = 0; count < countNum; ++count) {
		if (ret = SetData_Doa(id, data + count * length, length)) {
			return ret;
		}

		double tmpa = 0;
		// if (ret = AI_Doa(id, 1, &angle, &quality, &tmpa)) {
		if (ret = Interferometer_Doa(id, angle, quality)) {
			return ret;
		}

		cout << "angle=" << angle << ", quality=" << quality << endl;
	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 九通道干涉仪测向(待修改）
int main19(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double BW = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	string dirName = jsonData["dirName"];

	double at = jsonData["at"];		// 积分时间
	int countNum = jsonData["calculationTime"];		// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * countNum);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = tp;
			}
		}
	}

	start = 0;
	for (int count = 0; count < countNum; ++count) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置抛点数量
	if (ret = SetUsePointNum_Doa(id, beginCutNum, pointNum - endCutNum)) {
		return ret;
	}

	// 设置数据保存模式刀方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, BW)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 测向
	for (int count = 0; count < countNum; ++count) {
		if (ret = SetData_Doa(id, data + count * length, length)) {
			return ret;
		}

		if (ret = Interferometer_Doa(id, angle, quality)) {
			return ret;
		}

		cout << "angle=" << angle << ", quality=" << quality << endl;
	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 双通道MuSic算法（待补充）
int main22(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double BW = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	string dirName = jsonData["dirName"];

	double at = jsonData["at"];		// 积分时间
	int countNum = jsonData["calculationTime"];		// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	int sigNum = jsonData["sigNum"];

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * countNum);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = tp;
			}
		}
	}

	start = 0;
	for (int count = 0; count < countNum; ++count) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double amplitudes[antennaNum];

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置抛点数量
	if (ret = SetUsePointNum_Doa(id, beginCutNum, pointNum - endCutNum)) {
		return ret;
	}

	// 设置数据保存模式刀方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, BW)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	if (ret = SetFrequencyBasedOn_Doa(id)) {
		return ret;
	}
	// 测向
	for (int count = 0; count < countNum; ++count) {
		if (ret = SetData_Doa(id, data + count * length, length)) {
			return ret;
		}

		if (ret = MUSIC_Doa(id, sigNum, angles, amplitudes)) {
			return ret;
		}

		for (size_t i = 0; i < sigNum; i++) {
			cout << "angle=" << angles[i] << endl;
		}
	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);
	return 0;
}

// 三通道MuSic算法(调通，无误）
int main23(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double BW = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	string dirName = jsonData["dirName"];

	double at = jsonData["at"];		// 积分时间
	int countNum = jsonData["calculationTime"];		// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	int sigNum = jsonData["sigNum"];

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * countNum);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = 0;
			}
		}
	}

	start = 0;
	for (int count = 0; count < countNum; ++count) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = 0;
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double amplitudes[antennaNum];

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置抛点数量
	if (ret = SetUsePointNum_Doa(id, beginCutNum, pointNum - endCutNum)) {
		return ret;
	}

	// 设置数据保存模式刀方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, BW)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 测向
	for (int count = 0; count < countNum; ++count) {
		if (ret = SetData_Doa(id, data + count * length, length)) {
			return ret;
		}

		if (ret = MUSIC_Doa(id, sigNum, angles, amplitudes)) {
			return ret;
		}

		for (size_t i = 0; i < sigNum; i++) {
			cout << "angle=" << angles[i] << endl;
		}
	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);
	return 0;
}

// 九通道MuSic算法
int main29(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double BW = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	string dirName = jsonData["dirName"];

	double at = jsonData["at"];		// 积分时间
	int countNum = jsonData["calculationTime"];		// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	int sigNum = jsonData["sigNum"];

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * countNum);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < channelNum; ++i) {
		for (int j = 0; j < pointNum * 2; ++j) {
			fread(&tp, sizeof(short), 1, fps[i]);
			correction[start++] = tp;
		}
	}

	start = 0;
	for (int count = 0; count < countNum; ++count) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double amplitudes[antennaNum];

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置抛点数量
	if (ret = SetUsePointNum_Doa(id, beginCutNum, pointNum - endCutNum)) {
		return ret;
	}

	// 设置数据保存模式刀方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, BW)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 测向
	for (int count = 0; count < countNum; ++count) {
		if (ret = SetData_Doa(id, data + count * length, length)) {
			return ret;
		}

		if (ret = MUSIC_Doa(id, sigNum, angles, amplitudes)) {
			return ret;
		}

		for (size_t i = 0; i < sigNum; i++) {
			cout << "angle=" << angles[i] << endl;
		}

	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);
	return 0;
}

// 双通道空域滤波（暂无）
int main62(json jsonData) {
	return 0;
}

// 三通道空域滤波
int main63(json jsonData) {
//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double bw = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
		
	int fn = jsonData["calculationTime"];	// 计算次数
	double at = jsonData["at"];	// 积分时间
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	int dataType = jsonData["dataType"];

	string dirName = jsonData["dirName"];
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];
	string modelDir = jsonData["modelDir"];

	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);
	
	// 0 fft data, 1 _0 data
	if (dataType == 0){
		FILE* fps[channelNum];
		string fileName;
		for (int i = 0; i < channelNum; ++i) {
			fileName = dirName + "fft" + to_string(i + 1) + ".dat";
			fps[i] = fopen(fileName.c_str(), "rb");
		}

		vector<double> correctionVec;
		int start = 0;
		short tp = 0;
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					correction[start++] = tp;
				}
			}
		}
		vector<double> dataVec;
		start = 0;
		for (int n = 0; n < fn; ++n) {
			for (int i = 0; i < cutNum; ++i) {
				for (int j = 0; j < channelNum; ++j) {
					for (int k = 0; k < pointNum * 2; ++k) {
						fread(&tp, sizeof(short), 1, fps[j]);
						data[start++] = tp;
					}
				}
			}
		}

		for (int i = 0; i < channelNum; ++i) {
			fclose(fps[i]);
		}
	}
	else{
		FILE* corFp = fopen(corDirName.c_str(), "rb");
		FILE* doaFp = fopen(doaDirName.c_str(), "rb");
		string fileName;

		vector<double> correctionVec;
		short tp = 0;
		for (int i = 0; i < length; ++i) {
			fread(&tp, sizeof(short), 1, corFp);
			correction[i] = tp;
		}

		vector<double> dataVec;
		for (int i = 0; i < length * fn; ++i) {
			fread(&tp, sizeof(short), 1, doaFp);
			data[i] = tp;
		}

		fclose(corFp);
		fclose(doaFp);
	}	

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double results[antennaNum];
	double qualitys[antennaNum];
	double amplitudes[antennaNum];
	complex<double> coeff[channelNum * cutNum * antennaNum];
	int coeffNum = -1;

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置数据保存模式刀方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	ret = SetThrowPointNum_Doa(id, beginCutNum, endCutNum);

	// 设置测向带宽
	// if (ret = SetBW_Doa(id, BW)) {
	// 	return ret;
	// }

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 测向
	for (int count = 0; count < fn; ++count) {
		if (count >= 0) {
			if (ret = SetData_Doa(id, data + count * length, length)) {
				return ret;
			}

			if (ret = DBF_Doa(id, sigNum, angles, qualitys, amplitudes, coeff, coeffNum)) {
				return ret;
			}

			for (size_t i = 0; i < sigNum; i++) {
				cout << "angle=" << angles[i] << endl;
			}
			cout << endl;
		}

	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);
	return 0;


	// 清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 九通道空域滤波（待补全）
int main69(json jsonData) {
//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double BW = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	string dirName = jsonData["dirName"];

	int countNum = jsonData["calculationTime"];		// 计算次数
	int sigNum = jsonData["sigNum"];

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * countNum);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = tp;
			}
		}
	}

	start = 0;
	int dataCount = 0;
	for (int count = 0; count < countNum; ++count) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
				}
			}
		}
		if (!feof(fps[channelNum - 1])) {
			dataCount++;
		}
	}

	countNum = min(countNum, dataCount); // 有效数据量

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double results[antennaNum];
	double qualitys[antennaNum];
	double amplitudes[antennaNum];
	complex<double> coeff[channelNum * cutNum * antennaNum];
	int coeffNum = -1;

	

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置数据保存模式刀方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, BW)) {
		return ret;
	}

	// 计算校正系数
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 测向
	for (int count = 0; count < countNum; ++count) {
		if (count >= 13) {
			if (ret = SetData_Doa(id, data + count * length, length)) {
				return ret;
			}

			if (ret = DBF_Doa(id, sigNum, angles, qualitys, amplitudes, coeff, coeffNum)) {
				return ret;
			}

			for (size_t i = 0; i < sigNum; i++) {
				cout << "angle=" << angles[i] << endl;
			}
			cout << endl;
		}

	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);
	return 0;


	// 清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 双通道AI测向 （待补全）
int main72(json jsonData) {
	return 0;
}

// 三通道AI测向
int main73(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double bw = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
		
	int fn = jsonData["calculationTime"];	// 计算次数
	double at = jsonData["at"];	// 积分时间
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	int dataType = jsonData["dataType"];

	int errorAntNum = jsonData["errorAntNum"];
	vector<int> errorAntIndex = jsonData["errorAntIndex"];
	int* errorAntIndexPtr = &errorAntIndex[0];

	string dirName = jsonData["dirName"];
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];
	string modelDir = jsonData["modelDir"];

	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);
	
	// 0 fft data, 1 _0 data
	if (dataType == 0){
		FILE* fps[channelNum];
		string fileName;
		for (int i = 0; i < channelNum; ++i) {
			fileName = dirName + "fft" + to_string(i + 1) + ".dat";
			fps[i] = fopen(fileName.c_str(), "rb");
		}

		vector<double> correctionVec;
		int start = 0;
		short tp = 0;
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					correction[start++] = tp;
				}
			}
		}
		vector<double> dataVec;
		start = 0;
		for (int n = 0; n < fn; ++n) {
			for (int i = 0; i < cutNum; ++i) {
				for (int j = 0; j < channelNum; ++j) {
					for (int k = 0; k < pointNum * 2; ++k) {
						fread(&tp, sizeof(short), 1, fps[j]);
						data[start++] = tp;
					}
				}
			}
		}

		for (int i = 0; i < channelNum; ++i) {
			fclose(fps[i]);
		}
	}
	else{
		FILE* corFp = fopen(corDirName.c_str(), "rb");
		FILE* doaFp = fopen(doaDirName.c_str(), "rb");
		string fileName;

		vector<double> correctionVec;
		short tp = 0;
		for (int i = 0; i < length; ++i) {
			fread(&tp, sizeof(short), 1, corFp);
			correction[i] = tp;
		}

		vector<double> dataVec;
		for (int i = 0; i < length * fn; ++i) {
			fread(&tp, sizeof(short), 1, doaFp);
			data[i] = tp;
		}

		fclose(corFp);
		fclose(doaFp);
	}	

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double qualitys[antennaNum];
	double amplitudes[antennaNum];

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置抛点数量
	// if (ret = SetUsePointNum_Doa(id, beginCutNum, pointNum - endCutNum)) {
	// 	return ret;
	// }
	if (ret = SetThrowPointNum_Doa(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 设置积分时间
	// if (ret = SetAverageTime_Doa(id, at)) {
	// 	return ret;
	// }

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置数据保存、读取方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 设置测向带宽
	if (ret = SetBW_Doa(id, bw)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 设置快速AI模式
	// if (ret = SetFastAIMode_Doa(id)) {
	// 	return ret;
	// }
	
	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_Doa(id, data + length * i, length)) {
			return ret;
		}
		
		// ret = SetErrorAntennaIndex_Doa(id, errorAntNum, errorAntIndexPtr);
		// if (ret != 0) {
		// 	return ret;
		// }

		// AI测向
		if (ret = AI_Doa(id, sigNum, angles, qualitys, amplitudes)) {
			return ret;
		}

		for (int i = 0; i < sigNum; ++i) {
			cout << "sigNum" << i + 1 << ", angle=" << angles[i] << endl;
		}
	}


	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 九通道AI测向
int main79(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string dirName = jsonData["dirName"];
	
	int fn = jsonData["calculationTime"];	// 计算次数
	double at = jsonData["at"];	// 积分时间
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	vector<double> correctionVec;
	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = tp;
			}
		}
	}
	vector<double> dataVec;
	start = 0;
	for (int n = 0; n < fn; ++n) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double qualitys[antennaNum];
	double amplitudes[antennaNum];

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置抛点数量
	if (ret = SetUsePointNum_Doa(id, beginCutNum, pointNum - endCutNum)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置数据保存、读取方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	// 设置快速模式
	if (ret = SetFastAIMode_Doa(id)) {
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_Doa(id, data + length * i, length)) {
			return ret;
		}

		// AI测向
		if (ret = AI_Doa(id, sigNum, angles, qualitys, amplitudes)) {
			return ret;
		}

		for (int i = 0; i < sigNum; ++i) {
			cout << "sigNum" << i + 1 << ", angle=" << angles[i] << endl;
		}
	}


	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}


// 双通道FFT
int main82(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	int treshold = jsonData["fftTreshold"];
	int threadNum = 4;
	const int length = cutNum * channelNum * pointNum * 2;
	double band = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	string dirName = jsonData["dirName"];

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < channelNum; ++i) {
		for (int j = 0; j < cutNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[i]);
				if (k % 2 == 0)
					correction[start++] = double(tp) / 100.0;
				else
					correction[start++] = double(tp) / 8192.0;
				// correction[start++] = 10;
			}
		}
	}

	//调用算法接口
	int id;
	int ret;
	int fftMode = jsonData["fftMode"];
	int angleNum = 0;
	int time = 0;
	double angles[4096];
	double qualitys[4096];
	int frequencies[4096];

	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	if (ret = SetThrowPointNum_Doa(id, 0, 0))
	{
		return ret;
	}

	if (ret = SetFFTTreshold_Doa(id, treshold)) {
		return ret;
	}

	if (ret = SetBW_Doa(id, band)) {
		return ret;
	}

	// 老版本FFT稳定
	//if (ret = SetUseDensitiesFFT_Doa(id, 10))
	//{
	//	return ret;
	//}

	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (jsonData["byChannel"]) {
		if (ret = SetByChannel_Doa(id)) {
			return ret;
		}
	}
	else{
		if (ret = SetByCut_Doa(id)) {
			return ret;
		}
	}

	if (ret = SetFFTMode_Doa(id, fftMode)) {
		return ret;
	}

	if (ret = SetThreadNum_Doa(id, 20)) {
		return ret;
	}

	if (ret = SetCorrectionFFT_Doa(id, correction, length)) {
		return ret;
	}

	for (int calTime = 0; calTime < 1; calTime++) {
		start = 0;
		for (int i = 0; i < channelNum; ++i) {
			for (int j = 0; j < cutNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[i]);
					if (k % 2 == 0)
						data[start++] = double(tp) / 100.0;
					else
						data[start++] = double(tp) / 8192.0;
					// data[start++] = 20;
				}
			}
		}

        // auto beginTime = chrono::high_resolution_clock::now();
		
        if (ret = SetDataFFT_Doa(id, data, length)) {
			return ret;
		}
        
        // auto endTime = chrono::high_resolution_clock::now();
        // chrono::duration<double> dur = endTime - beginTime;
        // cout << "SetData_Doa Cost " << dur.count() << " s.\n";

        // beginTime = chrono::high_resolution_clock::now();
		if (ret = InterferometerFFT_Doa(id, angleNum, angles, qualitys, frequencies)) {
			return ret;
		}
        // endTime = chrono::high_resolution_clock::now();
        // dur = endTime - beginTime;
        // cout << "InterferometerFFT_Doa Cost " << dur.count() << " s.\n";

		// 老版本积分
		//if (ret = SignalCombine_Doa(id, angleNum, angles, qualitys, frequencies))
		//{
		//	return ret;
		//}

		//if (ret = SetTimeFactor_Doa(id, fTime, 40000000))
		//{
		//	return ret;
		//}

		//if (ret = SetUpdateDensitiesFFT_Doa(id, angleNum, angles, frequencies))
		//{
		//	return ret;
		//}

	}

	// for (int i = 0; i < channelNum; ++i) {
	// 	fclose(fps[i]);
	// }

	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 三通道FFT
int main83(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	int treshold = -100;
	int threadNum = 1;
	const int length = cutNum * channelNum * pointNum * 2;
	double band = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];
	double* phaseDiff = new double[pointNum * antennaNum * (antennaNum - 1) / 2];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	string dirName = jsonData["dirName"];

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				if (k % 2 == 0)
					correction[start++] = tp / 100.0;
				else
					correction[start++] = tp / 8192.0;
			}
		}
	}

	//调用算法接口
	int id;
	int ret;
	int fftMode = 0;
	int angleNum = 0;
	int time = 0;
	double angles[4096];
	double qualitys[4096];
	int frequencies[4096];

	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	if (ret = SetThrowPointNum_Doa(id, 0, 0))
	{
		return ret;
	}

	if (ret = SetFFTTreshold_Doa(id, treshold)) {
		return ret;
	}

	if (ret = SetBW_Doa(id, band)) {
		return ret;
	}

	// 老版本FFT稳定
	//if (ret = SetUseDensitiesFFT_Doa(id, 10))
	//{
	//	return ret;
	//}


	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	if (ret = SetFFTMode_Doa(id, fftMode)) {
		return ret;
	}

	int fftAverageNum = 1;
	// 新版本FFT积分
	if (ret = SetFFTAverageNum_Doa(id, fftAverageNum)){
		return ret;
	}

	// if (ret = SetThreadNum_Doa(id, 20)) {
	// 	return ret;
	// }

	if (ret = SetCorrectionFFT_Doa(id, correction, length)) {
		return ret;
	}

	for (int calTime = 0; calTime < 50; calTime++) {
		start = 0;
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					if (k % 2 == 0)
						data[start++] = tp / 100.0;
					else
						data[start++] = tp / 8192.0;			
				}
			}
		}
        auto beginTime = chrono::high_resolution_clock::now();
		if (ret = SetDataFFT_Doa(id, data, length)) {
			return ret;
		}
        auto endTime = chrono::high_resolution_clock::now();
        // chrono::duration<double> dur = endTime - beginTime;
        // cout << "Set Data Cost " << dur.count() << " s." << endl;

		if (ret = GetPhaseDifferenceFFT_Doa(id, phaseDiff)) {
			return ret;
		}

        beginTime = chrono::high_resolution_clock::now();
		if (ret = InterferometerFFT_Doa(id, angleNum, angles, qualitys, frequencies)) {
			return ret;
		}
        // endTime = chrono::high_resolution_clock::now();
        // dur = endTime - beginTime;
        // cout << "InterferometerFFT Cost " << dur.count() << "s." << endl;
		// cout << calTime << endl;
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}


// 九通道FFT
int main89(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	int treshold = 7;
	int threadNum = 1;
	const int length = cutNum * channelNum * pointNum * 2;
	double band = jsonData["BW"];
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	string dirName = jsonData["dirName"];

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				// correction[start++] = tp;
				if (k % 2 == 0)
					correction[start++] = tp / 100.0;
				else
					correction[start++] = tp / 8192.0;
			}
		}
	}

	//调用算法接口
	int id;
	int ret;
	int fftMode = 0;
	int angleNum = 0;
	int time = 0;
	double angles[4096];
	double qualitys[4096];
	int frequencies[4096];

	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	if (ret = SetThrowPointNum_Doa(id, 0, 0))
	{
		return ret;
	}

	if (ret = SetFFTTreshold_Doa(id, treshold)) {
		return ret;
	}

	if (ret = SetBW_Doa(id, band)) {
		return ret;
	}

	// 老版本FFT稳定
	//if (ret = SetUseDensitiesFFT_Doa(id, 10))
	//{
	//	return ret;
	//}

	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	if (ret = SetFFTMode_Doa(id, fftMode)) {
		return ret;
	}

	if (ret = SetThreadNum_Doa(id, threadNum)) {
		return ret;
	}

	if (ret = SetCorrectionFFT_Doa(id, correction, length)) {
		return ret;
	}


	for (int calTime = 0; calTime < 50; calTime++) {
		start = 0;
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					if (k % 2 == 0)
						data[start++] = tp / 100.0;
					else
						data[start++] = tp / 8192.0;
				}
			}
		}

		if (ret = SetDataFFT_Doa(id, data, length)) {
			return ret;
		}

		auto time1 = std::chrono::high_resolution_clock::now();

		if (ret = InterferometerFFT_Doa(id, angleNum, angles, qualitys, frequencies)) {
			return ret;
		}
		
		auto time2 = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double>  dur = time2 - time1;
		cout << calTime << endl;
		for (int ii = 0; ii < angleNum; ii++)
			cout << angles[ii] << "\t";
		cout << endl;
		// 老版本积分
		//if (ret = SignalCombine_Doa(id, angleNum, angles, qualitys, frequencies))
		//{
		//	return ret;
		//}

		//if (ret = SetTimeFactor_Doa(id, fTime, 40000000))
		//{
		//	return ret;
		//}

		//if (ret = SetUpdateDensitiesFFT_Doa(id, angleNum, angles, frequencies))
		//{
		//	return ret;
		//}
	}


	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 八阵元比幅（待补全）
int main98(json jsonData) {
	return 0;
}


// 九阵元比幅 
int main99(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int pointNum = jsonData["pointNum"];
	const int length = antennaNum * pointNum * 2;

	double f = jsonData["frequency"];
	f *= 1e6;	

	string modelFile = "G:\\比幅 model\\9元\\81\\8-26.5\\model.dat";
	string dataFile = "C:\\Users\\Administrator\\Desktop\\39bf\\胥腾\\18G、20G\\30度\\18G\\18000000000-25000.dat";

	//读取IQ数据
	double* data = (double*)malloc(sizeof(double) * length);
	short tp;
	FILE* fp = fopen(dataFile.c_str(), "rb");
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, fp);
		data[i] = tp;
	}
	fclose(fp);

	//调用比幅测向算法
	int id;
	int ret;
	int type = -1;
	int startPointNum = 100;
	int endPointNum = pointNum;
	double angle;

	if (ret = Create_MicroDoa(id, antennaNum, pointNum, f)) {
		cout << "Create_MicroDoa failed" << endl;
		return 1;
	}
	if (ret = Init_MicroDoa(id, type, modelFile.c_str())) {
		cout << "Init_MicroDoa failed" << endl;
		return 1;
	}
	if (ret = SetData_MicroDoa(id, data, length)) {
		cout << "SetData_MicroDoa failed" << endl;
		return 1;
	}
	if (ret = GetAngle_MicroDoa(id, angle)) {
		cout << "GetAngle_MicroDoa failed" << endl;
		return 1;
	}
	cout << "angle=" << angle << endl;

	return 0;
}

// 信号个数自动估计
int main991(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	double r = jsonData["r"];

	int sigNum;
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	string dirName = jsonData["dirName"];

	int fn = jsonData["calculationTime"];// 计算次数
	double at = jsonData["at"];// 积分时间
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	vector<double> correctionVec;
	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = tp;
				correctionVec.push_back(tp);
			}
		}
	}
	vector<double> dataVec;
	start = 0;
	for (int n = 0; n < fn; ++n) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
					dataVec.push_back(tp);
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angles[antennaNum];
	double qualitys[antennaNum];
	double amplitudes[antennaNum];

	// 创建测向对象
	if (ret = Create_Doa(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_Doa(id, f, r)) {
		return ret;
	}

	// 设置抛点数量
	if (ret = SetThrowPointNum_Doa(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_Doa(id, at)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_Doa(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置数据保存、读取方式
	if (ret = SetByCut_Doa(id)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_Doa(id, correction, length)) {
		return ret;
	}

	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_Doa(id, data + length * i, length)) {
			return ret;
		}

		// AI测向
		if (ret = EstSigNum_Doa(id, sigNum)) {
			return ret;
		}

		cout << "sigNum = " << sigNum << endl;
	}


	//清理
	free(correction);
	free(data);
	Release_Doa(id);

	return 0;
}

// 三通道定向天线幅相法, 切刀数12, 使用SVD获取幅相值
int main101(json jsonData){
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];

	string modelDir;
	if (f < 3600e6){
		if (antType == 2)
			modelDir = "./data/1_3G_V_model_YF.dat";
		else if (antType == 3)
			modelDir = "./data/1_3G_H_model_YF.dat";
	}
	else{
		if (antType == 2)
			modelDir = "./data/3_8G_H_model_YF.dat";
		else if (antType == 3)
			modelDir = "./data/3_8G_H_model_YF.dat";
	}
	modelDir = jsonData["modelDir"];
	int fn = jsonData["calculationTime"];// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* corFp = fopen(corDirName.c_str(), "rb");
	FILE* doaFp = fopen(doaDirName.c_str(), "rb");
	string fileName;

	vector<double> correctionVec;
	short tp = 0;
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, corFp);
		correction[i] = tp;
	}

	vector<double> dataVec;
	for (int i = 0; i < length * fn; ++i) {
		fread(&tp, sizeof(short), 1, doaFp);
		data[i] = tp;
	}

	fclose(corFp);
	fclose(doaFp);

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;
	double angles[9];
	double qualities[9];
	double amplitudes[9];
	int newCut[3];
	int newCutMode[3];
	int antCode;

	// 创建测向对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_AS(id, antType, f, r)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_AS(id, at)){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// if (i == fn - 1){
		// 	cout << endl;
		// }

		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// 测向
		// if (ret = AI_AS(id, 1, angles, qualities, amplitudes, newCut, newCutMode, antCode)){
		// 	return ret;
		// }
		if (ret = AmplitutdePhaseAlg_AS(id, angles[0], qualities[0])){
			return ret;
		}

	}


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}

// 三通道定向天线 幅相法 4刀
int main102(json jsonData){
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string dirName = jsonData["dirName"];
	string modelDir = "./data/";

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	vector<double> correctionVec;
	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = tp;
			}
		}
	}
	vector<double> dataVec;
	start = 0;
	for (int n = 0; n < fn; ++n) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;
	double angles[9];
	double qualities[9];
	double amplitudes[9];
	int newCut[3];
	int newCutMode[3];
	int antCode;

	// 创建测向对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_AS(id, 0, f, r)) {
		return ret;
	}

	if (ret = SetAverageTime_AS(id, at)){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, "/Users/tengxu/Documents/workspace/new3/data/")){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// if (i == fn - 1){
		// 	cout << endl;
		// }

		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// 测向
		if (ret = AI_AS(id, 1, angles, qualities, amplitudes, newCut, newCutMode, antCode)){
			return ret;
		}
	}


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}


// 三通道定向天线 AI测向
int main103(json jsonData){
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	// 0 - fftx.dat 1 - _0.dat
	int dataType = jsonData["dataType"];

	// 天线类型包含： 0:垂直-福建天线产线 1:水平-福建天线产线 2:垂直-福建天线研发 3:水平-福建天线研发 4:8-26.5G测向 5-I型平台 6-II型平台，垂直 7-II型平台，垂直
	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"]; 	   // 积分时间

	int fastAIMode = jsonData["fastAIMode"];

	int sigNum = jsonData["sigNum"];
	// int thw[] = { 1, 4, 7, 2, 5, 8, 3, 6, 9, 1, 5, 9, 2, 6, 7, 3, 4, 8, 1, 6, 8, 2, 4, 9, 3, 5, 7, 1, 2, 3, 4, 5, 6, 7, 8, 9 }; 
	// int thw[] = { 1, 2, 3, 4, 5, 3, 7, 5, 6, 1, 8, 7};
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];

	// string dirName = "./data/ch3/IQ/Dire/data0925_3个信号幅度分辨率/三个幅度相近/";
	string dirName = jsonData["dirName"];
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];
	string modelDir;
	if (antType == 0){
		if (f < 3500e6){
			modelDir = "./data/1_3G_V_model.dat";
		}
		else{
			modelDir = "./data/3_8G_V_model.dat";
		}
	}
	else if (antType == 1){
		if (f < 3500e6){
			modelDir = "./data/1_3G_H_model.dat";
		}
		else{
			modelDir = "./data/3_8G_H_model.dat";
		}
	}
	else if (antType == 2){
		if (f <= 3600e6)
			modelDir = "./data/1_3G_V_model_YF.dat";
		else
			modelDir = "./data/3_8G_V_model_YF.dat";
	}
	else if (antType == 3){
		if (f <= 3600e6)
			modelDir = "./data/1_3G_H_model_YF.dat";
			// modelDir = "./data/1_4G_Model_H_39_S2.dat";
		else
			modelDir = "./data/3_8G_H_model_YF.dat";
	}
	// modelDir = "./data/dk_3_8.dat";
	if (jsonData["modelDir"] != ""){
		modelDir = jsonData["modelDir"];
	}

	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);
	
	// 以下，_0, _1 data数据读取
	if (dataType == 0){
		FILE* fps[channelNum];
		string fileName;
		for (int i = 0; i < channelNum; ++i) {
			fileName = dirName + "fft" + to_string(i + 1) + ".dat";
			fps[i] = fopen(fileName.c_str(), "rb");
		}

		vector<double> correctionVec;
		int start = 0;
		short tp = 0;
		for (int i = 0; i < cutNum; ++i) {
				for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					correction[start++] = tp;
				}
			}
		}
		vector<double> dataVec;
		start = 0;
		for (int n = 0; n < fn; ++n) {
			for (int i = 0; i < cutNum; ++i) {
				for (int j = 0; j < channelNum; ++j) {
					for (int k = 0; k < pointNum * 2; ++k) {
						fread(&tp, sizeof(short), 1, fps[j]);
						data[start++] = tp;
					}
				}
			}
		}

		for (int i = 0; i < channelNum; ++i) {
			fclose(fps[i]);
		}
	}
	else{
		FILE* corFp = fopen(corDirName.c_str(), "rb");
		FILE* doaFp = fopen(doaDirName.c_str(), "rb");
		string fileName;

		vector<double> correctionVec;
		short tp = 0;
		for (int i = 0; i < length; ++i) {
			fread(&tp, sizeof(short), 1, corFp);
			correction[i] = tp;
		}

		vector<double> dataVec;
		for (int i = 0; i < length * fn; ++i) {
			fread(&tp, sizeof(short), 1, doaFp);
			data[i] = tp;
			dataVec.push_back(tp);
		}

		fclose(corFp);
		fclose(doaFp);
	}	

	//调用算法接口
	int id;
	int ret;
	double angles[9];
	double qualitys[9];
	double amplitudes[9];
	int newCut[3];
	int newCutAntCodes[14];
	int antCode;
	int tmpValue[3];
	double angle;
	double quality;

	// 创建测向对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// type : 天线类型 - 0:垂直-频谱 1:水平-频谱 2:垂直-研发 3:水平-研发
	// 初始化测向对象
	if (ret = Init_AS(id, antType, f, r)) {
		return ret;
	}

	// 设置模型
	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 设置积分时间 
	// if (ret = SetAverageTime_AS(id, at)){
	// 	return ret;
	// }


	// 设置空域滤波flag
	// if (ret = SetDbfFlag_AS(id, 1)) {
	// 	return ret;
	// }

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}

	if (fastAIMode == 1){
		if (ret = SetFastAIMode_AS(id)){
			return ret;
		}
	}
	
	for (size_t i = 0; i < fn; i++) {
		// if (i == fn - 1){
		// 	cout << endl;
		// }
		// 导入数据
		if (ret = SetData_AS(id, data + i * length, length)) {
			return ret;
		}

		// 测向
		if (ret = AI_AS(id, sigNum, angles, qualitys, amplitudes, newCut, newCutAntCodes, antCode)){
			return ret;
		}
	}
	
	//清理内存
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}


// 三通道 定向天线 空域滤波
int main104(json jsonData){
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string dirName = jsonData["dirName"];
	string modelDir = "./data/";

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	vector<double> correctionVec;
	int start = 0;
	short tp = 0;
	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				correction[start++] = tp;
			}
		}
	}
	vector<double> dataVec;
	start = 0;
	for (int n = 0; n < fn; ++n) {
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					data[start++] = tp;
				}
			}
		}
	}

	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}

	//调用算法接口
	int id;
	int ret;
	double angles[9];
	double qualitys[9];
	double amplitudes[9];
	int newCut[3];
	int newCutAntCodes[14];
	int antCode;
	int tmpValue[3];
	double angle;
	double quality;

	// 创建测向对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// type : 天线类型 - 0:垂直-频谱 1:水平-频谱 2:垂直-研发 3:水平-研发
	// 初始化测向对象
	if (ret = Init_AS(id, 1, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, "/Users/tengxu/Documents/workspace/new3/data/")){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}

	ret = SetDbfFlag_AS(id, 1);
	
	for (size_t i = 0; i < 1; i++) {
		// if (i == fn - 1){
		// 	cout << endl;
		// }

		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// AI测向
		if (ret = AI_AS(id, sigNum, angles, qualitys, amplitudes, newCut, newCutAntCodes, antCode)){
			return ret;
		}

	}

	// 空域滤波
	for (size_t i = 0; i < fn; i++){
		// int newthw[] = {1, 2, 3};
		complex<double> coeef[30];
		bool flag;
		if (ret = SetThw_AS(id, 1, newCut, 1 * channelNum)) {
			return ret;
		}

		if (ret = SetData_AS(id, data + length * 1, 2048 * 2 * 3)) {
			return ret;
		}

		if (ret = DBF_AS(id, sigNum, coeef, flag)){
			return ret;
		}
	}
	


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}


// 三通道 定向天线 FFT测向
int main105(json jsonData){
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;

	double threshold = 38;

	double f = jsonData["frequency"];
	f *= 1e6;
	double r = jsonData["r"];

	// int thw[] = { 1, 2, 3, 4, 5, 3, 7, 5, 6, 1, 8, 7 };
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string dirName = jsonData["dirName"];
	string modelDir = "data/1_3G_V_model.dat";

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);
	double* spectrum = (double*)malloc(sizeof(double) * pointNum);

	FILE* fps[channelNum];
	string fileName;
	for (int i = 0; i < channelNum; ++i) {
		fileName = dirName + "fft" + to_string(i + 1) + ".dat";
		fps[i] = fopen(fileName.c_str(), "rb");
	}

	vector<double> correctionVec;
	int start = 0;
	short tp = 0;
	// for (int i = 0; i < 1; ++i) {
	// 	for (int j = 0; j < channelNum; ++j) {
	// 		for (int k = 0; k < pointNum * 2; ++k) {
	// 			fread(&tp, sizeof(short), 1, fps[j]);
	// 		}
	// 	}
	// }

	for (int i = 0; i < cutNum; ++i) {
		for (int j = 0; j < channelNum; ++j) {
			for (int k = 0; k < pointNum * 2; ++k) {
				fread(&tp, sizeof(short), 1, fps[j]);
				if (k % 2 == 0){
					correction[start++] = (double)tp / 100.0;
					correctionVec.push_back((double)tp / 100.0);
				}
				else{
					correction[start++] = (double)tp / 8192.0;
					correctionVec.push_back((double)tp / 8192.0);
				}
			}
		}
	}

	vector<double> dataVec;
	std::vector<double> dataVecAll;
	std::vector<double> specVec(pointNum, -9999.0);

	start = 0;
	for (int n = 0; n < fn; ++n) {
		for (int i = 0; i < 1; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					if (j == 2 && k % 2 == 0){
						specVec[k / 2] = (double)tp / 100.0;
					}
				}
			}
		}
		// for (int kk = 0; kk < 10; kk++)
		// 	cout << specVec[1020 + kk] << endl;
		
		for (int i = 0; i < cutNum; ++i) {
			for (int j = 0; j < channelNum; ++j) {
				for (int k = 0; k < pointNum * 2; ++k) {
					fread(&tp, sizeof(short), 1, fps[j]);
					// data[start++] = tp;
					if (k % 2 == 0){
						data[start++] = (double)tp / 100.0;
						dataVec.push_back((double)tp / 100.0);
						dataVecAll.push_back((double)tp / 100.0);
					}
					else{
						data[start++] = (double)tp / 8192.0;
						dataVecAll.push_back((double)tp / 8192.0);
					}
				}
			}
		}
	}


	for (int i = 0; i < channelNum; ++i) {
		fclose(fps[i]);
	}


	// 构建频谱数据 [1, 2, 3 // 4, 5, 3 // 7, 5, 6 // 1, 7, 8]
	// std:vector<std::vector<double>> ampAll(antennaNum, std::vector<double>(pointNum));
	// std::vector<double> specVec(pointNum, -9999.0);

	// for (int i = 0; i < pointNum; i++){
	// 	for (int j = 0; j < cutNum * channelNum; j++){
	// 		int curIndex = j * pointNum + i;
	// 		if (specVec[i] < dataVec[curIndex]){
	// 			specVec[i] = dataVec[curIndex];
	// 		}
	// 	}
	// }

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;
	int angleNum;
	double angles[pointNum];
	double qualities[pointNum];
	double amplitudes[pointNum];
	int frequencies[pointNum];

	// 创建对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}


	// type : 天线类型 - 0:垂直-频谱 1:水平-频谱 2:垂直-研发 3:水平-研发
	// 初始化对象
	if (ret = Init_AS(id, 0, f, r)) {
		return ret;
	}

	// 设置模版路径
	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	// 设置FFT门限
	if (ret = SetThresholdFFT_AS(id, threshold)) {
		return ret;
	}

	// 设置校正数据
	if (ret = SetCorrectionFFT_AS(id, correction, length)) {
		return ret;
	}

	// 设置FFT数据
	if (ret = SetDataFFT_AS(id, data, length)) {
		return ret;
	}

	// 设置频谱数据
	if (ret = SetAmplitudeDataFFT_AS(id, &specVec[0], pointNum)) {
		return ret;
	}

	// 获取FFT测向结果
	if (ret = GetAngleFFT_AS(id, angleNum, angles, frequencies)) {
		return ret;
	}

	// 打印结果
	for (int i = 0; i < angleNum; i++) {
		cout << "angle=" << angles[i] << ", frequency=" << frequencies[i] << endl;
	}

	// 释放内存
	free(data);
	free(correction);
	Release_AS(id);

	return 0;
}


// 定向天线8-26.5G
int main111(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	// string corDirName = "./data/ch2/IQ/Dire/test/1200.000000_125.000000_0.dat";
	// string doaDirName = "./data/ch2/IQ/Dire/test/1200.000000_125.000000_1.dat";
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];
	// string modelDir = "./data/platformII-V-1-4.dat";
	// string modelDir = "./data/platformII-H-1-4.dat";
	// string modelDir = "./data/platformII-V-3-8.dat";
	// string modelDir = "./data/platformII-H-3-8.dat";
	string modelDir = "./data/8_26dot5_model.dat";
	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* corFp = fopen(corDirName.c_str(), "rb");
	FILE* doaFp = fopen(doaDirName.c_str(), "rb");
	string fileName;

	vector<double> correctionVec;
	short tp = 0;
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, corFp);
		correction[i] = tp;
	}

	vector<double> dataVec;
	for (int i = 0; i < length * fn; ++i) {
		fread(&tp, sizeof(short), 1, doaFp);
		data[i] = tp;
	}

	fclose(corFp);
	fclose(doaFp);

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;
	double angles[10];
	double qualities[10];


	// 创建测向对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_AS(id, antType, f, r)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_AS(id, at)){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// 测向
		double amplitudes[10];
		int* newCut;
		int* newCutAntCodes;
		int antCode;
		if (ret = AI_AS(id, 1, angles, qualities, amplitudes, newCut, newCutAntCodes, antCode)){
			return ret;
		}
		// if (ret = AmplitutdePhaseAlg_AS(id, angles, qualitys));

	}


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}


int main112(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	// string corDirName = "./data/ch2/IQ/Dire/test/1200.000000_125.000000_0.dat";
	// string doaDirName = "./data/ch2/IQ/Dire/test/1200.000000_125.000000_1.dat";
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];
	// string modelDir = "./data/platformII-V-1-4.dat";
	// string modelDir = "./data/platformII-H-1-4.dat";
	// string modelDir = "./data/platformII-V-3-8.dat";
	// string modelDir = "./data/platformII-H-3-8.dat";
	string modelDir;
	if (f < 8000e6){
		modelDir = "./data/platformI-3-8.dat";
	}
	else{
		modelDir = "./data/platformI-8-18.dat"; 
	}
	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* corFp = fopen(corDirName.c_str(), "rb");
	FILE* doaFp = fopen(doaDirName.c_str(), "rb");
	string fileName;

	vector<double> correctionVec;
	short tp = 0;
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, corFp);
		correction[i] = tp;
	}

	vector<double> dataVec;
	for (int i = 0; i < length * fn; ++i) {
		fread(&tp, sizeof(short), 1, doaFp);
		data[i] = tp;
	}

	fclose(corFp);
	fclose(doaFp);

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;

	// 创建测向对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_AS(id, antType, f, r)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_AS(id, at)){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// 测向
		if (ret = AmplitutdePhaseAlg_AS(id, angle, quality)){
			return ret;
		}
		// if (ret = AmplitutdePhaseAlg_AS(id, angles, qualitys));

	}


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}


int main113(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	// 6 - V, 7 - H
	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];

	string modelDir;

	// if (antType == 6 && f < 3000e6)
	// 	// modelDir = "./data/platformII-V-1-4.dat";
	// 	modelDir = "./data/1_3G_V_model_YF.dat";
	// else if(antType == 6 && f >= 3000e6)
	// 	// modelDir = "./data/platformII-V-3-8.dat";
	// 	modelDir = "./data/3_8G_V_model_YF.dat";
	// else if(antType == 7 && f < 3000e6)
	// 	// modelDir = "./data/platformII-H-1-4.dat";
	// 	modelDir = "./data/1_3G_H_model_YF.dat";
	// else if(antType == 7 && f >= 3000e6)
	// 	modelDir = "./data/platformII-H-3-8.dat";
	// string modelDir = "./data/platformII-V-1-4.dat";
	// string modelDir = "./data/platformII-H-1-4.dat";
	// string modelDir = "./data/platformII-V-3-8.dat";
	modelDir = jsonData["modelDir"];

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* corFp = fopen(corDirName.c_str(), "rb");
	FILE* doaFp = fopen(doaDirName.c_str(), "rb");
	string fileName;

	vector<double> correctionVec;
	short tp = 0;
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, corFp);
		correction[i] = tp;
		// correction[i] = 0;

	}

	vector<double> dataVec;
	for (int i = 0; i < length * fn; ++i) {
		fread(&tp, sizeof(short), 1, doaFp);
		data[i] = tp;
		// data[i] = 0;
	}

	fclose(corFp);
	fclose(doaFp);

	//调用算法接口
	int id;
	int ret;
	double angle;
	double quality;

	// 创建测向对象

	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_AS(id, antType, f, r)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_AS(id, at)){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// 测向
		if (ret = AmplitutdePhaseAlg_AS(id, angle, quality)){
			return ret;
		}
		// if (ret = AmplitutdePhaseAlg_AS(id, angles, qualitys));

	}


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}

int main114(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	// 2 - V, 3 - H
	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];

	string modelDir;
	if (f < 3000e6){
		modelDir = "./data/1_3G_V_model_YF.dat";
	}
	else{
		modelDir = "./data/3_8G_V_model_YF.dat";
	}

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* corFp = fopen(corDirName.c_str(), "rb");
	FILE* doaFp = fopen(doaDirName.c_str(), "rb");
	string fileName;

	vector<double> correctionVec;
	short tp = 0;
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, corFp);
		correction[i] = tp;
	}

	vector<double> dataVec;
	for (int i = 0; i < length * fn; ++i) {
		fread(&tp, sizeof(short), 1, doaFp);
		data[i] = tp;
	}

	fclose(corFp);
	fclose(doaFp);

	//调用算法接口
	int id;
	int ret;
	double angles[10];
	double qualitys[10];
	double amptitudes[10];
	int newcut[10];
	int newCUtAntCodes[10];
	int antCode;

	// 创建测向对象

	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_AS(id, antType, f, r)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_AS(id, at)){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}

	// 设置dbfflag
	if (ret = SetDbfFlag_AS(id, 0)){
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// 测向
		// if (ret = AmplitutdePhaseAlg_AS(id, angle, quality)){
		// 	return ret;
		// }

		if (ret = AI_AS(id, sigNum, angles, qualitys, amptitudes, newcut, newCUtAntCodes, antCode)){
			return ret;
		}
		// if (ret = AmplitutdePhaseAlg_AS(id, angles, qualitys));

	}


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}

int main139(json jsonData) {
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	// 6 - V, 7 - H
	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];

	string modelDir;

	// if (antType == 6 && f < 3000e6)
	// 	// modelDir = "./data/platformII-V-1-4.dat";
	// 	modelDir = "./data/1_3G_V_model_YF.dat";
	// else if(antType == 6 && f >= 3000e6)
	// 	// modelDir = "./data/platformII-V-3-8.dat";
	// 	modelDir = "./data/3_8G_V_model_YF.dat";
	// else if(antType == 7 && f < 3000e6)
	// 	// modelDir = "./data/platformII-H-1-4.dat";
	// 	modelDir = "./data/1_3G_H_model_YF.dat";
	// else if(antType == 7 && f >= 3000e6)
	// 	modelDir = "./data/platformII-H-3-8.dat";
	// string modelDir = "./data/platformII-V-1-4.dat";
	// string modelDir = "./data/platformII-H-1-4.dat";
	// string modelDir = "./data/platformII-V-3-8.dat";
	modelDir = jsonData["modelDir"];

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* corFp = fopen(corDirName.c_str(), "rb");
	FILE* doaFp = fopen(doaDirName.c_str(), "rb");
	string fileName;

	vector<double> correctionVec;
	short tp = 0;
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, corFp);
		correction[i] = tp;
		// correction[i] = 0;

	}

	vector<double> dataVec;
	for (int i = 0; i < length * fn; ++i) {
		fread(&tp, sizeof(short), 1, doaFp);
		data[i] = tp;
		// data[i] = 0;
	}

	fclose(corFp);
	fclose(doaFp);

	//调用算法接口
	int id;
	int ret;
	double angles[9];
	double qualitys[9];

	// 创建测向对象

	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// 初始化测向对象
	if (ret = Init_AS(id, antType, f, r)) {
		return ret;
	}

	// 设置积分时间
	if (ret = SetAverageTime_AS(id, at)){
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}
	
	for (size_t i = 0; i < fn; i++) {
		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// 测向
		if (ret = Coherent_AS(id, sigNum, angles, qualitys)){
			return ret;
		}
		// if (ret = AmplitutdePhaseAlg_AS(id, angles, qualitys));

	}


	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}

// 九通道 定向天线 空域滤波
int main141(json jsonData){
	//设置参数
	const int antennaNum = jsonData["antennaNum"];
	const int channelNum = jsonData["channelNum"];
	const int pointNum = jsonData["pointNum"];
	const int cutNum = jsonData["cutNum"];
	const int length = cutNum * channelNum * pointNum * 2;
	double f = jsonData["frequency"];
	f *= 1e6;

	// 6 - V, 7 - H
	int antType = jsonData["antType"];
	double r = jsonData["r"];

	int sigNum = jsonData["sigNum"];
	vector<int> thwVec = jsonData["thw"];
	int* thw = &thwVec[0];
	
	string corDirName = jsonData["corDirName"];
	string doaDirName = jsonData["doaDirName"];

	std::cout << "corDirName: " << corDirName << std::endl;
	std::cout << "doaDirName: " << doaDirName << std::endl;

	string modelDir;

	// if (antType == 6 && f < 3000e6)
	// 	// modelDir = "./data/platformII-V-1-4.dat";
	// 	modelDir = "./data/1_3G_V_model_YF.dat";
	// else if(antType == 6 && f >= 3000e6)
	// 	// modelDir = "./data/platformII-V-3-8.dat";
	// 	modelDir = "./data/3_8G_V_model_YF.dat";
	// else if(antType == 7 && f < 3000e6)
	// 	// modelDir = "./data/platformII-H-1-4.dat";
	// 	modelDir = "./data/1_3G_H_model_YF.dat";
	// else if(antType == 7 && f >= 3000e6)
	// 	modelDir = "./data/platformII-H-3-8.dat";
	// string modelDir = "./data/platformII-V-1-4.dat";
	// string modelDir = "./data/platformII-H-1-4.dat";
	// string modelDir = "./data/platformII-V-3-8.dat";
	modelDir = jsonData["modelDir"];

	int fn = jsonData["calculationTime"];	// 计算次数
	int beginCutNum = jsonData["beginCutNum"];   // 前抛点数
	int endCutNum = jsonData["endCutNum"];     // 后抛点数
	double at = jsonData["at"];	// 积分时间

	//读取数据
	double* correction = (double*)malloc(sizeof(double) * length);
	double* data = (double*)malloc(sizeof(double) * length * fn);

	FILE* corFp = fopen(corDirName.c_str(), "rb");
	FILE* doaFp = fopen(doaDirName.c_str(), "rb");
	string fileName;

	vector<double> correctionVec;
	short tp = 0;
	for (int i = 0; i < length; ++i) {
		fread(&tp, sizeof(short), 1, corFp);
		correction[i] = tp;
		// correction[i] = 0;

	}

	vector<double> dataVec;
	for (int i = 0; i < length * fn; ++i) {
		fread(&tp, sizeof(short), 1, doaFp);
		data[i] = tp;
		// data[i] = 0;
	}

	fclose(corFp);
	fclose(doaFp);

	//调用算法接口
	int id;
	int ret;
	double angles[9];
	double qualitys[9];
	double amplitudes[9];
	int newCut[3];
	int newCutAntCodes[14];
	int antCode;
	int tmpValue[3];
	double angle;
	double quality;

	// 创建测向对象
	if (ret = Create_AS(id, channelNum, antennaNum, pointNum)) {
		return ret;
	}

	// type : 天线类型 - 0:垂直-频谱 1:水平-频谱 2:垂直-研发 3:水平-研发
	// 初始化测向对象
	if (ret = Init_AS(id, 1, f, r)) {
		return ret;
	}

	// 设置切刀方式
	if (ret = SetThw_AS(id, cutNum, thw, cutNum * channelNum)) {
		return ret;
	}

	if (ret = SetModel_AS(id, modelDir.c_str())){
		return ret;
	}
	
	// 设置抛点数量
	if (ret = SetThrowPointNum_AS(id, beginCutNum, endCutNum)) {
		return ret;
	}

	// 计算校正数据
	if (ret = SetCorrection_AS(id, correction, length)) {
		return ret;
	}

	ret = SetDbfFlag_AS(id, 1);
			
	bool flag;
	complex<double> coeef[100];

	for (size_t i = 0; i < fn; i++) {
		// if (i == fn - 1){
		// 	cout << endl;
		// }

		// 导入数据
		if (ret = SetData_AS(id, data + length * i, length)) {
			return ret;
		}

		// AI测向
		if (ret = AI_AS(id, sigNum, angles, qualitys, amplitudes, newCut, newCutAntCodes, antCode)){
			return ret;
		}

		if (ret = DBF_AS(id, sigNum, coeef, flag)){
			return ret;
		}

	}
	
	//清理
	free(correction);
	free(data);
	Release_AS(id);

	return 0;
}


// =============================================================================
// 卫导 GN902 欺骗检测与测向测试
// 解析 K827 双端口 NovAtel Msg43 dat + 配套 debug 切刀时刻表，构造与欺骗检测/测向
// 流程传入数据结构一致的 GNSSData，并流式喂入 GN902 引擎验证 C++ 算法。
//
// 用法（usedConfig.json）:
//   "method": 90, "method type": 0
//   "datDir"          : K827Data_0/1_*.dat 所在目录（与 port0File/port1File 二选一）
//   "port0File"       : 端口0 dat 文件（可选，优先于 datDir）
//   "port1File"       : 端口1 dat 文件（可选，优先于 datDir）
//   "debugFile"       : 切刀时刻表 debug 文件（含 OpenAntenna code 标记）
//   "logFile"         : 结果日志输出路径（默认 ./gn902_result.log）
//   "switchSeconds"   : 每刀保留末尾稳定秒数（默认 1，与 Python 流程对齐）
//   可选阈值覆盖（不设则用引擎默认，与 Python 硬编码阈值一致）:
//   "phsDiffThreshold" / "satelliteCountThreshold" / "cutCountThreshold" /
//   "sysEnum" / "typeEnum"
// =============================================================================

// 同时输出到命令行与日志文件
static void gn902LogLine(FILE* logFp, const char* fmt, ...)
{
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
    if (logFp)
    {
        va_start(args, fmt);
        vfprintf(logFp, fmt, args);
        va_end(args);
        fflush(logFp);
    }
}

// 打印/记录一轮测向结果
static void gn902LogResult(FILE* logFp, int round, const SpoofingResult& r)
{
    gn902LogLine(logFp, "\n---- 测向轮 %d: 报警频点数 = %d ----\n", round, r.i_Count);
    for (int i = 0; i < r.i_Count; ++i)
    {
        const SatelliteAngle& sa = r.i_SatelliteAngle[i];
        gn902LogLine(logFp, "  频点 Sys=%d Type=%d Alarm=%d 来向角度=%.2f° 被欺骗卫星数=%d\n",
                     sa.i_Sys, sa.i_Type, sa.i_Alarm, sa.i_Angle, sa.i_Count);
        for (int j = 0; j < sa.i_Count; ++j)
        {
            const AlarmData& ad = sa.i_AlarmData[j];
            gn902LogLine(logFp, "    卫星 Prn=%d Snr=%.1f Angle=%d Quality=%.2f\n",
                         ad.i_Prn, ad.i_Snr, ad.i_Angle, ad.i_Quality);
        }
    }
}

int main902(json jsonData)
{
    using namespace gn902test;

    // ---- 1. 读取配置 ----
    std::string datDir, port0File, port1File, debugFile, logFile;
    if (jsonData.count("datDir"))    datDir    = jsonData["datDir"].get<std::string>();
    if (jsonData.count("port0File")) port0File = jsonData["port0File"].get<std::string>();
    if (jsonData.count("port1File")) port1File = jsonData["port1File"].get<std::string>();
    if (jsonData.count("debugFile")) debugFile = jsonData["debugFile"].get<std::string>();
    logFile = jsonData.count("logFile") ? jsonData["logFile"].get<std::string>() : "./gn902_result.log";
    int switchSeconds = jsonData.count("switchSeconds") ? jsonData["switchSeconds"].get<int>() : 1;

    // 固定基线采集模式(可选): 配置里提供 fixedPair=[通道1天线,通道2天线] 时，
    // 不按 debug 切刀时刻表做循环切刀，而是把每个公共 GPS 秒作为该固定天线对
    // 喂入 SetData_GN902(仅采集相位差，不检测/测向)。
    std::vector<int> fixedPair;
    if (jsonData.count("fixedPair") && jsonData["fixedPair"].is_array())
    {
        for (auto &v : jsonData["fixedPair"])
        {
            fixedPair.push_back(v.get<int>());
        }
    }
    bool collectMode = (fixedPair.size() == 2);
    if (collectMode && !((fixedPair[0] == 8 && fixedPair[1] == 9) || (fixedPair[0] == 9 && fixedPair[1] == 8)))
    {
        printf("[GN902] fixedPair 仅支持 [8,9] 或 [9,8]（当前为 [%d,%d]）\n", fixedPair[0], fixedPair[1]);
        return 1;
    }

    // 定位 dat 文件
    if (port0File.empty() || port1File.empty())
    {
        if (datDir.empty())
        {
            printf("[GN902] 需提供 datDir 或 port0File/port1File\n");
            return 1;
        }
        if (!findPortDatFiles(datDir, port0File, port1File))
        {
            printf("[GN902] 目录缺少 K827Data_0/1_*.dat: %s\n", datDir.c_str());
            return 1;
        }
    }
    if (!collectMode && debugFile.empty())
    {
        printf("[GN902] 需提供 debugFile\n");
        return 1;
    }

    printf("================================================================\n");
    printf("GN902 欺骗检测与测向（解析 K827 双端口 dat 流式喂入）\n");
    printf("  端口0 dat : %s\n", port0File.c_str());
    printf("  端口1 dat : %s\n", port1File.c_str());
    printf("  debug     : %s\n", debugFile.c_str());
    printf("  日志      : %s\n", logFile.c_str());
    printf("  模式      : %s\n", collectMode ? "固定基线采集(相位差)" : "循环切刀检测+测向");
    if (collectMode)
    {
        printf("  固定天线对: (%d,%d)\n", fixedPair[0], fixedPair[1]);
    }
    printf("================================================================\n");

    // ---- 2. 打开结果日志 ----
    FILE* logFp = fopen(logFile.c_str(), "w");

    // ---- 3. 解析两端口 dat ----
    std::map<double, std::vector<SatelliteData>> port0 = parseDatPort(port0File);
    std::map<double, std::vector<SatelliteData>> port1 = parseDatPort(port1File);
    gn902LogLine(logFp, "端口0 帧数: %d, 端口1 帧数: %d\n", (int)port0.size(), (int)port1.size());

    // ---- 4. 公共 GPS 秒（两端口同时出现的时刻）----
    std::vector<double> common;
    for (auto& kv : port0) if (port1.count(kv.first)) common.push_back(kv.first);
    std::sort(common.begin(), common.end());

    // ---- 5. 生成喂帧序列 ----
    std::vector<std::pair<double, int>> feedFrames;   // (sec, code) 时间升序
    if (collectMode)
    {
        // 固定基线采集: 每个公共 GPS 秒都作为该固定天线对喂入(code 仅占位)
        for (double sec : common)
        {
            feedFrames.push_back({ sec, 0 });
        }
        gn902LogLine(logFp, "固定基线采集喂帧: %d 帧(天线对 %d,%d)\n",
                     (int)feedFrames.size(), fixedPair[0], fixedPair[1]);
    }
    else
    {
        // ---- 解析切刀时刻表 ----
        std::vector<std::pair<double, int>> schedule = parseDebugSchedule(debugFile);
        gn902LogLine(logFp, "切刀切换点: %d 个\n", (int)schedule.size());

        // 赋 code
        std::vector<std::pair<double, int>> tagged;   // (sec, code)
        for (double sec : common)
        {
            int code = activeCode(sec, schedule);
            if (code < 0) continue;
            tagged.push_back({ sec, code });
        }

        // 按 code 分刀(runs)，每刀去末尾过渡秒、保留末尾 switchSeconds 个稳定秒
        struct Run { int code; std::vector<double> secs; };
        std::vector<Run> runs;
        for (auto& t : tagged)
        {
            if (!runs.empty() && runs.back().code == t.second) runs.back().secs.push_back(t.first);
            else runs.push_back({ t.second, { t.first } });
        }
        int settleN = (switchSeconds > 0) ? switchSeconds : 1;
        for (auto& run : runs)
        {
            std::vector<double> settled;
            if (run.secs.size() > 1) settled.assign(run.secs.begin(), run.secs.end() - 1);
            else settled = run.secs;
            int start = std::max(0, (int)settled.size() - settleN);
            for (int i = start; i < (int)settled.size(); ++i) feedFrames.push_back({ settled[i], run.code });
        }
        gn902LogLine(logFp, "检测喂帧: %d 帧\n", (int)feedFrames.size());
    }

    // ---- 7. 创建 GN902 对象 ----
    int id = 0, ret = 0;
    if ((ret = Create_GN902(id)) != 0)
    {
        gn902LogLine(logFp, "[GN902] Create_GN902 失败 ret=%d\n", ret);
        if (logFp) fclose(logFp);
        return ret;
    }

    // 可选阈值覆盖（不设则用引擎默认，与 Python 硬编码阈值一致）
    if (jsonData.count("phsDiffThreshold") && jsonData.count("satelliteCountThreshold"))
    {
        double phsTh = jsonData["phsDiffThreshold"].get<double>();
        double satTh = jsonData["satelliteCountThreshold"].get<double>();
        double cutTh = jsonData.count("cutCountThreshold") ? jsonData["cutCountThreshold"].get<double>() : 0.0;
        int sys  = jsonData.count("sysEnum")  ? jsonData["sysEnum"].get<int>()  : -1;
        int type = jsonData.count("typeEnum") ? jsonData["typeEnum"].get<int>() : -1;
        ret = SetThresholdDetection_GN902(id, phsTh, satTh, cutTh, sys, type);
        if (ret != 0) gn902LogLine(logFp, "[GN902] SetThresholdDetection_GN902 失败 ret=%d\n", ret);
    }

    // ---- 8. 流式喂入 ----
    if (collectMode)
    {
        // 固定基线采集: 每个公共 GPS 秒作为固定天线对喂入，仅采集相位差(写入日志)
        int collectCount = 0;
        for (auto& fr : feedFrames)
        {
            auto it0 = port0.find(fr.first);
            auto it1 = port1.find(fr.first);
            GNSSData frame = buildGnssData(
                it0 != port0.end() ? it0->second : std::vector<SatelliteData>(),
                it1 != port1.end() ? it1->second : std::vector<SatelliteData>());
            SetData_GN902(id, &frame, fixedPair[0], fixedPair[1]);
            collectCount++;
        }
        gn902LogLine(logFp, "固定基线采集完成: 共 %d 帧(逐星相位差见 ./gn902_phasediff.log)\n", collectCount);
    }
    else
    {
        // 一轮 = 校正刀(1,1) + 六刀测向(1,2)..(1,7)；校正刀再次出现触发上一轮检测+测向。
        // 缺刀的轮次（如末尾不完整周期）会被引擎丢弃，与 Python 流程一致。
        int roundIdx = 0;
        int doaMask = 0;
        bool inRound = false;
        int totalAlarmCount = 0;

        for (auto& fr : feedFrames)
        {
            int cutIdx2 = codeToPair(fr.second);
            if (cutIdx2 < 1 || cutIdx2 > 7) continue;

            auto it0 = port0.find(fr.first);
            auto it1 = port1.find(fr.first);
            GNSSData frame = buildGnssData(
                it0 != port0.end() ? it0->second : std::vector<SatelliteData>(),
                it1 != port1.end() ? it1->second : std::vector<SatelliteData>());

            if (cutIdx2 == 1)
            {
                // 校正刀：先喂（触发上一轮检测+测向），再结算上一轮结果
                SetData_GN902(id, &frame, 1, 1);
                if (inRound && (doaMask & 0xFC) == 0xFC)
                {
                    SpoofingResult r;
                    GetResult_GN902(id, r);
                    gn902LogResult(logFp, roundIdx, r);
                    totalAlarmCount += r.i_Count;
                    roundIdx++;
                }
                inRound = true;
                doaMask = 0;
            }
            else
            {
                SetData_GN902(id, &frame, 1, cutIdx2);
                inRound = true;
                doaMask |= (1 << cutIdx2);
            }
        }

        // 末尾完整轮：用一帧空校正触发上一轮检测+测向
        if (inRound && (doaMask & 0xFC) == 0xFC)
        {
            GNSSData dummyCal;
            memset(&dummyCal, 0, sizeof(dummyCal));
            SetData_GN902(id, &dummyCal, 1, 1);
            SpoofingResult r;
            GetResult_GN902(id, r);
            gn902LogResult(logFp, roundIdx, r);
            totalAlarmCount += r.i_Count;
            roundIdx++;
        }

        // ---- 9. 汇总 ----
        gn902LogLine(logFp, "\n================================================================\n");
        gn902LogLine(logFp, "GN902 检测完成: 处理 %d 个完整测向轮, 共 %d 个报警频点\n", roundIdx, totalAlarmCount);
        gn902LogLine(logFp, "================================================================\n");
    }

    Release_GN902(id);
    if (logFp) fclose(logFp);
    return 0;
}
