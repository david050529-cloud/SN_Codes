#include "gn902_interface.h"
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

int main902(json jsonData);

int main()
{
	std::ifstream f("usedConfig.json");
	if (!f.is_open())
	{
		cout << "Error: Failed to open config.json file." << endl;
		return 1;
	}
	json jsonData;
	f >> jsonData;
	int method = jsonData["method"];
	int type = jsonData["method type"];

	// 只有902模式
	switch (method)
	{
	case 90:
		main902(jsonData);
		break;
	default:
		break;
	}
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
//   "getResultFile"   : ★ 新增 ★ GetResult_GN902 返回结果专用保存路径
//                       （默认 ./gn902_getresult.txt）。该文件只存放
//                       GetResult_GN902 的返回值快照，与 logFile 完全分离。
//   "switchSeconds"   : 每刀保留末尾稳定秒数（默认 1，与 Python 流程对齐）
//   可选阈值覆盖（不设则用引擎默认，与 Python 硬编码阈值一致）:
//   "phsDiffThreshold" / "satelliteCountThreshold" / "sysEnum" / "typeEnum"
//   "cutCountThreshold": 连续切刀数(连续报警确认次数)，由 SetCutnumThreshold_GN902
//                        接口单独传入(不再走 txt 配置文件)
// =============================================================================

// 同时输出到命令行与日志文件。
// printf 格式检查(GCC/Clang): 让编译器核对实参与格式串是否匹配 —— 没有它,
// 类型不匹配只会静默打印错位值(AlarmData::i_Angle 由 double 改 int 后就踩过一次:
// Angle 列打印出质量分、Quality 列打印出寄存器残留)。
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 2, 3)))
#endif
static void gn902LogLine(FILE *logFp, const char *fmt, ...)
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

// =============================================================================
// ★ 新增: GetResult_GN902 返回值专用保存
// -----------------------------------------------------------------------------
// 每次调用 GetResult_GN902 成功后, 把返回的 SpoofingResult 独立写入专用文件,
// 与主结果日志(gn902_result.log)完全分离, 便于单独分析/比对。
// 文件内容采用逐字段可读格式, 一行一条, 方便 diff / 后续脚本处理。
// =============================================================================

// 在专用文件中写一行(带时间戳前缀)
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 2, 3)))
#endif
static void gn902GetResultLine(FILE *fp, const char *fmt, ...)
{
	if (fp == 0) return;

	// 时间戳
	std::time_t t = std::time(0);
	std::tm *lt = std::localtime(&t);
	char ts[32] = {0};
	if (lt != 0)
	{
		std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", lt);
	}
	fprintf(fp, "[%s] ", ts);

	va_list args;
	va_start(args, fmt);
	vfprintf(fp, fmt, args);
	va_end(args);
	fflush(fp);
}

// 把 SpoofingResult 单独保存到专用文件。
// @param fp       专用文件句柄(由 main902 打开, 追加模式)
// @param result   库写入的结果结构体
// @param roundIdx 轮序号(从 1 开始, 每完成一轮 +1)
// @param roundSec 本轮最后一刀时刻(GPS 秒, 0=未知)
static void gn902SaveSpoofingResult(FILE *fp, const SpoofingResult &result,
                                    int roundIdx, double roundSec)
{
	if (fp == 0)
	{
		return;
	}

	gn902GetResultLine(fp,
		"==== Round=%d  RoundSec=%.1f  AlarmFreqCount=%d ====\n",
		roundIdx, roundSec, result.i_Count);

	// 越界夹紧: 库最多写 24 个频点 / 每频点最多 32 颗卫星
	int freqNum = result.i_Count;
	if (freqNum > 24) freqNum = 24;
	if (freqNum < 0)  freqNum = 0;

	for (int i = 0; i < freqNum; ++i)
	{
		const SatelliteAngle &sa = result.i_SatelliteAngle[i];
		gn902GetResultLine(fp,
			"  Freq[%d]: Sys=%s(%d) Type=%s(%d) Alarm=%d Angle=%.2f SatCount=%d\n",
			i,
			GetSysName(sa.i_Sys), sa.i_Sys,
			GetTypeName(sa.i_Sys, sa.i_Type), sa.i_Type,
			sa.i_Alarm, sa.i_Angle, sa.i_Count);

		int satNum = sa.i_Count;
		if (satNum > 32) satNum = 32;
		if (satNum < 0)  satNum = 0;

		for (int j = 0; j < satNum; ++j)
		{
			const AlarmData &ad = sa.i_AlarmData[j];
			// AlarmData::i_Angle 是 int(度), 不能用 %f 打印
			gn902GetResultLine(fp,
				"    Sat[%d]: Prn=%d Snr=%.1f Angle=%d Quality=%.2f\n",
				j, ad.i_Prn, ad.i_Snr, ad.i_Angle, ad.i_Quality);
		}
	}
	gn902GetResultLine(fp, "----\n");
}

// 打印/记录一轮内的全部"报警时刻"结果。
// 输出单位是"某切刀时刻某频点是否给出报警"：给出报警才打印，不再按整轮罗列
// 已跟踪频点(未给出报警的频点不再输出 -1 占位)。
// @param cutSec  本轮各切刀时刻(GPS秒), 索引 = 通道2天线号(1..7)
// @param cutCode 本轮各切刀的原始 code, 索引同上
static void gn902LogAlarmMoments(FILE *logFp, const std::vector<AlarmMoment> &moments,
								 const double *cutSec, const int *cutCode, int &totalFreq)
{
	for (size_t i = 0; i < moments.size(); ++i)
	{
		const AlarmMoment &am = moments[i];
		int pair2 = am.i_Cut + 1; // 切刀序号(1..6) -> 通道2天线号 2..7
		double sec = 0.0;
		int code = -1;
		if (cutSec != 0 && pair2 >= 2 && pair2 <= 7)
		{
			sec = cutSec[pair2];
			code = (cutCode != 0) ? cutCode[pair2] : -1;
		}

		gn902LogLine(logFp, "\n---- 报警时刻 code=%d 天线对(1,%d) t=%.1fs ----\n", code, pair2, sec);
		if (am.i_Angle < 0)
		{
			// 该时刻尚未测出来向角度 → 角度输出 -1；欺骗卫星数照常给出，不为 0
			gn902LogLine(logFp, "  频点 Sys=%s Type=%s Alarm=%d 来向角度=-1 被欺骗卫星数=%d\n",
						 GetSysName(am.i_Sys), GetTypeName(am.i_Sys, am.i_Type), am.i_Alarm, am.i_Count);
		}
		else
		{
			gn902LogLine(logFp, "  频点 Sys=%s Type=%s Alarm=%d 来向角度=%.2f° 被欺骗卫星数=%d\n",
						 GetSysName(am.i_Sys), GetTypeName(am.i_Sys, am.i_Type), am.i_Alarm, am.i_Angle, am.i_Count);
		}
		for (int j = 0; j < am.i_Count; ++j)
		{
			const AlarmData &ad = am.i_AlarmData[j];
			if (ad.i_Angle < 0)
			{
				gn902LogLine(logFp, "    卫星 Prn=%d Snr=%.1f Angle=-1 Quality=-1\n",
							 ad.i_Prn, ad.i_Snr);
				continue;
			}
			// 注意: AlarmData::i_Angle 是 int(度), 不能用 %f 打印(变参类型不匹配)
			gn902LogLine(logFp, "    卫星 Prn=%d Snr=%.1f Angle=%d Quality=%.2f\n",
						 ad.i_Prn, ad.i_Snr, ad.i_Angle, ad.i_Quality);
		}
		totalFreq++;
	}
}

// 本轮最后一刀(测向刀)的时刻(GPS秒), 0=本轮没有测向刀时刻。
// 频点级结果不带时刻, 用它标识"这一轮"。
static double gn902LastCutSec(const double *cutSec)
{
	double last = 0.0;
	for (int i = 2; i <= 7; ++i)
	{
		if (cutSec[i] > last)
			last = cutSec[i];
	}
	return last;
}

// 打印/记录一次 GetResult_GN902 取回的频点级测向结果(每完成一轮取一次)。
// 这是对外 C 接口的正式取结果方式: 宿主在自己的栈上定义 SpoofingResult,
// 由库按引用整体写入 —— 需要两端 ALG/头文件同一次构建(见 main902 开头的 ABI 自检)。
// @param result   库写入的结果结构体
// @param roundSec 本轮最后一刀时刻(GPS秒, 0=未知)
static void gn902LogSpoofingResult(FILE *logFp, const SpoofingResult &result, double roundSec)
{
	gn902LogLine(logFp, "\n==== GetResult_GN902: 报警频点数=%d (本轮最后一刀 t=%.1fs) ====\n",
				 result.i_Count, roundSec);
	if (result.i_Count <= 0)
	{
		gn902LogLine(logFp, "  (本轮无报警频点)\n");
		return;
	}
	// 库最多写 24 个频点 / 每频点最多 32 颗卫星; 越界即为 ABI 不一致的信号,
	// 这里夹住不下溢, 免得把宿主读崩。
	if (result.i_Count > 24)
	{
		gn902LogLine(logFp, "  [警告] i_Count=%d 超出数组长度 24, 只输出前 24 个\n", result.i_Count);
	}
	int freqNum = (result.i_Count > 24) ? 24 : result.i_Count;
	for (int i = 0; i < freqNum; ++i)
	{
		const SatelliteAngle &sa = result.i_SatelliteAngle[i];
		if (sa.i_Angle < 0)
		{
			gn902LogLine(logFp, "  频点 Sys=%s Type=%s Alarm=%d 来向角度=-1 被欺骗卫星数=%d\n",
						 GetSysName(sa.i_Sys), GetTypeName(sa.i_Sys, sa.i_Type), sa.i_Alarm, sa.i_Count);
		}
		else
		{
			gn902LogLine(logFp, "  频点 Sys=%s Type=%s Alarm=%d 来向角度=%.2f° 被欺骗卫星数=%d\n",
						 GetSysName(sa.i_Sys), GetTypeName(sa.i_Sys, sa.i_Type), sa.i_Alarm, sa.i_Angle, sa.i_Count);
		}

		if (sa.i_Count > 32 || sa.i_Count < 0)
		{
			gn902LogLine(logFp, "  [警告] 该频点卫星数=%d 超出数组长度 32\n", sa.i_Count);
		}
		int satNum = (sa.i_Count > 32) ? 32 : ((sa.i_Count < 0) ? 0 : sa.i_Count);
		for (int j = 0; j < satNum; ++j)
		{
			const AlarmData &ad = sa.i_AlarmData[j];
			if (ad.i_Angle < 0)
			{
				gn902LogLine(logFp, "    卫星 Prn=%d Snr=%.1f Angle=-1 Quality=-1\n",
							 ad.i_Prn, ad.i_Snr);
				continue;
			}
			gn902LogLine(logFp, "    卫星 Prn=%d Snr=%.1f Angle=%d Quality=%.2f\n",
						 ad.i_Prn, ad.i_Snr, ad.i_Angle, ad.i_Quality);
		}
	}
}

int main902(json jsonData)
{
	using namespace gn902test;

	// ---- 0. ABI 自检 ----
	// 宿主按自己的头文件算出指纹, 与库编译时的指纹比对。不相等说明"头文件与库不是
	// 同一次构建"(典型: 结构体改了, 但链接/加载的还是旧库), 此时 GetResult_GN902
	// 会按库的结构体尺寸写宿主的栈对象(栈溢出), 表现为
	//   *** stack smashing detected ***: terminated
	// 这种情况下必须用同一份头文件重新编译库, 而不是继续跑。
	{
		unsigned int localSig = GN902AbiSignature();
		unsigned int libSig = GetAbiSignature_GN902();
		if (localSig != libSig)
		{
			printf("[GN902] ABI 不一致: 宿主=%u 库=%u, 请用同一份头文件重新编译库!\n",
				   localSig, libSig);
			return 1;
		}
		printf("[GN902] ABI 自检通过 (sig=%u, sizeof(SpoofingResult)=%d)\n",
			   localSig, (int)sizeof(SpoofingResult));
	}

	// ---- 1. 读取配置 ----
	std::string datDir, port0File, port1File, debugFile, logFile, getResultFile;
	if (jsonData.count("datDir"))
		datDir = jsonData["datDir"].get<std::string>();
	if (jsonData.count("port0File"))
		port0File = jsonData["port0File"].get<std::string>();
	if (jsonData.count("port1File"))
		port1File = jsonData["port1File"].get<std::string>();
	if (jsonData.count("debugFile"))
		debugFile = jsonData["debugFile"].get<std::string>();
	logFile = jsonData.count("logFile") ? jsonData["logFile"].get<std::string>() : "./gn902_result.log";
	// ★ 新增: GetResult_GN902 返回值专用保存文件路径(可选, 默认 ./gn902_getresult.txt)
	getResultFile = jsonData.count("getResultFile")
		? jsonData["getResultFile"].get<std::string>()
		: std::string("./gn902_getresult.txt");

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
	printf("  GetResult : %s\n", getResultFile.c_str());
	printf("  模式      : %s\n", collectMode ? "固定基线采集(相位差)" : "循环切刀检测+测向");
	if (collectMode)
	{
		printf("  固定天线对: (%d,%d)\n", fixedPair[0], fixedPair[1]);
	}
	printf("================================================================\n");

	// ---- 2. 打开结果日志 ----
	FILE *logFp = fopen(logFile.c_str(), "w");

	// ★ 新增: 打开 GetResult_GN902 返回值专用保存文件(追加模式)
	//    写前先写一段头部说明, 便于后续阅读。
	FILE *getResultFp = fopen(getResultFile.c_str(), "w");
	if (getResultFp != 0)
	{
		gn902GetResultLine(getResultFp,
			"=== GN902 GetResult_GN902 结果快照 ===\n");
		gn902GetResultLine(getResultFp,
			"     port0=%s\n", port0File.c_str());
		gn902GetResultLine(getResultFp,
			"     port1=%s\n", port1File.c_str());
		gn902GetResultLine(getResultFp,
			"     debug=%s\n", debugFile.c_str());
		gn902GetResultLine(getResultFp,
			"     sizeof(SpoofingResult)=%d\n", (int)sizeof(SpoofingResult));
		gn902GetResultLine(getResultFp, "==================================\n");
	}

	// ---- 3. 解析两端口 dat ----
	std::map<double, std::vector<SatelliteData>> port0 = parseDatPort(port0File);
	std::map<double, std::vector<SatelliteData>> port1 = parseDatPort(port1File);
	gn902LogLine(logFp, "端口0 帧数: %d, 端口1 帧数: %d\n", (int)port0.size(), (int)port1.size());

	// ---- 4. 公共 GPS 秒（两端口同时出现的时刻）----
	std::vector<double> common;
	for (auto &kv : port0)
		if (port1.count(kv.first))
			common.push_back(kv.first);
	std::sort(common.begin(), common.end());

	// ---- 5. 生成喂帧序列 ----
	std::vector<std::pair<double, int>> feedFrames; // (sec, code) 时间升序
	if (collectMode)
	{
		// 固定基线采集: 每个公共 GPS 秒都作为该固定天线对喂入(code 仅占位)
		for (double sec : common)
		{
			feedFrames.push_back({sec, 0});
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
		std::vector<std::pair<double, int>> tagged; // (sec, code)
		for (double sec : common)
		{
			int code = activeCode(sec, schedule);
			if (code < 0)
				continue;
			tagged.push_back({sec, code});
		}

		// 按 code 分刀(runs)，每刀去末尾过渡秒、保留末尾 switchSeconds 个稳定秒
		struct Run
		{
			int code;
			std::vector<double> secs;
		};
		std::vector<Run> runs;
		for (auto &t : tagged)
		{
			if (!runs.empty() && runs.back().code == t.second)
				runs.back().secs.push_back(t.first);
			else
				runs.push_back({t.second, {t.first}});
		}
		int settleN = (switchSeconds > 0) ? switchSeconds : 1;
		for (auto &run : runs)
		{
			std::vector<double> settled;
			if (run.secs.size() > 1)
				settled.assign(run.secs.begin(), run.secs.end() - 1);
			else
				settled = run.secs;
			int start = std::max(0, (int)settled.size() - settleN);
			for (int i = start; i < (int)settled.size(); ++i)
				feedFrames.push_back({settled[i], run.code});
		}
		gn902LogLine(logFp, "检测喂帧: %d 帧\n", (int)feedFrames.size());
	}

	// ---- 7. 创建 GN902 对象 ----
	int id = 0, ret = 0;
	if ((ret = Create_GN902(id)) != 0)
	{
		gn902LogLine(logFp, "[GN902] Create_GN902 失败 ret=%d\n", ret);
		if (logFp)
			fclose(logFp);
		if (getResultFp)
			fclose(getResultFp);
		return ret;
	}

	// 可选阈值覆盖（不设则用引擎默认，与 Python 硬编码阈值一致）
	if (jsonData.count("phsDiffThreshold") && jsonData.count("satelliteCountThreshold"))
	{
		double phsTh = jsonData["phsDiffThreshold"].get<double>();
		double satTh = jsonData["satelliteCountThreshold"].get<double>();
		int sys = jsonData.count("sysEnum") ? jsonData["sysEnum"].get<int>() : -1;
		int type = jsonData.count("typeEnum") ? jsonData["typeEnum"].get<int>() : -1;
		// ret = SetThresholdDetection_GN902(id, phsTh, satTh, cutTh, sys, type);
		ret = SetThresholdDetection_GN902(id, satTh, phsTh, sys, type);
		if (ret != 0)
			gn902LogLine(logFp, "[GN902] SetThresholdDetection_GN902 失败 ret=%d\n", ret);
	}

	// 连续切刀数(连续报警确认次数): 由接口传入(>0 时生效)，不设则用引擎默认值
	if (jsonData.count("cutCountThreshold"))
	{
		int cutTh = jsonData["cutCountThreshold"].get<int>();
		ret = SetCutnumThreshold_GN902(id, cutTh);
		if (ret != 0)
			gn902LogLine(logFp, "[GN902] SetCutnumThreshold_GN902 失败 ret=%d\n", ret);
	}

	// ---- 8. 流式喂入 ----
	if (collectMode)
	{
		// 固定基线采集: 每个公共 GPS 秒作为固定天线对喂入，仅采集相位差(写入日志)
		int collectCount = 0;
		for (auto &fr : feedFrames)
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
		// 输出按"报警时刻"给出：每个给出报警的切刀时刻打印一条记录，与轮次无关。
		int roundIdx = 0;
		int doaMask = 0;
		bool inRound = false;
		int totalAlarmCount = 0;
		int totalResultRound = 0; // 成功调用 GetResult_GN902 取到结果的次数

		double cutSec[8] = {0.0};  // 当前轮各切刀的时刻(GPS秒), 索引 = 通道2天线号
		int cutCode[8] = {0};	   // 当前轮各切刀的原始 code, 索引同上
		double doneSec[8] = {0.0}; // 即将结算的那一轮的各切刀时刻
		int doneCode[8] = {0};	   // 即将结算的那一轮的各切刀 code

		for (auto &fr : feedFrames)
		{
			int cutIdx2 = codeToPair(fr.second);
			if (cutIdx2 < 1 || cutIdx2 > 7)
				continue;

			auto it0 = port0.find(fr.first);
			auto it1 = port1.find(fr.first);
			GNSSData frame = buildGnssData(
				it0 != port0.end() ? it0->second : std::vector<SatelliteData>(),
				it1 != port1.end() ? it1->second : std::vector<SatelliteData>());

			if (cutIdx2 == 1)
			{
				// 校正刀：先记下上一轮各切刀的时刻，再喂（触发上一轮检测+测向），
				// 然后按"报警时刻"结算上一轮结果。
				memcpy(doneSec, cutSec, sizeof(cutSec));
				memcpy(doneCode, cutCode, sizeof(cutCode));
				memset(cutSec, 0, sizeof(cutSec));
				memset(cutCode, 0, sizeof(cutCode));

				SetData_GN902(id, &frame, 7, 7); // 校正刀: 天线对(7,7)
				if (inRound && (doaMask & 0xFC) == 0xFC)
				{
					// 上一轮已组批检测+测向完成, 通过对外 C 接口取频点级结果。
					// SpoofingResult 就定义在宿主自己的栈上, 由库按引用整体写入。
					SpoofingResult result;
					memset(&result, 0, sizeof(result)); // 先清零: 若输出全 0, 说明库少写了(ABI 不一致)
					int getRet = GetResult_GN902(id, result);
					if (getRet != 0)
					{
						gn902LogLine(logFp, "[GN902] GetResult_GN902 失败 ret=%d\n", getRet);
					}
					else
					{
						double roundSec = gn902LastCutSec(doneSec);
						gn902LogSpoofingResult(logFp, result, roundSec);
						// ★ 新增: 单独保存 GetResult_GN902 返回结果
						gn902SaveSpoofingResult(getResultFp, result, roundIdx + 1, roundSec);
						totalResultRound++;
					}

					// 报警时刻级明细仍走类接口(GN902Container), 与上面频点级结果互补
					std::vector<AlarmMoment> moments;
					if (id >= 0 && id < (int)GN902Container.size() && GN902Container[id] != 0)
					{
						GN902Container[id]->GetAlarmMoments(moments);
					}
					gn902LogAlarmMoments(logFp, moments, doneSec, doneCode, totalAlarmCount);
					roundIdx++;
				}
				inRound = true;
				doaMask = 0;
			}
			else
			{
				SetData_GN902(id, &frame, 1, cutIdx2);
				cutSec[cutIdx2] = fr.first;
				cutCode[cutIdx2] = fr.second;
				inRound = true;
				doaMask |= (1 << cutIdx2);
			}
		}

		// 末尾轮：与 Python 一致，末尾不完整周期也逐刀处理。缺失的测向刀用空帧(无卫星)
		// 补齐，使引擎仍按 7 刀(校正 + 六测向)组批；缺刀位的相位差由跨周期基线 m_Baselines
		// 补缺，随后用一帧空校正触发上一轮检测+测向。
		// 注意：仅当本周期已喂入至少一刀测向(doaMask!=0)时才flush，避免"最后恰为一校正刀"
		// 时凭空多出一轮空结果。
		if (doaMask != 0)
		{
			memcpy(doneSec, cutSec, sizeof(cutSec));
			memcpy(doneCode, cutCode, sizeof(cutCode));

			for (int cut = 2; cut <= 7; ++cut)
			{
				if (!(doaMask & (1 << cut)))
				{
					GNSSData emptyCut;
					memset(&emptyCut, 0, sizeof(emptyCut));
					SetData_GN902(id, &emptyCut, 1, cut);
				}
			}
			GNSSData dummyCal;
			memset(&dummyCal, 0, sizeof(dummyCal));
			SetData_GN902(id, &dummyCal, 7, 7); // 末尾补一帧空校正刀(天线对 7,7)

			// 末尾轮同样用对外接口取频点级结果
			SpoofingResult result;
			memset(&result, 0, sizeof(result)); // 先清零: 若输出全 0, 说明库少写了(ABI 不一致)
			int getRet = GetResult_GN902(id, result);
			if (getRet != 0)
			{
				gn902LogLine(logFp, "[GN902] GetResult_GN902 失败 ret=%d\n", getRet);
			}
			else
			{
				double roundSec = gn902LastCutSec(doneSec);
				gn902LogSpoofingResult(logFp, result, roundSec);
				// ★ 新增: 单独保存 GetResult_GN902 返回结果(末尾轮)
				gn902SaveSpoofingResult(getResultFp, result, roundIdx + 1, roundSec);
				totalResultRound++;
			}

			std::vector<AlarmMoment> moments;
			if (id >= 0 && id < (int)GN902Container.size() && GN902Container[id] != 0)
			{
				GN902Container[id]->GetAlarmMoments(moments);
			}
			gn902LogAlarmMoments(logFp, moments, doneSec, doneCode, totalAlarmCount);
			roundIdx++;
		}

		// ---- 9. 汇总 ----
		gn902LogLine(logFp, "\n================================================================\n");
		gn902LogLine(logFp, "GN902 检测完成: 处理 %d 个完整测向轮, 共 %d 个报警时刻记录\n", roundIdx, totalAlarmCount);
		gn902LogLine(logFp, "GetResult_GN902 取结果成功 %d 次(每完成一轮一次)\n", totalResultRound);
		gn902LogLine(logFp, "================================================================\n");

		// ★ 新增: 在专用文件末尾也写一段汇总
		if (getResultFp != 0)
		{
			gn902GetResultLine(getResultFp, "==================================\n");
			gn902GetResultLine(getResultFp,
				"总计: 处理 %d 轮, GetResult_GN902 成功取回结果 %d 次\n",
				roundIdx, totalResultRound);
			gn902GetResultLine(getResultFp, "==================================\n");
		}
	}

	Release_GN902(id);
	if (logFp)
		fclose(logFp);
	if (getResultFp)
		fclose(getResultFp);
	return 0;
}