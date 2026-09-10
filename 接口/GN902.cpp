// =============================================================================
// 文件名: GN902.cpp
// 功能描述: GN902 欺骗检测/测向模块 —— 自包含实现文件(消除编译隔离)
//
// 本文件收纳了以下原独立源文件的全部函数实现(逐字保留, 便于跨函数内联优化):
//   公共工具   publicFunctionDoa.cpp  (PublicSpace 命名空间)
//   测向算法   arithmetic.cpp         (ArithmeticDoa 类)
//   核心引擎   SpoofingDoa.cpp        InitData.cpp     PreparationData.cpp
//              Corrected.cpp          Interf.cpp       AmpPhase.cpp
//              Omni.cpp               Directed.cpp     Alarm.cpp
//              SpectrumDesity.cpp     WriteLog.cpp
//   接口实现   GN902.cpp (GN902 类: 实例状态机 + 整轮帧缓冲 + 检测/测向调度)
//
// 所有类型与类声明见配套头文件 GN902.h。
// =============================================================================
#include "GN902.h"

using namespace std;

// =============================================================================
// == 原 publicFunctionDoa.cpp —— PublicSpace 命名空间实现 ===========================
// =============================================================================
namespace PublicSpace
{
    map<string, FILE *> m_Log_fp;
    string m_LogCount = "";
    int m_logFlg = 0;
    int m_save_data_Flg = 0;
    std::mutex m_fileMutex;

    int Round360(int x)
    {
        x = x % 360;
        if (x < 0)
        {
            x += 360;
        }
        return x;
    }

    string getNowTime() {

        if (m_logFlg != 0){
            auto now = std::chrono::system_clock::now();
            std::time_t current_time = std::chrono::system_clock::to_time_t(now);
            std::tm *local_time = std::localtime(&current_time);

            auto now_ms = std::chrono::time_point_cast<std::chrono::milliseconds>(now);
            auto epoch = now_ms.time_since_epoch();
            auto value = std::chrono::duration_cast<std::chrono::milliseconds>(epoch);
            long long milliseconds = value.count() % 1000;

            char buffer[80];
            std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", local_time);

            std::stringstream ss;
            ss << buffer << "." << std::setfill('0') << std::setw(3) << milliseconds;
            return ss.str();
        }
        else{
            return "";
        }
    }

    void LogCreat(const string path) {
        if (0 != m_logFlg){
            string logstr = ".log";
            string tp_path = "";
            tp_path.append(path);
            tp_path.append(m_LogCount);
            tp_path.append(logstr);

            FILE *fp;

            if (1 == m_logFlg) {
                fp = fopen(tp_path.c_str(), "w");
            }
            if (2 == m_logFlg) {
                fp = fopen(tp_path.c_str(), "a");
            }

            if (fp == nullptr){
                return;
            }

            fprintf(fp, "Log File create\n");
            fclose(fp);

            FILE *m_fp = fopen(tp_path.c_str(), "a");
            m_Log_fp[m_LogCount] = m_fp;
            if (fp == nullptr){
                return;
            }

        }
    }

    void Log(const char *format, ...){
        std::lock_guard<std::mutex> lock(m_fileMutex);

        if (0 != m_logFlg){
            FILE *m_fp = m_Log_fp[m_LogCount];
            if (nullptr == m_fp){
                cout << "Log is null:" << m_LogCount << endl;
                return;
            }
            va_list args;
            va_start(args, format);
            vfprintf(m_fp, format, args);
            va_end(args);
            fflush(m_fp);
        }

    }

}


// =============================================================================
// == 原 arithmetic.cpp —— ArithmeticDoa 类实现 ====================================
// =============================================================================
ArithmeticDoa::ArithmeticDoa()
{
}

ArithmeticDoa::~ArithmeticDoa()
{
}

void ArithmeticDoa::setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
{
    map<int, map<int, double>> tp_antenna_map;
    tp_antenna_map.clear();
    map<int, double> tp1;
    for (int i = 0; i < (int)phaseDiff.size(); i++)
    {
        tp1.clear();
        int ant1 = cutSequence[i][0];
        if (tp_antenna_map.find(ant1) != tp_antenna_map.end())
        {
            tp1 = tp_antenna_map[ant1];
        }
        tp1[cutSequence[i][1]] = phaseDiff[i];
        tp_antenna_map[ant1] = tp1;
    }
    int count = 1;
    int size = 0;
    int start_index = 0;
    int tp_ant1 = 0;
    int tp_ant2 = 0;
    double tp2_diff = 0.0;
    while (0 != count && count < 50)
    {
        count = 0;
        size = (int)phaseDiff.size();
        for (int i = 0; i < size; i++)
        {
            if (start_index <= i)
            {
                start_index = i + 1;
            }

            for (int j = start_index; j < size; j++)
            {
                tp1.clear();
                if (cutSequence[i][1] == cutSequence[j][0])
                {
                    tp_ant1 = cutSequence[i][0];
                    tp_ant2 = cutSequence[j][1];
                    tp2_diff = phaseDiff[i] + phaseDiff[j];
                }
                if (cutSequence[i][0] == cutSequence[j][1])
                {
                    tp_ant1 = cutSequence[j][0];
                    tp_ant2 = cutSequence[i][1];
                    tp2_diff = phaseDiff[i] + phaseDiff[j];
                }
                if (cutSequence[i][0] == cutSequence[j][0])
                {
                    tp_ant1 = cutSequence[i][1];
                    tp_ant2 = cutSequence[j][1];
                    tp2_diff = phaseDiff[j] - phaseDiff[i];
                }
                if (cutSequence[i][1] == cutSequence[j][1])
                {
                    tp_ant1 = cutSequence[i][0];
                    tp_ant2 = cutSequence[j][0];
                    tp2_diff = phaseDiff[i] - phaseDiff[j];
                }
                if (tp_ant1 == tp_ant2)
                {
                    continue;
                }
                if (tp_antenna_map.find(tp_ant2) != tp_antenna_map.end())
                {
                    tp1 = tp_antenna_map[tp_ant2];
                    if (tp1.find(tp_ant1) != tp1.end())
                    {
                        continue;
                    }
                    tp1.clear();
                }
                if (tp_antenna_map.find(tp_ant1) != tp_antenna_map.end())
                {
                    tp1 = tp_antenna_map[tp_ant1];
                    if (tp1.find(tp_ant2) != tp1.end())
                    {
                        continue;
                    }
                }

                tp1[tp_ant2] = tp2_diff;
                tp_antenna_map[tp_ant1] = tp1;
                vector<int> tp2;
                tp2.clear();
                tp2.resize(2);

                tp2[0] = tp_ant1;
                tp2[1] = tp_ant2;
                cutSequence.emplace_back(tp2);
                phaseDiff.emplace_back(tp2_diff);
                ++count;
            }
        }
        start_index = size;
    }
}

void ArithmeticDoa::calPseudoByInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, vector<double> &diff)
{
    int startAngle = (int)data.i_Start;
    int endAngle = (int)data.i_End;
    int size = data.i_Phase_Len;
    diff.clear();
    vector<double>().swap(diff);
    diff.resize(360);
    std::fill(diff.begin(), diff.end(), 0.0);
    int tp_ang = 0;
    for (int ang = startAngle; ang < endAngle + 1; ang++)
    {
        double sumDiff = 0.0;
        tp_ang = Round360(ang);
        for (int i = 0; i < size; i++)
        {
            int antn1 = data.i_AntennaSq[i][0];
            int antn2 = data.i_AntennaSq[i][1];
            double theoryPhase = phaseTheory[tp_ang][antn1 - 1] - phaseTheory[tp_ang][antn2 - 1];
            sumDiff = sumDiff + cos(theoryPhase - data.i_Phase_Diff[i]);
        }
        diff[tp_ang] = sumDiff;
    }
}

/**
 * @brief 相关干涉仪测向: 在搜索角度范围内, 用理论相位差与实际相位差做相关匹配
 * @param phaseTheory 理论相位差模板 [角度][阵元]
 * @param data        实测信息(天线对、相位差、搜索范围)
 * @param angle       输出: 最匹配的来波角度(度)
 * @param quality     输出: 测向质量(0-100)
 * @param diff2       输出: 各角度相关度伪谱
 */
void ArithmeticDoa::calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2)
{
    diff2.resize(360);
    vector<double> diff;
    int startAngle = (int)data.i_Start;
    int endAngle = (int)data.i_End;
    int size = data.i_Phase_Len;
    double max_val = -9999.9;
    int ang_val = 0;
    diff.clear();
    vector<double>().swap(diff);
    diff.resize(360);
    int tp_ang = 0;
    InterferInfo tp_data = data;
    double tp_Phase_theory[200];
    for (int ang = startAngle; ang < endAngle + 1; ang++)
    {
        double sumDiff = 0.0;
        tp_ang = Round360(ang);
        for (int i = 0; i < size; i++)
        {
            int antn1 = data.i_AntennaSq[i][0];
            int antn2 = data.i_AntennaSq[i][1];
            double theoryPhase = phaseTheory[tp_ang][antn1 - 1] - phaseTheory[tp_ang][antn2 - 1];
            tp_Phase_theory[i] = theoryPhase;

            sumDiff = sumDiff + cos(theoryPhase - data.i_Phase_Diff[i]);
        }
        if (sumDiff > max_val)
        {
            max_val = sumDiff;
            ang_val = tp_ang;
            for (int j = 0; j < data.i_Phase_Len; j++)
            {
                tp_data.i_Phase_Diff[j] = tp_Phase_theory[j];
            }
        }
        diff[tp_ang] = sumDiff;
        diff2[tp_ang] = (sumDiff / size + 1) / 2;
    }

    angle = Round360(ang_val);
    vector<double> tp_theory_diff;
    calPseudoByInterfer(phaseTheory, tp_data, tp_theory_diff);
    quality = getDoaMass(tp_theory_diff, diff, startAngle, endAngle);
    for (int i = 0; i < 359; i++)
    {
        diff2[i] *= quality;
    }
}

/**
 * @brief 计算均匀圆阵的理论相位差模板
 * @param f         载波频率(Hz)
 * @param r         阵列半径(米)
 * @param antnnaNum 阵元数量
 * @param theory    输出: 理论相位差模板 [360角度][阵元](弧度)
 */
void ArithmeticDoa::calPhaseTheory(const double f, const double r, const int antnnaNum, std::vector<std::vector<double>> &theory)
{
    double perAngel = 2 * m_PI / antnnaNum;
    theory.resize(360);
    for (int ang = 0; ang < 360; ang++)
    {
        theory[ang].resize(antnnaNum);
        for (int i = 0; i < antnnaNum; i++)
        {
            double theoryPhase = 2 * m_PI * f * r / m_C * cos(i * perAngel - ang * m_PI / 180);
            theory[ang][i] = theoryPhase;
        }
    }
}

double ArithmeticDoa::getDoaMass(const vector<double> diffTheory, const vector<double> diff, int starAngle, int endAngle)
{
    double quality;
    int size = (int)diffTheory.size();
    double tp_sum = 0.0;
    double tp_theory_sum = 0.0;
    double tp_diff_sum = 0.0;
    int tp_ang = 0;
    for (int i = starAngle; i < endAngle + 1; i++)
    {
        tp_ang = Round360(i);
        tp_sum = tp_sum + diffTheory[tp_ang] * diff[tp_ang];
        tp_theory_sum = tp_theory_sum + diffTheory[tp_ang] * diffTheory[tp_ang];
        tp_diff_sum = tp_diff_sum + diff[tp_ang] * diff[tp_ang];
    }
    double tp_mode_sum = sqrt(tp_theory_sum) * sqrt(tp_diff_sum);
    if (tp_mode_sum == 0)
    {
        quality = 0;
    }
    else
    {
        quality = (tp_sum / tp_mode_sum + 1) / 2 * 100;
    }

    return quality;
}

double ArithmeticDoa::getDoaMass(const vector<double> diffTheory, const vector<double> diff)
{
    double quality;
    int size = (int)diffTheory.size();
    double tp_sum = 0.0;
    double tp_theory_sum = 0.0;
    double tp_diff_sum = 0.0;
    for (int i = 0; i < size; i++)
    {
        tp_sum = tp_sum + diffTheory[i] * diff[i];
        tp_theory_sum = tp_theory_sum + diffTheory[i] * diffTheory[i];
        tp_diff_sum = tp_diff_sum + diff[i] * diff[i];
    }
    double tp_mode_sum = sqrt(tp_theory_sum) * sqrt(tp_diff_sum);
    if (tp_mode_sum == 0)
    {
        quality = 0;
    }
    else
    {
        quality = (tp_sum / tp_mode_sum + 1) / 2 * 100;
    }

    return quality;
}


// =============================================================================
// == 原 SpoofingDoa.cpp —— 核心引擎实现 =================================================
// =============================================================================

/**
 * @brief 构造函数: 初始化引擎并设置各系统频点的默认欺骗检测阈值
 * @note 对 GPS/GLONASS/Galileo/BDS 各频点调用 setThresholdDetectionDoa 设置默认卫星数阈值与相位差阈值
 */
SpoofingDoa::SpoofingDoa(void){
    Init();
    setThresholdDetectionDoa(0, 2, 4, -1);    // GPS L5
    setThresholdDetectionDoa(1, 0, 2, 5.0);   // GLONASS G1
    setThresholdDetectionDoa(1, 1, 3, 5.0);   // GLONASS G2
    setThresholdDetectionDoa(3, 2, 3, 3.6);   // Galileo E1C
    setThresholdDetectionDoa(3, 12, 4, 3.6);  // Galileo E5a
    setThresholdDetectionDoa(3, 17, 4, 3.6);  // Galileo E5b
    setThresholdDetectionDoa(4, 17, 3, -1);   // BDS B2I
    setThresholdDetectionDoa(4, 0, 3, -1);    // BDS B1I
    setThresholdDetectionDoa(4, 2, 3, -1);    // BDS B3I
    setThresholdDetectionDoa(4, 8, 2, -1);    // BDS B1C
    setThresholdDetectionDoa(4, 19, 3, -1);   // BDS B2b
    setThresholdDetectionDoa(4, 34, 3, 3.6);  // BDS B1X
}

/**
 * @brief 初始化引擎全部运行状态
 * @note 主要完成: 清空历史结果 → 设置阵列/切刀/检测等默认参数 → 建立频率表(initType)
 *       → 初始化各频点检测阈值 → 建立相关干涉仪理论模板(initTheory)
 */
void SpoofingDoa::Init(void){

    m_CorrectionData.clear();
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);

    m_LogFile = "./spoofingDoaLog_";
    PublicSpace::m_logFlg = 0;
    LogCreat(m_LogFile);

    m_AntennaNum = 7;
    m_omni_R = 0.1865;

    m_cutSequence = { {1,1}, {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7} };

    m_OneCut_Frams = 8;
    m_Smooth_Flag = 1;

    m_Doa_Cut_min_Num = 6;
    if (m_Doa_Cut_min_Num >= (int)m_cutSequence.size()) {
        m_Doa_Cut_min_Num = (int)m_cutSequence.size() - 1;
    }

    m_Phasediff_Threshold = 5.0;
    m_Detection_Threshold_Num = 2;
    m_Snr_Threshold = 35.0;
    m_Qulity_Threshold = 0.0;

    m_Cyclic_Detection_Flag = 1;
    m_Delete_Prn_Flag = 1;

    m_Detection_Recodds_Num = 2;

    m_Save_Original_Flg = 0;
    PublicSpace::m_save_data_Flg = 0;

    initType();
    resetCyclicDetection();
    setR();

    initDetectionThreshold(m_Detection_Threshold_Num, m_Phasediff_Threshold);

    initTheory();
}

SpoofingDoa::~SpoofingDoa(void)
{
}

/**
 * @brief 喂入一批 GNSS 原始数据并触发检测/测向
 * @param data    GNSS 数据数组(按切刀序列排列)
 * @param dataLen 数据条数(切刀数量)
 */
void SpoofingDoa::setGNSSData(const GNSSData *data, int dataLen)
{
    int cutNum = (int)m_cutSequence.size();
    if (cutNum < 2)
    {
        PublicSpace::Log("error:cutSequence is mistake!!!\n");
        cout << "error:cutSequence is mistake!!!" << endl;
        return;
    }
    PublicSpace::Log("star Doa.........................\n");
    setDataAngle(data, dataLen);
    saveGNSSData(data, dataLen);
}

/**
 * @brief 取出最近一轮的测向/报警结果
 * @param result 输出结果(含各频点报警角度、卫星明细)
 * @return 0=成功
 */
int SpoofingDoa::getAngleSpoofingDoa(SpoofingResult &result)
{
    setSpoofingResult(result);
    PublicSpace::Log("Doa result:\n");
    LogSpoofingResult(result);
    return 0;
}

void SpoofingDoa::setSpoofingResult(SpoofingResult &result)
{
    int count = 0;
    int typeInt = 0;
    double angle = 0.0;
    int prn = 0;
    for (auto it = m_AngleResultData.begin(); it != m_AngleResultData.end(); ++it)
    {
        typeInt = it->first;
        result.i_SatelliteAngle[count].i_Sys = typeInt / 100;
        result.i_SatelliteAngle[count].i_Type = typeInt % 100;
        result.i_SatelliteAngle[count].i_Alarm = 1;
        vector<AlarmData> tp = it->second;
        vector<double> doas;
        doas.reserve(tp.size());
        for (unsigned int i = 0; i < tp.size(); i++)
        {
            doas.push_back((double)tp[i].i_Angle);
        }
        angle = tp.empty() ? -1.0 : circularMeanDeg(doas);
        if (angle < 0)
        {
            angle += 360.0;
        }
        result.i_SatelliteAngle[count].i_Angle = angle;
        result.i_SatelliteAngle[count].i_Count = (int)tp.size();
        for (unsigned int i = 0; i < tp.size(); i++)
        {
            prn = tp[i].i_Prn;
            result.i_SatelliteAngle[count].i_AlarmData[i] = tp[i];
            result.i_SatelliteAngle[count].i_AlarmData[i].i_Snr = m_Max_Snr[typeInt][prn];
        }
        ++count;
    }
    result.i_Count = count;
}

/**
 * @brief 测向主流程: 接收整轮切刀数据, 依次完成相位差计算→平滑→校正→欺骗检测→测向
 * @param data    切刀数据数组(每项对应一个切刀位置)
 * @param dataLen 切刀数量
 * @note 流程: getSatelliteDataPhaseDiffA 提取相位差 → (可选)平滑 → 通道校正 →
 *       循环切刀检测(累积跨轮基线) → 相关干涉仪测向 → 更新跟踪状态
 */
void SpoofingDoa::setDataAngle(const GNSSData *data, int dataLen)
{
    string nowT = getNowTime();

    PublicSpace::Log("cal phase diff cutNum:  %s \n", nowT.c_str());
    vector<vector<SatelliteDataPhaseDiffA>> dataA;
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);

    for (int i = 0; i < dataLen; i++){
        PublicSpace::Log("cut: %d   PortOneNum: %d PortTwoNum: %d  \n", i + 1, data[i].i_PortOneNum, data[i].i_PortTwoNum);
        vector<SatelliteDataPhaseDiffA> tp;
        tp.clear();
        vector<SatelliteDataPhaseDiffA>().swap(tp);
        getSatelliteDataPhaseDiffA(data[i], tp);
        dataA.emplace_back(tp);
        LogGNSSData(data[i], i + 1);
    }

    if (0 != m_logFlg){
        vector<SatelliteDataPhaseDiffB> tpB;
        tpB.clear();
        getSatelliteDataPhaseDiffB(dataA, tpB);
        LogSatelliteDataPhaseDiffB(tpB);
    }

    if (m_OneCut_Frams > 1 && 1 == m_Smooth_Flag){
        nowT = getNowTime();
        PublicSpace::Log("get smooth data:  %s \n", nowT.c_str());
        getSmoothData(dataA);
    }

    if (m_OneCut_Frams > 1 && 0 == m_Smooth_Flag)
    {
        PublicSpace::Log("get end fram data:  %s \n", nowT.c_str());
        getEndFramData(dataA);
    }

    nowT = getNowTime();
    PublicSpace::Log("get Correct Data:  %s \n", nowT.c_str());
    setCorrectionData(dataA);
    nowT = getNowTime();
    PublicSpace::Log("get Corrected Gnss Data:  %s \n", nowT.c_str());
    getCorrectedGnssData(dataA);

    if (m_Cyclic_Detection_Flag != 0)
    {
        nowT = getNowTime();
        PublicSpace::Log("get Cyclic Detection Data:  %s \n", nowT.c_str());
        getCyclicDetectionData(dataA);
    }

    vector<SatelliteDataPhaseDiffB> dataB;
    dataB.clear();
    getSatelliteDataPhaseDiffB(dataA, dataB);
    LogSatelliteDataPhaseDiffB(dataB);

    vector<SatelliteDataPhaseDiffB> doaDataB;
    doaDataB.clear();
    if (m_Cyclic_Detection_Flag != 0)
    {
        accumulateBaselines(dataB);
        getCrossCycleDataB(doaDataB);
    }
    else
    {
        doaDataB = dataB;
    }

    if (0 == m_Theory.size()){
        cout << "error:theory phase diff have not!!!!" << endl;
        return;
    }

    getResultInterferDoa(doaDataB);

    if (m_Cyclic_Detection_Flag != 0)
    {
        for (auto &tp : m_Tracking)
        {
            int typeInt = tp.first;
            TrackingInfo &t = tp.second;

            if (m_AngleResultData.find(typeInt) != m_AngleResultData.end() && !m_AngleResultData[typeInt].empty())
            {
                vector<double> doas;
                double sumQ = 0.0;
                for (auto &ad : m_AngleResultData[typeInt])
                {
                    doas.push_back((double)ad.i_Angle);
                    sumQ += ad.i_Quality;
                }
                if (!doas.empty())
                {
                    double mean = circularMeanDeg(doas);
                    if (mean < 0)
                    {
                        mean += 360.0;
                    }
                    t.doa_deg = mean;
                    t.quality = sumQ / doas.size();
                }
            }
            else if (m_ConsecutiveAlarm[typeInt] == 0 && t.doa_deg >= 0.0)
            {
                vector<AlarmData> kept;
                for (int sid : t.cluster_sats)
                {
                    AlarmData ad;
                    ad.i_Prn = sid;
                    ad.i_Angle = Round360((int)t.doa_deg);
                    ad.i_Quality = t.quality;
                    ad.i_Snr = 0.0f;
                    if (m_Max_Snr.find(typeInt) != m_Max_Snr.end() && m_Max_Snr[typeInt].find(sid) != m_Max_Snr[typeInt].end())
                    {
                        ad.i_Snr = (float)m_Max_Snr[typeInt][sid];
                    }
                    kept.emplace_back(ad);
                }
                m_AngleResultData[typeInt] = kept;
            }
        }
    }
}

void SpoofingDoa::calAngle(std::map<int, std::map<int, InterferInfo>> inferInfoData)
{
    m_AngleResultData.clear();
    int typeInt = 0;
    int prn = 0;
    map<int, InterferInfo> tp;
    InterferInfo tp_info;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    vector<vector<double>> phaseTheory;
    vector<double> pseudoValue;

    for (auto it = inferInfoData.begin(); it != inferInfoData.end(); ++it)
    {
        typeInt = it->first;
        tp = it->second;
        phaseTheory.clear();
        phaseTheory = m_Theory[typeInt];
        tp_alarms.clear();
        for (auto itt = tp.begin(); itt != tp.end(); ++itt)
        {
            pseudoValue.clear();
            prn = itt->first;
            tp_info = itt->second;

            double angle;
            double quality;
            ArithmeticDoa::calInterfer(phaseTheory, tp_info, angle, quality, pseudoValue);
            PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,Fre=%.1f,R=%.4f,startAngle=%d,endAngle=%d,angle=%.2f,quality=%.2f\n",
                             typeInt / 100, typeInt % 100, prn, m_F[typeInt], m_R[typeInt],
                             tp_info.i_Start, tp_info.i_End, angle, quality);
            PublicSpace::Log("antenna and phasediff:[\n");
            for (int i = 0; i < tp_info.i_Phase_Len; i++)
            {
                PublicSpace::Log("   antenna1=%d,antenna2=%d,phasediff=%.2f\n",
                                 tp_info.i_AntennaSq[i][0], tp_info.i_AntennaSq[i][1], tp_info.i_Phase_Diff[i] * 180 / PI);
            }
            PublicSpace::Log("]\n");

            if (quality < m_Qulity_Threshold)
            {
                continue;
            }

            tp_alarm.i_Prn = prn;
            tp_alarm.i_Angle = (int)angle;
            tp_alarm.i_Quality = quality;
            tp_alarms.emplace_back(tp_alarm);
        }
        m_AngleResultData[typeInt] = tp_alarms;
    }
}

/**
 * @brief 从一条基线数据提取测向所需的天线对与相位差, 填入 InterferInfo
 * @param dataB  单星各切刀相位差数据
 * @param info   输出: 测向输入信息(天线对/相位差/条数)
 * @param doaFlg 输出: 1=有效(切刀数足够), 0=无效(切刀数不足)
 */
void SpoofingDoa::calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg)
{
    doaFlg = 1;
    int diffLen = dataB.i_diffLen;
    int count = 0;
    map<int, map<int, double>> tp_antenna_map;
    map<int, double> tp1;
    float maxSnr = 99;

    vector<vector<int>> tp_antnna;
    vector<double> tp_diff;

    for (int j = 0; j < diffLen; j++)
    {
        if (dataB.i_Snr1[j] < 1e-6 || dataB.i_Snr2[j] < 1e-6)
        {
            continue;
        }
        if (m_cutSequence[j][0] == m_cutSequence[j][1])
        {
            continue;
        }

        if (dataB.i_Snr1[j] < maxSnr)
        {
            maxSnr = dataB.i_Snr1[j];
        }
        if (dataB.i_Snr2[j] < maxSnr)
        {
            maxSnr = dataB.i_Snr2[j];
        }
        tp_antnna.emplace_back(m_cutSequence[j]);
        tp_diff.emplace_back(dataB.i_phase_diff[j]);
        tp1[m_cutSequence[j][1]] = dataB.i_phase_diff[j];
        tp_antenna_map[m_cutSequence[j][0]] = tp1;
        ++count;
    }
    if (count < m_Doa_Cut_min_Num)
    {
        doaFlg = 0;
        return;
    }
    ArithmeticDoa::setUseAntennaAndPhaseAll(tp_antnna, tp_diff);
    int size = (int)tp_diff.size();
    if (size < m_Doa_Cut_min_Num)
    {
        doaFlg = 0;
        return;
    }
    int prn = dataB.i_Prn;
    int typeInt = TypeInt(dataB.i_Sys, dataB.i_Type);
    if (diffLen > 1)
    {
        maxSnr = dataB.i_Snr1[1];
    }
    m_Max_Snr[typeInt][prn] = maxSnr;
    for (int i = 0; i < size; i++)
    {
        info.i_Phase_Diff[i] = tp_diff[i] * 2 * PI;
        info.i_AntennaSq[i][0] = tp_antnna[i][0];
        info.i_AntennaSq[i][1] = tp_antnna[i][1];
    }
    info.i_Phase_Len = size;
}

void SpoofingDoa::getSmoothData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int cutNum = (int)m_cutSequence.size();
    vector<SatelliteDataPhaseDiffB> dataB;
    vector<vector<SatelliteDataPhaseDiffA>> oneCutData;
    oneCutData.resize(m_OneCut_Frams);
    vector<vector<SatelliteDataPhaseDiffA>> resultData;
    SatelliteDataPhaseDiffA tpA;
    vector<SatelliteDataPhaseDiffA> tpA2;
    for (int j = 0; j < cutNum; j++)
    {
        dataB.clear();
        oneCutData.clear();
        int index = 0;
        for (int k = 0; k < m_OneCut_Frams; k++)
        {
            index = j * m_OneCut_Frams + k;
            oneCutData[k] = dataA[index];
        }
        int savedDeleteFlag = m_Delete_Prn_Flag;
        m_Delete_Prn_Flag = 0;
        getSatelliteDataPhaseDiffB(oneCutData, dataB);
        m_Delete_Prn_Flag = savedDeleteFlag;
        for (unsigned int i = 0; i < dataB.size(); i++)
        {
            calSmoothData(dataB[i], tpA);
            tpA2.emplace_back(tpA);
        }
        resultData.emplace_back(tpA2);
    }
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA = resultData;
}

void SpoofingDoa::calSmoothData(SatelliteDataPhaseDiffB dataB, SatelliteDataPhaseDiffA &dataA)
{
    vector<double> samples;
    double sumSnr1 = 0.0;
    double sumSnr2 = 0.0;
    int length = dataB.i_diffLen;
    for (int i = 0; i < length; ++i)
    {
        if (dataB.i_Snr1[i] < 1e-3 || dataB.i_Snr2[i] < 1e-3)
        {
            continue;
        }
        samples.emplace_back(dataB.i_phase_diff[i] * 360.0);
        sumSnr1 += dataB.i_Snr1[i];
        sumSnr2 += dataB.i_Snr2[i];
    }

    dataA.i_Sys = dataB.i_Sys;
    dataA.i_Type = dataB.i_Type;
    dataA.i_Prn = dataB.i_Prn;

    int n = (int)samples.size();
    if (n <= 0)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    int required = (length < MIN_STABLE_SAMPLES) ? length : MIN_STABLE_SAMPLES;
    if (n < required)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    if (circularSpanDeg(samples) < STABILITY_RANGE_DEG)
    {
        double smoothDeg = circularMeanDeg(samples);
        double smoothCycle = smoothDeg / 360.0;
        if (smoothCycle < 0)
        {
            smoothCycle += 1.0;
        }
        dataA.i_phase_diff = smoothCycle;
        dataA.i_Snr1 = sumSnr1 / n;
        dataA.i_Snr2 = sumSnr2 / n;
        return;
    }

    if (circularSpan180Deg(samples) < STABILITY_RANGE_DEG)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    dataA.i_phase_diff = 0.0;
    dataA.i_Snr1 = 0.0;
    dataA.i_Snr2 = 0.0;
}

void SpoofingDoa::getEndFramData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    vector<vector<SatelliteDataPhaseDiffA>> resultDataA;
    int cutNum = (int)m_cutSequence.size();
    for (int j = 0; j < cutNum; j++)
    {
        int index = 0;
        index = (j + 1) * m_OneCut_Frams - 1;
        resultDataA.emplace_back(dataA[index]);
    }

    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA = resultDataA;
}

void SpoofingDoa::setR(void){
    // 全向天线: 所有频点使用同一阵列半径 m_omni_R
    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        m_R[it->first] = m_omni_R;
    }
}

/**
 * @brief 设置某系统频点的欺骗检测阈值
 * @param sys         卫星系统编码
 * @param type        频点编码
 * @param threshold   卫星数阈值(-1 不修改)
 * @param phsThreshold 相位差阈值(度, <=0 不修改)
 * @note sys==-1 && type==-1 时初始化全部频点
 */
void SpoofingDoa::setThresholdDetectionDoa(int sys, int type, int threshold, double phsThreshold)
{
    PublicSpace::Log("Sys=%i,Type=%i,coutThreshold=%i,phsThreshold=%.1f\n", sys, type, threshold, phsThreshold);
    if (-1 == sys && -1 == type)
    {
        initDetectionThreshold(threshold, phsThreshold);
        return;
    }

    if (-1 != threshold)
    {
        m_Detection_Threshold[TypeInt(sys, type)] = threshold;
    }
    if (phsThreshold > 0)
    {
        m_Detection_PhsThreshold[TypeInt(sys, type)] = phsThreshold / 360.0;
    }
}

void SpoofingDoa::resetCyclicDetection(void)
{
    m_ConsecutiveAlarm.clear();
    m_Tracking.clear();
    m_Baselines.clear();
}

/**
 * @brief 配置循环切刀运行方式
 * @param cyclic      是否启用循环切刀检测
 * @param oneCutFrams 每个切刀帧数(>0 生效)
 * @param smooth      是否多帧平滑
 * @param omniR       全向天线阵列半径(米, >0 且全向时重建理论模板)
 */
void SpoofingDoa::configCyclicRuntime(bool cyclic, int oneCutFrams, bool smooth, double omniR)
{
    m_Cyclic_Detection_Flag = cyclic ? 1 : 0;
    if (oneCutFrams > 0)
    {
        m_OneCut_Frams = oneCutFrams;
    }
    m_Smooth_Flag = smooth ? 1 : 0;

    if (omniR > 0)
    {
        m_omni_R = omniR;
        m_R.clear();
        setR();
        m_Theory.clear();
        initTheory();
    }
    PublicSpace::Log("configCyclicRuntime: cyclic=%d oneCutFrams=%d smooth=%d omniR=%.4f\n",
                     m_Cyclic_Detection_Flag, m_OneCut_Frams, m_Smooth_Flag, m_omni_R);
}

void SpoofingDoa::getCyclicDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int cutNum = (int)dataA.size();

    for (int j = 0; j < cutNum; ++j)
    {
        if (j < (int)m_cutSequence.size() && m_cutSequence[j][0] == m_cutSequence[j][1])
        {
            continue;
        }

        std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
        getSatelliteDataByType(dataA[j], dataT);

        std::map<int, std::set<int>> cutAlarms;
        for (auto &kv : dataT)
        {
            int typeInt = kv.first;
            vector<SatelliteDataPhaseDiffA> alarmSats;
            int alarm = 0;
            calAlarmByPhaseDiff(typeInt, kv.second, alarmSats, alarm);
            if (alarm)
            {
                std::set<int> sids;
                for (auto &s : alarmSats)
                {
                    sids.insert(s.i_Prn);
                }
                cutAlarms[typeInt] = sids;
            }
        }

        for (auto &kv : m_ConsecutiveAlarm)
        {
            if (cutAlarms.find(kv.first) == cutAlarms.end())
            {
                kv.second = 0;
                if (m_Tracking.find(kv.first) == m_Tracking.end())
                {
                    m_Baselines.erase(kv.first);
                }
            }
        }
        for (auto &kv : cutAlarms)
        {
            int typeInt = kv.first;
            int c = m_ConsecutiveAlarm[typeInt] + 1;
            m_ConsecutiveAlarm[typeInt] = c;
            if (c >= m_Detection_Recodds_Num)
            {
                TrackingInfo &t = m_Tracking[typeInt];
                for (int sid : kv.second)
                {
                    t.cluster_sats.insert(sid);
                }
            }
        }
    }

    for (int j = 0; j < cutNum; ++j)
    {
        vector<SatelliteDataPhaseDiffA> filtered;
        for (auto &sat : dataA[j])
        {
            int typeInt = TypeInt(sat.i_Sys, sat.i_Type);
            auto it = m_Tracking.find(typeInt);
            if (it != m_Tracking.end() &&
                it->second.cluster_sats.find(sat.i_Prn) != it->second.cluster_sats.end())
            {
                filtered.emplace_back(sat);
            }
        }
        dataA[j] = filtered;
    }
}

void SpoofingDoa::accumulateBaselines(const std::vector<SatelliteDataPhaseDiffB> &dataB)
{
    for (const auto &b : dataB)
    {
        int typeInt = TypeInt(b.i_Sys, b.i_Type);
        int prn = b.i_Prn;
        auto &inner = m_Baselines[typeInt];
        auto itp = inner.find(prn);
        if (itp == inner.end())
        {
            inner[prn] = b;
            continue;
        }
        SatelliteDataPhaseDiffB &acc = itp->second;
        acc.i_diffLen = b.i_diffLen;
        for (int j = 0; j < b.i_diffLen && j < 100; ++j)
        {
            if (b.i_Snr1[j] > 1e-6 && b.i_Snr2[j] > 1e-6)
            {
                acc.i_phase_diff[j] = b.i_phase_diff[j];
                acc.i_Snr1[j] = b.i_Snr1[j];
                acc.i_Snr2[j] = b.i_Snr2[j];
            }
        }
    }
}

void SpoofingDoa::getCrossCycleDataB(std::vector<SatelliteDataPhaseDiffB> &doaDataB)
{
    doaDataB.clear();
    for (const auto &kv : m_Tracking)
    {
        int typeInt = kv.first;
        const TrackingInfo &t = kv.second;
        auto itt = m_Baselines.find(typeInt);
        if (itt == m_Baselines.end())
        {
            continue;
        }
        for (int prn : t.cluster_sats)
        {
            auto itp = itt->second.find(prn);
            if (itp != itt->second.end())
            {
                doaDataB.emplace_back(itp->second);
            }
        }
    }
}

void SpoofingDoa::setCutSquence(int len, const int *cutSq)
{
    m_cutSequence.clear();
    vector<vector<int>>().swap(m_cutSequence);
    int size = len / 2;
    m_cutSequence.resize(size);
    int tp_index = 0;
    for (int i = 0; i < size; i++)
    {
        m_cutSequence[i].resize(2);
        tp_index = i * 2;
        if (cutSq[tp_index] < 1 || cutSq[tp_index + 1] < 1)
        {
            m_cutSequence.clear();
            return;
        }
        m_cutSequence[i][0] = cutSq[tp_index];
        m_cutSequence[i][1] = cutSq[tp_index + 1];
        PublicSpace::Log("set cutSequence:%d,m_cutSequence1 = %d,m_cutSequence2 = %d\n", i + 1, m_cutSequence[i][0], m_cutSequence[i][1]);
    }
}

void SpoofingDoa::saveGNSSData(const GNSSData *data, int dataLen)
{
    string fileName = "GNSSData_";
    fileName.append(to_string(dataLen));
    fileName.append(".dat");
    PublicSpace::saveArrayToBinary(fileName, data, dataLen);
}

int SpoofingDoa::TypeInt(int sys, int type)
{
    return sys * 100 + type;
}

void SpoofingDoa::initType(void)
{
    m_F[TypeInt(0, 0)] = 1575.42e6;
    m_F[TypeInt(0, 2)] = 1176.45e6;
    m_F[TypeInt(0, 5)] = 1227.6e6;
    m_F[TypeInt(0, 9)] = 1227.6e6;
    m_F[TypeInt(0, 14)] = 1176.45e6;
    m_F[TypeInt(0, 16)] = 1575.42e6;
    m_F[TypeInt(0, 17)] = 1227.6e6;

    m_F[TypeInt(1, 0)] = 1602.0e6;
    m_F[TypeInt(1, 1)] = 1246.0e6;
    m_F[TypeInt(1, 5)] = 1246.0e6;
    m_F[TypeInt(1, 6)] = 1277.85e6;

    m_F[TypeInt(2, 0)] = 1575.42e6;
    m_F[TypeInt(2, 6)] = 1176.45e6;

    m_F[TypeInt(3, 1)] = 1575.42e6;
    m_F[TypeInt(3, 2)] = 1575.42e6;
    m_F[TypeInt(3, 7)] = 1278.75e6;
    m_F[TypeInt(3, 12)] = 1176.45e6;
    m_F[TypeInt(3, 17)] = 1207.14e6;
    m_F[TypeInt(3, 20)] = 1191.795e6;

    m_F[TypeInt(4, 0)] = 1561.098e6;
    m_F[TypeInt(4, 17)] = 1207.14e6;
    m_F[TypeInt(4, 2)] = 1268.52e6;
    m_F[TypeInt(4, 8)] = 1575.42e6;
    m_F[TypeInt(4, 12)] = 1176.45e6;
    m_F[TypeInt(4, 19)] = 1207.14e6;

    m_F[TypeInt(4, 34)] = 1575.42e6;
    m_F[TypeInt(4, 49)] = 1268.52e6;
    m_F[TypeInt(4, 47)] = 1268.52e6;

    m_F[TypeInt(5, 0)] = 1575.42e6;
    m_F[TypeInt(5, 14)] = 1176.45e6;
    m_F[TypeInt(5, 17)] = 1227.6e6;
    m_F[TypeInt(5, 16)] = 1575.42e6;
}

void SpoofingDoa::initDetectionThreshold(int threshold, double phsThreshold){

    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        int intType = it->first;

        if (-1 != threshold){
            m_Detection_Threshold[intType] = threshold;
        }

        if (phsThreshold > 0){
            m_Detection_PhsThreshold[intType] = phsThreshold / 360.0;
        }

    }

}

void SpoofingDoa::setDetectionRecordNum(int num)
{
    if (num < 1 || num > 10)
    {
        m_Detection_Recodds_Num = 1;
    }
    else
    {
        m_Detection_Recodds_Num = num;
    }
}

/**
 * @brief 从单帧 GNSS 双通道数据提取逐星相位差
 * @param data  单帧 GNSS 数据(Port1/Port2 两通道)
 * @param dataA 输出: 各卫星的相位差、信噪比(按 PRN/系统/频点对齐)
 * @note 取两通道同名卫星的载波相位差(取小数部分, 归一化到 [0,1) 周)
 */
void SpoofingDoa::getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA)
{
    dataA.clear();
    int size1 = data.i_PortOneNum;
    int size2 = data.i_PortTwoNum;
    int typInt = 0;
    bool flg_E = true;
    vector<int> data2_index;
    data2_index.clear();
    for (int j = 0; j < size2; ++j)
    {
        data2_index.emplace_back(j);
    }
    for (int i = 0; i < size1; ++i)
    {
        SatelliteDataPhaseDiffA tp;
        flg_E = true;
        typInt = TypeInt(data.i_PortOne[i].i_Sys, data.i_PortOne[i].i_Type);
        if (m_F.find(typInt) == m_F.end())
        {
            continue;
        }

        int find = 0;
        for (int j = 0; j < i; ++j)
        {
            if (data.i_PortOne[j].i_Prn == data.i_PortOne[i].i_Prn && data.i_PortOne[j].i_Sys == data.i_PortOne[i].i_Sys && data.i_PortOne[j].i_Type == data.i_PortOne[i].i_Type)
            {
                ++find;
                break;
            }
        }
        if (find)
        {
            continue;
        }
        for (int j = 0; j < size2; ++j)
        {
            typInt = TypeInt(data.i_PortTwo[j].i_Sys, data.i_PortTwo[j].i_Type);
            if (m_F.find(typInt) == m_F.end())
            {
                flg_E = false;
                continue;
            }
            if (data.i_PortOne[i].i_Prn == data.i_PortTwo[j].i_Prn && data.i_PortOne[i].i_Sys == data.i_PortTwo[j].i_Sys && data.i_PortOne[i].i_Type == data.i_PortTwo[j].i_Type)
            {
                flg_E = false;

                if (data.i_PortOne[i].i_Snr >= m_Snr_Threshold && data.i_PortTwo[j].i_Snr >= m_Snr_Threshold)
                {
                    data2_index.erase(std::remove(data2_index.begin(), data2_index.end(), j), data2_index.end());

                    tp.i_Snr1 = data.i_PortOne[i].i_Snr;
                    tp.i_Snr2 = data.i_PortTwo[j].i_Snr;
                    tp.i_Prn = data.i_PortOne[i].i_Prn;
                    tp.i_Sys = data.i_PortOne[i].i_Sys;
                    tp.i_Type = data.i_PortOne[i].i_Type;

                    double phs_tp = data.i_PortOne[i].i_Phase - data.i_PortTwo[j].i_Phase;

                    double diff = phs_tp - (long long int)phs_tp;
                    if (diff < 0)
                    {
                        diff = diff + 1;
                    }
                    tp.i_phase_diff = diff;
                    dataA.emplace_back(tp);
                }
                break;
            }
        }
        if (flg_E)
        {
            tp.i_Snr1 = data.i_PortOne[i].i_Snr;
            tp.i_Snr2 = 0;
            tp.i_Prn = data.i_PortOne[i].i_Prn;
            tp.i_Sys = data.i_PortOne[i].i_Sys;
            tp.i_Type = data.i_PortOne[i].i_Type;
            tp.i_phase_diff = -1;
            dataA.emplace_back(tp);
        }
    }
    for (int j = 0; j < (int)data2_index.size(); ++j)
    {
        typInt = TypeInt(data.i_PortTwo[j].i_Sys, data.i_PortTwo[j].i_Type);
        if (m_F.find(typInt) == m_F.end())
        {
            continue;
        }
        SatelliteDataPhaseDiffA tp;
        tp.i_Snr1 = 0;
        tp.i_Snr2 = data.i_PortTwo[data2_index[j]].i_Snr;
        tp.i_Prn = data.i_PortTwo[data2_index[j]].i_Prn;
        tp.i_Sys = data.i_PortTwo[data2_index[j]].i_Sys;
        tp.i_Type = data.i_PortTwo[data2_index[j]].i_Type;
        tp.i_phase_diff = -1;
        dataA.emplace_back(tp);
    }
}

void SpoofingDoa::getSatelliteDataPhaseDiffB(const vector<vector<SatelliteDataPhaseDiffA>> &dataA, vector<SatelliteDataPhaseDiffB> &dataB)
{
    vector<SatelliteDataPhaseDiffB> tp_dataB;
    tp_dataB.clear();
    int size = (int)dataA.size();
    SatelliteDataPhaseDiffA tpA;
    SatelliteDataPhaseDiffB tpB;
    for (int i = 0; i < size; i++)
    {
        int size2 = (int)dataA[i].size();
        for (int j = 0; j < size2; j++)
        {
            tpA = dataA[i][j];
            bool flg = true;
            int tpBsize = (int)tp_dataB.size();
            for (unsigned int k = 0; k < tpBsize; k++)
            {

                if (tpA.i_Sys == tp_dataB[k].i_Sys && tpA.i_Type == tp_dataB[k].i_Type && tpA.i_Prn == tp_dataB[k].i_Prn)
                {
                    tp_dataB[k].i_Snr1[i] = tpA.i_Snr1;
                    tp_dataB[k].i_Snr2[i] = tpA.i_Snr2;
                    tp_dataB[k].i_phase_diff[i] = tpA.i_phase_diff;
                    flg = false;
                    break;
                }
            }
            if (flg)
            {
                clearSatelliteDataPhaseDiffB(tpB);
                tpB.i_Sys = tpA.i_Sys;
                tpB.i_Type = tpA.i_Type;
                tpB.i_Prn = tpA.i_Prn;
                tpB.i_Snr1[i] = tpA.i_Snr1;
                tpB.i_Snr2[i] = tpA.i_Snr2;
                tpB.i_phase_diff[i] = tpA.i_phase_diff;
                tpB.i_diffLen = size;
                tp_dataB.emplace_back(tpB);
            }
        }
    }
    if (1 == m_Delete_Prn_Flag)
    {
        bool tp_flg = true;
        for (int i = 0; i < (int)tp_dataB.size(); i++)
        {
            tp_flg = true;
            tpB = tp_dataB[i];
            for (int j = 0; j < tpB.i_diffLen; j++)
            {
                if (tpB.i_Snr1[j] < 1e-6 || tpB.i_Snr2[j] < 1e-6)
                {
                    tp_flg = false;
                    break;
                }
            }
            if (tp_flg)
            {
                dataB.emplace_back(tpB);
            }
        }
    }
    else
    {
        dataB = tp_dataB;
    }
    tp_dataB.clear();
}

void SpoofingDoa::getSatelliteDataByType(const std::vector<SatelliteDataPhaseDiffA> &dataA, std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT){
    int typeInt = 0;
    SatelliteDataPhaseDiffA tp;
    vector<SatelliteDataPhaseDiffA> tpVec;
    for (unsigned int i = 0; i < dataA.size(); i++){

        tp = dataA[i];
        typeInt = TypeInt(tp.i_Sys, tp.i_Type);

        if (dataT.find(typeInt) == dataT.end()){
            tpVec.clear();
            tpVec.push_back(tp);
            dataT[typeInt] = tpVec;
        }
        else{
            dataT[typeInt].push_back(tp);
        }

    }
}

void SpoofingDoa::clearSatelliteDataPhaseDiffB(SatelliteDataPhaseDiffB &dataB)
{
    dataB.i_Prn = -1;
    dataB.i_Sys = -1;
    dataB.i_Type = -1;
    dataB.i_diffLen = 0;
    for (int i = 0; i < 100; i++)
    {
        dataB.i_Snr1[i] = 0.0;
        dataB.i_Snr2[i] = 0.0;
        dataB.i_phase_diff[i] = 0.0;
    }
}

void SpoofingDoa::setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA)
{
    if (dataA.size() != m_cutSequence.size())
    {
        return;
    }

    vector<vector<SatelliteDataPhaseDiffA>> calCuts;
    for (unsigned int i = 0; i < dataA.size(); i++)
    {
        if (m_cutSequence[i][0] == m_cutSequence[i][1])
        {
            calCuts.emplace_back(dataA[i]);
        }
    }
    calCorrectionOffset(calCuts);
}

void SpoofingDoa::calCorrectionOffset(const vector<vector<SatelliteDataPhaseDiffA>> &calCuts)
{
    map<int, map<int, vector<double>>> samples;
    map<int, map<int, double>> sumSnr;
    int nCalCuts = 0;

    for (unsigned int c = 0; c < calCuts.size(); ++c)
    {
        ++nCalCuts;
        for (const auto &sat : calCuts[c])
        {
            if (sat.i_Snr1 < 1e-3 || sat.i_Snr2 < 1e-3)
            {
                continue;
            }
            int typeInt = TypeInt(sat.i_Sys, sat.i_Type);
            int prn = sat.i_Prn;
            samples[typeInt][prn].emplace_back(sat.i_phase_diff * 360.0);
            sumSnr[typeInt][prn] += (sat.i_Snr1 + sat.i_Snr2) / 2.0;
        }
    }

    if (nCalCuts <= 0)
    {
        return;
    }
    int required = (nCalCuts < MIN_STABLE_SAMPLES) ? nCalCuts : MIN_STABLE_SAMPLES;

    map<int, map<int, SatelliteDataPhaseDiffA>> newCorrection;
    map<int, vector<double>> freqVals;

    for (auto &tkv : samples)
    {
        int typeInt = tkv.first;
        int sys = typeInt / 100;
        for (auto &skv : tkv.second)
        {
            int prn = skv.first;
            const vector<double> &samp = skv.second;

            if (circularSpanDeg(samp) >= STABILITY_RANGE_DEG)
            {
                continue;
            }
            if ((int)samp.size() < required)
            {
                continue;
            }

            double meanDeg = circularMeanDeg(samp);
            double meanSnr = sumSnr[typeInt][prn] / samp.size();

            SatelliteDataPhaseDiffA tp;
            tp.i_Sys = sys;
            tp.i_Type = typeInt % 100;
            tp.i_Prn = prn;
            double cyc = fmod(meanDeg / 360.0, 1.0);
            if (cyc < 0)
            {
                cyc += 1.0;
            }
            tp.i_phase_diff = cyc;
            tp.i_Snr1 = meanSnr;
            tp.i_Snr2 = meanSnr;

            if (sys == 1)
            {
                newCorrection[typeInt][prn] = tp;
            }
            else
            {
                freqVals[typeInt].emplace_back(meanDeg);
            }
        }
    }

    for (auto &fv : freqVals)
    {
        int typeInt = fv.first;
        double meanDeg = circularMeanDeg(fv.second);
        double cyc = fmod(meanDeg / 360.0, 1.0);
        if (cyc < 0)
        {
            cyc += 1.0;
        }
        SatelliteDataPhaseDiffA tp;
        tp.i_Sys = typeInt / 100;
        tp.i_Type = typeInt % 100;
        tp.i_Prn = -1;
        tp.i_phase_diff = cyc;
        tp.i_Snr1 = 0;
        tp.i_Snr2 = 0;
        newCorrection[typeInt][-1] = tp;
    }

    if (newCorrection.empty())
    {
        return;
    }

    m_CorrectionData.swap(newCorrection);

    for (auto it = m_CorrectionData.begin(); it != m_CorrectionData.end(); ++it)
    {
        map<int, SatelliteDataPhaseDiffA> tpA;
        tpA = it->second;
        for (auto itt = tpA.begin(); itt != tpA.end(); ++itt)
        {
            SatelliteDataPhaseDiffA tp = itt->second;
            LogSatelliteDataPhaseDiffA(tp);
        }
    }
}

void SpoofingDoa::getCorrectedGnssData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int size = (int)dataA.size();
    for (int i = 0; i < size; i++)
    {
        for (unsigned int j = 0; j < dataA[i].size(); j++)
        {
            calCorrecteData(dataA[i][j]);
        }
    }
}

void SpoofingDoa::calCorrecteData(SatelliteDataPhaseDiffA &dataA)
{
    int typeInt = TypeInt(dataA.i_Sys, dataA.i_Type);
    int prn = dataA.i_Prn;

    if (dataA.i_Snr1 < 1e-3 || dataA.i_Snr2 < 1e-3)
    {
        return;
    }

    auto it = m_CorrectionData.find(typeInt);
    if (it == m_CorrectionData.end())
    {
        return;
    }

    const map<int, SatelliteDataPhaseDiffA> &tp_prnData = it->second;
    const SatelliteDataPhaseDiffA *off = nullptr;
    if (dataA.i_Sys == 1)
    {
        auto ps = tp_prnData.find(prn);
        if (ps != tp_prnData.end())
        {
            off = &(ps->second);
        }
    }
    else
    {
        auto pf = tp_prnData.find(-1);
        if (pf != tp_prnData.end())
        {
            off = &(pf->second);
        }
    }
    if (off != nullptr)
    {
        dataA.i_phase_diff = dataA.i_phase_diff - off->i_phase_diff;
    }
}

/**
 * @brief 相关干涉仪测向调度: 组测向输入信息 → 逐星测向
 * @param dataB 各星相位差数据
 * @note 按天线类型(全向/定向)选择不同的组信息方式, 再统一调用 calAngle 逐星测向
 */
void SpoofingDoa::getResultInterferDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    string nowT = getNowTime();
    std::map<int, std::map<int, InterferInfo>> inferInfoData;
    PublicSpace::Log("set Interfer Info Data Omni   %s\n", nowT.c_str());
    setInterferInfoDataOmni(dataB, inferInfoData);
    nowT = getNowTime();
    PublicSpace::Log("cal angle    %s\n", nowT.c_str());
    calAngle(inferInfoData);
}

/**
 * @brief 建立相关干涉仪理论相位差模板
 * @note 对每个频点调用 calPhaseTheory 生成 [360角度][阵元] 理论模板
 */
void SpoofingDoa::initTheory(void){
    int typeInt = 0;
    double f = 0;
    double r = 0.0;

    vector<vector<double>> theory;
    for (auto it = m_F.begin(); it != m_F.end(); ++it)
    {
        theory.clear();
        typeInt = it->first;
        f = it->second;
        r = m_R[typeInt];
        ArithmeticDoa::calPhaseTheory(f, r, m_AntennaNum, theory);

        m_Theory[typeInt] = theory;
    }
}

void SpoofingDoa::setInterferInfoDataOmni(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData)
{
    int size = (int)dataB.size();
    int typInt = 0;
    int prn = 0;
    SatelliteDataPhaseDiffB tp1;
    int startAngle = 0;
    int endAngle = 359;
    for (int i = 0; i < size; i++)
    {
        tp1 = dataB[i];
        typInt = TypeInt(tp1.i_Sys, tp1.i_Type);
        prn = tp1.i_Prn;

        InterferInfo tp2;
        int flg;
        calAngleUseAntenna(tp1, tp2, flg);
        if (0 == flg)
        {
            continue;
        }
        tp2.i_Start = startAngle;
        tp2.i_End = endAngle;
        inferInfoData[typInt][prn] = tp2;
    }
    Log("set Interfer Info Data sucess....\n");
}

