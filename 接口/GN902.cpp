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
//
// 新增: GN902 独立日志(与 PublicSpace::Log 分离)。
//   - 由外部 .txt 配置文件控制开关;
//   - 未读到配置文件 -> 默认不打印独立日志;
//   - 日志文件路径也写在配置文件里(可自定义命名)。
//
// 新增: cutCountThreshold (连续切刀数, 对应引擎 m_Detection_Recodds_Num)
//   - 改为通过接口 SetCutnumThreshold_GN902 传入(GN902::SetCutnumThreshold);
//   - 取值 1..10; 不设置(<=0)时使用引擎默认值(2);
//   - 不再从 txt 配置文件读取。
// =============================================================================
#include "GN902.h"
#include <cctype>      // 新增: std::tolower

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

    // printf 格式检查(GCC/Clang): 让编译器在编译期核对可变参数与格式串是否匹配。
#if defined(__GNUC__) || defined(__clang__)
    void Log(const char *format, ...) __attribute__((format(printf, 1, 2)));
#else
    void Log(const char *format, ...);
#endif

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
 */
void ArithmeticDoa::calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2)
{
    diff2.resize(360);
    vector<double> diff;
    int startAngle = (int)data.i_Start;
    int endAngle = (int)data.i_End;
    int size = data.i_Phase_Len;
    if (size > 200)
    {
        size = 200;
    }
    if (size < 0)
    {
        size = 0;
    }
    double max_val = -9999.9;
    int ang_val = 0;
    diff.clear();
    vector<double>().swap(diff);
    diff.resize(360);
    int tp_ang = 0;
    for (int ang = startAngle; ang < endAngle + 1; ang++)
    {
        double sumDiff = 0.0;
        tp_ang = Round360(ang);
        if (tp_ang < 0 || tp_ang >= (int)phaseTheory.size())
        {
            continue;
        }
        for (int i = 0; i < size; i++)
        {
            int antn1 = data.i_AntennaSq[i][0];
            int antn2 = data.i_AntennaSq[i][1];
            if (antn1 < 1 || antn2 < 1 ||
                antn1 > (int)phaseTheory[tp_ang].size() ||
                antn2 > (int)phaseTheory[tp_ang].size())
            {
                continue;
            }
            double theoryPhase = phaseTheory[tp_ang][antn1 - 1] - phaseTheory[tp_ang][antn2 - 1];
            sumDiff = sumDiff + cos(theoryPhase - data.i_Phase_Diff[i]);
        }
        if (sumDiff > max_val)
        {
            max_val = sumDiff;
            ang_val = tp_ang;
        }
        diff[tp_ang] = sumDiff;
        diff2[tp_ang] = (sumDiff / size + 1) / 2;
    }

    angle = Round360(ang_val);
    if (size > 0)
    {
        quality = (max_val / size + 1.0) / 2.0 * 100.0;
    }
    else
    {
        quality = 0.0;
    }
    for (int i = 0; i < 360; i++)
    {
        diff2[i] = (diff[i] / size + 1.0) / 2.0 * 100.0;
    }
}

/**
 * @brief 计算均匀圆阵的理论相位差模板
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

SpoofingDoa::SpoofingDoa(void){
    Init();
    setThresholdDetectionDoa(0, 2, 3, -1);    // GPS L5
    setThresholdDetectionDoa(1, 0, 2, 30.0);   // GLONASS G1
    setThresholdDetectionDoa(1, 1, 3, 30.0);   // GLONASS G2
    setThresholdDetectionDoa(3, 2, 3, 5.0);   // Galileo E1C
    setThresholdDetectionDoa(3, 12, 3, 5.0);  // Galileo E5a
    setThresholdDetectionDoa(3, 17, 3, 5.0);  // Galileo E5b
    setThresholdDetectionDoa(4, 17, 3, -1);   // BDS B2I
    setThresholdDetectionDoa(4, 0, 3, -1);    // BDS B1I
    setThresholdDetectionDoa(4, 2, 3, -1);    // BDS B3I
    setThresholdDetectionDoa(4, 8, 2, -1);    // BDS B1C
    setThresholdDetectionDoa(4, 19, 3, -1);   // BDS B2b
    setThresholdDetectionDoa(4, 34, 3, 5.0);  // BDS B1X
}

void SpoofingDoa::Init(void){

    m_CorrectionData.clear();
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);

    m_LogFile = "./spoofingDoaLog_";
    PublicSpace::m_logFlg = 0;
    LogCreat(m_LogFile);

    m_AntennaNum = 7;
    m_omni_R = 0.1865;

    m_cutSequence = { {7,7}, {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7} };

    m_OneCut_Frams = 8;
    m_Smooth_Flag = 1;

    m_Doa_Cut_min_Num = 6;
    if (m_Doa_Cut_min_Num >= (int)m_cutSequence.size()) {
        m_Doa_Cut_min_Num = (int)m_cutSequence.size() - 1;
    }

    m_Phasediff_Threshold = 5.0;
    m_Detection_Threshold_Num = 2;
    m_Snr_Threshold = 35.0;
    m_Qulity_Threshold = 10.0;

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

void SpoofingDoa::collectPhaseDiffData(const GNSSData &data, std::vector<SatelliteDataPhaseDiffA> &dataA)
{
    vector<SatelliteDataPhaseDiffA> raw;
    getSatelliteDataPhaseDiffA(data, raw, false);
    dataA.clear();
    for (size_t i = 0; i < raw.size(); i++)
    {
        if (raw[i].i_phase_diff < 0)
        {
            continue;
        }
        dataA.emplace_back(raw[i]);
    }
}

int SpoofingDoa::getAngleSpoofingDoa(SpoofingResult &result)
{
    setSpoofingResult(result);
    cout << " changdu::"<< sizeof(SpoofingResult) << endl;
    PublicSpace::Log("Doa result:\n");
    LogSpoofingResult(result);
    return 0;
}

void SpoofingDoa::setSpoofingResult(SpoofingResult &result)
{
    int count = 0;
    for (auto &kv : m_Tracking)
    {
        if (count >= 24)
        {
            break;
        }
        int typeInt = kv.first;
        const TrackingInfo &t = kv.second;
        auto it = m_AngleResultData.find(typeInt);

        result.i_SatelliteAngle[count].i_Sys = typeInt / 100;
        result.i_SatelliteAngle[count].i_Type = typeInt % 100;
        result.i_SatelliteAngle[count].i_Alarm = 1;

        std::set<int> sats = t.cluster_sats;
        if (sats.empty() && it != m_AngleResultData.end())
        {
            for (unsigned int i = 0; i < it->second.size(); i++)
            {
                sats.insert(it->second[i].i_Prn);
            }
        }

        vector<double> doas;
        int n = 0;
        for (int prn : sats)
        {
            if (n >= 32)
            {
                break;
            }
            AlarmData ad;
            ad.i_Prn = prn;
            ad.i_Snr = (float)getSatMaxSnr(typeInt, prn);
            ad.i_Angle = -1;
            ad.i_Quality = -1.0;

            if (it != m_AngleResultData.end())
            {
                for (unsigned int i = 0; i < it->second.size(); i++)
                {
                    if (it->second[i].i_Prn == prn)
                    {
                        ad.i_Angle = it->second[i].i_Angle;
                        ad.i_Quality = it->second[i].i_Quality;
                        doas.push_back(ad.i_Angle);
                        break;
                    }
                }
            }
            result.i_SatelliteAngle[count].i_AlarmData[n] = ad;
            ++n;
        }
        result.i_SatelliteAngle[count].i_Count = n;

        if (doas.empty())
        {
            result.i_SatelliteAngle[count].i_Angle = -1;
        }
        else
        {
            double angle = circularMeanDeg(doas);
            if (angle < 0)
            {
                angle += 360.0;
            }
            result.i_SatelliteAngle[count].i_Angle = angle;
        }
        ++count;
    }
    result.i_Count = count;
}

double SpoofingDoa::getSatMaxSnr(int typeInt, int prn)
{
    double snr = 0.0;
    auto its = m_Baselines.find(typeInt);
    if (its != m_Baselines.end())
    {
        auto itp = its->second.find(prn);
        if (itp != its->second.end())
        {
            const SatelliteDataPhaseDiffB &b = itp->second;
            for (int j = 0; j < b.i_diffLen && j < 100; ++j)
            {
                if (b.i_Snr1[j] > snr)
                {
                    snr = b.i_Snr1[j];
                }
                if (b.i_Snr2[j] > snr)
                {
                    snr = b.i_Snr2[j];
                }
            }
        }
    }
    if (snr <= 0.0)
    {
        auto itm = m_Max_Snr.find(typeInt);
        if (itm != m_Max_Snr.end())
        {
            auto itmp = itm->second.find(prn);
            if (itmp != itm->second.end())
            {
                snr = itmp->second;
            }
        }
    }
    return snr;
}

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
        bool isCalCut = (i < (int)m_cutSequence.size() && m_cutSequence[i][0] == m_cutSequence[i][1]);
        getSatelliteDataPhaseDiffA(data[i], tp, !isCalCut);
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
    int savedDeletePrnFlag = m_Delete_Prn_Flag;
    m_Delete_Prn_Flag = 0;
    getSatelliteDataPhaseDiffB(dataA, dataB);
    m_Delete_Prn_Flag = savedDeletePrnFlag;
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
                    doas.push_back(ad.i_Angle);
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

void SpoofingDoa::calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg)
{
    doaFlg = 1;
    int diffLen = dataB.i_diffLen;
    if (diffLen < 0)
    {
        diffLen = 0;
    }
    if (diffLen > 100)
    {
        diffLen = 100;
    }
    if (diffLen > (int)m_cutSequence.size())
    {
        diffLen = (int)m_cutSequence.size();
    }
    int count = 0;
    float maxSnr = 0.0f;

    vector<vector<int>> tp_antnna;
    vector<double> tp_diff;

    for (int j = 0; j < diffLen; j++)
    {
        if (dataB.i_Snr1[j] < 1e-6 || dataB.i_Snr2[j] < 1e-6)
        {
            continue;
        }
        int ant1 = m_cutSequence[j][0];
        int ant2 = m_cutSequence[j][1];
        if (ant1 == ant2)
        {
            continue;
        }
        if (ant1 < 1 || ant2 < 1 || ant1 > m_AntennaNum || ant2 > m_AntennaNum)
        {
            continue;
        }

        if (dataB.i_Snr1[j] > maxSnr)
        {
            maxSnr = dataB.i_Snr1[j];
        }
        if (dataB.i_Snr2[j] > maxSnr)
        {
            maxSnr = dataB.i_Snr2[j];
        }
        tp_antnna.emplace_back(m_cutSequence[j]);
        tp_diff.emplace_back(dataB.i_phase_diff[j]);
        ++count;
    }
    if (count < m_Doa_Cut_min_Num)
    {
        doaFlg = 0;
        return;
    }
    int size = (int)tp_diff.size();
    int prn = dataB.i_Prn;
    int typeInt = TypeInt(dataB.i_Sys, dataB.i_Type);
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
    if (cutNum <= 0 || m_OneCut_Frams <= 0)
    {
        return;
    }
    if ((int)dataA.size() < cutNum * m_OneCut_Frams)
    {
        PublicSpace::Log("error: getSmoothData dataA.size()=%d < cutNum*oneCutFrams=%d\n",
                         (int)dataA.size(), cutNum * m_OneCut_Frams);
        return;
    }

    vector<SatelliteDataPhaseDiffB> dataB;
    vector<vector<SatelliteDataPhaseDiffA>> oneCutData;
    oneCutData.resize(m_OneCut_Frams);
    vector<vector<SatelliteDataPhaseDiffA>> resultData;
    SatelliteDataPhaseDiffA tpA;
    for (int j = 0; j < cutNum; j++)
    {
        dataB.clear();
        if ((int)oneCutData.size() != m_OneCut_Frams)
        {
            oneCutData.resize(m_OneCut_Frams);
        }
        for (int k = 0; k < m_OneCut_Frams; k++)
        {
            int index = j * m_OneCut_Frams + k;
            oneCutData[k] = dataA[index];
        }
        int savedDeleteFlag = m_Delete_Prn_Flag;
        m_Delete_Prn_Flag = 0;
        getSatelliteDataPhaseDiffB(oneCutData, dataB);
        m_Delete_Prn_Flag = savedDeleteFlag;

        vector<SatelliteDataPhaseDiffA> tpA2;
        tpA2.reserve(dataB.size());
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
    if (m_OneCut_Frams <= 0 || cutNum <= 0 ||
        (int)dataA.size() < cutNum * m_OneCut_Frams)
    {
        PublicSpace::Log("error: getEndFramData dataA.size()=%d < cutNum*oneCutFrams=%d\n",
                         (int)dataA.size(), cutNum * m_OneCut_Frams);
        return;
    }
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
    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        m_R[it->first] = m_omni_R;
    }
}

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
    m_AlarmMoments.clear();

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

        vector<SatelliteDataPhaseDiffA> filtered;
        for (auto &sat : dataA[j])
        {
            int typeInt = TypeInt(sat.i_Sys, sat.i_Type);
            auto ca = cutAlarms.find(typeInt);
            if (ca != cutAlarms.end() &&
                ca->second.find(sat.i_Prn) != ca->second.end())
            {
                filtered.emplace_back(sat);
            }
        }
        dataA[j] = filtered;

        if (filtered.empty())
        {
            continue;
        }

        {
            vector<vector<SatelliteDataPhaseDiffA>> oneCut(cutNum);
            oneCut[j] = filtered;
            vector<SatelliteDataPhaseDiffB> cutDataB;
            int savedDeletePrnFlag = m_Delete_Prn_Flag;
            m_Delete_Prn_Flag = 0;
            getSatelliteDataPhaseDiffB(oneCut, cutDataB);
            m_Delete_Prn_Flag = savedDeletePrnFlag;
            accumulateBaselines(cutDataB);
        }

        for (auto &kv : cutAlarms)
        {
            int typeInt = kv.first;

            AlarmMoment am = {};
            am.i_Cut = j;
            am.i_Sys = typeInt / 100;
            am.i_Type = typeInt % 100;
            am.i_Alarm = 1;
            am.i_Angle = -1;
            am.i_Count = 0;

            vector<SatelliteDataPhaseDiffB> doaDataB;
            getCrossCycleDataBByType(typeInt, kv.second, doaDataB);

            m_AngleResultData.clear();
            if (!doaDataB.empty())
            {
                getResultInterferDoa(doaDataB);
            }

            const vector<AlarmData> *alarms = 0;
            auto ita = m_AngleResultData.find(typeInt);
            if (ita != m_AngleResultData.end())
            {
                alarms = &ita->second;
            }

            vector<double> doas;
            int n = 0;
            for (int prn : kv.second)
            {
                if (n >= 32)
                {
                    break;
                }
                AlarmData ad;
                ad.i_Prn = prn;
                ad.i_Snr = (float)getSatMaxSnr(typeInt, prn);
                ad.i_Angle = -1;
                ad.i_Quality = -1;
                if (alarms != 0)
                {
                    for (unsigned int i = 0; i < alarms->size(); i++)
                    {
                        if ((*alarms)[i].i_Prn == prn)
                        {
                            ad.i_Angle = (*alarms)[i].i_Angle;
                            ad.i_Quality = (*alarms)[i].i_Quality;
                            doas.push_back(ad.i_Angle);
                            break;
                        }
                    }
                }
                am.i_AlarmData[n] = ad;
                ++n;
            }
            am.i_Count = n;

            if (!doas.empty())
            {
                double angle = circularMeanDeg(doas);
                if (angle < 0)
                {
                    angle += 360.0;
                }
                am.i_Angle = angle;
            }
            m_AlarmMoments.emplace_back(am);
        }
    }
}

const std::vector<AlarmMoment> &SpoofingDoa::getAlarmMoments(void) const
{
    return m_AlarmMoments;
}

void SpoofingDoa::accumulateBaselines(const std::vector<SatelliteDataPhaseDiffB> &dataB)
{
    for (const auto &b : dataB)
    {
        int typeInt = TypeInt(b.i_Sys, b.i_Type);
        int prn = b.i_Prn;

        auto itt = m_Tracking.find(typeInt);
        if (itt == m_Tracking.end()) continue;
        if (itt->second.cluster_sats.find(prn) == itt->second.cluster_sats.end()) continue;

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
void SpoofingDoa::getCrossCycleDataBByType(int typeInt, const std::set<int> &sats,
                                           std::vector<SatelliteDataPhaseDiffB> &doaDataB)
{
    doaDataB.clear();
    auto itt = m_Baselines.find(typeInt);
    if (itt == m_Baselines.end())
    {
        return;
    }
    for (int prn : sats)
    {
        auto itp = itt->second.find(prn);
        if (itp != itt->second.end())
        {
            doaDataB.emplace_back(itp->second);
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

void SpoofingDoa::getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA, bool snrFilter)
{
    dataA.clear();
    int size1 = data.i_PortOneNum;
    int size2 = data.i_PortTwoNum;
    if (size1 < 0 || size1 > GN902_MAX_PORT_SAT)
    {
        PublicSpace::Log("warn: i_PortOneNum=%d out of range, clamped\n", size1);
        size1 = (size1 < 0) ? 0 : GN902_MAX_PORT_SAT;
    }
    if (size2 < 0 || size2 > GN902_MAX_PORT_SAT)
    {
        PublicSpace::Log("warn: i_PortTwoNum=%d out of range, clamped\n", size2);
        size2 = (size2 < 0) ? 0 : GN902_MAX_PORT_SAT;
    }
    int typInt = 0;
    bool flg_E = true;
    vector<int> data2_index;
    data2_index.clear();
    data2_index.reserve(size2);
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

                if (!snrFilter || (data.i_PortOne[i].i_Snr >= m_Snr_Threshold && data.i_PortTwo[j].i_Snr >= m_Snr_Threshold))
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

void SpoofingDoa::getSatelliteDataPhaseDiffB(
        const vector<vector<SatelliteDataPhaseDiffA>> &dataA,
        vector<SatelliteDataPhaseDiffB> &dataB)
{
    vector<SatelliteDataPhaseDiffB> tp_dataB;
    tp_dataB.clear();

    int size = (int)dataA.size();

    const int MAX_CUT = 100;
    if (size > MAX_CUT)
    {
        PublicSpace::Log("warning: dataA.size()=%d > %d, truncated\n",
                         size, MAX_CUT);
        cout << "warning: dataA.size()=" << size
             << " > " << MAX_CUT << ", truncated" << endl;
        size = MAX_CUT;
    }

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
                if (tpA.i_Sys == tp_dataB[k].i_Sys &&
                    tpA.i_Type == tp_dataB[k].i_Type &&
                    tpA.i_Prn == tp_dataB[k].i_Prn)
                {
                    tp_dataB[k].i_Snr1[i]       = tpA.i_Snr1;
                    tp_dataB[k].i_Snr2[i]       = tpA.i_Snr2;
                    tp_dataB[k].i_phase_diff[i] = tpA.i_phase_diff;
                    flg = false;
                    break;
                }
            }
            if (flg)
            {
                clearSatelliteDataPhaseDiffB(tpB);
                tpB.i_Sys  = tpA.i_Sys;
                tpB.i_Type = tpA.i_Type;
                tpB.i_Prn  = tpA.i_Prn;
                tpB.i_Snr1[i]       = tpA.i_Snr1;
                tpB.i_Snr2[i]       = tpA.i_Snr2;
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

const double SpoofingDoa::CNR_MIN_DB = 35.0;
const double SpoofingDoa::STABILITY_RANGE_DEG = 10.0;
const int SpoofingDoa::MIN_STABLE_SAMPLES = 3;

double SpoofingDoa::normalizeAngle180(double deg)
{
    double r = fmod(deg + 180.0, 360.0);
    if (r < 0)
    {
        r += 360.0;
    }
    return r - 180.0;
}

double SpoofingDoa::circularMeanDeg(const std::vector<double> &degs)
{
    double s = 0.0;
    double c = 0.0;
    for (double d : degs)
    {
        double rad = d * PI / 180.0;
        s += sin(rad);
        c += cos(rad);
    }
    return atan2(s, c) * 180.0 / PI;
}

double SpoofingDoa::circularSpanDeg(const std::vector<double> &degs)
{
    int n = (int)degs.size();
    if (n <= 1)
    {
        return 0.0;
    }
    vector<double> a;
    a.reserve(n);
    for (double d : degs)
    {
        a.push_back(normalizeAngle180(d));
    }
    sort(a.begin(), a.end());
    double maxGap = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double gap = fmod(a[(i + 1) % n] - a[i], 360.0);
        if (gap < 0)
        {
            gap += 360.0;
        }
        if (gap > maxGap)
        {
            maxGap = gap;
        }
    }
    return 360.0 - maxGap;
}

double SpoofingDoa::circularSpan180Deg(const std::vector<double> &degs)
{
    int n = (int)degs.size();
    if (n <= 1)
    {
        return 0.0;
    }
    vector<double> a;
    a.reserve(n);
    for (double d : degs)
    {
        double r = fmod(d, 180.0);
        if (r < 0)
        {
            r += 180.0;
        }
        a.push_back(r);
    }
    sort(a.begin(), a.end());
    double maxGap = 0.0;
    for (int i = 0; i < n; ++i)
    {
        double gap = fmod(a[(i + 1) % n] - a[i], 180.0);
        if (gap < 0)
        {
            gap += 180.0;
        }
        if (gap > maxGap)
        {
            maxGap = gap;
        }
    }
    return 180.0 - maxGap;
}

void SpoofingDoa::calAlarmByPhaseDiff(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm)
{
    alarmSatelliteData.clear();
    alarm = 0;

    double phsThresholdDeg = m_Detection_PhsThreshold[typeInt] * 360.0;

    vector<pair<double, int>> valid;
    for (unsigned int i = 0; i < dataA.size(); ++i)
    {
        if (dataA[i].i_Snr1 < CNR_MIN_DB || dataA[i].i_Snr2 < CNR_MIN_DB)
        {
            continue;
        }
        double ph = normalizeAngle180(dataA[i].i_phase_diff * 360.0);
        valid.emplace_back(ph, (int)i);
    }

    int n = (int)valid.size();
    if (n < 2)
    {
        return;
    }

    sort(valid.begin(), valid.end());
    vector<pair<double, int>> ext;
    ext.reserve(2 * n);
    for (const auto &p : valid)
    {
        ext.emplace_back(p.first, p.second);
    }
    for (const auto &p : valid)
    {
        ext.emplace_back(p.first + 360.0, p.second);
    }

    int bestCount = 0;
    vector<int> bestIds;
    int left = 0;
    for (int right = 0; right < (int)ext.size(); ++right)
    {
        while (ext[right].first - ext[left].first >= phsThresholdDeg)
        {
            ++left;
        }
        while (right - left + 1 > n)
        {
            ++left;
        }
        set<int> ids;
        for (int k = left; k <= right; ++k)
        {
            ids.insert(ext[k].second);
        }
        if ((int)ids.size() > bestCount)
        {
            bestCount = (int)ids.size();
            bestIds.assign(ids.begin(), ids.end());
        }
    }

    if (bestCount > m_Detection_Threshold[typeInt])
    {
        alarm = 1;
        for (int idx : bestIds)
        {
            alarmSatelliteData.emplace_back(dataA[idx]);
        }
    }
}

void SpoofingDoa::LogSpoofingResult(const SpoofingResult result)
{
    string nowT = getNowTime();
    PublicSpace::Log(" %s   Spoofing num = %d\n", nowT.c_str(), result.i_Count);
    for (int i = 0; i < result.i_Count; i++)
    {
        PublicSpace::Log("Sys=%d,Type=%d,Count=%d,Angle=%.2f\n",
                         result.i_SatelliteAngle[i].i_Sys, result.i_SatelliteAngle[i].i_Type, result.i_SatelliteAngle[i].i_Count, result.i_SatelliteAngle[i].i_Angle);
        PublicSpace::Log("{\n");
        for (int j = 0; j < result.i_SatelliteAngle[i].i_Count; j++)
        {
            PublicSpace::Log("Prn=%d,Snr=%.1f,Angle=%d,Quality=%.2f;\n",
                             result.i_SatelliteAngle[i].i_AlarmData[j].i_Prn, result.i_SatelliteAngle[i].i_AlarmData[j].i_Snr, result.i_SatelliteAngle[i].i_AlarmData[j].i_Angle, result.i_SatelliteAngle[i].i_AlarmData[j].i_Quality);
        }
        PublicSpace::Log("}\n");
    }
}

void SpoofingDoa::LogGNSSData(const GNSSData data, int n)
{
    SatelliteData tp;
    if (0 != m_Save_Original_Flg)
    {
        int size1 = data.i_PortOneNum;
        int size2 = data.i_PortTwoNum;
        if (size1 < 0 || size1 > GN902_MAX_PORT_SAT)
        {
            size1 = (size1 < 0) ? 0 : GN902_MAX_PORT_SAT;
        }
        if (size2 < 0 || size2 > GN902_MAX_PORT_SAT)
        {
            size2 = (size2 < 0) ? 0 : GN902_MAX_PORT_SAT;
        }
        PublicSpace::Log("%d-1,i_PortOneNum:%d\n", n, size1);
        for (int i = 0; i < size1; i++)
        {
            tp = data.i_PortOne[i];
            PublicSpace::Log("%d-1,Sys=%d,Type=%d,Prn=%d,Psr=%.5f,Snr=%.1f,Phase=%.5f,Dop=%.5f\n", n, tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Psr, tp.i_Snr, tp.i_Phase, tp.i_Dop);
        }
        PublicSpace::Log("%d-2,i_PortTwoNum:%d\n", n, size2);
        for (int i = 0; i < size2; i++)
        {
            tp = data.i_PortTwo[i];
            PublicSpace::Log("%d-2,Sys=%d,Type=%d,Prn=%d,Psr=%.5f,Snr=%.1f,Phase=%.5f,Dop=%.5f\n", n, tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Psr, tp.i_Snr, tp.i_Phase, tp.i_Dop);
        }
    }
}

void SpoofingDoa::LogSatelliteDataPhaseDiffB(const SatelliteDataPhaseDiffB tp)
{
    PublicSpace::Log("Sys=%d,Type=%d,Prn=%d\n",
                     tp.i_Sys, tp.i_Type, tp.i_Prn);
    PublicSpace::Log("       snr1=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_Snr1[j]);
    }
    PublicSpace::Log("]\n");
    PublicSpace::Log("       snr2=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_Snr2[j]);
    }
    PublicSpace::Log("]\n");
    PublicSpace::Log("  phasediff=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_phase_diff[j] * 360);
    }
    PublicSpace::Log("]\n");
}

void SpoofingDoa::LogSatelliteDataPhaseDiffB(const vector<SatelliteDataPhaseDiffB> &dataB)
{
    SatelliteDataPhaseDiffB tp;

    for (unsigned int i = 0; i < dataB.size(); i++)
    {
        tp = dataB[i];
        PublicSpace::Log("Sys=%d,Type=%d,Prn=%d\n",
                         tp.i_Sys, tp.i_Type, tp.i_Prn);
        PublicSpace::Log("       snr1=[");
        for (int j = 0; j < tp.i_diffLen; j++)
        {
            PublicSpace::Log("%.2f,", tp.i_Snr1[j]);
        }
        PublicSpace::Log("]\n");
        PublicSpace::Log("       snr2=[");
        for (int j = 0; j < tp.i_diffLen; j++)
        {
            PublicSpace::Log("%.2f,", tp.i_Snr2[j]);
        }
        PublicSpace::Log("]\n");
        PublicSpace::Log("  phasediff=[");
        for (int j = 0; j < tp.i_diffLen; j++)
        {
            PublicSpace::Log("%.2f,", tp.i_phase_diff[j] * 360);
        }
        PublicSpace::Log("]\n");
    }
}

void SpoofingDoa::LogSatelliteDataPhaseDiffA(const vector<SatelliteDataPhaseDiffA> &dataA)
{

    for (unsigned int i = 0; i < dataA.size(); i++)
    {
        SatelliteDataPhaseDiffA tp = dataA[i];
        PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,snr1=%.2f,snr2=%.2f,phasediff=%.2f\n",
                         tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Snr1, tp.i_Snr2, tp.i_phase_diff * 360);
    }
}

void SpoofingDoa::LogSatelliteDataPhaseDiffA(const SatelliteDataPhaseDiffA dataA)
{

    SatelliteDataPhaseDiffA tp = dataA;
    PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,snr1=%.2f,snr2=%.2f,phasediff=%.2f\n",
                     tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Snr1, tp.i_Snr2, tp.i_phase_diff * 360);
}

void SpoofingDoa::LogSatelliteDataPhaseDiffType(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT)
{
    std::vector<SatelliteDataPhaseDiffA> tp_dataA;
    for (auto it = dataT.begin(); it != dataT.end(); it++)
    {
        tp_dataA.clear();
        tp_dataA = it->second;
        LogSatelliteDataPhaseDiffA(tp_dataA);
    }
    Log("\n");
}


// =============================================================================
// == 原 GN902.cpp —— 接口层实现(GN902 类) =================================================
// =============================================================================

// =============================================================================
// ★ GN902 独立日志 & 配置文件(.txt) ★
//   - 与 PublicSpace::Log 完全分离，使用独立的文件句柄与独立命名；
//   - 由外部 .txt 配置文件控制开关；未读到配置文件则默认不打印；
//   - 日志文件路径由配置文件 log_path 指定，可自定义命名；
//   - 注意: 连续切刀数 cutCountThreshold(对应引擎 m_Detection_Recodds_Num)已改为
//     通过接口 SetCutnumThreshold_GN902 传入, 不再由本配置文件读取。
//
// 配置文件按以下顺序查找(第一个可读文件生效)：
//     1) 环境变量 GN902_CONFIG 指定的路径
//     2) ./gn902_config.txt
//     3) ./config/gn902_config.txt
//     4) ../config/gn902_config.txt
//
// 配置文件内容(以 '#' 或 ';' 作为注释起始)：
//     # 是否启用独立日志，1=开启，0=关闭(默认关闭)
//     log_enable=1
//     # 独立日志文件路径(相对/绝对路径, 需带文件名)
//     log_path=./logs/gn902_doa.log
//     # 是否保存上位机传入算法的原始数据，1=开启，0=关闭(默认关闭)
//     save_data_enable=1
//     # 原始输入数据文件路径(相对/绝对路径, 需带文件名)
//     save_data_path=./gn902_input.log
// =============================================================================
namespace {

struct GN902RuntimeConfig
{
    bool        logEnable;          // 独立日志开关 (默认 false)
    std::string logPath;            // 独立日志文件路径(含文件名)
    std::string cfgPath;            // 实际加载到的配置文件路径 (""=未找到)
    bool        saveDataEnable;     // 原始输入数据保存开关 (默认 false)
    std::string saveDataPath;       // 原始输入数据文件路径(含文件名)

    GN902RuntimeConfig()
        : logEnable(false),
          logPath("./gn902_debug.log"),
          cfgPath(""),
          saveDataEnable(false),
          saveDataPath("./gn902_input.log") {}
};

GN902RuntimeConfig g_gn902Cfg;
bool              g_gn902CfgLoaded = false;
FILE             *g_gn902LogFp     = 0;
std::mutex        g_gn902LogMutex;

// 去除首尾空白
std::string gn902Trim(const std::string &s)
{
    size_t b = s.find_first_not_of(" \t\r\n");
    if (b == std::string::npos) return "";
    size_t e = s.find_last_not_of(" \t\r\n");
    return s.substr(b, e - b + 1);
}

// 转小写
std::string gn902Lower(std::string s)
{
    for (size_t i = 0; i < s.size(); ++i)
    {
        s[i] = (char)std::tolower((unsigned char)s[i]);
    }
    return s;
}

// 解析一个 .txt 配置文件, 成功打开并读取返回 true (即使文件为空也算成功)
bool gn902ParseConfigFile(const std::string &path, GN902RuntimeConfig &cfg)
{
    try
    {
        std::ifstream fin(path.c_str());
        if (!fin.is_open())
        {
            return false;
        }

        std::string line;
        while (std::getline(fin, line))
        {
            // 去注释('#',';')
            size_t cut = line.find_first_of("#;");
            if (cut != std::string::npos)
            {
                line = line.substr(0, cut);
            }
            line = gn902Trim(line);
            if (line.empty())
            {
                continue;
            }

            size_t eq = line.find('=');
            if (eq == std::string::npos)
            {
                continue;
            }

            std::string key = gn902Lower(gn902Trim(line.substr(0, eq)));
            std::string val = gn902Trim(line.substr(eq + 1));

            if (key == "log_enable" || key == "log_enabled" || key == "log")
            {
                std::string lv = gn902Lower(val);
                cfg.logEnable = !(lv == "0" || lv == "false" ||
                                  lv == "no"  || lv == "off" || lv.empty());
            }
            else if (key == "log_path" || key == "log_file" || key == "logpath")
            {
                if (!val.empty())
                {
                    cfg.logPath = val;
                }
            }
            // 是否保存上位机传入算法的原始数据
            else if (key == "save_data_enable" || key == "save_data_enabled" ||
                     key == "save_data"        || key == "savedata")
            {
                std::string sv = gn902Lower(val);
                cfg.saveDataEnable = !(sv == "0" || sv == "false" ||
                                       sv == "no"  || sv == "off" || sv.empty());
            }
            // 原始输入数据文件路径
            else if (key == "save_data_path" || key == "save_data_file" ||
                     key == "savedatapath")
            {
                if (!val.empty())
                {
                    cfg.saveDataPath = val;
                }
            }
        }
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << "[GN902] 解析配置文件异常: " << e.what() << std::endl;
        return false;
    }
}

// 懒加载配置文件，只执行一次
void gn902LoadConfig()
{
    if (g_gn902CfgLoaded)
    {
        return;
    }
    g_gn902CfgLoaded = true;

    // 默认值: 不打印独立日志, 不保存原始输入数据
    g_gn902Cfg.logEnable = false;
    g_gn902Cfg.logPath   = "./gn902_debug.log";
    g_gn902Cfg.cfgPath   = "";
    g_gn902Cfg.saveDataEnable = false;
    g_gn902Cfg.saveDataPath   = "./gn902_input.log";

    // 1) 环境变量优先
    const char *envCfg = std::getenv("GN902_CONFIG");
    if (envCfg != 0 && *envCfg != '\0')
    {
        if (gn902ParseConfigFile(envCfg, g_gn902Cfg))
        {
            g_gn902Cfg.cfgPath = envCfg;
            return;
        }
    }

    // 2) 依次尝试常见路径(全部使用 .txt)
    const char *candidates[] = {
        "./gn902_config.txt",
        "./config/gn902_config.txt",
        "../config/gn902_config.txt"
    };
    const size_t nCand = sizeof(candidates) / sizeof(candidates[0]);
    for (size_t i = 0; i < nCand; ++i)
    {
        if (gn902ParseConfigFile(candidates[i], g_gn902Cfg))
        {
            g_gn902Cfg.cfgPath = candidates[i];
            return;
        }
    }
    // 一个都没读到: 保持默认(不打印), cfgPath 保持空
}

// 独立命名日志：与 PublicSpace::Log 完全分离
#if defined(__GNUC__) || defined(__clang__)
__attribute__((format(printf, 1, 2)))
#endif
void GN902Log(const char *fmt, ...)
{
    if (!g_gn902Cfg.logEnable)
    {
        return;
    }
    std::lock_guard<std::mutex> lk(g_gn902LogMutex);

    if (g_gn902LogFp == 0)
    {
        g_gn902LogFp = fopen(g_gn902Cfg.logPath.c_str(), "a");
        if (g_gn902LogFp == 0)
        {
            std::cerr << "[GN902] 无法打开独立日志文件: "
                      << g_gn902Cfg.logPath << std::endl;
            return;
        }
    }

    // 时间戳
    std::time_t t = std::time(0);
    std::tm    *lt = std::localtime(&t);
    char ts[32] = {0};
    if (lt != 0)
    {
        std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", lt);
    }
    fprintf(g_gn902LogFp, "[%s] ", ts);

    va_list ap;
    va_start(ap, fmt);
    vfprintf(g_gn902LogFp, fmt, ap);
    va_end(ap);

    fflush(g_gn902LogFp);
}

} // namespace

// =============================================================================
// GN902State 结构定义
// =============================================================================
namespace {
struct GN902State
{
    SpoofingDoa *eng;                // 欺骗检测+测向引擎(循环切刀)
    std::vector<GNSSData> buf;       // 当前轮待处理帧(原始到达顺序)
    std::vector<int> cutIndex;       // 每帧对应切刀序号(0=校正刀, 1..6=六测向刀)
    int curCut;                      // 当前帧所在切刀序号(-1=初始)
    int doaMask;                     // 本轮已到位的测向刀掩码: bit1..bit6 <-> cut=1..6, 0x7E=六刀到齐
    std::vector<GNSSData> calFrames; // 储存的校正刀数据
    SpoofingResult result;           // 最近一轮测向结果
    FILE *phaseDiffFp;               // 固定基线相位差采集日志(追加模式)
    FILE *inputDataFp;               // 原始输入数据保存文件(追加模式, 0=未打开)
    long long inputDataCount;        // 已保存的输入帧数

    GN902State() : eng(0), curCut(-1), doaMask(0), phaseDiffFp(0), inputDataFp(0), inputDataCount(0) { clearResult(); }

    ~GN902State()
    {
        if (eng)
        {
            delete eng;
            eng = 0;
        }
        if (phaseDiffFp)
        {
            fclose(phaseDiffFp);
            phaseDiffFp = 0;
        }
        if (inputDataFp)
        {
            fclose(inputDataFp);
            inputDataFp = 0;
        }
    }

    void clearResult()
    {
        result.i_Count = 0;
        for (int i = 0; i < 24; ++i)
        {
            result.i_SatelliteAngle[i].i_Sys = 0;
            result.i_SatelliteAngle[i].i_Type = 0;
            result.i_SatelliteAngle[i].i_Alarm = 0;
            result.i_SatelliteAngle[i].i_Angle = -1;
            result.i_SatelliteAngle[i].i_Count = 0;
        }
    }
};

// 实例容器: 对象地址 -> 状态(不改动 GN902.h 的既有类定义/成员)
std::map<const GN902 *, GN902State *> g_gn902State;
} // namespace

// GN902 实例容器(声明于 interface.h)，供 Create_GN902/Release_GN902 等 C 接口使用
std::vector<GN902 *> GN902Container;

// 天线对 -> 切刀序号：校正刀 {7,7} -> 0；六测向刀 {1,2}..{1,7} -> 1..6。
//   校正刀: 同一根天线(7)接两个端口, 测的是通道固有相差, 与测向刀的天线对无关;
//           兼容旧约定 {1,1}。
//   测向刀: 参考天线(通道1)固定为天线 1, 通道2 在天线 2..7 间循环切换。
// 注意: 天线对映射失败时 GN902::SetData 会整帧丢弃。校正刀({7,7})映射失败只影响
//       校正数据刷新; 但六测向刀({1,2}..{1,7})映射失败会让轮边界(见下方 SetData 中
//       doaMask==0x7E 的判断)凑不齐, Detect()/Doa() 只能靠校正刀兜底触发, 表现为
//       "输出稀疏/完全无输出"。改天线对约定时这里必须同步。
static int mapPairToCutIndex(int cutIdx_1, int cutIdx_2)
{
    if (cutIdx_1 == 7 && cutIdx_2 == 7)
    {
        return 0; // 校正刀
    }
    if (cutIdx_1 == 1 && cutIdx_2 >= 1 && cutIdx_2 <= 7)
    {
        return cutIdx_2 - 1; // {1,1}=旧约定校正刀(0), {1,2}..{1,7}=六测向刀(1..6)
    }
    return -1;  // 非标准天线对，忽略该帧
}

// 固定基线相位差采集：天线对 {8,9}/{9,8} 时不参与循环切刀、不进入测向，
// 只逐星采集两端口载波相位差并写入日志文件(追加模式)。
static const char *g_phaseDiffLogPath = "./gn902_phasediff.log";

static void collectFixedPairPhaseDiff(GN902State *st, const GNSSData *data, int cutIdx_1, int cutIdx_2)
{
    if (st == 0 || data == 0 || st->eng == 0)
    {
        return;
    }
    if (st->phaseDiffFp == 0)
    {
        st->phaseDiffFp = fopen(g_phaseDiffLogPath, "a");
    }
    FILE *fp = st->phaseDiffFp;
    if (fp == 0)
    {
        return;
    }

    std::vector<SatelliteDataPhaseDiffA> diff;
    st->eng->collectPhaseDiffData(*data, diff);

    fprintf(fp, "[固定基线采集] 天线对=(%d,%d) 双端卫星数=%d\n", cutIdx_1, cutIdx_2, (int)diff.size());
    for (size_t i = 0; i < diff.size(); ++i)
    {
        const SatelliteDataPhaseDiffA &s = diff[i];
        fprintf(fp, "  PRN=%d, Sys=%d, Type=%d, Snr1=%.1f, Snr2=%.1f, PhaseDiff=%.6f周(%.2f度)\n",
                s.i_Prn, s.i_Sys, s.i_Type, s.i_Snr1, s.i_Snr2,
                s.i_phase_diff, s.i_phase_diff * 360.0);
    }
    fflush(fp);
}

// =============================================================================
// 保存上位机传入算法的原始数据(SetData_GN902 入口逐帧落盘)
// -----------------------------------------------------------------------------
// 目的: 上位机直接调用 C 接口时, 逐帧记录"实际喂进算法的是什么"(天线对/切刀序号、
//       两端口卫星数、逐星字段), 便于与其自身源数据比对, 排查"每刀只喂最后一秒、
//       缺刀、天线对不符"这类问题。
// 与 SpoofingDoa::saveGNSSData(PublicSpace::saveArrayToBinary, 二进制、按轮组批后)
// 是两回事: 本函数在接口入口处记录, 因此也包含被引擎忽略的非标准天线对帧。
// 由 gn902_config.txt 的 save_data_enable / save_data_path 控制, 默认关闭。
// 文本、追加模式; 首次写入时打印一段文件头。
// =============================================================================
static void saveInputGnssData(GN902State *st, const GNSSData *data, int cutIdx_1, int cutIdx_2)
{
    if (st == 0 || data == 0 || !g_gn902Cfg.saveDataEnable)
    {
        return;
    }

    char ts[32] = {0};
    std::time_t now = std::time(0);
    std::tm *lt = std::localtime(&now);
    if (lt != 0)
    {
        std::strftime(ts, sizeof(ts), "%Y-%m-%d %H:%M:%S", lt);
    }

    if (st->inputDataFp == 0)
    {
        st->inputDataFp = fopen(g_gn902Cfg.saveDataPath.c_str(), "a");
        if (st->inputDataFp == 0)
        {
            std::cerr << "[GN902] 无法打开原始输入数据文件: "
                      << g_gn902Cfg.saveDataPath << std::endl;
            return;
        }
        fprintf(st->inputDataFp,
                "=== GN902 原始输入数据(SetData_GN902 入口逐帧记录) ===\n"
                "    起始时间=%s sizeof(SatelliteData)=%d sizeof(GNSSData)=%d\n"
                "    帧行: [帧序号] 时间 天线对=(通道1,通道2) 切刀序号 帧类型 双端口卫星数\n"
                "    星行: 端口Prn Sys Type Snr Psr Phase Dop SpoofFlag\n"
                "========================================================\n",
                ts, (int)sizeof(SatelliteData), (int)sizeof(GNSSData));
    }

    FILE *fp = st->inputDataFp;
    long long idx = ++st->inputDataCount;

    // 切刀序号(0=校正刀, 1..6=六测向刀, -1=非标准天线对 => 引擎忽略该帧)
    int cut = mapPairToCutIndex(cutIdx_1, cutIdx_2);
    const char *kind = "";
    if ((cutIdx_1 == 8 && cutIdx_2 == 9) || (cutIdx_1 == 9 && cutIdx_2 == 8))
    {
        kind = " 固定基线采集";
    }
    else if (cut < 0)
    {
        kind = " 非标准天线对(引擎忽略)";
    }
    else if (cut == 0)
    {
        kind = " 校正刀";
    }

    // 越界夹紧: 引擎侧同样按 GN902_MAX_PORT_SAT 夹紧, 这里只是防止按非法计数遍历
    int n1 = data->i_PortOneNum;
    int n2 = data->i_PortTwoNum;
    if (n1 < 0 || n1 > GN902_MAX_PORT_SAT) { n1 = (n1 < 0) ? 0 : GN902_MAX_PORT_SAT; }
    if (n2 < 0 || n2 > GN902_MAX_PORT_SAT) { n2 = (n2 < 0) ? 0 : GN902_MAX_PORT_SAT; }

    fprintf(fp, "[%lld] %s 天线对=(%d,%d) 切刀=%d%s 通道1卫星数=%d 通道2卫星数=%d\n",
            idx, ts, cutIdx_1, cutIdx_2, cut, kind, n1, n2);
    for (int i = 0; i < n1; ++i)
    {
        const SatelliteData &s = data->i_PortOne[i];
        fprintf(fp, "    1: Prn=%d Sys=%d Type=%d Snr=%.1f Psr=%.3f Phase=%.3f Dop=%.3f SpoofFlag=%d\n",
                s.i_Prn, s.i_Sys, s.i_Type, s.i_Snr, s.i_Psr, s.i_Phase, s.i_Dop, s.i_SpoofingFlag);
    }
    for (int i = 0; i < n2; ++i)
    {
        const SatelliteData &s = data->i_PortTwo[i];
        fprintf(fp, "    2: Prn=%d Sys=%d Type=%d Snr=%.1f Psr=%.3f Phase=%.3f Dop=%.3f SpoofFlag=%d\n",
                s.i_Prn, s.i_Sys, s.i_Type, s.i_Snr, s.i_Psr, s.i_Phase, s.i_Dop, s.i_SpoofingFlag);
    }
    fflush(fp);
}

GN902::GN902(){
    GN902State *st = new GN902State();
    // SpoofingDoa 构造即 Init()：运行参数已内置在引擎 Init() 中(不读配置文件)，
    // 并按 Python 流程对齐各频点阈值。
    st->eng = new SpoofingDoa();
    if (st->eng != 0)
    {
        // GN902 运行参数：循环切刀检测开、全向半径 0.1865m(重建理论模板)。
        st->eng->configCyclicRuntime(true, 0, false, 0.1865);
        // 切刀顺序 = {7,7},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7}
        // (7 组天线对 = 校正刀 + 六刀测向，对齐 Python code 0/9/57/17/25/33/1)
        const int cutSeq[14] = {7, 7, 1, 2, 1, 3, 1, 4, 1, 5, 1, 6, 1, 7};
        st->eng->setCutSquence(14, cutSeq);
    }
    g_gn902State[this] = st;

    // ★ 加载独立日志配置(懒加载, 只执行一次)
    gn902LoadConfig();

    // 注意: 连续切刀数 cutCountThreshold 已改为由接口 SetCutnumThreshold_GN902 传入
    //       (见 GN902::SetCutnumThreshold)，此处不再从 txt 配置读取。
    //       未调用该接口时, 引擎保持自身默认值(m_Detection_Recodds_Num=2)。

    if (g_gn902Cfg.cfgPath.empty())
    {
        std::cout << "[GN902] 未找到配置文件(.txt), 独立日志默认关闭" << std::endl;
    }
    else
    {
        std::cout << "[GN902] 已加载配置: " << g_gn902Cfg.cfgPath
                  << " | 独立日志=" << (g_gn902Cfg.logEnable ? "开启" : "关闭")
                  << " | 路径=" << g_gn902Cfg.logPath
                  << std::endl;
    }

    // ★ 首次写入独立日志(若开启)，标记实例创建
    GN902Log("=== GN902 实例创建: cfg=%s logEnable=%d logPath=%s ===\n",
             g_gn902Cfg.cfgPath.empty() ? "(未找到)" : g_gn902Cfg.cfgPath.c_str(),
             (int)g_gn902Cfg.logEnable,
             g_gn902Cfg.logPath.c_str());
}

GN902::~GN902(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it != g_gn902State.end())
    {
        delete it->second; // 内部释放引擎
        g_gn902State.erase(it);
    }
}


// 设置阈值检测参数
// @param phsDiffThreshold 位相差阈值(度, 有效范围 0~360)
// @param satelliteCountThreshold 卫星数阈值(有效范围 0~GN902_MAX_PORT_SAT)
// @param sysEnum 系统类型
// @param typeEnum 类型
void GN902::SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, int sysEnum, int typeEnum){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }

    // ★ 参数合法性校验
    //   目的: 过滤调用方传入的无效值(未初始化变量 / 参数顺序写反 / 头文件与库 ABI
    //         不一致导致 double 位模式被误解释)。这些垃圾值一旦进入引擎就会污染
    //         阈值, 并让日志打印出 "203747...49216.000" 这样的超长数字。
    //   合法范围: 相位差阈值 0~360 度, 卫星数阈值 0~GN902_MAX_PORT_SAT。
    if (!std::isfinite(phsDiffThreshold) || phsDiffThreshold < 0.0 || phsDiffThreshold > 360.0)
    {
        GN902Log("SetThresholdDetection: INVALID phsDiff=%.3g (out of [0,360]), ignored. "
                 "sys=%d type=%d satCount=%.3g\n",
                 phsDiffThreshold, sysEnum, typeEnum, satelliteCountThreshold);
        return;
    }
    if (!std::isfinite(satelliteCountThreshold) || satelliteCountThreshold < 0.0 ||
        satelliteCountThreshold > (double)GN902_MAX_PORT_SAT)
    {
        GN902Log("SetThresholdDetection: INVALID satCount=%.3g (out of [0,%d]), ignored. "
                 "sys=%d type=%d phsDiff=%.3g\n",
                 satelliteCountThreshold, GN902_MAX_PORT_SAT, sysEnum, typeEnum, phsDiffThreshold);
        return;
    }

    SpoofingDoa *eng = it->second->eng;
    // 参数顺序: (系统sysEnum, 频点typeEnum, 卫星数阈值satelliteCountThreshold, 相位差阈值phsDiffThreshold)
    eng->setThresholdDetectionDoa(sysEnum, typeEnum, (int)satelliteCountThreshold, phsDiffThreshold);

    // 注意: 连续切刀数 cutCountThreshold 不由本接口传入, 见 GN902::SetCutnumThreshold。
    // ★ 末尾补 '\n': 原来用空格结尾, 多条日志会粘在同一行, 看起来像一条超长日志。
    // ★ 用 %.3g 代替 %.3f: 配合上面的校验, 正常值打印不受影响; 即使异常大也不会打印几百位数字。
    GN902Log("SetThresholdDetection: sys=%d type=%d phsDiff=%.3g satCount=%.3g\n",
             sysEnum, typeEnum, phsDiffThreshold, satelliteCountThreshold);
    return;
}

// 设置连续切刀数(连续报警确认次数)
// @param thresholdCount 连续报警确认次数; >0 时生效(对应引擎 m_Detection_Recodds_Num),
//                       超出 1..10 的范围时按 1 处理(见 setDetectionRecordNum)
void GN902::SetCutnumThreshold(int thresholdCount){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    if (thresholdCount > 0)
    {
        it->second->eng->setDetectionRecordNum(thresholdCount);
    }
    // ★ 末尾补 '\n', 避免与下一条日志粘行
    GN902Log("SetCutnumThreshold: cutCountThreshold=%d\n", thresholdCount);
}
// 设置数据
// @param data 数据指针
// @param cutIdx_1 通道1天线索引(参考天线，固定为1)
// @param cutIdx_2 通道2天线索引(1=校正, 2..7=六测向刀)
void GN902::SetData(const GNSSData* data, int cutIdx_1, int cutIdx_2){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        return;
    }
    GN902State *st = it->second;
    // 帧空则忽略
    if (data == 0)
    {
        return;
    }
    // ★ 保存上位机传入的原始帧(由 gn902_config.txt 的 save_data_enable 控制, 默认关闭)
    //   放在天线对判定之前: 被引擎忽略的帧也要记录, 否则"缺刀/天线对不符"看不出来。
    saveInputGnssData(st, data, cutIdx_1, cutIdx_2);
    // 特殊天线对 {8,9}/{9,8}：固定基线采集相位差，不参与循环切刀、不进入测向。
    if ((cutIdx_1 == 8 && cutIdx_2 == 9) || (cutIdx_1 == 9 && cutIdx_2 == 8))
    {
        collectFixedPairPhaseDiff(st, data, cutIdx_1, cutIdx_2);
        return;
    }
    // 天线对 -> 切刀序号(0=校正{7,7}, 1..6=测向{1,2}..{1,7})；非标准对忽略该帧
    int cut = mapPairToCutIndex(cutIdx_1, cutIdx_2);
    if (cut < 0)
    {
        return;
    }

    // 一轮结束的处理: 组批喂入引擎 -> 取测向结果 -> 清空本轮缓冲。
    // 清空 buf/cutIndex 是必须的: 缓冲若跨轮累积, Detect() 里的 C(每刀帧数)会
    // 涨到几十, 各帧来自不同轮, 时间上不连贯, 检测聚不到一起。
    auto finishRound = [&]()
    {
        Detect();
        Doa();
        st->buf.clear();
        st->cutIndex.clear();
        st->doaMask = 0;
    };

    // 轮边界(兜底)：校正刀(天线对{7,7})再次出现且上一帧非校正刀时，
    // 说明上一轮已结束。上位机切刀顺序异常(缺刀/乱序)导致六刀掩码凑不齐时，
    // 靠这一条兜底, 避免 Detect()/Doa() 一次都不执行。
    if (cut == 0 && st->curCut != 0 && !st->buf.empty())
    {
        finishRound();
    }
    st->curCut = cut;
    st->buf.push_back(*data);
    st->cutIndex.push_back(cut);

    // 轮边界(主)：六测向刀 cut=1..6 全部到达即一轮结束。
    // 校正刀(cut=0)只负责刷新 st->calFrames, 不再承担轮边界职责 ——
    // 设备上校正刀几十轮才来一次, 以它为边界会让检测/测向几乎不执行。
    bool roundDone = false;
    if (cut >= 1 && cut <= 6)
    {
        st->doaMask |= (1 << cut);
        roundDone = (st->doaMask == 0x7E);
    }

    // ★ 独立日志记录一帧到达(doaMask = 收到本帧后的掩码, 0x7E 即六刀到齐)
    GN902Log("SetData: pair=(%d,%d) cut=%d doaMask=0x%02X roundDone=%d PortOneNum=%d PortTwoNum=%d\n",
             cutIdx_1, cutIdx_2, cut, st->doaMask, (int)roundDone,
             data->i_PortOneNum, data->i_PortTwoNum);

    if (roundDone)
    {
        finishRound();
    }
    return;
}

// 获取结果
void GN902::GetResult(SpoofingResult& result){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        result.i_Count = 0;
        // ★ 实例不存在(未创建/已释放)也要留痕, 便于排查调用时序问题
        GN902Log("===== GetResult_GN902 called: instance NOT FOUND, return empty =====\n");
        return;
    }
    // 取最近一轮(喂入引擎后)的测向结果
    result = it->second->result;

    // ★ 每调用一次 GetResult_GN902 打印一次标识
    GN902Log("===== GetResult_GN902 called: alarmCount=%d =====\n", result.i_Count);
    for (int i = 0; i < result.i_Count; ++i)
    {
        GN902Log("      [%d] Sys=%d Type=%d Count=%d Angle=%.2f\n",
                 i,
                 result.i_SatelliteAngle[i].i_Sys,
                 result.i_SatelliteAngle[i].i_Type,
                 result.i_SatelliteAngle[i].i_Count,
                 result.i_SatelliteAngle[i].i_Angle);
    }
    return;
}
// 取最近一轮内所有"报警时刻"的测向结果(按时刻先后排列)
void GN902::GetAlarmMoments(std::vector<AlarmMoment>& out){
    out.clear();
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    out = it->second->eng->getAlarmMoments();
    return;
}

// 检测
void GN902::Detect(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    GN902State *st = it->second;

    // 取出本轮缓冲(swap 后引用安全)
    std::vector<GNSSData> frameData;
    std::vector<int> frameCuts;
    frameData.swap(st->buf);
    frameCuts.swap(st->cutIndex);

    // 拆帧：cut=0 -> 校正刀；cut=1..6 -> 六刀测向刀
    std::vector<GNSSData> calNew;
    std::vector<std::vector<int> > detByCut(6); // index = cut-1 (0..5)
    for (size_t i = 0; i < frameCuts.size(); ++i)
    {
        int c = frameCuts[i];
        if (c == 0)
        {
            calNew.push_back(frameData[i]);
        }
        else
        {
            detByCut[c - 1].push_back((int)i);
        }
    }

    // 本轮出现了校正刀数据 -> 刷新储存的校正数据
    if (!calNew.empty())
    {
        st->calFrames = calNew;
    }
    // 从未收到过校正刀({7,7}, 兼容旧约定 {1,1}) 时不再整轮丢弃:
    // 存一帧全零 GNSSData(0 星)充当校正行, 让本轮照常组批。
    //   引擎侧影响: setCorrectionData 找到的校正刀没有任何卫星 -> calCorrectionOffset
    //   收不到样本 -> m_CorrectionData 保持不变, 等价于本轮不做通道校正。
    //   相位差因此少了一个常量平移, 但该平移对同频所有卫星一致, 不影响相位差聚簇
    //   检测 -> 报警与测向照常给出(测向角度带一个常量偏移)。
    // 之所以要把这帧存进 calFrames 而不是用局部变量: 下面组批的 else 分支会退回到
    // calRow = st->calFrames 并取 calRow.back(), 空 vector 取 back() 是未定义行为。
    // 设备上校正刀几十轮才来一次, 若在这里直接 return, 开机后到第一把校正刀之间
    // 的所有轮都静默无结果; 若设备根本不发校正刀, 则永远无结果。
    if (st->calFrames.empty())
    {
        GNSSData noCal;
        memset(&noCal, 0, sizeof(noCal));
        st->calFrames.push_back(noCal);
        GN902Log("Detect: 无校正刀数据(calFrames empty) -> 用全零帧充当校正行, "
                 "本轮不做通道校正。本轮帧数=%d\n", (int)frameCuts.size());
    }

    // 测向轮缺刀不再整轮丢弃：缺失的测向刀用空帧补齐
    GNSSData emptyCut;
    memset(&emptyCut, 0, sizeof(emptyCut));
    bool cutComplete = true;
    for (int r = 0; r < 6; ++r)
    {
        if (detByCut[r].empty())
        {
            cutComplete = false;
            break;
        }
    }

    // 六测向刀每刀帧数是否一致
    int C = (int)detByCut[0].size();
    bool detUniform = cutComplete;
    for (int r = 1; r < 6; ++r)
    {
        if ((int)detByCut[r].size() != C)
        {
            detUniform = false;
            break;
        }
    }

    // batch 大小 = 7*C，会被引擎内部的 SatelliteDataPhaseDiffB 逐帧写入
    // 长度 100 的栈数组(见 getSatelliteDataPhaseDiffB)。必须保证 7*C <= 100。
    // 硬上限只作防御: 正常情况下每轮结束即清空缓冲, C 就是"本轮每刀位的帧数"。
    // 一旦真的截断, 说明缓冲仍在跨轮累积(轮边界没触发), 日志里必须能看出来。
    const int MAX_C = 14;  // 7*14 = 98 <= 100
    bool truncated = false;
    if (detUniform && C > MAX_C)
    {
        for (int r = 0; r < 6; ++r)
        {
            detByCut[r].erase(detByCut[r].begin(),
                              detByCut[r].end() - MAX_C);
        }
        C = MAX_C;
        truncated = true;
    }

    // 校正行：六刀帧数统一为 C 时，校正帧数也必须凑满 C 帧。
    std::vector<GNSSData> calRow;
    if (detUniform && C > 0)
    {
        if ((int)st->calFrames.size() >= C)
        {
            calRow.assign(st->calFrames.end() - C, st->calFrames.end());
        }
        else
        {
            // 校正帧不足 C: 用最近一帧校正复制 C 份填满, 仍然走标准多帧路径。
            // 此处若退化为单帧路径(uniform=false), oneCutFrams=1/smooth=false,
            // 本轮 C 帧会被压成 7 帧拼批, 各刀位不同轮的数据拼在一起, 检测聚不到一起。
            // 代价: 复制校正会放大校正误差 —— 配合"每轮结束即清空缓冲"使用, C 天然
            //       就是本轮每刀位的帧数(通常 1), 该分支只在上位机重复喂同一刀位时走到。
            calRow.assign((size_t)C, st->calFrames.back());
        }
    }
    else
    {
        calRow = st->calFrames;
    }

    // C == 0 时(六刀均无帧)才允许退化为单帧路径
    bool uniform = detUniform && (C > 0) && ((int)calRow.size() == C);
    int oneCutFrams = uniform ? C : 1;
    bool smooth = uniform;

    // 按引擎行序组批
    std::vector<GNSSData> batch;
    if (uniform)
    {
        batch.reserve((size_t)7 * C);
        for (int k = 0; k < C; ++k)
        {
            batch.push_back(calRow[k]);
        }
        for (int r = 0; r < 6; ++r)
        {
            for (int k = 0; k < C; ++k)
            {
                batch.push_back(frameData[detByCut[r][k]]);
            }
        }
    }
    else
    {
        batch.reserve(7);
        batch.push_back(calRow.back());
        for (int r = 0; r < 6; ++r)
        {
            if (detByCut[r].empty())
            {
                batch.push_back(emptyCut);
            }
            else
            {
                batch.push_back(frameData[detByCut[r].back()]);
            }
        }
    }

    // ★ 独立日志记录本轮组批信息
    //   C = 本轮每个刀位的帧数(轮结束后缓冲已清空, 因此不会跨轮累积);
    //   uniform=1 表示走标准多帧路径, batchSize 应为 7*C(7 <= batchSize <= 98)。
    GN902Log("Detect: batchSize=%d uniform=%d C=%d oneCutFrams=%d smooth=%d cutComplete=%d truncated=%d calFrames=%d\n",
             (int)batch.size(), (int)uniform, C, oneCutFrams, (int)smooth,
             (int)cutComplete, (int)truncated, (int)st->calFrames.size());

    st->eng->configCyclicRuntime(true, oneCutFrams, smooth, 0.0);
    st->eng->setGNSSData(batch.data(), (int)batch.size());
    return;
}
// 测向
void GN902::Doa(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    GN902State *st = it->second;
    st->clearResult();
    st->eng->getAngleSpoofingDoa(st->result);

    // ★ 独立日志记录本轮测向结果
    GN902Log("Doa: 报警频点数=%d\n", st->result.i_Count);
    for (int i = 0; i < st->result.i_Count; ++i)
    {
        GN902Log("  Sys=%d Type=%d Count=%d Angle=%.2f Alarm=%d\n",
                 st->result.i_SatelliteAngle[i].i_Sys,
                 st->result.i_SatelliteAngle[i].i_Type,
                 st->result.i_SatelliteAngle[i].i_Count,
                 st->result.i_SatelliteAngle[i].i_Angle,
                 st->result.i_SatelliteAngle[i].i_Alarm);
    }
    return;
}