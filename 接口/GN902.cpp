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

// =============================================================================
// 模块级说明 —— 数据流与算法约定（重要）
// =============================================================================
//
// 【切刀(天线对)循环】
//   一轮 = 1 个校正刀 + 6 个测向刀，共 7 刀：
//     校正刀 {1,1}：两端口接同一根天线，测的是通道间固定相位差，用于"通道校正"。
//     测向刀 {1,2}..{1,7}：通道1固定接参考天线1，通道2依次切换到天线2..7，
//                          用于相位差检测(聚类)与相关干涉仪测向(DOA)。
//   切刀序列由 m_cutSequence 表示，顺序对齐 Python 的 CODE_TO_PAIR:
//     code 0→{1,1}(校正), 9→{1,2}, 57→{1,3}, 17→{1,4}, 25→{1,5}, 33→{1,6}, 1→{1,7}。
//
// 【相位差符号约定】
//   本模块 i_phase_diff = frac(PortOne.Phase − PortTwo.Phase)，单位"周"，归一化到 [0,1)。
//   与 Python detection_lib 的 phase_diff = (port2 − port1) mod 360° 符号相反，
//   但本模块内部"数据生成 / 校正偏移 / DOA 理论模板"三者统一用 port1−port2 约定，
//   符号翻转在相关干涉仪的 cos() 中互相抵消，最终测向角度与 Python 一致。
//
// 【一轮处理流水线】(SpoofingDoa::setDataAngle)
//   1. getSatelliteDataPhaseDiffA   逐刀提取两端口同名卫星的载波相位差(周)。
//   2. (可选)getSmoothData / getEndFramData  每刀多帧平滑或取末帧(每刀1帧时跳过)。
//   3. setCorrectionData / getCorrectedGnssData  用校正刀算通道偏移并扣除。
//   4. getCyclicDetectionData  逐刀按相位差聚类做欺骗检测 + 连续确认 + 跟踪；
//                              并只保留"本轮报警 ∪ 已跟踪"频点的卫星供基线累积。
//   5. accumulateBaselines + getCrossCycleDataB  跨周期累积每星 6 条测向基线，
//                              缺刀位用上一周期相位差补缺。
//   6. getResultInterferDoa  相关干涉仪逐星测向，汇总报警角度。
// =============================================================================

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
        if (sumDiff > max_val)
        {
            max_val = sumDiff;
            ang_val = tp_ang;
        }
        diff[tp_ang] = sumDiff;
        diff2[tp_ang] = (sumDiff / size + 1) / 2;
    }

    angle = Round360(ang_val);
    // 与 Python correlative_doa 对齐: quality = (corr + 1)/2 * 100,
    // 其中 corr = max(sum cos)/size。原 getDoaMass 采用伪谱归一化相关口径,
    // 与 Python 的 (best_corr+1)/2*100 不一致, 导致质量分与角度置信度不符。
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

// 欺骗测向算法对象构造函数
/**
 * @brief 构造函数: 初始化引擎并设置各系统频点的默认欺骗检测阈值
 * @note 对 GPS/GLONASS/Galileo/BDS 各频点调用 setThresholdDetectionDoa 设置默认阈值
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

void SpoofingDoa::collectPhaseDiffData(const GNSSData &data, std::vector<SatelliteDataPhaseDiffA> &dataA)
{
    // 固定基线原始相位差采集: 不按载噪比过滤(保留所有两端口同时出现的卫星)、
    // 不做通道校正、不进入检测/测向。仅提取两端口同名卫星的载波相位差(小数部分)。
    vector<SatelliteDataPhaseDiffA> raw;
    getSatelliteDataPhaseDiffA(data, raw, false);
    dataA.clear();
    for (size_t i = 0; i < raw.size(); i++)
    {
        if (raw[i].i_phase_diff < 0)
        {
            continue; // 仅单端口出现的卫星, 无相位差, 跳过
        }
        dataA.emplace_back(raw[i]);
    }
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

/**
 * @brief 汇总最近一轮的报警结果: 每个报警频点输出欺骗来向角度(逐星圆周均值)与卫星明细
 *
 * 遍历 m_AngleResultData(各频点报警卫星列表)，对每个频点：
 *   - 把该频点各报警卫星的测向角做圆周均值(circularMeanDeg)，归一化到 [0,360) 作为来向角度；
 *   - 逐星填入 AlarmData(PRN/角度/质量)，并用 m_Max_Snr 覆盖其信噪比；
 *   - i_Alarm 恒为 1(报警)，i_Count 为该频点报警卫星数。
 * 注意来向角度是"报警卫星测向角的圆周均值"，若个别卫星因半周模糊偏 180°，会拉偏均值。
 */
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
            doas.push_back(tp[i].i_Angle);
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
        // 校正刀(自校准 {1,1})不按载噪比过滤: 与 Python compute_calibration 一致,
        // 用全部匹配卫星(含低载噪比)计算通道校正偏移; 否则低载噪比卫星被剔除会使
        // 校正偏移整体漂移(可达上百度), 导致测向角度错位。测向刀仍按载噪比过滤。
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
    // 测向数据放宽口径: 不要求卫星在全部 7 刀(含校正刀)都出现, 只要求 6 条测向
    // 基线齐全(由 calAngleUseAntenna 的 m_Doa_Cut_min_Num 把关), 与 Python
    // run_doa_one 只要求 6 个测向 code(9/57/17/25/33/1) 一致。否则缺失校正刀
    // 的卫星会被整体丢弃, 导致报警频点/卫星数偏少(如 B2b 整点缺失)。
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
                    ad.i_Angle = t.doa_deg;
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

/**
 * @brief 对每个频点的可疑卫星做相关干涉仪测向, 汇总报警结果
 * @param inferInfoData 各频点各星的测向输入信息(天线对、相位差、搜索范围)
 */
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
            tp_alarm.i_Angle = angle;
            tp_alarm.i_Quality = quality;
            tp_alarms.emplace_back(tp_alarm);
        }
        m_AngleResultData[typeInt] = tp_alarms;
    }
}

/**
 * @brief 从一条基线数据提取测向所需的天线对与相位差, 填入 InterferInfo
 *
 * 从 dataB 的 7 刀中跳过校正刀(j=0)，收集 6 条测向刀(天线对 {1,2}..{1,7})的有效相位差
 * (两端口信噪比都 >1e-6)。仅使用这 6 条基线，不做 setUseAntennaAndPhaseAll 的天线对传递
 * 扩展——扩展会引入半周歧义(180° 翻转)，使同一卫星在测向轮之间角度来回跳变(352°↔172°)。
 * 有效切刀数不足 m_Doa_Cut_min_Num(默认6)则 doaFlg=0，本轮该星不参与测向。
 * 顺带统计该星最大信噪比(取 6 条测向刀两端口中的最大值)写入 m_Max_Snr，供结果输出。
 * 相位差由"周"转"弧度"(×2π)后填入 InterferInfo.i_Phase_Diff(与理论模板单位一致)。
 *
 * @param dataB  单星各切刀相位差数据
 * @param info   输出: 测向输入信息(天线对/相位差/条数)
 * @param doaFlg 输出: 1=有效(切刀数足够), 0=无效(切刀数不足)
 */
void SpoofingDoa::calAngleUseAntenna(const SatelliteDataPhaseDiffB dataB, InterferInfo &info, int &doaFlg)
{
    doaFlg = 1;
    int diffLen = dataB.i_diffLen;
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
        if (m_cutSequence[j][0] == m_cutSequence[j][1])
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
    // 与 Python correlative_doa 对齐: 仅使用 6 条测向基线(参考天线1 到 天线2..7)，
    // 不做 setUseAntennaAndPhaseAll 天线对传递扩展。扩展会引入半周歧义(180° 翻转)，
    // 使同一卫星在不同测向轮之间角度来回跳变(如 352°↔172°)。
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
 * @param omniR       全向天线阵列半径(米, >0 时重建理论模板)
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

/**
 * @brief 循环切刀欺骗检测 + 连续确认 + 跟踪 + 基线筛选
 *
 * 分两阶段：
 *   第一阶段(逐刀)：对每个测向刀按频点聚类(calAlarmByPhaseDiff)判报警；维护各频点连续报警
 *   计数 m_ConsecutiveAlarm——本刀报警则 +1，连续达到 m_Detection_Recodds_Num 后把该刀报警
 *   卫星并入 m_Tracking 的 cluster_sats(欺骗卫星簇)；本刀未报警则清零，且未进入跟踪的频点
 *   其跨周期基线 m_Baselines 被清空。同时收集本轮所有报警频点到 roundAlarms。
 *   第二阶段(筛选)：只保留 (m_Tracking ∪ roundAlarms) 频点的卫星，供后续 accumulateBaselines
 *   累积跨周期基线。这样首次报警(连续=1)那一轮也能累积基线，对齐 Python 的 current_alarms∪tracking。
 *
 * @param dataA 各切刀逐星相位差(已校正)，本函数会原地筛选为"本轮报警∪已跟踪"频点的卫星
 */
void SpoofingDoa::getCyclicDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int cutNum = (int)dataA.size();
    std::set<int> roundAlarms; // 本轮任意测向刀报警的频点(对应 Python current_alarms)

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
                roundAlarms.insert(typeInt);
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
            // 与 Python 对齐: 基线累积针对 (current_alarms ∪ tracking) 频点的全部稳定卫星
            // (而非仅 cluster_sats 或仅已跟踪频点)。current_alarms 即本轮任意测向刀报警的
            // 频点(roundAlarms), 首次报警(连续=1)那一轮也要累积基线, 否则会漏掉该轮跨周期
            // 相位差。测向候选星仍由 getCrossCycleDataB 限定为 cluster_sats, 不影响检测结果。
            if (m_Tracking.find(typeInt) != m_Tracking.end() || roundAlarms.find(typeInt) != roundAlarms.end())
            {
                filtered.emplace_back(sat);
            }
        }
        dataA[j] = filtered;
    }
}

/**
 * @brief 跨周期累积每星各切刀的基线相位差(补缺刀用)
 *
 * 把本轮得到的每星相位差(dataB)写入 m_Baselines[typeInt][prn]：
 *   - 首次出现 → 整体存入；
 *   - 已存在 → 仅覆盖本轮信噪比有效的切刀位(有效位才更新)，其余切刀位保留上一周期的旧值。
 * 这样当某星在本轮缺某刀时，该刀位自动用上一周期(甚至更早)的相位差补缺，使测向凑齐 6 条基线。
 * 注意：基线存的是"校正后"相位差；校正偏移每轮由校正刀重算，若偏移跨轮漂移，
 * 旧基线可能与新基线口径不一致(该行为与 Python 一致)。
 */
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

/**
 * @brief 从跨周期基线库中取出当前跟踪频点 cluster_sats 各星的基线数据(供测向)
 *
 * 测向候选星只取"被跟踪(cluster_sats)"的卫星，逐星从 m_Baselines 取已累积的基线
 * (可能混有上一周期补缺的刀位)，交给 getResultInterferDoa 测向。
 * 注意：哪些频点/卫星被写入 m_Baselines 由 getCyclicDetectionData 决定(本轮报警 ∪ 已跟踪)，
 * 这里只做"取用"——不在 cluster_sats 里的卫星即便有基线也不会参与测向。
 */
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
 * @brief 从单帧 GNSS 双通道数据提取逐星相位差(细粒度 SatelliteDataPhaseDiffA)
 *
 * 遍历 PortOne 中每个"未重复"的卫星(按 PRN/系统/频点去重)，在 PortTwo 中找同名卫星：
 *   - 匹配成功 → i_phase_diff = frac(PortOne.Phase − PortTwo.Phase)，单位"周"，归一化到 [0,1)。
 *     注意符号：这是 Port1−Port2，与 Python 的 port2−port1 相反(见文件头部"符号约定")；
 *     但"数据/校正/模板"三者统一该约定，相关干涉仪 cos() 中符号翻转互相抵消，测向角一致。
 *   - 匹配失败 → i_phase_diff = −1，标记"仅单端口出现"，后续被 getSatelliteDataPhaseDiffB
 *     按信噪比缺省剔除(仅此单端口存在，无有效相位差)。
 * 只处理 m_F 中登记过的频点；PortTwo 中剩余未匹配的卫星也以 i_phase_diff=−1 补入(补全另一端口)。
 *
 * @param data     单帧 GNSS 数据(Port1/Port2 两通道)
 * @param dataA    输出: 各卫星相位差/信噪比列表(按 PRN/系统/频点对齐)
 * @param snrFilter true=仅保留两端口信噪比都 ≥ m_Snr_Threshold 的卫星(测向刀);
 *                  false=不过滤(校正刀, 用全部匹配卫星算通道偏移, 与 Python compute_calibration 一致)
 * @note 载波相位差取小数部分(周)：整数周(整周期模糊度)在相关干涉仪中不贡献方向信息，直接丢弃。
 */
void SpoofingDoa::getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA, bool snrFilter)
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

/**
 * @brief 从校正刀数据计算并设置通道校正偏移
 * @param dataA 各切刀相位差数据(取其中校正刀 {i,i} 计算偏移)
 */
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

/**
 * @brief 由校正刀数据计算各频点的通道校正偏移(相位差, 单位"周")
 *
 * 校正刀 {1,1} 两端口接同一根天线，因此同星相位差只反映两通道之间的固定相位偏移。
 * 对每颗"稳定"卫星(相位差最小覆盖弧 circularSpanDeg < STABILITY_RANGE_DEG 且采样数足够)
 * 取圆周均值作为该星的通道偏移；再对同频点各星偏移取圆周均值，得到该频点的统一校正偏移。
 *
 * 关键区别(与 Python compute_calibration 一致)：
 *   - GLONASS(sys==1) 为 FDMA，各卫星频率不同、通道偏移逐星而异 → 按 (频点, PRN) 逐星存偏移。
 *   - 其余系统(CDMA) → 同频点各星共用同一偏移，存于 key=−1(PRN 占位)。
 *
 * 注意校正刀不按信噪比过滤(见 setDataAngle 的 isCalCut 分支)，否则低信噪比卫星被剔除
 * 会使校正偏移整体漂移(可达上百度)，导致测向角度错位。
 *
 * @param calCuts 校正刀数据(每个校正刀一帧的逐星相位差列表)
 */
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

/**
 * @brief 对单颗卫星的相位差扣除通道校正偏移(单位: 周)
 *
 * 从 m_CorrectionData 取出该星对应偏移并相减：
 *   - GLONASS(sys==1)：按 PRN 取逐星偏移(FDMA，各星通道偏移不同)。
 *   - 其余系统：取频点统一偏移(key=−1)。
 * 无偏移记录或该星信噪比不完整(任一端口 <1e-3，视为无有效相位差)则不改动。
 * 校正后 i_phase_diff ≈ 纯几何相位差(天线1 与 天线k 之间的来波相位差)。
 */
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
 * @note 全向天线: 直接组 6 条基线信息, 再统一调用 calAngle 逐星测向
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

/**
 * @brief 欺骗检测核心: 判断某频点内卫星相位差是否聚成一簇(欺骗特征)
 *
 * 算法(对应 Python cluster_satellites, fold_half=False)：
 *   1. 筛出两端口信噪比都 ≥ CNR_MIN_DB 的高信噪比卫星。
 *   2. 校正后相位差(周) ×360 转角度，再用 normalizeAngle180 归一化到 [-180°, 180°)。
 *   3. 排序后复制一份整体 +360°(解决簇跨 ±180°/0° 边界被漏掉的问题)，用滑动窗口找
 *      "最大覆盖弧 < phsThreshold" 的最大卫星集合；窗口最多含 n 颗，避免同一颗星被其
 *      复制份重复计数。
 *   4. 若最大集合卫星数 > m_Detection_Threshold[typeInt] 则判为欺骗报警。
 *
 * 物理含义：真实卫星来自不同方向，相位差分散；欺骗信号同源，相位差会聚成一簇。
 *
 * @param typeInt            频点编码(sys*100+type)
 * @param dataA              该频点各卫星相位差(已校正)
 * @param alarmSatelliteData 输出: 被判为欺骗的卫星列表
 * @param alarm              输出: 1=报警(欺骗), 0=正常
 */
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
            PublicSpace::Log("Prn=%d,Snr=%.1f,Angle=%.1f,Quality=%.2f;\n",
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
        PublicSpace::Log("%d-1,i_PortOneNum:%d\n", n, data.i_PortOneNum);
        for (int i = 0; i < data.i_PortOneNum; i++)
        {
            tp = data.i_PortOne[i];
            PublicSpace::Log("%d-1,Sys=%d,Type=%d,Prn=%d,Psr=%.5f,Snr=%.1f,Phase=%.5f,Dop=%.5f\n", n, tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Psr, tp.i_Snr, tp.i_Phase, tp.i_Dop);
        }
        PublicSpace::Log("%d-2,i_PortTwoNum:%d\n", n, data.i_PortTwoNum);
        for (int i = 0; i < data.i_PortTwoNum; i++)
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
// 每个 GN902 实例的内部状态(引擎 + 整轮帧缓冲)。
//   eng      : SpoofingDoa 引擎。构造时即 Init()，并按 Python 流程阈值
//              初始化各频点)。引擎内部持跨轮(跨周期)状态 m_ConsecutiveAlarm /
//              m_Tracking / m_Baselines，在本类多轮调用间持续累积。
//   buf/cutIndex : 当前轮逐帧送来的 GNSSData 与对应切刀序号(0=校正, 1..6=六测向刀)。
//   curCut   : 当前帧所在切刀序号，用于检测轮边界(校正刀 {1,1} 再次出现即新轮开始)。
//   calFrames : 已储存的校正刀({1,1})数据，每轮出现校正刀则整体刷新。
//   result   : 最近一轮喂入引擎后取出的测向结果(供 GetResult 返回)。
// =============================================================================
namespace {
struct GN902State
{
    SpoofingDoa *eng;                // 欺骗检测+测向引擎(循环切刀)
    std::vector<GNSSData> buf;       // 当前轮待处理帧(原始到达顺序)
    std::vector<int> cutIndex;       // 每帧对应切刀序号(0=校正刀, 1..6=六测向刀)
    int curCut;                      // 当前帧所在切刀序号(-1=初始)
    std::vector<GNSSData> calFrames; // 储存的校正刀数据
    SpoofingResult result;           // 最近一轮测向结果
    FILE *phaseDiffFp;               // 固定基线相位差采集日志(追加模式)

    GN902State() : eng(0), curCut(-1), phaseDiffFp(0) { clearResult(); }

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
    }

    void clearResult()
    {
        result.i_Count = 0;
        for (int i = 0; i < 24; ++i)
        {
            result.i_SatelliteAngle[i].i_Sys = 0;
            result.i_SatelliteAngle[i].i_Type = 0;
            result.i_SatelliteAngle[i].i_Alarm = 0;
            result.i_SatelliteAngle[i].i_Angle = -1.0;
            result.i_SatelliteAngle[i].i_Count = 0;
        }
    }
};

// 实例容器: 对象地址 -> 状态(不改动 GN902.h 的既有类定义/成员)
std::map<const GN902 *, GN902State *> g_gn902State;
} // namespace

// GN902 实例容器(声明于 interface.h)，供 Create_GN902/Release_GN902 等 C 接口使用
std::vector<GN902 *> GN902Container;

// 天线对 -> 切刀序号：校正刀 {1,1} -> 0；六测向刀 {1,2}..{1,7} -> 1..6。
// 参考天线(通道1)固定为天线1，通道2在天线 1..7 间循环切换。
static int mapPairToCutIndex(int cutIdx_1, int cutIdx_2)
{
    if (cutIdx_1 == 1 && cutIdx_2 >= 1 && cutIdx_2 <= 7)
    {
        return cutIdx_2 - 1;
    }
    return -1;  // 非标准天线对(参考天线须为1)，忽略该帧
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

GN902::GN902(){
    GN902State *st = new GN902State();
    // SpoofingDoa 构造即 Init()：运行参数已内置在引擎 Init() 中(不读配置文件)，
    // 并按 Python 流程对齐各频点阈值。
    st->eng = new SpoofingDoa();
    if (st->eng != 0)
    {
        // GN902 运行参数：循环切刀检测开、全向半径 0.1865m(重建理论模板)。
        st->eng->configCyclicRuntime(true, 0, false, 0.1865);
        // 切刀顺序 = {1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7}
        // (7 组天线对 = 校正刀 + 六刀测向，对齐 Python code 0/9/57/17/25/33/1)
        const int cutSeq[14] = {1, 1, 1, 2, 1, 3, 1, 4, 1, 5, 1, 6, 1, 7};
        st->eng->setCutSquence(14, cutSeq);
    }
    g_gn902State[this] = st;
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
// @param phsDiffThreshold 位相差阈值
// @param satelliteCountThreshold 卫星数阈值
// @param cutCountThreshold 通道2对应天线阈值(连续确认刀数)
// @param sysEnum 系统类型
// @param typeEnum 类型
void GN902::SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    SpoofingDoa *eng = it->second->eng;
    // 参数顺序: (系统sysEnum, 频点typeEnum, 卫星数阈值satelliteCountThreshold, 相位差阈值phsDiffThreshold)
    eng->setThresholdDetectionDoa(sysEnum, typeEnum, (int)satelliteCountThreshold, phsDiffThreshold);
    // cutCountThreshold = 连续确认刀数 p(对应 ALARM_CONSECUTIVE_P)，>0 时生效
    if (cutCountThreshold > 0)
    {
        eng->setDetectionRecordNum((int)cutCountThreshold);
    }
    return;
}

// 设置数据(流式喂入一帧)
// 轮边界 = 校正刀(天线对{1,1})再次出现：此时上一轮(校正+六测向)已完整结束，
// 触发 Detect()+Doa() 结算上一轮，然后清空缓冲、把当前校正刀作为新一轮的第一帧。
// 特殊天线对 {8,9}/{9,8} 为固定基线采集模式，不参与循环切刀、不进入测向。
// 注意：实际使用中每刀只喂最后一秒(1帧)，故每轮每刀只累积 1 帧(oneCutFrams=1, 不平滑)。
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
    // 特殊天线对 {8,9}/{9,8}：固定基线采集相位差，不参与循环切刀、不进入测向。
    if ((cutIdx_1 == 8 && cutIdx_2 == 9) || (cutIdx_1 == 9 && cutIdx_2 == 8))
    {
        collectFixedPairPhaseDiff(st, data, cutIdx_1, cutIdx_2);
        return;
    }
    // 天线对 -> 切刀序号(0=校正{1,1}, 1..6=测向{1,2}..{1,7})；非标准对忽略该帧
    int cut = mapPairToCutIndex(cutIdx_1, cutIdx_2);
    if (cut < 0)
    {
        return;
    }

    // 轮边界检测：校正刀(天线对{1,1})再次出现且上一帧非校正刀时，
    // 说明上一轮(校正刀 + 六测向刀)已完整结束 -> 处理上一轮并开始新轮。
    if (cut == 0 && st->curCut != 0 && !st->buf.empty())
    {
        Detect();
        Doa();
        st->buf.clear();
        st->cutIndex.clear();
    }
    st->curCut = cut;
    st->buf.push_back(*data);
    st->cutIndex.push_back(cut);
    return;
}

// 获取结果
// @param result 结果结构体
void GN902::GetResult(SpoofingResult& result){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        result.i_Count = 0;
        return;
    }
    // 取最近一轮(喂入引擎后)的测向结果
    result = it->second->result;
    return;
}

// 检测(整轮组批)
// 把当前轮缓冲的逐帧数据按"引擎行序"重新组批：row0=校正刀, row1..6=六测向刀，
// 然后一次性喂入引擎，完成循环切刀欺骗检测与跟踪(跨轮状态在引擎内连续累积)。
//
// 组批要点：
//   - 校正刀(cut=0)存入 st->calFrames(每轮刷新)；六测向刀(cut=1..6)按切刀序号分桶(detByCut)。
//   - 若六刀帧数一致(C)且校正帧数 ≥ C，则取校正帧末尾 C 帧 + 每刀 C 帧，得到 7×C 的 batch
//     (oneCutFrams=C, smooth=true)；否则每行取末帧(oneCutFrams=1, smooth=false)。
//   - 六刀必须齐全，缺刀则丢弃本轮(实际使用中每刀只喂 1 帧，main902 在末尾用空帧补齐缺刀)。
//   - 组批后调用 configCyclicRuntime 设定本轮 oneCutFrams/smooth，再 setGNSSData 喂入引擎。
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

    // 拆帧：
    //   cut=0  -> 校正刀({1,1})，不参与测向，先存为校正数据；
    //   cut=1..6 -> 六刀测向刀(对应 {1,2}..{1,7})。
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
            detByCut[c - 1].push_back((int)i); // c 已保证 1..6
        }
    }

    // 本轮出现了校正刀数据 -> 刷新储存的校正数据(替换旧校正，取本轮为准)
    if (!calNew.empty())
    {
        st->calFrames = calNew;
    }
    if (st->calFrames.empty())
    {
        return; // 尚无校正数据则丢弃本轮(跨轮连续/跟踪状态不变)
    }

    // 测向轮需六刀齐全；缺刀则丢弃本轮
    for (int r = 0; r < 6; ++r)
    {
        if (detByCut[r].empty())
        {
            return;
        }
    }

    // 六测向刀每刀帧数是否一致
    int C = (int)detByCut[0].size();
    bool detUniform = true;
    for (int r = 1; r < 6; ++r)
    {
        if ((int)detByCut[r].size() != C)
        {
            detUniform = false;
            break;
        }
    }

    // 校正行(引擎 row0)：六刀帧数统一为 C 且校正帧数 >= C 时，取校正帧末尾 C 帧
    // (最稳定尾段)以便整体平滑；否则使用全部校正帧(将落入“每刀取末帧”路径)。
    std::vector<GNSSData> calRow;
    if (detUniform && (int)st->calFrames.size() >= C)
    {
        calRow.assign(st->calFrames.end() - C, st->calFrames.end());
    }
    else
    {
        calRow = st->calFrames;
    }

    bool uniform = detUniform && ((int)calRow.size() == C) && C > 0;
    int oneCutFrams = uniform ? C : 1;
    bool smooth = uniform;

    // 按引擎行序组批：row0 = 校正(cut=0)，row1..6 = 六刀测向(cut=1..6)
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
        // 帧数不一致(或校正帧不足) -> 每行取末尾(最稳定)一帧
        batch.reserve(7);
        batch.push_back(calRow.back());
        for (int r = 0; r < 6; ++r)
        {
            batch.push_back(frameData[detByCut[r].back()]);
        }
    }

    // 运行一轮：引擎 row0 校正刀用于通道校正，六刀测向刀做检测+测向；
    // 跨轮 m_ConsecutiveAlarm / m_Tracking / m_Baselines 在引擎内持续累积，缺刀位
    // 用上一周期相位差补齐六条基线。
    st->eng->configCyclicRuntime(true, oneCutFrams, smooth, 0.0);
    st->eng->setGNSSData(batch.data(), (int)batch.size());
    return;
}

// 测向
// 从引擎取出本轮测向结果，存入内部 result 供 GetResult 返回。
void GN902::Doa(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    GN902State *st = it->second;
    st->clearResult();
    st->eng->getAngleSpoofingDoa(st->result);
    return;
}
