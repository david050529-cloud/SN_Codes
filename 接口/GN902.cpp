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

    void findPeaks(const std::vector<double> data, vector<int> &index2, vector<double> &vaules2)
    {
        if (data.empty())
            return;
        int tp = -1;
        vector<int> tp_index;
        vector<int> index;
        vector<double> vaules;
        for (size_t i = 1; i < data.size() - 1; ++i)
        {
            if (data[i] > data[i - 1] && data[i] > data[i + 1])
            {
                index.emplace_back(i);
                vaules.emplace_back(data[i]);
                tp = tp + 1;
                tp_index.emplace_back(tp);
            }
        }

        if (data.size() > 1 && (data[0] > data[1]) && (data[0] > data[data.size() - 1]))
        {
            index.emplace_back(0);
            vaules.emplace_back(data[0]);
            tp = tp + 1;
            tp_index.emplace_back(tp);
        }
        if (data.size() > 1 && (data[data.size() - 1] > data[data.size() - 2]) && (data[data.size() - 1] > data[0]))
        {
            index.emplace_back(data.size() - 1);
            vaules.emplace_back(data[data.size() - 1]);
            tp = tp + 1;
            tp_index.emplace_back(tp);
        }
        std::sort(tp_index.begin(), tp_index.end(), [&](int i, int j)
                  { return vaules[i] > vaules[j]; });

        index2.clear();
        vaules2.clear();
        int num = (int)tp_index.size();
        index2.resize(num);
        vaules2.resize(num);
        for (int i = 0; i < num; i++)
        {
            index2[i] = index[tp_index[i]];
            vaules2[i] = vaules[tp_index[i]];
        }
    }

    double getNorm(const vector<complex<double>> data)
    {
        double sum_of_squares = 0.0;
        for (int i = 0; i < (int)data.size(); i++)
        {
            sum_of_squares += std::norm(data[i]);
        }
        return std::sqrt(sum_of_squares);
    }

    void split(vector<string> &result, string str, char str1)
    {
        stringstream ss(str);
        string word;

        while (getline(ss, word, str1))
        {
            trim(word);
            if (!word.empty())
            {
                result.push_back(word);
            }
        }
    }

    void trim(string &str)
    {
        str.erase(str.begin(), find_if(str.begin(), str.end(), [](unsigned char ch)
                                       { return !isspace(ch); }));

        str.erase(find_if(str.rbegin(), str.rend(), [](unsigned char ch)
                          { return !isspace(ch); })
                      .base(),
                  str.end());
    }

    int readFile(const string adr, vector<string> &data){
        string path = "";
        path.append(adr);
        data.clear();
        ifstream in(path);
        if (!in.is_open()){
            cout << "*error:adr not found !!!" << adr << endl;
            return -1;
        }

        string line;
        string tp = "";

        while (getline(in, line)){
            istringstream strm(line);
            while (strm >> tp){
                data.emplace_back(tp);
            }
        }
        in.close();
        return 0;
    }

    double Round3600(double x)
    {
        x = std::fmod(x * 10, 3600);
        if (x < 0)
        {
            x += 3600;
        }
        return x / 10.0;
    }

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

void ArithmeticDoa::getVirtual(const double virMultiple, vector<vector<int>> &antnna, vector<double> &phase_diff)
{
    vector<vector<int>> tp_antnna;
    tp_antnna = antnna;
    vector<double> tp_diff;
    tp_diff = phase_diff;
    int count = tp_antnna.size();
    int virtualAnt = 0;
    double virtualDiff = 0.0;
    vector<int> tp;
    tp.resize(2);
    for (int i = 0; i < count; i++)
    {
        virtualAnt = getVirtualAntNum(tp_antnna[i][0], tp_antnna[i][1]);
        virtualDiff = tp_diff[i] * virMultiple;
        tp[0] = tp_antnna[i][1];
        tp[1] = virtualAnt;
        tp_antnna.emplace_back(tp);
        tp_diff.emplace_back(virtualDiff);

        virtualAnt = getVirtualAntNum(tp_antnna[i][1], tp_antnna[i][0]);
        virtualDiff = -tp_diff[i] * virMultiple;
        tp[0] = tp_antnna[i][0];
        tp[1] = virtualAnt;
        tp_antnna.emplace_back(tp);
        tp_diff.emplace_back(virtualDiff);
    }

    antnna.clear();
    phase_diff.clear();
    antnna = tp_antnna;
    phase_diff = tp_diff;
}

void ArithmeticDoa::getVirtualTheory(const int antnnaNum, const std::vector<std::vector<double>> tp_theory, const double virMultiple, std::vector<std::vector<double>> &virtualTheory)
{
    int num = antnnaNum * 10 + antnnaNum;
    int index = 0;
    virtualTheory.clear();
    virtualTheory.resize(360);
    for (int ang = 0; ang < 360; ang++)
    {
        virtualTheory[ang].resize(num);
        for (int i = 0; i < antnnaNum; i++)
        {
            for (int j = 0; j < antnnaNum; j++)
            {
                if (i == j)
                {
                    continue;
                }
                index = getVirtualAntNum(i + 1, j + 1) - 1;
                double tp_diff = virMultiple * (tp_theory[ang][i] - tp_theory[ang][j]);
                double tp_diff2 = tp_theory[ang][j] - tp_diff;
                virtualTheory[ang][index] = tp_diff2;
            }
        }
    }
}

void ArithmeticDoa::calAngleSerchRange(const vector<int> index, const vector<vector<int>> cutSequence, const int AntennaNum, int max_index, int &startAngle, int &endAngle)
{
    int size = (int)index.size();
    int perAngle = (int)(360 / AntennaNum);
    int startAntenna = -1;
    int endAntenna = -1;
    float th = AntennaNum / 2.0;
    int antenna_diff = cutSequence[max_index][0] - cutSequence[max_index][1];

    if (antenna_diff < (0 - th))
    {
        startAntenna = cutSequence[max_index][1];
        endAntenna = cutSequence[max_index][0];
    }
    if (antenna_diff > th)
    {
        startAntenna = cutSequence[max_index][0];
        endAntenna = cutSequence[max_index][1];
    }
    if (-th < antenna_diff && antenna_diff < 0)
    {
        startAntenna = cutSequence[max_index][0];
        endAntenna = cutSequence[max_index][1];
    }
    if (antenna_diff > 0 && antenna_diff < th)
    {
        startAntenna = cutSequence[max_index][1];
        endAntenna = cutSequence[max_index][0];
    }
    if (index[0] == max_index)
    {
        startAngle = (startAntenna - 1) * perAngle - 10;
        endAngle = endAntenna * perAngle;
    }
    else if (index[size - 1] == max_index)
    {
        startAngle = (startAntenna - 2) * perAngle;
        endAngle = (endAntenna - 1) * perAngle + 10;
    }
    else
    {
        startAngle = (startAntenna - 1) * perAngle - perAngle / 2;
        endAngle = (endAntenna - 1) * perAngle + perAngle / 2;
    }
    if (startAngle > endAngle)
    {
        startAngle = startAngle - 360;
    }
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

int ArithmeticDoa::getRData(const string adr, vector<Rs> &mR){
    vector<Rs>().swap(mR);
    vector<string> tpVec;
    int n = readFile(adr, tpVec);

    if (0 != n){
        Log("***error:read r adr is flase-----%s\n", adr.c_str());
        return -1;
    }

    for (int i = 0; i < tpVec.size(); i += 2){

        string &a = tpVec[i];
        vector<string> a2;

        split(a2, a, '-');

        if (a2.size() == 1){
            a2.clear();
            split(a2, a, '~');
        }

        if (a2.size() == 1){
            return -2;
        }

        double starF1 = stod(a2[0]) * 1e6;
        double endF1 = stod(a2[1]) * 1e6;

        Rs tmpR;
        tmpR.i_starF = starF1;
        tmpR.i_endF = endF1;
        tmpR.i_r = stod(tpVec[i + 1]);
        mR.emplace_back(tmpR);
    }

    tpVec.clear();
    vector<string>().swap(tpVec);
    Log("set fre and R is sucess...\n");
    return 0;
}

void ArithmeticDoa::calSecondDoaByVirInterf(const vector<vector<double>> phaseTheory, const double virMultiple, vector<double> diff, InterferInfo data, double &angle, double &quality)
{
    for (int i = 0; i < (int)diff.size(); i++)
    {
        if (diff[i] < 90)
        {
            diff[i] = 0.0;
        }
    }
    vector<int> index;
    vector<double> vaules;
    findPeaks(diff, index, vaules);
    if (index.empty() || index.size() == 1)
    {
        return;
    }

    int size = data.i_Phase_Len;
    vector<double> cos_diff;
    vector<int> diff_index;
    cos_diff.resize(size);
    diff_index.resize(size);

    vector<double> tp_theory_diff1;
    vector<double> tp_theory_diff2;
    tp_theory_diff1.resize(size);
    tp_theory_diff2.resize(size);
    for (int j = 0; j < size; j++)
    {
        tp_theory_diff1[j] = phaseTheory[index[0]][data.i_AntennaSq[j][0] - 1] - phaseTheory[index[0]][data.i_AntennaSq[j][1] - 1];
        tp_theory_diff2[j] = phaseTheory[index[1]][data.i_AntennaSq[j][0] - 1] - phaseTheory[index[1]][data.i_AntennaSq[j][1] - 1];
        cos_diff[j] = cos(tp_theory_diff1[j] - tp_theory_diff2[j]);
        diff_index[j] = j;
    }
    std::sort(diff_index.begin(), diff_index.end(), [&](int i, int j)
              { return cos_diff[i] < cos_diff[j]; });

    int num = 3;
    vector<vector<int>> antnna;
    antnna.resize(num);
    vector<double> phase_diff;
    phase_diff.resize(num);
    vector<double> tp_theory_diff3;
    vector<double> tp_theory_diff4;
    tp_theory_diff3.resize(num);
    tp_theory_diff4.resize(num);
    for (int i = 0; i < num; i++)
    {
        antnna[i].resize(2);
        antnna[i][0] = data.i_AntennaSq[diff_index[i]][0];
        antnna[i][1] = data.i_AntennaSq[diff_index[i]][1];
        phase_diff[i] = data.i_Phase_Diff[diff_index[i]];
        tp_theory_diff3[i] = tp_theory_diff1[diff_index[i]];
        tp_theory_diff4[i] = tp_theory_diff2[diff_index[i]];
    }
    vector<vector<int>> antnna1;
    vector<vector<int>> antnna2;
    antnna1 = antnna;
    antnna2 = antnna;
    getVirtual(virMultiple, antnna, phase_diff);
    getVirtual(virMultiple, antnna1, tp_theory_diff3);
    getVirtual(virMultiple, antnna2, tp_theory_diff4);
    double sum1 = 0.0;
    double sum2 = 0.0;
    for (int i = 0; i < (int)antnna.size(); i++)
    {
        sum1 = sum1 + cos(phase_diff[i] - tp_theory_diff3[i]);
        sum2 = sum2 + cos(phase_diff[i] - tp_theory_diff4[i]);
    }
    if (sum1 < sum2)
    {
        angle = index[1];
        for (int j = 0; j < size; j++)
        {
            data.i_Phase_Diff[j] = tp_theory_diff2[j];
        }
        vector<double> tp_theory_Doa_diff;
        calPseudoByInterfer(phaseTheory, data, tp_theory_Doa_diff);
        quality = getDoaMass(tp_theory_Doa_diff, diff);
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

void ArithmeticDoa::calPseudoByAmpPhase(const vector<vector<complex<double>>> simulateA, vector<complex<double>> theory_A, vector<double> &diff)
{
    double tp_ActualNorm = getNorm(theory_A);
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    diff.clear();
    diff.resize(360);
    vector<complex<double>> tp_A;
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        tp_A = simulateA[ang];
        double tp_ANorm = getNorm(tp_A);
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * theory_A[i];
        }
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm);
        diff[ang] = tp_diff * 100;
    }
}

void ArithmeticDoa::calAmpPhase(vector<vector<complex<double>>> simulateA, const InterferInfo data, double &angle, double &quality, vector<double> &diff)
{
    int num = data.i_Phase_Len;
    vector<double> amp;
    amp.resize(num);
    vector<double> phase;
    phase.resize(num);
    for (int i = 0; i < num; i++)
    {
        amp[i] = data.i_Amp[i];
        phase[i] = data.i_Phase_Diff[i];
    }
    vector<complex<double>> actual;
    getA(amp, phase, actual);
    double tp_ActualNorm = getNorm(actual);

    double tp_max = -999.0;
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    vector<complex<double>> tp_A;
    tp_A.clear();
    tp_A.resize(num);
    vector<complex<double>> tp_A2;
    tp_A2.clear();

    diff.clear();
    diff.resize(360);
    vector<vector<complex<double>>> tp_simulateA;
    tp_simulateA.resize(360);
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        for (int k = 0; k < num; k++)
        {
            tp_A[k] = simulateA[ang][data.i_AntennaSq[k][0] - 1] / simulateA[ang][data.i_AntennaSq[k][1] - 1];
        }
        tp_simulateA[ang] = tp_A;

        double tp_ANorm = getNorm(tp_A);
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * actual[i];
        }
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm);
        if (tp_max < tp_diff)
        {
            tp_max = tp_diff;
            angle = ang;
            tp_A2.clear();
            tp_A2 = tp_A;
        }
        diff[ang] = tp_diff * 100;
    }
    vector<double> tp_theory_diff;
    calPseudoByAmpPhase(tp_simulateA, tp_A2, tp_theory_diff);
    quality = getDoaMass(tp_theory_diff, diff, 0, 360);
}

bool ArithmeticDoa::existAmpPhsFile(const string path, const double f)
{
        bool tp_f = true;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(0);
        ss << f;
        string path1 = ss.str();
        string csv = ".csv";
        string Amp_File_Path = "";

        Amp_File_Path.append(path);
        Amp_File_Path.append("A-");
        Amp_File_Path.append(path1);
        Amp_File_Path.append(csv);
        ifstream in(Amp_File_Path);
        return in.is_open();

}

void ArithmeticDoa::calAmpPhaseSimulateA(const string path, const double f, vector<vector<complex<double>>> &simulateA)
{
    int num1 = 0;
    bool tp_bool = true;
    double tp_f1 = f;
    double tp_f = f;
    int diff = 10e3;
    while (num1<10000 )
    {
        tp_f1 = f + diff*num1;
        tp_bool = existAmpPhsFile(path, tp_f1);
        if (tp_bool)
        {
            tp_f = tp_f1;
            break;
        }
        tp_f1 = f - diff * num1;
        tp_bool = existAmpPhsFile(path, tp_f1);
        if (tp_bool)
        {
            tp_f = tp_f1;
            break;
        }
        num1++;
    }
    simulateA.clear();
    vector<vector<double>> AmpData;
    int flg_amp = getSimulateAmp(path, tp_f, AmpData);
    vector<vector<double>> PhsData;
    int flg_phase = getSimulatePhase(path, tp_f, PhsData);
    if (0 != flg_amp || 0 != flg_phase)
    {
        Log("*** error:simulate data is error!!!\n");
        return;
    }
    if (PhsData.size() != AmpData.size() || PhsData[0].size() != AmpData[0].size())
    {
        Log("*** error:simulate data is error!!!\n");
        return;
    }
    int num = PhsData[0].size();
    vector<double> tp_amp;
    vector<double> tp_phase;
    vector<complex<double>> tp_A;
    tp_amp.resize(num);
    tp_phase.resize(num);
    simulateA.resize(360);
    for (int i = 0; i < (int)PhsData.size(); i++)
    {
        for (int j = 0; j < num; j++)
        {
            tp_amp[j] = pow(10, (AmpData[i][j] / 20));
            tp_phase[j] = -PhsData[i][j];
        }
        tp_A.clear();
        getA(tp_amp, tp_phase, tp_A);
        simulateA[i] = tp_A;
    }
}

void ArithmeticDoa::getA(const vector<double> amp, const vector<double> phase, vector<complex<double>> &A)
{
    int size = amp.size();
    A.clear();
    A.resize(size);
    for (int i = 0; i < size; i++)
    {
        complex<double> tp(cos(phase[i]), sin(phase[i]));
        A[i] = amp[i] * tp;
    }
}

int ArithmeticDoa::getVirtualAntNum(const int ant1, const int ant2)
{
    return ant1 * 10 + ant2;
}

int ArithmeticDoa::getSimulatePhase(const string path, const double f, std::vector<std::vector<double>> &phase)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(0);
    ss << f;
    string path1 = ss.str();
    string csv = ".csv";
    string Phase_File_Path = "";
    Phase_File_Path.append(path);
    Phase_File_Path.append("P-");
    Phase_File_Path.append(path1);
    Phase_File_Path.append(csv);
    phase.clear();
    int flg = getModeData(Phase_File_Path, phase);
    if (0 != flg)
    {
        Log("***error:get simulate phase data  is error:%s\n", Phase_File_Path.c_str());
        cout << "***error:get simulate phase data is error!!!" << endl;
        phase.clear();
        return -1;
    }
    for (int i = 0; i < (int)phase.size(); i++)
    {
        for (int j = 0; j < (int)phase[i].size(); j++)
        {
            phase[i][j] = phase[i][j] * m_PI / 180;
        }
    }
    return 0;
}

int ArithmeticDoa::getSimulateAmp(const string path, const double f, std::vector<std::vector<double>> &amp)
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(0);
    ss << f;
    string path1 = ss.str();
    string csv = ".csv";
    string Amp_File_Path = "";

    Amp_File_Path.append(path);
    Amp_File_Path.append("A-");
    Amp_File_Path.append(path1);
    Amp_File_Path.append(csv);

    int flg = getModeData(Amp_File_Path, amp);
    if (flg != 0)
    {
        Log(" error:get simulate amp data  is error!!!\n");
        cout << "error:get simulate amp data  is error!!!" << endl;
        return -1;
    }
    return 0;
}

int ArithmeticDoa::getModeData(const string path, vector<vector<double>> &data)
{
    vector<string> tpVec;
    tpVec.clear();
    int flg = 0;
    flg = readFile(path, tpVec);

    if (0 != flg)
    {
        Log("***error:get sigle channel error paranmeter adr is false!!!\n");
        cout << "***error:get sigle channel error paranmeter adr is false!!!" << endl;
        return flg;
    }
    int num = tpVec.size();
    data.clear();
    data.resize(360);
    for (int i = 0; i < num; i++)
    {
        string &a1 = tpVec[i];
        vector<string> a2;
        a2.clear();
        vector<string>().swap(a2);
        split(a2, a1, ',');
        int ang = Round360(stoi(a2[0]));
        for (int j = 1; j < (int)a2.size(); j++)
        {
            data[ang].emplace_back(stod(a2[j]));
        }
    }
    return 0;
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
SpoofingDoa::SpoofingDoa(void){
    initProject();
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

void SpoofingDoa::Init(void){

    m_Angle_Accumulate.clear();
    m_Angle_Accumulate.resize(360);
    std::fill(m_Angle_Accumulate.begin(), m_Angle_Accumulate.end(), 0.0);

    m_CorrectionData.clear();
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
    m_infoData180.clear();
    std::map<int, std::map<int, InterferInfo>>().swap(m_infoData180);

    m_LogFile = "./spoofingDoaLog_";
    PublicSpace::m_logFlg = 0;
    LogCreat(m_LogFile);

    m_AntennaNum = 7;
    m_antnenaType = 0;
    m_omni_R = 0.1865;
    m_Radr.clear();

    m_cutSequence = { {1,1}, {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7} };

    m_OneCut_Frams = 8;
    m_Smooth_Flag = 1;

    m_Doa_Cut_Num = 6;
    m_Doa_Cut_min_Num = 6;
    if (m_Doa_Cut_min_Num >= (int)m_cutSequence.size()) {
        m_Doa_Cut_min_Num = (int)m_cutSequence.size() - 1;
    }

    m_Phasediff_Threshold = 5.0;
    m_Detection_Threshold_Num = 2;
    m_Snr_Threshold = 35.0;
    m_Qulity_Threshold = 0.0;

    m_Cyclic_Detection_Flag = 1;
    m_Doa_Detection_Flag = 0;
    m_Doa_Arithmetic = 1;
    m_PseudoSpectrum_Flag = 0;
    m_Secondary_Doa_Flag = 0;
    m_Delete_Prn_Flag = 1;
    m_getUseAntennaBySnr_Flag = 0;

    m_Virtual_Flag = 0;
    m_Virtual_Multiple = 0.94;

    m_Detection_Recodds_Num = 2;

    m_Detection180_Qulity_Threshold = 10.0;

    m_Save_Original_Flg = 0;
    PublicSpace::m_save_data_Flg = 0;
    m_Accumulate_multiplier = 0;

    m_Simulate_Data_file = "/simulateData/";
    m_All_Simulate_data_Fre = {1176e6, 1279e6, 1561e6, 1602e6};
    initType();
    resetCyclicDetection();
    setR(m_Radr);

    initDetectionThreshold(m_Detection_Threshold_Num, m_Phasediff_Threshold);

    if (1 == m_Doa_Arithmetic){
        initTheory();
    }
    if (2 == m_Doa_Arithmetic){
        initSimulateA();
    }
    if (3 == m_Doa_Arithmetic){
        initTheoryBySimulatePhase();
    }
}

SpoofingDoa::~SpoofingDoa(void)
{
}

void SpoofingDoa::setGNSSData(const GNSSData *data, int dataLen)
{
    if (1 == dataLen) {
        PublicSpace::Log("star detection GNSSdata...\n");
        setDataAlarm(data[0]);
        m_Detection_Tag = 1;
    }
    else if (2 == dataLen)
    {
        PublicSpace::Log("star detection GNSSdata & correction data...\n");
        setCorrectDetectionDataAlarm(data, dataLen);
        m_Detection_Tag = 1;
    }
    else
    {
        m_Detection_Tag = 0;
        int cutNum = (int)m_cutSequence.size();
        if (cutNum < 2)
        {
            PublicSpace::Log("error:cutSequence is mistake!!!\n");
            cout << "error:cutSequence is mistake!!!" << endl;
            return;
        }
        PublicSpace::Log("star Doa.........................\n");
        setDataAngle(data, dataLen);
    }
    saveGNSSData(data, dataLen);
}

int SpoofingDoa::getAngleSpoofingDoa(SpoofingResult &result)
{
    setSpoofingResult(result);
    if (0 == m_Detection_Tag)
    {
        PublicSpace::Log("Not SpectrumDesity result:\n");
        LogSpoofingResult(result);
    }
    if (1 == m_PseudoSpectrum_Flag && 0 == m_Detection_Tag)
    {
        setSpoofingResultPseudoSpectrumDesity(result);
    }

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
        angle = -1;
        if (0 == m_Detection_Tag)
        {
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
        }
        else
        {
            result.i_SatelliteAngle[count].i_Angle = -1;
            result.i_SatelliteAngle[count].i_Count = (int)tp.size();
            for (unsigned int i = 0; i < tp.size(); i++)
            {
                prn = tp[i].i_Prn;
                result.i_SatelliteAngle[count].i_AlarmData[i] = tp[i];
                result.i_SatelliteAngle[count].i_AlarmData[i].i_Snr = m_Max_Snr[typeInt][prn];
            }
        }
        ++count;
    }
    result.i_Count = count;
}

void SpoofingDoa::setDataAngle(const GNSSData *data, int dataLen)
{
    string nowT = getNowTime();

    PublicSpace::Log("cal phase diff cutNum:  %s \n", nowT.c_str());
    std::map<int, std::map<int, std::vector<double>>>().swap(m_Pseudo_Spectrum_Value);
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

    if (m_Project_flg == GN930)
    {
        int cutNum = 4;
        vector<vector<SatelliteDataPhaseDiffA>> dataA1;
        dataA1.clear();
        vector<vector<SatelliteDataPhaseDiffA>> dataA2;
        dataA2.clear();
        dataA1.resize(cutNum);
        dataA2.resize(cutNum);
        vector<vector<int>> cutSequence_all;
        cutSequence_all = m_cutSequence;
        vector<vector<int>> cutSequence1;
        vector<vector<int>> cutSequence2;
        cutSequence1.resize(cutNum);
        cutSequence2.resize(cutNum);
        for (int k = 0; k < cutNum; k++)
        {
            dataA1[k] = dataA[k];
            dataA2[k] = dataA[k + cutNum];
            cutSequence1[k] = m_cutSequence[k];
            cutSequence2[k] = m_cutSequence[k + cutNum];
        }
        nowT = getNowTime();
        m_CorrectionData.clear();
        std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
        m_cutSequence.clear();
        vector<vector<int>>().swap(m_cutSequence);
        m_cutSequence = cutSequence1;
        PublicSpace::Log("get Correct Data 1:  %s \n", nowT.c_str());
        setCorrectionData(dataA1);
        getCorrectedGnssData(dataA1);

        m_CorrectionData.clear();
        std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
        m_cutSequence.clear();
        vector<vector<int>>().swap(m_cutSequence);
        m_cutSequence = cutSequence2;
        nowT = getNowTime();
        PublicSpace::Log("get Correct Data 2:  %s \n", nowT.c_str());
        setCorrectionData(dataA2);
        nowT = getNowTime();
        PublicSpace::Log("get Corrected Gnss Data:  %s \n", nowT.c_str());
        getCorrectedGnssData(dataA2);
        m_cutSequence.clear();
        m_cutSequence = cutSequence_all;
        dataA.clear();
        vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
        dataA.resize(cutNum * 2);

        for (int k = 0; k < cutNum; k++)
        {
            dataA[k] = dataA1[k];
            dataA[k + cutNum] = dataA2[k];
        }

        cutSequence_all.clear();
        dataA1.clear();
        dataA2.clear();
        cutSequence1.clear();
        cutSequence2.clear();
    }
    else
    {
        nowT = getNowTime();
        PublicSpace::Log("get Correct Data:  %s \n", nowT.c_str());
        setCorrectionData(dataA);
        nowT = getNowTime();
        PublicSpace::Log("get Corrected Gnss Data:  %s \n", nowT.c_str());
        getCorrectedGnssData(dataA);
    }

    if (m_Cyclic_Detection_Flag != 0)
    {
        nowT = getNowTime();
        PublicSpace::Log("get Cyclic Detection Data:  %s \n", nowT.c_str());
        getCyclicDetectionData(dataA);
    }
    else if (m_Doa_Detection_Flag != 0)
    {
        nowT = getNowTime();
        PublicSpace::Log("get Spoofing Detection Data:  %s \n", nowT.c_str());
        getSpoofingDetectionData(dataA);
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

    if (1 == m_Doa_Arithmetic || 3 == m_Doa_Arithmetic){

        if (0 == m_Theory.size()){
            cout << "error:theory phase diff have not!!!!" << endl;
            return;
        }

        getResultInterferDoa(doaDataB);
    }

    if (2 == m_Doa_Arithmetic){

        if (0 == m_SimulateA.size()){
            cout << "error:SimulateA have not!!!!" << endl;
            return;
        }

        getResultAmpPhaseDoa(doaDataB);
    }

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

void SpoofingDoa::setDataAlarm(const GNSSData data)
{
    string nowT = getNowTime();

    vector<SatelliteDataPhaseDiffA> dataA;
    getSatelliteDataPhaseDiffA(data, dataA);

    std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;

    getSatelliteDataByType(dataA, dataT);
    PublicSpace::Log("set detection spoofing data:  %s  \n", nowT.c_str());

    LogSatelliteDataPhaseDiffType(dataT);
    m_AngleResultData.clear();
    getAlarm(dataT);
}

void SpoofingDoa::setCorrectDetectionDataAlarm(const GNSSData *data, int dataLen)
{
    string nowT = getNowTime();
    vector<vector<SatelliteDataPhaseDiffA>> dataA;
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA.resize(dataLen);
    for (int i = 0; i < dataLen; i++)
    {
        vector<SatelliteDataPhaseDiffA> tp;
        tp.clear();
        vector<SatelliteDataPhaseDiffA>().swap(tp);
        getSatelliteDataPhaseDiffA(data[i], tp);
        dataA[i] = tp;
        LogGNSSData(data[i], i + 1);
    }
    vector<vector<SatelliteDataPhaseDiffA>> calCuts;
    calCuts.emplace_back(dataA[0]);
    calCorrectionOffset(calCuts);

    PublicSpace::Log("correction data:  %s \n", nowT.c_str());
    vector<vector<SatelliteDataPhaseDiffA>> dataA2;
    dataA2.resize(1);
    dataA2[0] = dataA[1];
    nowT = getNowTime();
    getCorrectedGnssData(dataA2);
    vector<SatelliteDataPhaseDiffA> dataA3;
    dataA3 = dataA2[0];
    std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
    getSatelliteDataByType(dataA3, dataT);
    PublicSpace::Log("set detection spoofing data:  %s \n", nowT.c_str());
    LogSatelliteDataPhaseDiffType(dataT);
    m_AngleResultData.clear();
    getAlarm(dataT);
}

int SpoofingDoa::getDetection180(int typeInt, int prn, InterferInfo &info)
{
    int flg = 0;
    std::map<int, InterferInfo> tp_prn_info;
    InterferInfo tp_info;
    if (m_infoData180.find(typeInt) == m_infoData180.end())
    {
        m_infoData180[typeInt][prn] = info;
    }
    else
    {
        tp_prn_info = m_infoData180[typeInt];
        if (tp_prn_info.find(prn) != tp_prn_info.end())
        {
            tp_info = tp_prn_info[prn];
            for (int i = 0; i < tp_info.i_Phase_Len; i++)
            {

                for (int j = 0; j < info.i_Phase_Len; j++)
                {
                    if (tp_info.i_AntennaSq[i][0] == info.i_AntennaSq[j][0] && tp_info.i_AntennaSq[i][1] == info.i_AntennaSq[j][1])
                    {
                        if (cos(tp_info.i_Phase_Diff[i] - info.i_Phase_Diff[j]) < -0.95)
                        {
                            flg = 1;
                            info.i_Phase_Diff[j] += PI;
                        }
                        break;
                    }
                }
            }
        }
    }
    return flg;
}

void SpoofingDoa::calAngle(std::map<int, std::map<int, InterferInfo>> inferInfoData)
{
    m_AngleResultData.clear();
    int typeInt = 0;
    int prn = 0;
    map<int, InterferInfo> tp;
    InterferInfo tp_info;
    InterferInfo tp_info180;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    vector<vector<double>> phaseTheory;
    vector<double> pseudoValue;
    vector<double> pseudoValue180;
    map<int, vector<double>> tp_peseudo;

    for (auto it = inferInfoData.begin(); it != inferInfoData.end(); ++it)
    {
        typeInt = it->first;
        tp = it->second;
        phaseTheory.clear();
        phaseTheory = m_Theory[typeInt];
        tp_alarms.clear();
        tp_peseudo.clear();
        for (auto itt = tp.begin(); itt != tp.end(); ++itt)
        {
            pseudoValue.clear();
            pseudoValue180.clear();
            prn = itt->first;
            tp_info = itt->second;

            double angle;
            double quality;
            ArithmeticDoa::calInterfer(phaseTheory, tp_info, angle, quality, pseudoValue);
            PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,Fre=%.1f,R=%.4f,startAngle=%d,endAngle=%d,angle=%.2f,quality=%.2f\n",
                             typeInt / 100, typeInt % 100, prn, m_F[typeInt], m_R[typeInt],
                             tp_info.i_Start, tp_info.i_End, angle, quality);
            if (0 == m_Virtual_Flag)
            {
                PublicSpace::Log("antenna and phasediff:[\n");
                for (int i = 0; i < tp_info.i_Phase_Len; i++)
                {
                    PublicSpace::Log("   antenna1=%d,antenna2=%d,phasediff=%.2f\n",
                                     tp_info.i_AntennaSq[i][0], tp_info.i_AntennaSq[i][1], tp_info.i_Phase_Diff[i] * 180 / PI);
                }
                PublicSpace::Log("]\n");
            }

            int detection_flg = 0;
            tp_info180 = tp_info;
            detection_flg = getDetection180(typeInt, prn, tp_info180);
            PublicSpace::Log("detection180=%d\n", detection_flg);
            if (1 == detection_flg)
            {
                double angle180;
                double quality180;
                ArithmeticDoa::calInterfer(phaseTheory, tp_info180, angle180, quality180, pseudoValue180);
                int tp_start = Round360(tp_info.i_Start);
                int tp_end = Round360(tp_info.i_End);
                if (angle180 != tp_start && angle180 != tp_end)
                {
                    if ((quality180 - quality) > m_Detection180_Qulity_Threshold || angle == tp_start || angle == tp_end)
                    {
                        quality = quality180;
                        angle = angle180;
                        pseudoValue.clear();
                        pseudoValue = pseudoValue180;
                        tp_info = tp_info180;
                    }
                }
                PublicSpace::Log("180:Sys=%d,Type=%d,Prn=%d,Fre=%.1f,R=%.4f,startAngle=%d,endAngle=%d,angle=%.2f,quality=%.2f\n",
                                 typeInt / 100, typeInt % 100, prn, m_F[typeInt], m_R[typeInt],
                                 tp_info.i_Start, tp_info.i_End, angle180, quality180);
                if (0 == m_Virtual_Flag)
                {
                    PublicSpace::Log("antenna and phasediff:[\n");
                    for (int i = 0; i < tp_info180.i_Phase_Len; i++)
                    {
                        PublicSpace::Log("   antenna1=%d,antenna2=%d,phasediff=%.2f\n",
                                         tp_info180.i_AntennaSq[i][0], tp_info180.i_AntennaSq[i][1], tp_info180.i_Phase_Diff[i] * 180 / PI);
                    }
                    PublicSpace::Log("]\n");
                }
            }
            m_infoData180[typeInt][prn] = tp_info;

            if (1 == m_Secondary_Doa_Flag)
            {

                ArithmeticDoa::calSecondDoaByVirInterf(phaseTheory, m_Virtual_Multiple, pseudoValue, tp_info, angle, quality);
            }

            if (quality < m_Qulity_Threshold)
            {
                continue;
            }

            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo[prn] = pseudoValue;
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
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
    if (1 == m_Virtual_Flag)
    {
        ArithmeticDoa::getVirtual(m_Virtual_Multiple, tp_antnna, tp_diff);
        ArithmeticDoa::setUseAntennaAndPhaseAll(tp_antnna, tp_diff);
    }
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

void SpoofingDoa::getAlarm(const std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT)
{
    int typeInt = 0;
    vector<AlarmData> tp_alarmData;
    vector<SatelliteDataPhaseDiffA> tpA;
    vector<SatelliteDataPhaseDiffA> tpA2;
    int alarm = 0;

    for (auto it = dataT.begin(); it != dataT.end(); ++it){
        typeInt = it->first;
        tpA.clear();
        tpA2.clear();
        tpA2 = it->second;

        calAlarmByPhaseDiff(typeInt, tpA2, tpA, alarm);

        if (0 == alarm){
            continue;
        }

        AlarmData tp;
        tp_alarmData.clear();
        vector<AlarmData>().swap(tp_alarmData);
        if (m_AngleResultData.find(typeInt) != m_AngleResultData.end())
        {
            tp_alarmData = m_AngleResultData[typeInt];
        }
        string nowT = getNowTime();
        PublicSpace::Log("%s \n get spoofing detection alarm info:\n", nowT.c_str());
        LogSatelliteDataPhaseDiffA(tpA);
        for (unsigned int i = 0; i < tpA.size(); i++)
        {
            int prn = tpA[i].i_Prn;
            bool flg = false;
            for (unsigned int k = 0; k < tp_alarmData.size(); k++)
            {
                if (prn == tp_alarmData[k].i_Prn)
                {
                    flg = true;
                    break;
                }
            }
            if (flg)
            {
                continue;
            }
            tp.i_Prn = prn;
            tp.i_Angle = -1;
            tp.i_Quality = -1;
            tp.i_Snr = tpA[i].i_Snr1;
            m_Max_Snr[typeInt][tp.i_Prn] = tp.i_Snr;
            tp_alarmData.emplace_back(tp);
        }
        m_AngleResultData[typeInt] = tp_alarmData;
    }

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

void SpoofingDoa::setR(const string adr){
    vector<Rs> m_Rs;
    m_Rs.clear();
    if (1 == m_antnenaType){
        ArithmeticDoa::getRData(adr, m_Rs);
    }

    int typeInt = 0;
    double tpF = 0.0;

    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        typeInt = it->first;
        tpF = it->second;
        if (0 == m_antnenaType)
        {
            m_R[typeInt] = m_omni_R;
        }
        else
        {
            for (unsigned int i = 0; i < m_Rs.size(); i++)
            {
                if (tpF >= m_Rs[i].i_starF && tpF <= m_Rs[i].i_endF)
                {
                    m_R[typeInt] = m_Rs[i].i_r;
                    break;
                }
            }
        }
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
    m_Doa_Detection_Flag = 0;
    if (oneCutFrams > 0)
    {
        m_OneCut_Frams = oneCutFrams;
    }
    m_Smooth_Flag = smooth ? 1 : 0;

    if (omniR > 0 && 0 == m_antnenaType)
    {
        m_omni_R = omniR;
        m_R.clear();
        setR(m_Radr);
        m_Theory.clear();
        if (1 == m_Doa_Arithmetic)
        {
            initTheory();
        }
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

void SpoofingDoa::getSpoofingDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    m_AngleResultData.clear();
    int dataLen = (int)dataA.size();
    int cutNum = (int)m_cutSequence.size();
    if (1 == m_Doa_Detection_Flag)
    {
        for (int i = 0; i < dataLen; i++)
        {
            std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
            getSatelliteDataByType(dataA[i], dataT);
            getAlarm(dataT);
            setSpoofingDetectionData(dataA[i]);
            m_AngleResultData.clear();
        }
    }
    if (2 == m_Doa_Detection_Flag)
    {
        std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
        for (int i = 0; i < dataLen; i++)
        {
            if (cutNum == dataLen)
            {
                if (m_cutSequence[i][0] == m_cutSequence[i][1])
                {
                    continue;
                }
            }
            dataT.clear();
            getSatelliteDataByType(dataA[i], dataT);
            getAlarm(dataT);
        }

        for (int i = 0; i < dataLen; i++)
        {
            setSpoofingDetectionData(dataA[i]);
        }
    }
    m_AngleResultData.clear();
}

void SpoofingDoa::setSpoofingDetectionData(std::vector<SatelliteDataPhaseDiffA> &spoofingData)
{
    vector<SatelliteDataPhaseDiffA> resultData;
    resultData.clear();
    int size = (int)spoofingData.size();
    int typeInt = 0;
    int prn = 0;
    SatelliteDataPhaseDiffA tpA;
    vector<AlarmData> tp_alarm;
    for (int i = 0; i < size; i++)
    {
        tpA = spoofingData[i];

        typeInt = TypeInt(tpA.i_Sys, tpA.i_Type);
        prn = tpA.i_Prn;
        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            continue;
        }
        else
        {
            tp_alarm = m_AngleResultData[typeInt];
            for (unsigned int j = 0; j < tp_alarm.size(); j++)
            {
                if (tp_alarm[j].i_Prn == prn)
                {
                    resultData.emplace_back(tpA);
                    break;
                }
            }
        }
    }
    spoofingData.clear();
    vector<SatelliteDataPhaseDiffA>().swap(spoofingData);
    spoofingData = resultData;
    vector<SatelliteDataPhaseDiffA>().swap(resultData);
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

void SpoofingDoa::initProject(void)
{

    switch (m_Project_flg)
    {
    case GN902:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 6;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 6;
        m_omni_R = 0.1865;
        m_antnenaType = 0;
        m_cutSequence.clear();
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN930:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 6;
        m_omni_R = 0.1865;
        m_antnenaType = 0;
        m_cutSequence.clear();
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {7, 7}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN930U:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 6;
        m_omni_R = 0.2;
        m_antnenaType = 0;
        m_cutSequence.clear();
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN560:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 3;
        m_omni_R = 0.18;
        m_antnenaType = 1;
        m_cutSequence.clear();
        m_cutSequence = {{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 1}};

        break;
    default:
        break;
    }
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

void SpoofingDoa::getResultInterferDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    string nowT = getNowTime();
    std::map<int, std::map<int, InterferInfo>> inferInfoData;
    if (0 == m_antnenaType)
    {
        PublicSpace::Log("set Interfer Info Data Omni   %s\n", nowT.c_str());
        setInterferInfoDataOmni(dataB, inferInfoData);
    }
    else
    {
        nowT = getNowTime();
        PublicSpace::Log("set Interfer Info Data Direct   %s\n", nowT.c_str());
        setInterferInfoDataDirect(dataB, inferInfoData);
    }
    nowT = getNowTime();
    PublicSpace::Log("cal angle    %s\n", nowT.c_str());
    calAngle(inferInfoData);
}

void SpoofingDoa::initTheory(void){
    int typeInt = 0;
    double f = 0;
    double r = 0.0;

    vector<vector<double>> theory;
    vector<vector<double>> tp_theory;
    for (auto it = m_F.begin(); it != m_F.end(); ++it)
    {
        theory.clear();
        tp_theory.clear();
        typeInt = it->first;
        f = it->second;
        r = m_R[typeInt];
        ArithmeticDoa::calPhaseTheory(f, r, m_AntennaNum, tp_theory);

        theory = tp_theory;
        if (1 == m_Virtual_Flag)
        {
            ArithmeticDoa::getVirtualTheory(m_AntennaNum, tp_theory, m_Virtual_Multiple, theory);
        }

        m_Theory[typeInt] = theory;
    }
}

void SpoofingDoa::initTheoryBySimulatePhase(void){
    int typeInt = 0;
    double f = 0;
    std::map<double, std::vector<std::vector<double>>> tp_theory2;
    vector<vector<double>> tp_theory;
    vector<vector<double>> tp_theory3;
    int num = m_All_Simulate_data_Fre.size();
    vector<double> erse_index;
    erse_index.clear();
    for (int i = 0; i < num; i++){

        tp_theory3.clear();
        tp_theory.clear();
        f = m_All_Simulate_data_Fre[i];
        int flg = 0;
        flg = ArithmeticDoa::getSimulatePhase(m_Simulate_Data_file, f, tp_theory);

        if (-1 == flg){
            continue;
        }
        erse_index.emplace_back(f);
        tp_theory3 = tp_theory;

        if (1 == m_Virtual_Flag){
            ArithmeticDoa::getVirtualTheory(m_AntennaNum, tp_theory, m_Virtual_Multiple, tp_theory3);
        }

        tp_theory2[f] = tp_theory3;
    }
    if (0 == tp_theory2.size()){
        m_Doa_Arithmetic = 1;
        initTheory();
        return;
    }
    m_All_Simulate_data_Fre.clear();

    for (int i = 0; i < (int)erse_index.size(); i++){
        m_All_Simulate_data_Fre.emplace_back(erse_index[i]);
    }

    tp_theory.clear();
    double tp_f_min = 99999e8;
    double tp_f2 = 0;
    double tp_diff = 0.0;
    double f2 = 0.0;

    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        typeInt = it->first;
        f = it->second;
        tp_f_min = 99999e8;

        for (int i = 0; i < (int)m_All_Simulate_data_Fre.size(); i++){
            tp_f2 = m_All_Simulate_data_Fre[i];
            tp_diff = abs(tp_f2 - f);

            if (tp_f_min > tp_diff){
                tp_f_min = tp_diff;
                f2 = tp_f2;
            }

        }

        m_Theory[typeInt] = tp_theory2[f2];
    }

}

void SpoofingDoa::getResultAmpPhaseDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    m_AngleResultData.clear();
    int size = dataB.size();
    int typeInt = 0;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    std::vector<std::vector<complex<double>>> tp_SimulateA;
    vector<double> diff;
    SatelliteDataPhaseDiffB tpB;
    int num1 = (int)m_cutSequence.size();
    vector<double> amp_snr;
    amp_snr.resize(num1);
    vector<double> phase_diff;
    phase_diff.resize(num1);
    double tmp1 = 0.0;
    double tmp2 = 0.0;
    InterferInfo data;
    data.i_Start = 0;
    data.i_End = 359;
    map<int, vector<double>> tp_peseudo;

    for (int i = 0; i < num1; i++)
    {
        data.i_AntennaSq[i][0] = m_cutSequence[i][0];
        data.i_AntennaSq[i][1] = m_cutSequence[i][1];
    }
    data.i_Phase_Len = num1;

    for (int i = 0; i < size; i++)
    {
        tp_alarms.clear();
        tpB = dataB[i];
        typeInt = TypeInt(tpB.i_Sys, tpB.i_Type);
        tp_SimulateA.clear();
        tp_SimulateA = m_SimulateA[typeInt];
        double sum_snr = 0.0;
        int num2 = tpB.i_diffLen;
        int prn = dataB[i].i_Prn;
        tp_peseudo.clear();

        for (int j = 1; j < num2; j++)
        {
            sum_snr = sum_snr + tpB.i_Snr1[j];
            amp_snr[j] = pow(10, (tpB.i_Snr2[j] / 20));
            phase_diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
            data.i_Amp[j] = pow(10, (tpB.i_Snr2[j] / 20));
            data.i_Phase_Diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
        }
        tmp2 = sum_snr / (tpB.i_diffLen - 1);
        tmp1 = tmp2;
        amp_snr[0] = pow(10, (tmp1 / 20));
        phase_diff[0] = 0;
        data.i_Amp[0] = pow(10, (tmp1 / 20));
        data.i_Phase_Diff[0] = 0;

        double angle;
        double quality;
        diff.clear();

        ArithmeticDoa::calAmpPhase(tp_SimulateA, data, angle, quality, diff);

        m_Max_Snr[typeInt][tpB.i_Prn] = tmp2;

        tp_alarm.i_Prn = tpB.i_Prn;
        tp_alarm.i_Angle = (int)angle;
        tp_alarm.i_Quality = quality;

        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            tp_alarms.emplace_back(tp_alarm);
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo[prn] = diff;
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
            }
        }
        else
        {
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo = m_Pseudo_Spectrum_Value[typeInt];
                tp_peseudo[prn] = diff;
                m_Pseudo_Spectrum_Value[typeInt] = tp_peseudo;
            }
            tp_alarms = m_AngleResultData[typeInt];
            tp_alarms.emplace_back(tp_alarm);
        }

        m_AngleResultData[typeInt] = tp_alarms;
    }
}

void SpoofingDoa::initSimulateA(void){
    int typeInt = 0;
    double f = 0;
    std::map<double, std::vector<std::vector<complex<double>>>> tp_SimulateA;
    tp_SimulateA.clear();
    vector<vector<complex<double>>> simulateA;
    int num = m_All_Simulate_data_Fre.size();
    vector<double> erse_index;
    erse_index.clear();

    for (int i = 0; i < num; i++){
        f = m_All_Simulate_data_Fre[i];
        simulateA.clear();
        ArithmeticDoa::calAmpPhaseSimulateA(m_Simulate_Data_file, f, simulateA);
        if (0 == simulateA.size()){
            continue;
        }
        erse_index.emplace_back(f);
        tp_SimulateA[f] = simulateA;
    }

    if (0 == tp_SimulateA.size()){
        m_Doa_Arithmetic = 1;
        initTheory();
        return;
    }

    m_All_Simulate_data_Fre.clear();

    for (int i = 0; i < (int)erse_index.size(); i++){
        m_All_Simulate_data_Fre.emplace_back(erse_index[i]);
    }

    double tp_f_min = 999999e8;
    double tp_f2 = 0;
    double tp_diff = 0.0;
    double f2 = 0.0;

    for (auto it = m_F.begin(); it != m_F.end(); ++it){

        typeInt = it->first;
        f = it->second;
        tp_f_min = 999999e8;
        for (int i = 0; i < (int)m_All_Simulate_data_Fre.size(); i++){

            tp_f2 = m_All_Simulate_data_Fre[i];
            tp_diff = abs(tp_f2 - f);
            if (tp_f_min > tp_diff){
                tp_f_min = tp_diff;
                f2 = tp_f2;
            }

        }

        PublicSpace::Log("Fre = %.1f,f= %.1f\n", f, f2);
        m_SimulateA[typeInt] = tp_SimulateA[f2];
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

void SpoofingDoa::setInterferInfoDataDirect(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData)
{
    int size = (int)dataB.size();
    int typInt = 0;
    int prn = 0;
    SatelliteDataPhaseDiffB tp1;
    int startAngle = 0;
    int endAngle = 359;
    int max_index = 0;
    vector<int> index;
    for (int i = 0; i < size; i++)
    {
        tp1 = dataB[i];
        typInt = TypeInt(tp1.i_Sys, tp1.i_Type);
        prn = tp1.i_Prn;

        if (1 == m_getUseAntennaBySnr_Flag)
        {
            getUseAntennaBySnr(tp1, startAngle, endAngle);
        }
        else
        {
            int per = (int)360 / m_AntennaNum;

            int len = m_cutSequence.size();
            int doaNum = 0;
            vector<int> correction_index;
            vector<int> doa_index;

            for (int k1 = 0; k1 < len; k1++)
            {

                if (m_cutSequence[k1][0] == m_cutSequence[k1][1])
                {
                    correction_index.emplace_back(k1);
                    tp1.i_Snr1[k1] = 0.0;
                    tp1.i_Snr2[k1] = 0.0;
                }
                else
                {
                    doa_index.emplace_back(k1);
                    if (tp1.i_Snr1[k1] < 1e-6 || tp1.i_Snr2[k1] < 1e-6)
                    {
                        if (k1 == 2)
                        {
                            tp1.i_Snr1[1] = 0.0;
                            tp1.i_Snr2[1] = 0.0;
                        }
                        if (k1 == 3)
                        {
                            tp1.i_Snr1[4] = 0.0;
                            tp1.i_Snr2[4] = 0.0;
                        }
                        continue;
                    }

                    doaNum++;
                }
            }

            int mainAnten = m_cutSequence[correction_index[0]][0];
            startAngle = (mainAnten - 2) * per;
            endAngle = (mainAnten)*per;
            int doaNum2 = (int)doa_index.size();
            double tp_snr1 = tp1.i_Snr2[doa_index[0]];
            double tp_snr2 = tp1.i_Snr2[doa_index[doaNum2 - 1]];
            if (tp_snr1 < tp_snr2)
            {
                if (doaNum > m_Doa_Cut_Num)
                {
                    tp1.i_Snr1[doa_index[0]] = 0.0;
                    tp1.i_Snr2[doa_index[0]] = 0.0;
                }

                startAngle = startAngle + 30;
                endAngle = endAngle + 30;
            }
            else
            {
                if (doaNum > m_Doa_Cut_Num)
                {
                    tp1.i_Snr1[doa_index[doaNum - 1]] = 0.0;
                    tp1.i_Snr2[doa_index[doaNum - 1]] = 0.0;
                }
                startAngle = startAngle - 30;
                endAngle = endAngle - 30;
            }
        }

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
}

void SpoofingDoa::getUseAntennaBySnr(SatelliteDataPhaseDiffB &dataB, int &startAngle, int &endAngle)
{
    int len = dataB.i_diffLen;
    if (len != (int)m_cutSequence.size())
    {
        cout << "error:data len don't cutSequenceNum!!!" << endl;
        return;
    }
    vector<double> tp_Snr1;
    tp_Snr1.resize(len);
    vector<double> tp_Snr2;
    tp_Snr2.resize(len);

    vector<double> tp_Snr;
    tp_Snr.resize(len);
    vector<int> index;

    vector<int> correction_index;
    vector<int> doa_index;
    int doaNum = 0;
    for (int i = 0; i < len; i++)
    {

        if (m_cutSequence[i][0] == m_cutSequence[i][1])
        {
            correction_index.emplace_back(i);
        }
        else
        {
            doa_index.emplace_back(i);
            tp_Snr[doaNum] = (dataB.i_Snr1[i] + dataB.i_Snr2[i]) / 2;
            tp_Snr1[doaNum] = dataB.i_Snr1[i];
            tp_Snr2[doaNum] = dataB.i_Snr2[i];
            index.emplace_back(doaNum);
            doaNum++;
        }
        dataB.i_Snr1[i] = 0.0;
        dataB.i_Snr2[i] = 0.0;
    }
    std::sort(index.begin(), index.end(), [&](int i, int j)
              { return tp_Snr[i] > tp_Snr[j]; });

    int max_index = index[0];
    int last_index = 0;
    int next_index = 0;
    vector<int> tp_index;
    tp_index.emplace_back(max_index);
    int size = 1;
    int flg = 1;
    int num = 0;

    if (m_cutSequence[doa_index[0]][0] == m_cutSequence[doa_index[doaNum - 1]][1] || m_cutSequence[doa_index[0]][1] == m_cutSequence[doa_index[doaNum - 1]][0])
    {

        while (size < m_Doa_Cut_Num && flg > 0 && num < 20)
        {

            last_index = (tp_index[0] - 1 + doaNum) % doaNum;
            next_index = (tp_index[size - 1] + 1) % doaNum;
            flg = 0;
            for (int i = 1; i < doaNum; i++)
            {
                if (tp_Snr1[index[i]] < 1e-6 || tp_Snr2[index[i]] < 1e-6)
                {
                    break;
                }
                if (index[i] == last_index)
                {
                    tp_index.insert(tp_index.begin() + 0, index[i]);
                    flg = 1;
                    break;
                }
                if (index[i] == next_index)
                {
                    tp_index.emplace_back(index[i]);
                    flg = 1;
                    break;
                }
            }
            size = (int)tp_index.size();
            num++;
        }
        vector<int>().swap(index);
        index.resize(size);
        double maxSnr2 = -99;
        for (int i = 0; i < size; i++)
        {
            int tp_index2 = doa_index[tp_index[i]];
            dataB.i_Snr1[tp_index2] = tp_Snr1[tp_index[i]];
            dataB.i_Snr2[tp_index2] = tp_Snr2[tp_index[i]];
            index[i] = tp_index2;
            if (maxSnr2 < tp_Snr[tp_index[i]])
            {
                maxSnr2 = tp_Snr[tp_index[i]];
                max_index = tp_index2;
            }
        }
        ArithmeticDoa::calAngleSerchRange(index, m_cutSequence, m_AntennaNum, max_index, startAngle, endAngle);
    }
}

// 欺骗检测参数（对应 Python detection_lib.py）
const double SpoofingDoa::CNR_MIN_DB = 35.0;
const double SpoofingDoa::STABILITY_RANGE_DEG = 15.0;
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

void SpoofingDoa::setSpoofingResultPseudoSpectrumDesity(SpoofingResult &result)
{
    int count = result.i_Count;
    SatelliteAngle tp_Satellite;
    AlarmData tp_alarm;
    int prnNum = 0;
    int typeInt = 0;
    int prn = 0;
    vector<vector<double>> pseudoS;
    vector<double> tp_pseudo;
    vector<vector<double>> tp_pseudoS;
    vector<double> tp_pseudo2;
    int tp_Alarm_num = 0;
    for (int i = 0; i < count; i++)
    {

        tp_Satellite = result.i_SatelliteAngle[i];
        typeInt = TypeInt(tp_Satellite.i_Sys, tp_Satellite.i_Type);
        prnNum = tp_Satellite.i_Count;
        pseudoS.clear();
        for (int j = 0; j < prnNum; j++)
        {
            tp_pseudo2.clear();
            prn = tp_Satellite.i_AlarmData[j].i_Prn;
            tp_pseudo2 = m_Pseudo_Spectrum_Value[typeInt][prn];
            pseudoS.emplace_back(tp_pseudo2);
        }
        tp_Alarm_num = tp_Satellite.i_Alarm;
        tp_pseudo.clear();
        PublicSpace::Log("Sys=%d,Type=%d\n", tp_Satellite.i_Sys, tp_Satellite.i_Type);
        getPseudoSpectrumType(pseudoS, tp_pseudo, tp_Alarm_num);
        tp_pseudoS.emplace_back(tp_pseudo);
    }

    if (count > 0)
    {
        double tp_angle = 0;
        getPseudoSpectrumDesity(tp_pseudoS, tp_angle);
        int tp_angle_diff = 0;
        int tp_angle_diff2 = 0;

        m_Angle_Accumulate[tp_angle] += 50;
        double angle2 = tp_angle;
        double max_accu = m_Angle_Accumulate[tp_angle];
        for (int i = 1; i < 6; i++)
        {
            tp_angle_diff = Round360((tp_angle - i));
            tp_angle_diff2 = Round360((tp_angle + i));
            m_Angle_Accumulate[tp_angle_diff] += (50 - i);
            m_Angle_Accumulate[tp_angle_diff2] += (50 - i);
            if (m_Angle_Accumulate[tp_angle_diff] > max_accu)
            {
                angle2 = tp_angle_diff;
                max_accu = m_Angle_Accumulate[tp_angle_diff];
            }
            if (m_Angle_Accumulate[tp_angle_diff2] > max_accu)
            {
                angle2 = tp_angle_diff2;
                max_accu = m_Angle_Accumulate[tp_angle_diff2];
            }
        }

        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(1, 10);
        for (int i = 0; i < count; i++)
        {
            int rand = dis(gen);
            result.i_SatelliteAngle[i].i_Angle = angle2 + rand / 10.0;
            result.i_SatelliteAngle[i].i_Alarm = 1;
        }
    }
    PublicSpace::Log("Accumulate:\n");
    for (int i = 0; i < 360; i++)
    {
        PublicSpace::Log("%.2f,", m_Angle_Accumulate[i]);
        m_Angle_Accumulate[i] *= m_Accumulate_multiplier;
    }
    PublicSpace::Log("\n");
}

void SpoofingDoa::getPseudoSpectrumDesity(vector<vector<double>> diff, double &angle)
{
    int size = (int)diff.size();
    double max_diff = -9999.0;
    int max_angle = 0;
    PublicSpace::Log("diff sum all:\n");
    double tp_sum = 0.0;
    for (int k1 = 0; k1 < 360; k1++)
    {
        tp_sum = 0.0;
        for (int j = 0; j < size; j++)
        {

            tp_sum = tp_sum + diff[j][k1];
        }
        if (tp_sum > max_diff)
        {
            max_diff = tp_sum;
            max_angle = k1;
        }
        PublicSpace::Log("%.4f,", tp_sum);
    }
    PublicSpace::Log("\n");
    angle = max_angle;
}

void SpoofingDoa::getPseudoSpectrumType(const vector<vector<double>> diff, vector<double> &meanDiff, int Alarm)
{

    meanDiff.clear();
    meanDiff.resize(360);
    int size = (int)diff.size();
    vector<double> sum_diff;
    sum_diff.clear();
    sum_diff.resize(360);
    for (int k1 = 0; k1 < 360; k1++)
    {
        sum_diff[k1] = 0.0;
    }
    for (int j = 0; j < size; j++)
    {
        for (int k1 = 0; k1 < 360; k1++)
        {
            sum_diff[k1] = sum_diff[k1] + diff[j][k1];
            PublicSpace::Log("%.4f,", diff[j][k1]);
        }
        PublicSpace::Log("\n");
    }

    for (int k1 = 0; k1 < 360; k1++)
    {

        meanDiff[k1] = sum_diff[k1] / size;
        if (1 == Alarm)
        {
            meanDiff[k1] = meanDiff[k1] * 3;
        }
        PublicSpace::Log("%.4f,", meanDiff[k1]);
    }
    PublicSpace::Log("\n");
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
//   eng      : SpoofingDoa 引擎。构造时即 initProject()+Init()，并按 Python 流程阈值
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

    GN902State() : eng(0), curCut(-1) { clearResult(); }

    ~GN902State()
    {
        if (eng)
        {
            delete eng;
            eng = 0;
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

GN902::GN902(){
    GN902State *st = new GN902State();
    // SpoofingDoa 构造即 initProject()+Init()：运行参数已内置在引擎 Init() 中(不读配置文件)，
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

// 检测
// 整轮组批并喂入引擎，完成循环切刀欺骗检测与跟踪(跨轮状态在引擎内连续累积)。
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

