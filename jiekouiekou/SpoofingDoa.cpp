// =============================================================================
// 文件名: SpoofingDoa.cpp
// 功能描述: 欺骗干扰测向(DOA)主类的实现文件
// 系统角色: 本文件实现了欺骗测向系统的核心调度逻辑，包括:
//           1. 构造函数:初始化项目参数、频率表、检测阈值、理论相位差模板
//           2. setGNSSData:数据分发入口，根据dataLen区分检测/测向/校正模式
//           3. getAngleSpoofingDoa:输出最终测向结果，支持伪谱积分和外部筛选
//           4. setDataAngle:测向主流程(相位差计算→校正→检测→干涉仪/幅相法测向)
//           5. setDataAlarm:欺骗检测模式流程(相位差计算→按频点分类→三方法检测)
//           6. setConfigData:读取配置文件解析所有运行参数
//           7. calAngle:干涉仪测向汇总(按频点逐星计算+跳半周检测与修复)
//           8. 跳半周检测与优化:检测相位跳变并使用全排列组合进行修复
// =============================================================================
#include "SpoofingDoa.h"
#include "pch.h"


// 欺骗测向算法对象构造函数
// 初始化流程: initProject(设置硬件参数) -> Init(读取配置+频率表+理论模板)
SpoofingDoa::SpoofingDoa(void){
    initProject();
    Init();
    // m_Phasediff_Threshold = m_Phasediff_Threshold / 360.0; // 将度转化为周
    // 部分频点的检测阈值单独设置，与 Python detection_lib.ORIGINAL_CONFIG_TEXT 规则一致
    // （2026-09 对齐：GLONASS G1/G2、BDS B1C 的颗数阈值已按 Python 修正）
    // 判定为严格大于（bestCount > m_Detection_Threshold），对应 Python 的 cnt > count_thr
    setThresholdDetectionDoa(4, -1, 0, 2);    // GPS L5:     Python Sys=0,Type=2  count=4(触发≥5)
    setThresholdDetectionDoa(2, 5.0, 1, 0);   // GLONASS G1: Python Sys=1,Type=0  count=2(触发≥3) phs=5.0
    setThresholdDetectionDoa(3, 5.0, 1, 1);   // GLONASS G2: Python Sys=1,Type=1  count=3(触发≥4) phs=5.0
    setThresholdDetectionDoa(3, 3.6, 3, 2);   // Galileo E1C: Python Sys=3,Type=2 count=3(触发≥4) phs=3.6
    setThresholdDetectionDoa(4, 3.6, 3, 12);  // Galileo E5a: Python Sys=3,Type=12 count=4(触发≥5) phs=3.6
    setThresholdDetectionDoa(4, 3.6, 3, 17);  // Galileo E5b: Python Sys=3,Type=17 count=4(触发≥5) phs=3.6
    setThresholdDetectionDoa(3, -1, 4, 17);   // BDS B2I:     Python Sys=4,Type=17 count=3(触发≥4)
    setThresholdDetectionDoa(3, -1, 4, 0);    // BDS B1I:     Python Sys=4,Type=0  count=3(触发≥4)
    setThresholdDetectionDoa(3, -1, 4, 2);    // BDS B3I:     Python Sys=4,Type=2  count=3(触发≥4)
    setThresholdDetectionDoa(2, -1, 4, 8);    // BDS B1C:     Python Sys=4,Type=8  count=2(触发≥3)（新增）
    setThresholdDetectionDoa(3, -1, 4, 19);   // BDS B2b:     Python Sys=4,Type=19 count=3(触发≥4)
    setThresholdDetectionDoa(3, 3.6, 4, 34);  // BDS B1X:     Python Sys=4,Type=34 count=3(触发≥4) phs=3.6
    // setTypeDetectionBySnr(1, 1, 0);
}


// 完整初始化函数(可重复调用)
// 初始化顺序:
// 1. 清空累积数据(伪谱积分、校正数据、历史记录)
// 2. 读取配置文件设置所有运行参数(setConfigData)
// 3. 初始化GNSS频点频率映射表(initType)
// 4. 设置各频点阵列半径(setR)
// 5. 初始化检测阈值(initDetectionThreshold)
// 6. 根据算法类型初始化理论模板(理论相位差/仿真阵列流型)
void SpoofingDoa::Init(void){

    // 重置伪谱积分累积器(360个角度bin清零)
    m_Angle_Accumulate.clear();
    m_Angle_Accumulate.resize(360);
    std::fill(m_Angle_Accumulate.begin(), m_Angle_Accumulate.end(), 0.0);

    // 校正数据清零
    m_CorrectionData.clear();
    std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
    // 跳半周检测历史数据清零
    m_infoData180.clear();
    std::map<int, std::map<int, InterferInfo>>().swap(m_infoData180);

    // 读取配置文件(默认路径 ./DoaBSpoofingConfig.txt)
    string tp_config_adr = "./DoaBSpoofingConfig.txt";
    setConfigData(tp_config_adr);
    // 初始化GNSS频点频率映射表(m_F)
    initType();
    // 初始化循环切刀检测状态(连续报警计数与跟踪)
    resetCyclicDetection();
    // 设置各频点的阵列半径(m_R)
    setR(m_Radr);

    // 初始检测阈值设置(使用默认的卫星颗数阈值和相位差阈值)
    initDetectionThreshold(m_Detection_Threshold_Num, m_Phasediff_Threshold);

    // 默认所有频点不使用载噪比进行欺骗检测
    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        int intType = it->first;
        // -1 表示不使用载噪比检测欺骗信号
        m_Detection_snrThrehold[intType] = -1;
    }

    // 根据不同算法，初始化测向需要使用的模板
    // 注:三种算法互斥，仅初始化所选算法所需的模板数据
    if (1 == m_Doa_Arithmetic){
        initTheory();          // 干涉仪+理论相位:计算各频点的理论相位差模板
    }
    if (2 == m_Doa_Arithmetic){
        initSimulateA();       // 幅相法:加载仿真阵列流型
    }
    if (3 == m_Doa_Arithmetic){
        initTheoryBySimulatePhase(); // 干涉仪+阵列仿真相位:加载仿真相位差模板
    }
}

SpoofingDoa::~SpoofingDoa(void)
{
}


// =========================================================================
// 设置数据 - 系统主入口
// 根据dataLen区分三种运行模式:
//   dataLen=1: 欺骗检测模式(无校正) - 仅进行欺骗检测判断
//   dataLen=2: 欺骗检测模式(带校正) - 先计算校正数据再检测
//   dataLen>2: 完整测向模式 - 相位差计算→校正→检测→DOA计算
// 最后保存原始GNSS数据(可选)
// =========================================================================
void SpoofingDoa::setGNSSData(const GNSSData *data, int dataLen)
{
    // 检测数据
    // 若为检测数据，则设置为告警，并把结果写入到全局里面保存结果的数据中
    if (1 == dataLen) {
        PublicSpace::Log("star detection GNSSdata...\n");
        setDataAlarm(data[0]);
        m_Detection_Tag = 1;   // 标记为检测模式(不进行测向)
    }
    else if (2 == dataLen)
    {
        PublicSpace::Log("star detection GNSSdata & correction data...\n");
        setCorrectDetectionDataAlarm(data, dataLen);
        m_Detection_Tag = 1;   // 标记为检测模式
    }
    else
    {
        // 完整测向模式
        m_Detection_Tag = 0;   // 标记为测向模式
        int cutNum = (int)m_cutSequence.size();
        if (cutNum < 2)
        {
            PublicSpace::Log("error:cutSequence is mistake!!!\n");
            cout << "error:cutSequence is mistake!!!" << endl;
            return;
        }
        PublicSpace::Log("star Doa.........................\n");
        setDataAngle(data, dataLen);  // 执行完整测向流程
    }
    saveGNSSData(data, dataLen); // 保存原始数据(如果配置允许)
}

// =========================================================================
// 获取欺骗测向结果 - 主输出接口
// 流程: setSpoofingResult(汇总各频点测向结果) -> 外部结果筛选(可选)
//       -> 伪谱积分(可选) -> 日志输出
// =========================================================================
int SpoofingDoa::getAngleSpoofingDoa(SpoofingResult &result)
{
    // 汇总m_AngleResultData中的数据为SpoofingResult格式
    setSpoofingResult(result);
    // 根据外部欺骗检测结果筛选(如果启用了筛选功能)
    if (1 == m_Screen_detection_Flag && m_detectResult.size() > 0)
    {
        getSpoofingResultByDetection(result);
    }
    // 测向模式下输出非积分结果日志
    if (0 == m_Detection_Tag)
    {
        PublicSpace::Log("Not SpectrumDesity result:\n");
        LogSpoofingResult(result);
    }
    // 伪谱积分:累积多帧测向结果以提高稳定性
    if (1 == m_PseudoSpectrum_Flag && 0 == m_Detection_Tag)
    {
        setSpoofingResultPseudoSpectrumDesity(result);
    }

    PublicSpace::Log("Doa result:\n");
    LogSpoofingResult(result);
    return 0;
}

// =========================================================================
// 组装最终输出结果
// 将m_AngleResultData中每个频点的测向结果汇总为SpoofingResult
// 测向模式:直接输出每颗卫星的DOA，频点角度取各星DOA的圆形均值
// 检测模式:直接输出所有卫星的测向结果(角度可能为-1表示仅检测未测向)
// =========================================================================
void SpoofingDoa::setSpoofingResult(SpoofingResult &result)
{
    int count = 0;
    int typeInt = 0;
    double angle = 0.0;
    int prn = 0;
    // 遍历所有频点的测向结果
    for (auto it = m_AngleResultData.begin(); it != m_AngleResultData.end(); ++it)
    {
        typeInt = it->first;
        result.i_SatelliteAngle[count].i_Sys = typeInt / 100;   // 从编码恢复系统号
        result.i_SatelliteAngle[count].i_Type = typeInt % 100;  // 从编码恢复频点号
        result.i_SatelliteAngle[count].i_Alarm = 1;
        vector<AlarmData> tp = it->second;
        angle = -1;
        if (0 == m_Detection_Tag)
        {
            // 测向模式：直接输出每颗卫星的 DOA；频点角度取各星 DOA 的圆形均值
            // （原 calAlarmByAngle 角度聚类已按新流程移除，对应 Python circular_mean(doas)）
            vector<double> doas;
            doas.reserve(tp.size());
            for (unsigned int i = 0; i < tp.size(); i++)
            {
                doas.push_back((double)tp[i].i_Angle);
            }
            angle = tp.empty() ? -1.0 : circularMeanDeg(doas);
            if (angle < 0)
            {
                angle += 360.0;  // 归一化到 [0°, 360°)
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
            // 检测模式:无需角度聚类，直接输出所有结果
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
    result.i_Count = count; // 最终输出的频点个数
}


// =========================================================================
// 设置测向数据 - 完整DOA流程
// 这是测向模式的核心处理函数，完整流程:
// 1. 逐刀计算相位差(dataA)
// 2. 多帧数据平滑处理(可选)
// 3. GN930特殊处理:双板卡数据分离校正
// 4. 校正数据计算与相位校正
// 5. 欺骗信号检测与筛选(可选)
// 6. 相位差数据汇总为B格式(dataB)
// 7. 调用干涉仪或幅相法进行DOA计算
// =========================================================================
void SpoofingDoa::setDataAngle(const GNSSData *data, int dataLen)
{
    string nowT = getNowTime();

    PublicSpace::Log("cal phase diff cutNum:  %s \n", nowT.c_str());
    // 清空上一帧的伪谱值
    std::map<int, std::map<int, std::vector<double>>>().swap(m_Pseudo_Spectrum_Value);
    vector<vector<SatelliteDataPhaseDiffA>> dataA;
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);

    // 步骤1: 逐刀计算相位差
    for (int i = 0; i < dataLen; i++){
        PublicSpace::Log("cut: %d   PortOneNum: %d PortTwoNum: %d  \n", i + 1, data[i].i_PortOneNum, data[i].i_PortTwoNum);
        vector<SatelliteDataPhaseDiffA> tp;
        tp.clear();
        vector<SatelliteDataPhaseDiffA>().swap(tp);
        getSatelliteDataPhaseDiffA(data[i], tp);  // 计算两通道相位差
        dataA.emplace_back(tp);
        LogGNSSData(data[i], i + 1);              // 记录原始数据日志
    }

    // 日志输出B格式数据(调试用)
    if (0 != m_logFlg){
        vector<SatelliteDataPhaseDiffB> tpB;
        tpB.clear();
        getSatelliteDataPhaseDiffB(dataA, tpB);
        LogSatelliteDataPhaseDiffB(tpB);
    }

    // 步骤2: 多帧数据平滑处理
    if (m_OneCut_Frams > 1 && 1 == m_Smooth_Flag){
        nowT = getNowTime();
        PublicSpace::Log("get smooth data:  %s \n", nowT.c_str());
        // 多帧数据平滑,注意：计算相位的方式存在歧义，若启用该方法，则需注意验证
        getSmoothData(dataA);
    }

    // 若是有多帧数据，但不需要平滑，则取最后一帧数据
    if (m_OneCut_Frams > 1 && 0 == m_Smooth_Flag)
    {
        PublicSpace::Log("get end fram data:  %s \n", nowT.c_str());
        getEndFramData(dataA);
    }

    // 步骤3-4: 校正数据处理
    // GN930特殊处理:双板卡(两个独立接收通道)，需要分离两组校正数据
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
        // 将数据和切刀顺序分为两组(对应两个板卡)
        for (int k = 0; k < cutNum; k++)
        {
            dataA1[k] = dataA[k];
            dataA2[k] = dataA[k + cutNum];
            cutSequence1[k] = m_cutSequence[k];
            cutSequence2[k] = m_cutSequence[k + cutNum];
        }
        nowT = getNowTime();
        // 处理第一个板卡的校正数据
        m_CorrectionData.clear(); // 校正数据清零,防止上一帧数据另一个板卡的校正数据有残留
        std::map<int, std::map<int, SatelliteDataPhaseDiffA>>().swap(m_CorrectionData);
        m_cutSequence.clear();
        vector<vector<int>>().swap(m_cutSequence);
        m_cutSequence = cutSequence1;
        PublicSpace::Log("get Correct Data 1:  %s \n", nowT.c_str());
        setCorrectionData(dataA1);    // 设置校正数据(从同天线功分通道计算)
        getCorrectedGnssData(dataA1); // 获得校正后的数据

        // 处理第二个板卡的校正数据
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
        // 恢复完整切刀顺序
        m_cutSequence.clear();
        m_cutSequence = cutSequence_all;
        // 合并两组数据
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
        // 非GN930:正常校正流程
        nowT = getNowTime();
        PublicSpace::Log("get Correct Data:  %s \n", nowT.c_str());
        setCorrectionData(dataA);    // 设置校正数据
        nowT = getNowTime();
        PublicSpace::Log("get Corrected Gnss Data:  %s \n", nowT.c_str());
        getCorrectedGnssData(dataA); // 获得校正后的数据
    }

    // 步骤5: 欺骗信号检测与筛选(可选)
    if (m_Cyclic_Detection_Flag != 0)
    {
        nowT = getNowTime();
        PublicSpace::Log("get Cyclic Detection Data:  %s \n", nowT.c_str());
        getCyclicDetectionData(dataA); // 循环切刀检测：每刀聚类 + 跨刀连续确认 + 跟踪
    }
    else if (m_Doa_Detection_Flag != 0)
    {
        nowT = getNowTime();
        PublicSpace::Log("get Spoofing Detection Data:  %s \n", nowT.c_str());
        getSpoofingDetectionData(dataA); // 对每刀数据进行欺骗检测，只保留为欺骗信号的数据
    }

    // 步骤6: 相位差数据汇总为B格式(dataB)
    vector<SatelliteDataPhaseDiffB> dataB;
    dataB.clear();
    getSatelliteDataPhaseDiffB(dataA, dataB);
    LogSatelliteDataPhaseDiffB(dataB);

    // 步骤6.5: 跨周期基线累积(循环检测)：本轮相位差合并进持久基线，缺刀位复用
    // 上一周期的相位差，凑齐 6 条基线做测向(对应 Python baselines 跨周期复用)。
    vector<SatelliteDataPhaseDiffB> doaDataB;
    doaDataB.clear();
    if (m_Cyclic_Detection_Flag != 0)
    {
        accumulateBaselines(dataB);      // 本轮相位差合并进持久基线
        getCrossCycleDataB(doaDataB);    // 取当前跟踪中(系统,频点)的累积基线做测向
    }
    else
    {
        doaDataB = dataB;                // 非循环检测：保持单轮测向原行为
    }

    // 步骤7: 根据算法类型调用对应的DOA计算
    if (1 == m_Doa_Arithmetic || 3 == m_Doa_Arithmetic){

        if (0 == m_Theory.size()){
            cout << "error:theory phase diff have not!!!!" << endl;
            return;
        }

        getResultInterferDoa(doaDataB);  // 相关干涉仪测向
    }

    if (2 == m_Doa_Arithmetic){

        if (0 == m_SimulateA.size()){
            cout << "error:SimulateA have not!!!!" << endl;
            return;
        }

        getResultAmpPhaseDoa(doaDataB);  // 幅相法测向
    }

    // 步骤8: 更新循环切刀跟踪状态中的最近一次 DOA（对应 Python tracking 的 last_doa）
    if (m_Cyclic_Detection_Flag != 0)
    {
        for (auto &tp : m_Tracking)
        {
            int typeInt = tp.first;
            TrackingInfo &t = tp.second;

            if (m_AngleResultData.find(typeInt) != m_AngleResultData.end() && !m_AngleResultData[typeInt].empty())
            {
                // 本轮仍报警：更新最近一次 DOA（取各星 DOA 圆形均值）
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
                        mean += 360.0;  // 归一化到 [0°,360°)
                    }
                    t.doa_deg = mean;
                    t.quality = sumQ / doas.size();
                }
            }
            else if (m_ConsecutiveAlarm[typeInt] == 0 && t.doa_deg >= 0.0)
            {
                // 本轮已消失：把最近一次 DOA 作为该频点的报警结果保留在 m_AngleResultData
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


// =========================================================================
// 设置告警数据 - 欺骗检测模式(无校正)
// 流程: GNSSData -> 相位差计算 -> 按频点分类 -> 相位差法检测(getAlarm)
// 结果存储在 m_AngleResultData 中
// =========================================================================
void SpoofingDoa::setDataAlarm(const GNSSData data)
{
    string nowT = getNowTime();

    vector<SatelliteDataPhaseDiffA> dataA;
    getSatelliteDataPhaseDiffA(data, dataA);  // 计算相位差

    std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;

    //将设置的告警数据和id绑定(按频点分类)
    getSatelliteDataByType(dataA, dataT);
    PublicSpace::Log("set detection spoofing data:  %s  \n", nowT.c_str());

    LogSatelliteDataPhaseDiffType(dataT);
    m_AngleResultData.clear();
    getAlarm(dataT);  // 执行欺骗检测(相位差法+载噪比法)
}



// =========================================================================
// 带校正数据的欺骗检测模式
// 流程: GNSSData[0]用于计算校正数据 -> GNSSData[1]校正后用于欺骗检测
// 适用于需要消除通道差异的精确检测场景
// =========================================================================
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
    // 处理校正数据(GNSSData[0] = 同天线自校准刀 = Python code=0)
    vector<vector<SatelliteDataPhaseDiffA>> calCuts;
    calCuts.emplace_back(dataA[0]);
    calCorrectionOffset(calCuts);  // 计算校正偏移(对应 Python compute_calibration)

    PublicSpace::Log("correction data:  %s \n", nowT.c_str());
    // 对第二帧数据(检测用数据)进行校正
    vector<vector<SatelliteDataPhaseDiffA>> dataA2;
    dataA2.resize(1);
    dataA2[0] = dataA[1];
    nowT = getNowTime();
    getCorrectedGnssData(dataA2);
    // 校正后的数据进行欺骗检测
    vector<SatelliteDataPhaseDiffA> dataA3;
    dataA3 = dataA2[0];
    std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
    getSatelliteDataByType(dataA3, dataT);
    PublicSpace::Log("set detection spoofing data:  %s \n", nowT.c_str());
    LogSatelliteDataPhaseDiffType(dataT);
    m_AngleResultData.clear();
    getAlarm(dataT);
}

// =========================================================================
// 检测是否存在跳半周现象
// 原理: 比较当前帧与历史帧的同一卫星、同一天线对的相位差
//       如果余弦值 < -0.95 (即角度差接近180度)，判定为跳半周
// 返回: 1=存在跳半周(并自动修复info中的相位差), 0=正常
// =========================================================================
int SpoofingDoa::getDetection180(int typeInt, int prn, InterferInfo &info)
{
    int flg = 0;
    std::map<int, InterferInfo> tp_prn_info;
    InterferInfo tp_info;
    if (m_infoData180.find(typeInt) == m_infoData180.end())
    {
        // 首次出现此频点，保存当前数据作为参考
        m_infoData180[typeInt][prn] = info;
    }
    else
    {
        tp_prn_info = m_infoData180[typeInt];
        if (tp_prn_info.find(prn) != tp_prn_info.end())
        {
            tp_info = tp_prn_info[prn];
            // 遍历所有天线对，寻找匹配对并检查相位差变化
            for (int i = 0; i < tp_info.i_Phase_Len; i++)
            {

                for (int j = 0; j < info.i_Phase_Len; j++)
                {
                    if (tp_info.i_AntennaSq[i][0] == info.i_AntennaSq[j][0] && tp_info.i_AntennaSq[i][1] == info.i_AntennaSq[j][1])
                    {
                        // 余弦值 <-0.95 说明两次相位差接近180度(跳半周)
                        if (cos(tp_info.i_Phase_Diff[i] - info.i_Phase_Diff[j]) < -0.95)
                        {
                            flg = 1;
                            info.i_Phase_Diff[j] += PI;  // 补偿半周修复
                        }
                        break;
                    }
                }
            }
        }
    }
    return flg;
}

// =========================================================================
// 干涉仪测向汇总 - 对每个频点的每颗卫星进行DOA计算
// 流程:
// 1. 遍历所有频点和卫星
// 2. 调用ArithmeticDoa::calInterfer进行相关干涉仪匹配
// 3. 检测并修复跳半周问题(getDetection180)
// 4. 可选:二次确认测向(虚拟干涉仪)
// 5. 质量过滤(低于m_Qulity_Threshold则丢弃)
// 6. 可选:保存伪谱值用于积分
// 7. 结果存入m_AngleResultData
// =========================================================================
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
        phaseTheory = m_Theory[typeInt];  // 获取该频点的理论相位差模板
        tp_alarms.clear();
        tp_peseudo.clear();
        for (auto itt = tp.begin(); itt != tp.end(); ++itt)
        {
            pseudoValue.clear();
            pseudoValue180.clear();
            prn = itt->first;
            tp_info = itt->second;

            // 相关干涉仪匹配:计算实测相位差与理论模板的相关系数
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

            // 跳半周检测与修复
            int detection_flg = 0;
            tp_info180 = tp_info;
            detection_flg = getDetection180(typeInt, prn, tp_info180); // 检测是否存在跳半周
            PublicSpace::Log("detection180=%d\n", detection_flg);
            if (1 == detection_flg)
            {
                double angle180;
                double quality180;
                // 使用修复后的相位差重新测向
                ArithmeticDoa::calInterfer(phaseTheory, tp_info180, angle180, quality180, pseudoValue180);
                int tp_start = Round360(tp_info.i_Start);
                int tp_end = Round360(tp_info.i_End);
                if (angle180 != tp_start && angle180 != tp_end)
                {
                    // 若跳半周纠正后的测向质量大于纠正前的阈值，或原结果在边界，则采用修复结果
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
            // 保存当前数据用于下一帧的跳半周检测
            m_infoData180[typeInt][prn] = tp_info;

            // 二次确认测向(虚拟干涉仪)
            if (1 == m_Secondary_Doa_Flag)
            {

                ArithmeticDoa::calSecondDoaByVirInterf(phaseTheory, m_Virtual_Multiple, pseudoValue, tp_info, angle, quality);
            }

            // 质量过滤
            if (quality < m_Qulity_Threshold)
            {
                continue;  // 测向质量不达标，丢弃此结果
            }

            // 保存伪谱值用于多帧积分
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
        m_AngleResultData[typeInt] = tp_alarms;  // 保存该频点所有卫星的测向结果
    }
}

// =========================================================================
// 根据天线关系和相位差构建InterferInfo结构
// 这是从原始相位差数据到干涉仪测向输入的桥梁函数:
// 1. 验证数据有效性(载噪比>0，天线对有效)
// 2. 排除同天线自检刀(仅用于校正，不参与测向)
// 3. 构建天线对序列和相位差序列
// 4. 可选:应用虚拟阵列扩展
// 5. 记录该卫星的最大载噪比
// 返回: doaFlg=1表示数据有效可测向，0表示数据不足
// =========================================================================
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
        // 跳过无效数据(载噪比接近0)
        if (dataB.i_Snr1[j] < 1e-6 || dataB.i_Snr2[j] < 1e-6)
        {
            continue;
        }
        // 跳过同天线自检刀(用于校正，不参与测向)
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
        tp_antnna.emplace_back(m_cutSequence[j]);  // 记录天线对
        tp_diff.emplace_back(dataB.i_phase_diff[j]); // 记录相位差
        tp1[m_cutSequence[j][1]] = dataB.i_phase_diff[j];
        tp_antenna_map[m_cutSequence[j][0]] = tp1;
        ++count;
    }
    // 有效天线对数量不足，无法测向
    if (count < m_Doa_Cut_min_Num)
    {
        doaFlg = 0;
        return;
    }
    // 设置天线对和相位差用于底层算法
    ArithmeticDoa::setUseAntennaAndPhaseAll(tp_antnna, tp_diff);
    // 虚拟阵列扩展
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
    // 记录最大载噪比
    int prn = dataB.i_Prn;
    int typeInt = TypeInt(dataB.i_Sys, dataB.i_Type);
    if (diffLen > 1)
    {
        maxSnr = dataB.i_Snr1[1];
    }
    m_Max_Snr[typeInt][prn] = maxSnr;
    // 构建InterferInfo输出
    for (int i = 0; i < size; i++)
    {
        info.i_Phase_Diff[i] = tp_diff[i] * 2 * PI;    // 相位差转换为弧度
        info.i_AntennaSq[i][0] = tp_antnna[i][0];       // 天线对-天线1
        info.i_AntennaSq[i][1] = tp_antnna[i][1];       // 天线对-天线2
    }
    info.i_Phase_Len = size;
}


// =========================================================================
// 根据map保存的数据,计算告警值
// 对每个频点依次进行:
// 1. 相位差法检测(calAlarmByPhaseDiff，载噪比≥35dB 作为数据质量门限)
// 2. 将检测结果合并到m_AngleResultData
// 注: 载噪比聚类(calAlarmBySnr)与角度聚类(calAlarmByAngle)已按新流程移除;
//     本函数为旧(非循环)检测路径，逐帧判定(单帧确认);循环检测路径的跨刀
//     连续确认由 getCyclicDetectionData 的 m_ConsecutiveAlarm 完成。
// =========================================================================
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

        // 欺骗检查 - 相位差法
        calAlarmByPhaseDiff(typeInt, tpA2, tpA, alarm);

        // 载噪比聚类检测已按新流程移除（原方法2: calAlarmBySnr），载噪比仅作为
        // 数据质量门限（≥35dB）在 calAlarmByPhaseDiff 中生效。

        if (0 == alarm){
            continue;
        }

        // 构建告警结果
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
            // 检查该卫星是否已存在于结果中(避免重复)
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
            tp.i_Angle = -1;     // 检测模式下角度未知
            tp.i_Quality = -1;   // 检测模式下质量未知
            tp.i_Snr = tpA[i].i_Snr1;
            m_Max_Snr[typeInt][tp.i_Prn] = tp.i_Snr;
            tp_alarmData.emplace_back(tp);
        }
        m_AngleResultData[typeInt] = tp_alarmData;
    }

}


// =========================================================================
// 多帧数据平滑处理
// 对每个切刀位置的多帧数据进行卫星相位差平滑
// 平滑方法: 排除差异大的帧，将剩余帧的相位转换为复数后取均值
// 注: 该方法用于减少噪声对相位差的影响，提高测向精度
// =========================================================================
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
        // 对每个切刀位置的多帧数据汇总
        for (int k = 0; k < m_OneCut_Frams; k++)
        {
            index = j * m_OneCut_Frams + k;
            oneCutData[k] = dataA[index];
        }
        // 汇总同一卫星在该刀各帧(秒)的相位差样本。按帧聚合时不套用
        // m_Delete_Prn_Flag 的「要求全部帧有效」过滤：某几帧缺失的卫星也保留，
        // 交给 calSmoothData 的覆盖判定 min(刀内帧数, MIN_STABLE_SAMPLES) 决定取舍，
        // 对应当前 Python build_vectors_and_detect / compute_calibration 的采样覆盖规则。
        // （原实现此处按 deletePrnFlag=1 会丢弃缺任一帧的卫星，比 Python 严苛太多）
        int savedDeleteFlag = m_Delete_Prn_Flag;
        m_Delete_Prn_Flag = 0;
        getSatelliteDataPhaseDiffB(oneCutData, dataB);  // 按卫星汇总
        m_Delete_Prn_Flag = savedDeleteFlag;
        // 校正刀与普通刀一样，只要求有效采样覆盖 min(刀内帧数, MIN_STABLE_SAMPLES) 帧，
        // 不再要求卫星从该刀第一帧(第一秒)就存在（Python compute_calibration 亦不要求）
        for (unsigned int i = 0; i < dataB.size(); i++)
        {
            calSmoothData(dataB[i], tpA);  // 对每颗卫星进行多帧平滑
            tpA2.emplace_back(tpA);
        }
        resultData.emplace_back(tpA2);
    }
    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA = resultData;
}

// =========================================================================
// 对单颗卫星多帧相位差做稳定性过滤 + 跳半周处理 + 圆形均值
// 对应 Python detection_lib.py 的 check_stability / circular_mean：
//   1. 收集载噪比有效(两端口 > 1e-3)的相位差样本（单位: 周 -> 度）；
//   2. 均匀分布：有效采样数 >= min(总帧数, MIN_STABLE_SAMPLES)；
//   3. 360° 圆上最小覆盖弧 < STABILITY_RANGE_DEG(15°) -> 稳定，取圆形均值；
//   4. 否则折叠到 [0,180) 后跨度 < 15° -> 检测到跳半周（HALF_CYCLE_CORRECT=false
//      时按波动大处理，不使用，载噪比置 0 表示无效）；
//   5. 其余不稳定样本丢弃（载噪比置 0 表示无效）。
// 不再要求卫星从该刀第一帧(第一秒)就存在（Python compute_calibration 亦不要求）。
// 载噪比置 0 后，下游(校正 calCorrecteData / 检测 calAlarmByPhaseDiff / 测向
// calAngleUseAntenna)都会跳过该卫星。
// =========================================================================
void SpoofingDoa::calSmoothData(SatelliteDataPhaseDiffB dataB, SatelliteDataPhaseDiffA &dataA)
{
    vector<double> samples;  // 相位差样本(度)
    double sumSnr1 = 0.0;
    double sumSnr2 = 0.0;
    int length = dataB.i_diffLen;
    for (int i = 0; i < length; ++i)
    {
        if (dataB.i_Snr1[i] < 1e-3 || dataB.i_Snr2[i] < 1e-3)
        {
            continue;  // 载噪比无效，不使用该样本
        }
        samples.emplace_back(dataB.i_phase_diff[i] * 360.0);  // 周 -> 度
        sumSnr1 += dataB.i_Snr1[i];
        sumSnr2 += dataB.i_Snr2[i];
    }

    // 组装输出基本字段
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

    // 均匀分布：至少覆盖 min(总帧数, MIN_STABLE_SAMPLES) 个有效采样
    int required = (length < MIN_STABLE_SAMPLES) ? length : MIN_STABLE_SAMPLES;
    if (n < required)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    // 稳定判定：360° 圆上最小覆盖弧 < STABILITY_RANGE_DEG(15°)
    if (circularSpanDeg(samples) < STABILITY_RANGE_DEG)
    {
        double smoothDeg = circularMeanDeg(samples);
        double smoothCycle = smoothDeg / 360.0;
        if (smoothCycle < 0)
        {
            smoothCycle += 1.0;  // 归一化到 [0,1) 周
        }
        dataA.i_phase_diff = smoothCycle;
        dataA.i_Snr1 = sumSnr1 / n;
        dataA.i_Snr2 = sumSnr2 / n;
        return;
    }

    // 跳半周检测：折叠到 [0,180) 后跨度 < STABILITY_RANGE_DEG(15°)。HALF_CYCLE_CORRECT=false 时
    // 跳半周按相位差波动大处理（不使用），载噪比置 0 表示无效。
    if (circularSpan180Deg(samples) < STABILITY_RANGE_DEG)
    {
        dataA.i_phase_diff = 0.0;
        dataA.i_Snr1 = 0.0;
        dataA.i_Snr2 = 0.0;
        return;
    }

    // 普通波动大，不使用
    dataA.i_phase_diff = 0.0;
    dataA.i_Snr1 = 0.0;
    dataA.i_Snr2 = 0.0;
}

// =========================================================================
// 取最后一帧数据(不进行平滑时使用)
// 当配置为多帧但不平滑时，仅保留每个切刀位置的最后一帧
// =========================================================================
void SpoofingDoa::getEndFramData(vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    vector<vector<SatelliteDataPhaseDiffA>> resultDataA;
    int cutNum = (int)m_cutSequence.size();
    for (int j = 0; j < cutNum; j++)
    {
        int index = 0;
        index = (j + 1) * m_OneCut_Frams - 1;  // 取最后一帧索引
        resultDataA.emplace_back(dataA[index]);
    }

    dataA.clear();
    vector<vector<SatelliteDataPhaseDiffA>>().swap(dataA);
    dataA = resultDataA;
}

// =========================================================================
// 设置各频点的阵列半径(m_R)
// 全向天线模式:所有频点使用统一半径(m_omni_R)
// 定向天线模式:根据频率从配置文件读取各频段的半径
// 定向天线的不同频率对应不同的有效阵列半径(天线方向图频率相关)
// =========================================================================
void SpoofingDoa::setR(const string adr){
    vector<Rs> m_Rs;
    m_Rs.clear();
    if (1 == m_antnenaType){
        ArithmeticDoa::getRData(adr, m_Rs);  // 读取定向天线频率-半径映射表
    }

    int typeInt = 0;
    double tpF = 0.0;

    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        typeInt = it->first;
        tpF = it->second;
        if (0 == m_antnenaType) // 全向天线
        {
            m_R[typeInt] = m_omni_R;  // 统一使用全向天线半径
        }
        else
        {
            // 定向天线:查找频率对应的可用半径
            for (unsigned int i = 0; i < m_Rs.size(); i++)
            {
                if (tpF >= m_Rs[i].i_starF && tpF <= m_Rs[i].i_endF)
                {
                    m_R[typeInt] = m_Rs[i].i_r;  // 使用该频段的半径
                    break;
                }
            }
        }
    }
}


// 设置告警门限
// @param threshold 卫星颗数阈值
// @param phsThreshold 相位差阈值(单位:度)
// @param sys 卫星系统(-1表示所有系统)
// @param type 卫星频点(-1表示所有频点)
// @note 当卫星系统和卫星频点均为-1时，表示对所有卫星系统和频率进行告警检测，否则对一个卫星系统和一个固定频率进行告警检测
void SpoofingDoa::setThresholdDetectionDoa(int threshold, double phsThreshold, int sys, int type)
{
    PublicSpace::Log("Sys=%i,Type=%i,coutThreshold=%i,phsThreshold=%.1f\n", sys, type, threshold, phsThreshold);
    if (-1 == sys && -1 == type)
    {
        // 对所有频点统一设置阈值
        initDetectionThreshold(threshold, phsThreshold);
        return;
    }

    // 对指定频点单独设置阈值
    if (-1 != threshold)
    {
        m_Detection_Threshold[TypeInt(sys, type)] = threshold;
    }
    if (phsThreshold > 0)
    {
        m_Detection_PhsThreshold[TypeInt(sys, type)] = phsThreshold / 360.0;  // 度转周
    }
}

// =========================================================================
// 设置是否使用载噪比进行欺骗检测
// 欺骗信号往往具有相同或相近的载噪比(同一干扰源发射)
// =========================================================================
void SpoofingDoa::setTypeDetectionBySnr(int thresholdNum, int sys, int type)
{
    if (-1 != thresholdNum)
    {
        m_Detection_snrThrehold[TypeInt(sys, type)] = thresholdNum;
    }
    PublicSpace::Log("join detection by snr:Sys=%i,Type=%i\n", sys, type);
}


// =========================================================================
// 读取配置文件并设置相关参数
// 配置文件格式: key=value 每行一个参数
// 配置参数包括:天线数量、角度阈值、相位差阈值、算法类型、切刀顺序等
// =========================================================================
void SpoofingDoa::setConfigData(const string adr)
{
    map<string, string> configMap;
    int flg = readConfigtxt(adr, configMap);

    if (0 != flg)
    {
        return;
    }

    // 将读取的配置参数写入参数中

    // 日志地址
    (void)getMapData(configMap, "logAdr", m_LogFile);

    // 日志标签 0 - 无需日志 1 - 清空重写 2 - 追加
    (void)getMapData(configMap, "logFlg", m_logFlg);

    LogCreat(m_LogFile);

    // 算法版本
    PublicSpace::Log("SpoofingDoa Version:%s\n", Version);

    // 天线数量
    (void)getMapData(configMap, "antennaNum", m_AntennaNum);
    PublicSpace::Log("antennaNum=%d\n", m_AntennaNum);

    // 角度门限(同一方向判定阈值，度)
    (void)getMapData(configMap, "angleThreshold", m_Angle_Threshold);
    PublicSpace::Log("angleThreshold=%.1f\n", m_Angle_Threshold);

    // 相差门限(相位差相近判定阈值，度)
    (void)getMapData(configMap, "phaseDiffThreshold", m_Phasediff_Threshold);
    PublicSpace::Log("phaseDiffThreshold=%.5f\n", m_Phasediff_Threshold);

    // 天线类型(0=全向, 1=定向)
    (void)getMapData(configMap, "antennaType", m_antnenaType);
    PublicSpace::Log("antennaType=%d\n", m_antnenaType);

    // 一刀帧数(每个切刀位置采集的帧数)
    (void)getMapData(configMap, "cutFrams", m_OneCut_Frams);
    PublicSpace::Log("cutFrams=%d\n", m_OneCut_Frams);

    // 平滑标签(0=不平滑, 1=平滑)
    (void)getMapData(configMap, "smoothFlg", m_Smooth_Flag);
    PublicSpace::Log("smoothFlg=%d\n", m_Smooth_Flag);

    // 切刀数量(用于测向的刀数)
    (void)getMapData(configMap, "doaCutNum", m_Doa_Cut_Num);
    PublicSpace::Log("doaCutNum=%d\n", m_Doa_Cut_Num);

    // 检测门限(欺骗检测的卫星颗数阈值)
    (void)getMapData(configMap, "detectionNum", m_Detection_Threshold_Num);
    PublicSpace::Log("detectionNum=%d\n", m_Detection_Threshold_Num);

    // 伪谱积分标签(0=不积分, 1=积分)
    (void)getMapData(configMap, "pseudoSpectrumFlag", m_PseudoSpectrum_Flag);
    PublicSpace::Log("pseudoSpectrumFlag=%d\n", m_PseudoSpectrum_Flag);

    // 检测标签(测向中是否进行欺骗检测)
    (void)getMapData(configMap, "doaDetectionFlag", m_Doa_Detection_Flag);
    PublicSpace::Log("doaDetectionFlag=%d\n", m_Doa_Detection_Flag);

    // 循环切刀检测标签(是否使用循环切刀检测流程)
    (void)getMapData(configMap, "cyclicDetectionFlag", m_Cyclic_Detection_Flag);
    PublicSpace::Log("cyclicDetectionFlag=%d\n", m_Cyclic_Detection_Flag);

    // 质量门限(低于此值的结果丢弃)
    (void)getMapData(configMap, "qulityThreshold", m_Qulity_Threshold);
    PublicSpace::Log("qulityThreshold=%.1f\n", m_Qulity_Threshold);

    // 信噪比门限(低于此值的数据丢弃)
    (void)getMapData(configMap, "snrThreshold", m_Snr_Threshold);
    PublicSpace::Log("snrThreshold=%.1f\n", m_Snr_Threshold);

    // 虚拟孔径标签(0=不使用, 1=使用虚拟阵列扩展)
    (void)getMapData(configMap, "virtualFlag", m_Virtual_Flag);
    PublicSpace::Log("virtualFlag=%d\n", m_Virtual_Flag);

    // 虚拟空间倍数
    (void)getMapData(configMap, "virtualMultiple", m_Virtual_Multiple);
    PublicSpace::Log("virtualMultiple=%.2f\n", m_Virtual_Multiple);

    // 原始数据保存标签
    (void)getMapData(configMap, "saveOriginalFlg", m_Save_Original_Flg);
    PublicSpace::Log("saveOriginalFlg=%.2f\n", m_Save_Original_Flg);

    // 算法类型(1=干涉仪, 2=幅相法, 3=干涉仪+仿真)
    (void)getMapData(configMap, "doaArithmetic", m_Doa_Arithmetic);
    PublicSpace::Log("doaArithmetic=%d\n", m_Doa_Arithmetic);

    // 数据删除标签(0=保留部分有效, 1=要求全部有效)
    (void)getMapData(configMap, "deletePrnFlag", m_Delete_Prn_Flag);
    PublicSpace::Log("deletePrnFlag=%d\n", m_Delete_Prn_Flag);

    // 二次确认标签(0=进行二次确认, 1=不进行)
    (void)getMapData(configMap, "secondaryDoaFlag", m_Secondary_Doa_Flag);
    PublicSpace::Log("secondaryDoaFlag=%d\n", m_Secondary_Doa_Flag);

    // 欺骗检测结果筛选标签
    (void)getMapData(configMap, "screendetectionFlag", m_Screen_detection_Flag);
    PublicSpace::Log("screendetectionFlag=%d\n", m_Screen_detection_Flag);

    // 用载噪比确定用于测向的天线序号的标签
    (void)getMapData(configMap, "getUseAntennaBySnrFlag", m_getUseAntennaBySnr_Flag);
    PublicSpace::Log("getUseAntennaBySnrFlag=%d\n", m_getUseAntennaBySnr_Flag);

    // 测向结果积分因子(伪谱指数衰减系数)
    (void)getMapData(configMap, "accumulateMultiplier", m_Accumulate_multiplier);
    PublicSpace::Log("accumulateMultiplier=%.2f\n", m_Accumulate_multiplier);

    // 用于测向的相位差的最少数量
    (void)getMapData(configMap, "doaMinCutNum", m_Doa_Cut_min_Num);
    PublicSpace::Log("doaMinCutNum=%d\n", m_Doa_Cut_min_Num);

    // 当修复跳半周后的欺骗测向阈值，比前一次测向结果要大于该值才替换为修复后的测向结果
    (void)getMapData(configMap, "detection180QulityThreshold", m_Detection180_Qulity_Threshold);
    PublicSpace::Log("detection180QulityThreshold=%.2f\n", m_Detection180_Qulity_Threshold);

    // 阵列仿真数据路径
    (void)getMapData(configMap, "simulateFile", m_Simulate_Data_file);
    PublicSpace::Log("simulateFile=%s\n", m_Simulate_Data_file.c_str());

    // 欺骗检测记录个数，当连续个数均检测为欺骗信号时，才判定为欺骗信号
    (void)getMapData(configMap, "detectionRecoddsNum", m_Detection_Recodds_Num);
    PublicSpace::Log("detectionRecoddsNum=%d\n", m_Detection_Recodds_Num);

    // 数据保存标签
    (void)getMapData(configMap, "saveDataFlg", m_save_data_Flg);
    PublicSpace::Log("saveDataFlg=%d\n", m_save_data_Flg);

    // 切刀方式(RF开关切换顺序)
    string tp;
    int flg2 = getMapData(configMap, "cutThw", tp);

    if (flg2 == 0){
        string2Vector(tp, m_cutSequence);  // 解析切刀顺序字符串
        int cutSequence_num = m_cutSequence.size();
        // 如果最小测向刀数超过总刀数，自动调整为总刀数-1
        if (m_Doa_Cut_min_Num >= cutSequence_num){
            m_Doa_Cut_min_Num = cutSequence_num - 1;
            PublicSpace::Log("doaMinCutNum=%d\n", m_Doa_Cut_min_Num);
        }
    }


    // 仿真数据频率
    string tp2;
    vector<vector<double>> tp4;
    int flg3 = getMapData(configMap, "SimulateFre", tp2);

    if (flg3 == 0)
    {
        string2Vector(tp, tp4);
        m_All_Simulate_data_Fre = tp4[0];
    }


    // 孔径设置
    if (m_antnenaType == 0)
    {
        getMapData(configMap, "r", m_omni_R);  // 全向天线:统一半径
    }
    else
    {
        getMapData(configMap, "Radr", m_Radr);  // 定向天线:半径配置文件路径
    }
}

// =========================================================================
// 清空循环切刀检测的连续报警计数与跟踪状态（对应 Python consecutive / tracking）
// =========================================================================
void SpoofingDoa::resetCyclicDetection(void)
{
    m_ConsecutiveAlarm.clear();
    m_Tracking.clear();
    m_Baselines.clear();   // 跨周期测向基线一并清空
}

// =========================================================================
// 设备 wrapper(GN902 等)覆盖循环切刀运行参数
// 供外部按“整轮(7 刀)流式喂帧”驱动引擎时使用；omniR>0 时按该半径重建
// m_R 与测向理论模板，确保与 Python ARRAY_RADIUS(=0.1865, GN902)一致。
// =========================================================================
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
        setR(m_Radr);           // 全向天线：m_R[typeInt] = m_omni_R
        m_Theory.clear();
        if (1 == m_Doa_Arithmetic)
        {
            initTheory();       // 按新半径重建 360° 理论相位模板
        }
    }
    PublicSpace::Log("configCyclicRuntime: cyclic=%d oneCutFrams=%d smooth=%d omniR=%.4f\n",
                     m_Cyclic_Detection_Flag, m_OneCut_Frams, m_Smooth_Flag, m_omni_R);
}

// =========================================================================
// 循环切刀欺骗检测（对应 Python detection_main.py / detection_lib.py 循环切刀主流程）
// 对每刀做相位差聚类检测，跨刀连续确认(连续 p=m_Detection_Recodds_Num 刀)，确认后进入
// 测向跟踪；只保留已确认欺骗的 (系统,频点) 卫星用于后续测向。
// 数据流: dataA 已按刀划分且完成校正，每刀 = 该刀稳定(校正后)相位差的卫星列表。
// =========================================================================
void SpoofingDoa::getCyclicDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    int cutNum = (int)dataA.size();

    for (int j = 0; j < cutNum; ++j)
    {
        // 跳过同天线自校准刀(仅用于校正，不参与检测)
        if (j < (int)m_cutSequence.size() && m_cutSequence[j][0] == m_cutSequence[j][1])
        {
            continue;
        }

        std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
        getSatelliteDataByType(dataA[j], dataT);

        // 本刀各频点报警的卫星号集合
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

        // 1. 本刀未报警的 (系统,频点) 中断连续
        for (auto &kv : m_ConsecutiveAlarm)
        {
            if (cutAlarms.find(kv.first) == cutAlarms.end())
            {
                kv.second = 0;
                // 连续报警被打断：仅当该 (系统,频点) 尚未确认(未进入跟踪)时才清空基线；
                // 已确认(跟踪中)的信号即使本刀聚类未达阈值(如严格阈值的 Galileo E1C
                // 某刀只聚集 3 颗)，也保留跨刀/跨周期累积的相位差(对应 Python 同修复)。
                if (m_Tracking.find(kv.first) == m_Tracking.end())
                {
                    m_Baselines.erase(kv.first);
                }
            }
        }
        // 2. 本刀报警的 (系统,频点) 连续 +1；达到 p 次确认进入跟踪
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

    // 3. 只保留已确认(跟踪中)频点的【聚集卫星(cluster_sats)】数据，供后续相关干涉仪测向
    //    （对齐 Python run_doa_one：只对 tracking.info.cluster_sats 里的卫星做基线测向，
    //      而不是给该频点的全部卫星测向）
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

// =========================================================================
// 把本轮 dataB 合并进跨周期测向基线(相位差跨轮复用做测向)
// 对应 Python detection_main.py 的 baselines 累积：
//   - 每轮对同一 (系统,频点,卫星号)，仅覆盖本轮载噪比有效的刀位相位差；
//     本轮缺失(载噪比=0)的刀位保留上一周期的相位差，从而凑齐 6 条基线。
//   - 前提：校正偏移相邻轮基本稳定(与 Python 跨周期复用一致)。
// =========================================================================
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
            inner[prn] = b;  // 新卫星：整体初始化
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

// =========================================================================
// 取出当前跟踪中(系统,频点)的跨周期累积基线，作为测向输入
// 对应 Python run_doa_one：只对 tracking.cluster_sats 里的卫星做基线测向。
// =========================================================================
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

// =========================================================================
// 对每刀数据进行欺骗检测，筛选出检测为欺骗的信号用于后续测向
// 两种模式:
//   mode=1: 每刀独立检测，各刀独立筛选
//   mode=2: 所有刀合并检测，筛选所有刀中共有的欺骗信号
// =========================================================================
void SpoofingDoa::getSpoofingDetectionData(std::vector<vector<SatelliteDataPhaseDiffA>> &dataA)
{
    m_AngleResultData.clear();
    int dataLen = (int)dataA.size();
    int cutNum = (int)m_cutSequence.size();
    if (1 == m_Doa_Detection_Flag)
    {
        // 模式1: 每刀独立检测
        for (int i = 0; i < dataLen; i++)
        {
            std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
            getSatelliteDataByType(dataA[i], dataT);   // 按频点分类
            getAlarm(dataT);                            // 欺骗检测
            setSpoofingDetectionData(dataA[i]);         // 筛选数据
            m_AngleResultData.clear();
        }
    }
    if (2 == m_Doa_Detection_Flag)
    {
        // 模式2: 所有刀合并检测
        std::map<int, std::vector<SatelliteDataPhaseDiffA>> dataT;
        for (int i = 0; i < dataLen; i++)
        {
            // 跳过同天线自检刀(这些刀不包含测向信息)
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

        // 对所有刀的数据进行筛选
        for (int i = 0; i < dataLen; i++)
        {
            setSpoofingDetectionData(dataA[i]);
        }
    }
    m_AngleResultData.clear();
}

// =========================================================================
// 根据欺骗检测结果筛选卫星数据
// 仅保留在m_AngleResultData中标记为欺骗的卫星数据
// =========================================================================
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
            continue;  // 该频点未检测到欺骗
        }
        else
        {
            tp_alarm = m_AngleResultData[typeInt];
            for (unsigned int j = 0; j < tp_alarm.size(); j++)
            {
                if (tp_alarm[j].i_Prn == prn)
                {
                    resultData.emplace_back(tpA);  // 保留标记为欺骗的卫星
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

// =========================================================================
// 全排列组合方式优化跳半周
// 原理: 对每刀相位差尝试±0.5周(半周)的补偿，枚举所有组合
//       对每种组合进行干涉仪测向，选取测向质量最高的那个组合
// 适用场景: 当载波相位存在整周模糊或半周跳变时
// 注意: 2^n种组合(n=相位差数量-1)，组合数随刀数指数增长
// =========================================================================
void SpoofingDoa::setPermutationOptimiz180(SatelliteDataPhaseDiffB dataB, const vector<vector<double>> phaseTheory, double &angle, double &qulity)
{

    int length = dataB.i_diffLen - 1;  // 排除参考刀(第0刀)
    int combinations = std::pow(2, length);  // 全排列组合数 = 2^n
    qulity = -99.0;
    SatelliteDataPhaseDiffB tpB;
    for (int i = 0; i < combinations; ++i)
    {
        tpB = dataB;
        // 根据二进制位决定每刀是否加半周
        for (int j = 1; j < length + 1; ++j)
        {
            double tp_phs_diff_original = dataB.i_phase_diff[j];
            if (i & (1 << (j - 1)))
            {
                tpB.i_phase_diff[j] = tp_phs_diff_original + 0.5;  // 加半周
            }
            else
            {
                tpB.i_phase_diff[j] = tp_phs_diff_original;  // 保持不变
            }
        }
        PublicSpace::Log("optimiz:num=%d\n", i);
        LogSatelliteDataPhaseDiffB(tpB);
        vector<double> pseudoValue;
        pseudoValue.clear();
        int doaFlg = 1;
        InterferInfo tp_info1;
        tp_info1.i_Start = 0;
        tp_info1.i_End = 359;
        tp_info1.i_Phase_Len = 0;

        double tp_angle1;
        double tp_quality1;
        calAngleUseAntenna(tpB, tp_info1, doaFlg);               // 构建InterferInfo
        ArithmeticDoa::calInterfer(phaseTheory, tp_info1, tp_angle1, tp_quality1, pseudoValue);  // 干涉仪测向
        PublicSpace::Log("Angle=%.1f,Quality=%.1f\n", tp_angle1, tp_quality1);
        // 质量达到最优立即返回
        if (tp_quality1 >= 99.9)
        {
            angle = tp_angle1;
            qulity = tp_quality1;
            return;
        }
        // 保留质量最高的组合
        if (tp_quality1 > qulity)
        {
            angle = tp_angle1;
            qulity = tp_quality1;
        }
    }
}

// =========================================================================
// 对所有卫星进行跳半周优化测向
// 对每颗卫星使用全排列组合方式尝试修复跳半周
// 结果存入m_AngleResultData
// =========================================================================
void SpoofingDoa::getOptimizResultDoa180(vector<SatelliteDataPhaseDiffB> dataB)
{
    int size = (int)dataB.size();
    int typeInt = 0;
    SatelliteDataPhaseDiffB tpB;
    vector<vector<double>> phaseTheory;
    AlarmData tp_alarm;
    vector<AlarmData> tp_alarms;
    for (int i = 0; i < size; i++)
    {
        tp_alarms.clear();
        tpB = dataB[i];
        typeInt = TypeInt(tpB.i_Sys, tpB.i_Type);
        phaseTheory.clear();
        phaseTheory = m_Theory[typeInt];
        double angle;
        double quality = 0.0;
        setPermutationOptimiz180(tpB, phaseTheory, angle, quality);  // 全排列优化

        tp_alarm.i_Prn = tpB.i_Prn;
        tp_alarm.i_Angle = (int)angle;
        tp_alarm.i_Quality = quality;

        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            tp_alarms.emplace_back(tp_alarm);
            m_AngleResultData[typeInt] = tp_alarms;
        }
        else
        {
            tp_alarms = m_AngleResultData[typeInt];
            tp_alarms.emplace_back(tp_alarm);
            m_AngleResultData[typeInt] = tp_alarms;
        }
    }
}

// =========================================================================
// 设置切刀顺序(外部接口调用)
// 接收一维数组表示的天线对序列，转换为二维向量
// @param len: 数组长度(天线序号个数，需为偶数)
// @param cutSq: 天线序号数组[天线1a, 天线1b, 天线2a, 天线2b, ...]
// =========================================================================
void SpoofingDoa::setCutSquence(int len, const int *cutSq)
{
    m_cutSequence.clear();
    vector<vector<int>>().swap(m_cutSequence);
    int size = len / 2;  // 天线对数量 = 总长度 / 2
    m_cutSequence.resize(size);
    int tp_index = 0;
    for (int i = 0; i < size; i++)
    {
        m_cutSequence[i].resize(2);
        tp_index = i * 2;
        if (cutSq[tp_index] < 1 || cutSq[tp_index + 1] < 1)
        {
            m_cutSequence.clear();  // 天线序号无效，清空
            return;
        }
        m_cutSequence[i][0] = cutSq[tp_index];      // 天线对-天线1
        m_cutSequence[i][1] = cutSq[tp_index + 1];  // 天线对-天线2
        PublicSpace::Log("set cutSequence:%d,m_cutSequence1 = %d,m_cutSequence2 = %d\n", i + 1, m_cutSequence[i][0], m_cutSequence[i][1]);
    }
}

// =========================================================================
// 设置外部欺骗检测结果(由其他检测模块传入)
// 用于多模块联合检测，交叉验证
// =========================================================================
void SpoofingDoa::setSpoofingDetecteResult(const vector<SingleDeceptiveResult> detectResult)
{
    m_detectResult.clear();
    vector<SingleDeceptiveResult>().swap(m_detectResult);
    if (detectResult.size() > 0)
    {
        m_detectResult = detectResult;
    }

    if (0 != m_logFlg)
    {
        PublicSpace::Log("detecion Spoofing result by other arithmetic:\n");
        for (int i = 0; i < detectResult.size(); i++)
        {
            PublicSpace::Log("Sys=%d,Type=%d,Prn={", detectResult[i].r_Sys, detectResult[i].r_Type);
            for (int j = 0; j < detectResult[i].prncount; j++)
            {
                PublicSpace::Log("%d,", detectResult[i].r_Prn[j]);
            }
            PublicSpace::Log("}\n");
        }
    }
}

// =========================================================================
// 根据外部欺骗检测结果筛选GNSS数据
// 仅保留外部检测结果中标记为欺骗的系统和频点的卫星数据
// =========================================================================
void SpoofingDoa::getDataByDetectionResult(const GNSSData data, GNSSData &resultData)
{

    GNSSData detetectData;
    int detect_num = (int)m_detectResult.size();
    int tp_result_num = 0;
    detetectData.i_PortOneNum = 0;
    detetectData.i_PortTwoNum = 0;
    for (int i = 0; i < detect_num; i++)
    {
        // 筛选端口1的数据
        for (int j = 0; j < data.i_PortOneNum; j++)
        {
            if (data.i_PortOne[j].i_Type == m_detectResult[i].r_Type && data.i_PortOne[j].i_Sys == m_detectResult[i].r_Sys)
            {
                tp_result_num = 0;
                for (int k1 = 0; k1 < m_detectResult[i].prncount; k1++)
                {
                    if (data.i_PortOne[j].i_Prn == m_detectResult[i].r_Prn[k1])
                    {
                        tp_result_num = detetectData.i_PortOneNum;
                        detetectData.i_PortOne[tp_result_num] = data.i_PortOne[j];

                        detetectData.i_PortOneNum = tp_result_num + 1;
                        break;
                    }
                }
            }
        }
        // 筛选端口2的数据
        for (int j = 0; j < data.i_PortTwoNum; j++)
        {
            tp_result_num = 0;
            if (data.i_PortTwo[j].i_Type == m_detectResult[i].r_Type && data.i_PortTwo[j].i_Sys == m_detectResult[i].r_Sys)
            {
                for (int k1 = 0; k1 < m_detectResult[i].prncount; k1++)
                {
                    if (data.i_PortTwo[j].i_Prn == m_detectResult[i].r_Prn[k1])
                    {
                        tp_result_num = detetectData.i_PortTwoNum;
                        detetectData.i_PortTwo[tp_result_num] = data.i_PortTwo[j];

                        detetectData.i_PortTwoNum = tp_result_num + 1;
                        break;
                    }
                }
            }
        }
    }
    resultData = detetectData;
}

// =========================================================================
// 根据外部欺骗检测结果筛选测向结果
// 将外部检测模块标记为欺骗但本模块未计算测向结果的频点补充到输出中
// 通过角度加权投票确定补充频点的到达角
// =========================================================================
void SpoofingDoa::getSpoofingResultByDetection(SpoofingResult &result)
{
    SpoofingResult tp_result = result;
    int detect_num = (int)m_detectResult.size();
    int count = result.i_Count;
    SatelliteAngle tp_Satellite;
    bool flg = false;
    int typeInt = 0;
    vector<AlarmData> tp_alarms;
    SingleDeceptiveResult tp_decepResult;

    AlarmData tp_result_alarm;

    for (int i = 0; i < detect_num; i++)
    {
        tp_decepResult = m_detectResult[i];
        int tp_sys = tp_decepResult.r_Sys;
        int tp_type = tp_decepResult.r_Type;
        flg = true;
        // 检查该频点是否已在测向结果中
        for (int j = 0; j < count; j++)
        {
            tp_Satellite = tp_result.i_SatelliteAngle[j];
            if (tp_Satellite.i_Sys == tp_sys && tp_Satellite.i_Type == tp_type)
            {
                flg = false;  // 已存在
                break;
            }
        }
        if (flg)
        {
            // 该频点不在测向结果中，需补充
            typeInt = TypeInt(tp_sys, tp_type);
            if (m_AngleResultData.find(typeInt) != m_AngleResultData.end())
            {
                SatelliteAngle tp_result_SateA;
                int SateASize = 0;
                vector<double> angles(360, 0.0);  // 角度加权投票累积器
                tp_alarms = m_AngleResultData[typeInt];
                int PrnNum = (int)tp_alarms.size();
                int detec_PrnNum = tp_decepResult.prncount;
                for (int k1 = 0; k1 < detec_PrnNum; k1++)
                {
                    for (int k2 = 0; k2 < PrnNum; k2++)
                    {
                        if (tp_alarms[k2].i_Prn == tp_decepResult.r_Prn[k1])
                        {
                            tp_alarms[k2].i_Snr = m_Max_Snr[typeInt][tp_alarms[k2].i_Prn];
                            tp_result_SateA.i_AlarmData[SateASize] = tp_alarms[k2];
                            SateASize = SateASize + 1;
                            // 角度加权投票:以角度为中心，±阈值范围内按距离和测向质量加权
                            int tp_angle = tp_alarms[k2].i_Angle;
                            double tp_qulity = tp_alarms[k2].i_Quality / 100;
                            for (int k3 = (0 - m_Angle_Threshold); k3 < m_Angle_Threshold; k3++)
                            {
                                int ang = tp_angle + k3;
                                int ang2 = Round360(ang);
                                angles[ang2] = angles[ang2] + (1 - fabs(k3) * 0.1) * tp_qulity;
                            }

                            break;
                        }
                    }
                }
                if (SateASize > 0)
                {
                    // 找到投票最高的角度
                    double maxAngl = -999.9;
                    int tp_angle = -1;
                    for (int k3 = 0; k3 < 360; k3++)
                    {
                        if (angles[k3] > maxAngl)
                        {
                            maxAngl = angles[k3];
                            tp_angle = k3;
                        }
                    }
                    int tp_count = result.i_Count;
                    tp_result_SateA.i_Sys = tp_sys;
                    tp_result_SateA.i_Type = tp_type;
                    tp_result_SateA.i_Alarm = 0;
                    tp_result_SateA.i_Count = SateASize;
                    tp_result_SateA.i_Angle = tp_angle;
                    result.i_Count = result.i_Count + 1;
                    result.i_SatelliteAngle[tp_count] = tp_result_SateA;
                }
            }
        }
    }
}

// =========================================================================
// 保存原始GNSS数据到二进制文件
// 用于离线回放和调试分析
// =========================================================================
void SpoofingDoa::saveGNSSData(const GNSSData *data, int dataLen)
{
    string fileName = "GNSSData_";
    fileName.append(to_string(dataLen));
    fileName.append(".dat");
    PublicSpace::saveArrayToBinary(fileName, data, dataLen);
}
