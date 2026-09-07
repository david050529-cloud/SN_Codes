// =============================================================================
// 文件名: GN902.cpp
// 功能描述: GN902 欺骗检测/测向模块 —— 自包含实现文件(消除编译隔离)
//
// 本文件收纳了以下原独立源文件的全部函数实现(逐字保留, 便于跨函数内联优化):
//   核心引擎   SpoofingDoa.cpp      InitData.cpp     PreparationData.cpp
//              Corrected.cpp        Interf.cpp       AmpPhase.cpp
//              Omni.cpp             Directed.cpp     Alarm.cpp
//              SpectrumDesity.cpp   WriteLog.cpp
//   接口实现   GN902.cpp (GN902 类: 实例状态机 + 整轮帧缓冲 + 检测/测向调度)
//
// 所有类型与类声明见配套头文件 GN902.h; 外部依赖(Arithmetic / publicFunctionDoa
// 静态库) 见 GN902.h 头部注释。
// =============================================================================
#include "GN902.h"


// =============================================================================
// == 原 SpoofingDoa.cpp —— 核心引擎实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: SpoofingDoa.cpp
// 功能描述: 欺骗干扰测向(DOA)主类的实现文件
// 系统角色: 本文件实现了欺骗测向系统的核心调度逻辑，包括:
//           1. 构造函数:初始化项目参数、频率表、检测阈值、理论相位差模板
//           2. setGNSSData:数据分发入口，根据dataLen区分检测/测向/校正模式
//           3. getAngleSpoofingDoa:输出最终测向结果，支持伪谱积分和外部筛选
//           4. setDataAngle:测向主流程(相位差计算→校正→检测→干涉仪/幅相法测向)
//           5. setDataAlarm:欺骗检测模式流程(相位差计算→按频点分类→三方法检测)
//           6. calAngle:干涉仪测向汇总(按频点逐星计算+跳半周检测与修复)
//           7. 跳半周检测与优化:检测相位跳变并使用全排列组合进行修复
// 配置说明: 各运行参数(阵列/切刀/阈值/流程开关)直接在 Init() 中内置, 不读取外部配置文件
// =============================================================================


// 欺骗测向算法对象构造函数
// 初始化流程: initProject(设置硬件参数) -> Init(内置运行参数+频率表+理论模板)
SpoofingDoa::SpoofingDoa(void){
    initProject();
    Init();
    // m_Phasediff_Threshold = m_Phasediff_Threshold / 360.0; // 将度转化为周
    // 部分频点的检测阈值单独设置，与 Python detection_lib.ORIGINAL_CONFIG_TEXT 规则一致
    // （2026-09 对齐：GLONASS G1/G2、BDS B1C 的颗数阈值已按 Python 修正）
    // 判定为严格大于（bestCount > m_Detection_Threshold），对应 Python 的 cnt > count_thr
    // 实参顺序固定为: (系统sys, 频点type, 卫星颗数阈值threshold, 相位差阈值phsThreshold)
    setThresholdDetectionDoa(0, 2, 4, -1);    // GPS L5:     Python Sys=0,Type=2  count=4(触发≥5)
    setThresholdDetectionDoa(1, 0, 2, 5.0);   // GLONASS G1: Python Sys=1,Type=0  count=2(触发≥3) phs=5.0
    setThresholdDetectionDoa(1, 1, 3, 5.0);   // GLONASS G2: Python Sys=1,Type=1  count=3(触发≥4) phs=5.0
    setThresholdDetectionDoa(3, 2, 3, 3.6);   // Galileo E1C: Python Sys=3,Type=2 count=3(触发≥4) phs=3.6
    setThresholdDetectionDoa(3, 12, 4, 3.6);  // Galileo E5a: Python Sys=3,Type=12 count=4(触发≥5) phs=3.6
    setThresholdDetectionDoa(3, 17, 4, 3.6);  // Galileo E5b: Python Sys=3,Type=17 count=4(触发≥5) phs=3.6
    setThresholdDetectionDoa(4, 17, 3, -1);   // BDS B2I:     Python Sys=4,Type=17 count=3(触发≥4)
    setThresholdDetectionDoa(4, 0, 3, -1);    // BDS B1I:     Python Sys=4,Type=0  count=3(触发≥4)
    setThresholdDetectionDoa(4, 2, 3, -1);    // BDS B3I:     Python Sys=4,Type=2  count=3(触发≥4)
    setThresholdDetectionDoa(4, 8, 2, -1);    // BDS B1C:     Python Sys=4,Type=8  count=2(触发≥3)（新增）
    setThresholdDetectionDoa(4, 19, 3, -1);   // BDS B2b:     Python Sys=4,Type=19 count=3(触发≥4)
    setThresholdDetectionDoa(4, 34, 3, 3.6);  // BDS B1X:     Python Sys=4,Type=34 count=3(触发≥4) phs=3.6
    // setTypeDetectionBySnr(1, 1, 0);
}


// 完整初始化函数(可重复调用)
// 初始化顺序:
// 1. 清空累积数据(伪谱积分、校正数据、历史记录)
// 2. 内置运行参数(原 DoaBSpoofingConfig.txt 内容, 直接内嵌于本函数)
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

    // =========================================================================
    // 内置运行参数(不读取外部配置文件)
    // 以下取值与原始 DoaBSpoofingConfig.txt(2026-09 快照) 逐项一致，直接内嵌在代码中，
    // 消除运行时对配置文件的依赖；需要调整参数时改这里即可(括号内为原配置文件 key)。
    // 说明: initProject() 默认走 GN930U 分支(半径0.2/切刀{7,7}...)，此处按下方的
    //       GN902 配置覆盖，之后 setR()/initTheory() 按覆盖后的 m_omni_R 重建理论模板。
    // =========================================================================

    // ---- 日志: logAdr / logFlg ----
    m_LogFile = "./spoofingDoaLog_";
    PublicSpace::m_logFlg = 0;   // 0=关闭日志
    LogCreat(m_LogFile);

    // ---- 阵列物理参数: antennaNum / antennaType / r ----
    m_AntennaNum = 7;            // 天线阵元个数
    m_antnenaType = 0;           // 0=全向天线, 1=定向天线
    m_omni_R = 0.1865;           // 全向天线阵列半径(米) = GN902 硬件参数(与 Python ARRAY_RADIUS 一致)
    m_Radr.clear();              // 定向天线半径文件路径(全向天线不使用)

    // ---- 切刀顺序: cutThw ----
    // {1,1} = 自校准刀(天线对相同，用于通道校正，不参与测向)，
    // {1,2}~{1,7} = 六刀测向基线(对应 Python code 0/9/57/17/25/33/1)。
    m_cutSequence = { {1,1}, {1,2}, {1,3}, {1,4}, {1,5}, {1,6}, {1,7} };

    // ---- 平滑/帧数: cutFrams / smoothFlg ----
    m_OneCut_Frams = 8;          // 一刀帧数(循环切刀检测每刀 8 帧, 与 dat 8 帧/刀一致)
    m_Smooth_Flag = 1;           // 1=多帧平滑(稳定性过滤+圆形均值)

    // ---- 测向刀数/最少相位差数: doaCutNum / doaMinCutNum ----
    m_Doa_Cut_Num = 6;
    m_Doa_Cut_min_Num = 6;
    if (m_Doa_Cut_min_Num >= (int)m_cutSequence.size()) {
        m_Doa_Cut_min_Num = (int)m_cutSequence.size() - 1;  // 自动限制不超过总刀数-1
    }

    // ---- 检测门限: angleThreshold / phaseDiffThreshold / detectionNum / snrThreshold ----
    m_Angle_Threshold = 3.0;     // 同一方向判定阈值(度)
    m_Phasediff_Threshold = 5.0; // 相位差相近判定阈值(度)
    m_Detection_Threshold_Num = 2; // 卫星颗数默认阈值(严格大于判定, 触发≥3)
    m_Snr_Threshold = 35.0;      // 载噪比门限(dB-Hz), 低于该值不参与处理
    m_Qulity_Threshold = 0.0;    // 测向质量门限

    // ---- 流程开关 ----
    m_Cyclic_Detection_Flag = 1; // cyclicDetectionFlag: 1=循环切刀检测(每刀聚类+跨刀连续确认+测向跟踪)
    m_Doa_Detection_Flag = 0;    // doaDetectionFlag: 0=测向中不额外做欺骗检测(循环流程覆盖)
    m_Doa_Arithmetic = 1;        // doaArithmetic: 1=相关干涉仪
    m_PseudoSpectrum_Flag = 0;   // pseudoSpectrumFlag: 0=不做伪谱积分
    m_Screen_detection_Flag = 0; // screendetectionFlag: 0=不按外部欺骗结果筛选
    m_Secondary_Doa_Flag = 0;    // secondaryDoaFlag: 0=进行二次确认测向
    m_Delete_Prn_Flag = 1;       // deletePrnFlag: 1=删除存在缺失刀数据的卫星
    m_getUseAntennaBySnr_Flag = 0; // getUseAntennaBySnrFlag: 0=不用载噪比定测向天线序号

    // ---- 虚拟阵列: virtualFlag / virtualMultiple ----
    m_Virtual_Flag = 0;          // 0=不使用虚拟阵列
    m_Virtual_Multiple = 0.94;   // 虚拟阵列扩展倍数(未使用)

    // ---- 跨刀连续确认: detectionRecoddsNum ----
    m_Detection_Recodds_Num = 2; // 连续多少刀判为欺骗才最终确认(对应 Python ALARM_CONSECUTIVE_P)

    // ---- 跳半周修复阈值: detection180QulityThreshold ----
    m_Detection180_Qulity_Threshold = 10.0;

    // ---- 数据保存: saveOriginalFlg / saveDataFlg / accumulateMultiplier ----
    m_Save_Original_Flg = 0;     // 0=不保存原始GNSS数据
    PublicSpace::m_save_data_Flg = 0; // 0=不保存数据
    m_Accumulate_multiplier = 0; // 伪谱积分衰减因子(未启用积分)

    // ---- 阵列仿真(仅 doaArithmetic=2/3 使用, 干涉仪+全向不依赖): simulateFile / SimulateFre ----
    m_Simulate_Data_file = "/simulateData/";
    m_All_Simulate_data_Fre = {1176e6, 1279e6, 1561e6, 1602e6};
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
// @param sys 卫星系统(-1表示所有系统)
// @param type 卫星频点(-1表示所有频点)
// @param threshold 卫星颗数阈值
// @param phsThreshold 相位差阈值(单位:度)
// @note 当卫星系统和卫星频点均为-1时，表示对所有卫星系统和频率进行告警检测，否则对一个卫星系统和一个固定频率进行告警检测
void SpoofingDoa::setThresholdDetectionDoa(int sys, int type, int threshold, double phsThreshold)
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

// =============================================================================
// == 原 InitData.cpp —— 初始化与配置模块实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: InitData.cpp
// 功能描述: 初始化与配置模块
// 系统角色: 负责欺骗测向系统的所有初始化工作，包括:
//           1. 硬件平台参数初始化(天线数、切刀数、阵列半径等)
//           2. GNSS频点频率映射表初始化(m_F)
//           3. 欺骗检测阈值初始化
//           4. 检测历史记录初始化(滑动窗口队列)
//
// 支持设备: GN902, GN930U, GN930, GN560
// 支持星座: GPS, GLONASS, SBAS, Galileo, BDS, QZSS
// =============================================================================


// =========================================================================
// 初始化测向对象(根据项目号设置硬件参数)
// 不同项目的硬件配置差异:
//   GN902:   全向天线，7阵元，半径0.1865m，切刀 7+1刀
//   GN930:   全向天线，7阵元，半径0.1865m，切刀 7+1刀(带双板卡)
//   GN930U:  全向天线，7阵元，半径0.2000m，切刀 7刀
//   GN560:   定向天线，7阵元，半径0.1800m，切刀 7刀(环形连接)
// 关键参数:
//   m_AntennaNum:     天线阵元数
//   m_Doa_Cut_Num:    用于测向的刀数
//   m_Doa_Cut_min_Num: 最少有效相位差数量
//   m_omni_R:         全向天线阵列半径(米)
//   m_antnenaType:    天线类型(0全向/1定向)
//   m_cutSequence:    切刀顺序(RF开关切换序列)
// =========================================================================
void SpoofingDoa::initProject(void)
{

    switch (m_Project_flg)
    {
    case GN902:
        m_AntennaNum = 7;  // 阵列中的天线个数
        m_Doa_Cut_Num = 6; // 用于测向的刀数
        m_Doa_Arithmetic = 1;  // 使用相关干涉仪算法
        m_Doa_Cut_min_Num = 6; // 用于测向的相位差的最少数量
        m_omni_R = 0.1865;    // 全向天线阵列半径(米)
        m_antnenaType = 0;    // 全向天线
        m_cutSequence.clear();
        // 切刀顺序: {7,7}同天线功分(校正), {1,2}~{1,7}天线1与其他天线组成基线
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
        // GN930的切刀顺序: 中间多一个{7,7}校正刀(双板卡各需要一次校正)
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {7, 7}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN930U:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 6;
        m_omni_R = 0.2;      // 比GN902/GN930的半径(0.1865m)略大
        m_antnenaType = 0;
        m_cutSequence.clear();
        m_cutSequence = {{1, 1}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN560:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 3; // 定向天线最少3对即可测向(利用了方向性信息)
        m_omni_R = 0.18;
        m_antnenaType = 1;     // 定向天线
        m_cutSequence.clear();
        // 环形天线阵列的切刀顺序:相邻天线对形成环形基线
        // {1,2},{2,3},{3,4},{4,5},{5,6},{6,7},{7,1}
        m_cutSequence = {{1, 2}, {2, 3}, {3, 4}, {4, 5}, {5, 6}, {6, 7}, {7, 1}};

        break;
    default:
        break;
    }
}


// =========================================================================
// 设置频率索引(频点唯一编码)
// 编码规则: TypeInt = sys*100 + type
// 例如: GPS L1C/A: sys=0, type=0 -> TypeInt=0
//       BDS B1I:    sys=4, type=0 -> TypeInt=400
//       BDS B2a:    sys=4, type=12 -> TypeInt=412
// 此编码用于map的key，实现按频点的快速查找
// =========================================================================
int SpoofingDoa::TypeInt(int sys, int type)
{
    return sys * 100 + type;
}



// =========================================================================
// 初始化频率map(m_F)
// 功能: 建立所有支持GNSS频点到其中心频率(Hz)的映射
// 支持六大GNSS星座的所有主要民用和部分军用频点:
//
// GPS(系统号=0):  L1 C/A(1575.42MHz), L5(1176.45MHz), L2 P(1227.6MHz),
//                L1C(1575.42MHz), L2C(1227.6MHz)
// GLONASS(系统号=1): G1(1602.0MHz), G2(1246.0MHz), G3(1277.85MHz)
// SBAS(系统号=2): L1(1575.42MHz), L5(1176.45MHz)
// Galileo(系统号=3): E1B/E1C(1575.42MHz), E6C(1278.75MHz),
//                    E5a(1176.45MHz), E5b(1207.14MHz), AltBOC(1191.795MHz)
// BDS(系统号=4): B1I(1561.098MHz), B2I(1207.14MHz), B3I(1268.52MHz),
//                B1C(1575.42MHz), B2a(1176.45MHz), B2b(1207.14MHz)
// QZSS(系统号=5): L1(1575.42MHz), L5(1176.45MHz), L2C(1227.6MHz), L1C(1575.42MHz)
//
// 频率值用于理论相位差计算: phase_diff = 2*pi*r*sin(theta)*f/c
// =========================================================================
void SpoofingDoa::initType(void)
{
    /*0 = GPS
     1= GLONASS
     2 = SBAS
     3 = Galileo
     4 = BDS
     5 = QZSS
     6 = Reserved
     7 = Other
     */

    // ---------- GPS频点 ----------
    /*
    0 = L1 C/A
    2 = L5C
    5 = L2 P
    9 = L2 P codeless
    14 = L5 Q
    16=L1C
    17 = L2_C
    */

    m_F[TypeInt(0, 0)] = 1575.42e6;   // GPS L1 C/A
    m_F[TypeInt(0, 2)] = 1176.45e6;   // GPS L5
    m_F[TypeInt(0, 5)] = 1227.6e6;    // GPS L2 P
    m_F[TypeInt(0, 9)] = 1227.6e6;    // GPS L2 P codeless (同L2频率)
    m_F[TypeInt(0, 14)] = 1176.45e6;  // GPS L5 Q (同L5频率)
    m_F[TypeInt(0, 16)] = 1575.42e6;  // GPS L1C (同L1频率)
    m_F[TypeInt(0, 17)] = 1227.6e6;   // GPS L2C (同L2频率)

    // ---------- GLONASS频点 ----------
    /* 0 = L1 C/A  G1
    1 = L2 C/A  G2
    5 = L2 P  G2P
    6= G3 */
    m_F[TypeInt(1, 0)] = 1602.0e6;    // GLONASS G1 (FDMA中心频率)
    m_F[TypeInt(1, 1)] = 1246.0e6;    // GLONASS G2
    m_F[TypeInt(1, 5)] = 1246.0e6;    // GLONASS G2P (同G2频率)
    m_F[TypeInt(1, 6)] = 1277.85e6;   // GLONASS G3

    // ---------- SBAS频点 ----------
    /* 0 = L1 C/A
    6 = L5I
    */
    m_F[TypeInt(2, 0)] = 1575.42e6;   // SBAS L1
    m_F[TypeInt(2, 6)] = 1176.45e6;   // SBAS L5

    // ---------- Galileo频点 ----------
    /* 1= E1B
    2 = E1C
    7=E6C
    12 = E5a Q
    17 = E5b Q
    20 = AltBOC Q
    */
    m_F[TypeInt(3, 1)] = 1575.42e6;   // Galileo E1B
    m_F[TypeInt(3, 2)] = 1575.42e6;   // Galileo E1C (同E1频率)
    m_F[TypeInt(3, 7)] = 1278.75e6;   // Galileo E6C
    m_F[TypeInt(3, 12)] = 1176.45e6;  // Galileo E5a
    m_F[TypeInt(3, 17)] = 1207.14e6;  // Galileo E5b
    m_F[TypeInt(3, 20)] = 1191.795e6; // Galileo AltBOC

    // ---------- BDS频点 ----------
    /*
0 = B1 C/A (B1I)
    17 = B2 C/A (B2I)
    2 = B3 C/A (B3I)
    8 = B1C
    12 = B2a
    19 = B2b
    34=B1X(hg)
    49=B3X(hg)
    47 = B3I(hg)
    */
    m_F[TypeInt(4, 0)] = 1561.098e6;  // BDS B1I (B1频点)
    m_F[TypeInt(4, 17)] = 1207.14e6;  // BDS B2I
    m_F[TypeInt(4, 2)] = 1268.52e6;   // BDS B3I
    m_F[TypeInt(4, 8)] = 1575.42e6;   // BDS B1C (与GPS L1兼容)
    m_F[TypeInt(4, 12)] = 1176.45e6;  // BDS B2a (与GPS L5兼容)
    m_F[TypeInt(4, 19)] = 1207.14e6;  // BDS B2b

    /*hg - 军码频点*/
    m_F[TypeInt(4, 34)] = 1575.42e6;  // BDS B1X (军码)
    m_F[TypeInt(4, 49)] = 1268.52e6;  // BDS B3X (军码)
    m_F[TypeInt(4, 47)] = 1268.52e6;  // BDS B3I (军码)
    // ---------- QZSS频点 ----------
    /*0 = L1 C/A
    14 = L5Q
    17 = L2C
    16 = L1C
    */
    m_F[TypeInt(5, 0)] = 1575.42e6;   // QZSS L1
    m_F[TypeInt(5, 14)] = 1176.45e6;  // QZSS L5
    m_F[TypeInt(5, 17)] = 1227.6e6;   // QZSS L2C
    m_F[TypeInt(5, 16)] = 1575.42e6;  // QZSS L1C
}


// =========================================================================
// 初始化检测门限值(对所有频点统一设置)
// @param threshold: 卫星颗数阈值(>=此数量判定为欺骗)
// @param phsThreshold: 相位差门限值(度)(转换单位后: 周)
// 注: threshold需>=0, phsThreshold需>0
//     仅当threshold != -1 时才设置,仅当phsThreshold > 0 时才设置
// =========================================================================
void SpoofingDoa::initDetectionThreshold(int threshold, double phsThreshold){

    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        int intType = it->first;

        // 各个频点对应的卫星颗数阈值
        if (-1 != threshold){
            m_Detection_Threshold[intType] = threshold;
        }

        // 各个频点对应的相位差阈值(度 -> 周)
        if (phsThreshold > 0){
            m_Detection_PhsThreshold[intType] = phsThreshold / 360.0;
        }

    }

}

// =========================================================================
// 设置连续欺骗检测记录数(跨刀连续确认刀数)
// 取值范围: 1-10, 超出范围默认设为1
// 作用: 对应循环切刀检测中跨刀连续确认所需的连续刀数
//       (getCyclicDetectionData 中 m_ConsecutiveAlarm 累计阈值，
//        对应 Python detection_lib.ALARM_CONSECUTIVE_P 连续确认)
// =========================================================================
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

// =============================================================================
// == 原 PreparationData.cpp —— GNSS 数据预处理(相位差计算)实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: PreparationData.cpp
// 功能描述: GNSS数据预处理模块(相位差计算)
// 系统角色: 将原始GNSS观测数据转换为可用于DOA计算的相位差数据。
//           这是整个欺骗测向流程的第一步，负责:
//           1. 两通道卫星数据匹配(同卫星同频点配对)
//           2. 载波相位差计算(通道1相位 - 通道2相位)
//           3. 数据格式转换(A格式->B格式的汇总聚合)
//           4. 按频点分类组织数据
//
// 数据流转: GNSSData -> SatelliteDataPhaseDiffA(细粒度) -> SatelliteDataPhaseDiffB(粗粒度)
//   A格式: 每个卫星-切刀组合一条记录
//   B格式: 每个卫星一个对象，包含所有切刀的相位差数组
// =============================================================================
// 数据预处理
// 包括计算相位差


// =========================================================================
// 计算两个通道的相位差(A格式)
// 功能: 将单帧GNSS观测数据转换为SatelliteDataPhaseDiffA列表
// 处理步骤:
//   1. 遍历第一通道的所有卫星，对每颗卫星在第二通道中寻找匹配项
//   2. 匹配条件: 同系统(sys) + 同频点(type) + 同卫星号(prn)
//   3. 如果找到匹配且两通道载噪比均>=阈值，计算相位差并保存
//   4. 如果一通道有而二通道没有，保存为单通道数据(载噪比一方为0)
// 匹配过程中的检查:
//   - 频点有效性检查(频点编码是否在m_F中)
//   - 重复卫星检查(第一通道内同卫星号只保留第一条)
//   - 载噪比阈值过滤(低于m_Snr_Threshold的忽略)
// =========================================================================
void SpoofingDoa::getSatelliteDataPhaseDiffA(const GNSSData &data, vector<SatelliteDataPhaseDiffA> &dataA)
{
    dataA.clear();
    int size1 = data.i_PortOneNum;   // 第一通道卫星数
    int size2 = data.i_PortTwoNum;   // 第二通道卫星数
    int typInt = 0;
    bool flg_E = true;
    // data2_index: 记录第二通道中尚未匹配的卫星索引
    // 用于最后补充只在第二通道存在而第一通道不存在的卫星
    vector<int> data2_index;
    data2_index.clear();
    for (int j = 0; j < size2; ++j)
    {
        data2_index.emplace_back(j);
    }
    // 遍历第一通道的所有卫星
    for (int i = 0; i < size1; ++i)
    {
        SatelliteDataPhaseDiffA tp;
        flg_E = true;
        typInt = TypeInt(data.i_PortOne[i].i_Sys, data.i_PortOne[i].i_Type);
        // 频点有效性检查: 不支持的频点跳过
        if (m_F.find(typInt) == m_F.end())
        { // 若不存在系统或频点不进行处理
            continue;
        }

        // 重复卫星检查: 避免同PRN卫星在一帧中出现两次
        int find = 0;
        for (int j = 0; j < i; ++j)
        {
            if (data.i_PortOne[j].i_Prn == data.i_PortOne[i].i_Prn && data.i_PortOne[j].i_Sys == data.i_PortOne[i].i_Sys && data.i_PortOne[j].i_Type == data.i_PortOne[i].i_Type)
            {
                ++find; // 重复卫星
                break;
            }
        }
        if (find)
        {
            continue;  // 跳过重复卫星
        }
        // 在第二通道中寻找匹配的卫星
        for (int j = 0; j < size2; ++j)
        {
            typInt = TypeInt(data.i_PortTwo[j].i_Sys, data.i_PortTwo[j].i_Type);
            if (m_F.find(typInt) == m_F.end())
            { // 若不存在系统或频点不进行处理
                flg_E = false;
                continue;
            }
            // 匹配条件: 同系统 + 同频点 + 同卫星号
            if (data.i_PortOne[i].i_Prn == data.i_PortTwo[j].i_Prn && data.i_PortOne[i].i_Sys == data.i_PortTwo[j].i_Sys && data.i_PortOne[i].i_Type == data.i_PortTwo[j].i_Type)
            {
                flg_E = false;

                // 载噪比阈值过滤
                if (data.i_PortOne[i].i_Snr >= m_Snr_Threshold && data.i_PortTwo[j].i_Snr >= m_Snr_Threshold)
                {
                    // 从待匹配列表中移除(已匹配)
                    data2_index.erase(std::remove(data2_index.begin(), data2_index.end(), j), data2_index.end());

                    tp.i_Snr1 = data.i_PortOne[i].i_Snr;
                    tp.i_Snr2 = data.i_PortTwo[j].i_Snr;
                    tp.i_Prn = data.i_PortOne[i].i_Prn;
                    tp.i_Sys = data.i_PortOne[i].i_Sys;
                    tp.i_Type = data.i_PortOne[i].i_Type;

                    // 载波相位差计算: 通道1相位 - 通道2相位
                    double phs_tp = data.i_PortOne[i].i_Phase - data.i_PortTwo[j].i_Phase;

                    // 取小数部分，归一化到[0,1)周
                    double diff = phs_tp - (long long int)phs_tp;
                    if (diff < 0)
                    {
                        diff = diff + 1;  // 确保结果为非负
                    }
                    tp.i_phase_diff = diff;
                    dataA.emplace_back(tp);
                }
                break;
            }
        }
        if (flg_E)
        {
            // 第一通道有但第二通道没有: 保存为单通道数据
            tp.i_Snr1 = data.i_PortOne[i].i_Snr;
            tp.i_Snr2 = 0;           // 第二通道无数据
            tp.i_Prn = data.i_PortOne[i].i_Prn;
            tp.i_Sys = data.i_PortOne[i].i_Sys;
            tp.i_Type = data.i_PortOne[i].i_Type;
            tp.i_phase_diff = -1;    // 无效相位差
            dataA.emplace_back(tp);
        }
    }
    // 若某频点卫星，只有2通道有，而1通道不存在
    // 补充这些只在第二通道存在的卫星(单通道数据)
    for (int j = 0; j < (int)data2_index.size(); ++j)
    {
        typInt = TypeInt(data.i_PortTwo[j].i_Sys, data.i_PortTwo[j].i_Type);
        if (m_F.find(typInt) == m_F.end())
        { // 若不存在系统或频点不进行处理
            continue;
        }
        SatelliteDataPhaseDiffA tp;
        tp.i_Snr1 = 0;           // 第一通道无数据
        tp.i_Snr2 = data.i_PortTwo[data2_index[j]].i_Snr;
        tp.i_Prn = data.i_PortTwo[data2_index[j]].i_Prn;
        tp.i_Sys = data.i_PortTwo[data2_index[j]].i_Sys;
        tp.i_Type = data.i_PortTwo[data2_index[j]].i_Type;
        tp.i_phase_diff = -1;    // 无效相位差
        dataA.emplace_back(tp);
    }
}

// =========================================================================
// 将A格式数据汇总为B格式
// 功能: 将多帧(m_OneCut_Frams帧)的A格式数据按卫星聚合为B格式
//       A格式每条记录对应一颗星一个切刀位置
//       B格式每个对象对应一颗星所有切刀位置的汇总数组
//
// 聚合过程:
//   1. 遍历所有帧的所有卫星
//   2. 同一颗卫星(同系统+同频点+同PRN)的数据合并到一个B对象中
//   3. 载噪比和相位差按切刀序号(i)存入对应的数组索引
//   4. 如果启用了m_Delete_Prn_Flag，删除任何一刀数据缺失的卫星
// =========================================================================
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
            // 查找是否已有该卫星的B对象
            for (unsigned int k = 0; k < tpBsize; k++)
            {

                if (tpA.i_Sys == tp_dataB[k].i_Sys && tpA.i_Type == tp_dataB[k].i_Type && tpA.i_Prn == tp_dataB[k].i_Prn)
                {
                    // 找到已有对象，追加这个切刀位置的数据
                    tp_dataB[k].i_Snr1[i] = tpA.i_Snr1;
                    tp_dataB[k].i_Snr2[i] = tpA.i_Snr2;
                    tp_dataB[k].i_phase_diff[i] = tpA.i_phase_diff;
                    flg = false;
                    break;
                }
            }
            if (flg)
            {
                // 新卫星，创建新的B对象
                clearSatelliteDataPhaseDiffB(tpB);
                tpB.i_Sys = tpA.i_Sys;
                tpB.i_Type = tpA.i_Type;
                tpB.i_Prn = tpA.i_Prn;
                tpB.i_Snr1[i] = tpA.i_Snr1;
                tpB.i_Snr2[i] = tpA.i_Snr2;
                tpB.i_phase_diff[i] = tpA.i_phase_diff;
                tpB.i_diffLen = size;  // 记录总切刀数
                tp_dataB.emplace_back(tpB);
            }
        }
    }
    // 数据完整性过滤
    if (1 == m_Delete_Prn_Flag)
    {
        bool tp_flg = true;
        for (int i = 0; i < (int)tp_dataB.size(); i++)
        {
            tp_flg = true;
            tpB = tp_dataB[i];
            // 检查是否所有切刀位置都有有效数据
            for (int j = 0; j < tpB.i_diffLen; j++)
            {
                if (tpB.i_Snr1[j] < 1e-6 || tpB.i_Snr2[j] < 1e-6)
                {
                    tp_flg = false;  // 存在缺失刀
                    break;
                }
            }
            if (tp_flg)
            {
                dataB.emplace_back(tpB);  // 所有刀数据完整，保留
            }
        }
    }
    else
    {
        dataB = tp_dataB;  // 不做过滤，全部保留
    }
    tp_dataB.clear();
}


// =========================================================================
// 将告警数据和卫星系统、频率id绑定(按频点分类)
// 功能: 将A格式的卫星列表按频点编码(TypeInt)分组
// 输出: map<频点编码, vector<该频点的所有卫星相位差数据>>
// 用途: 欺骗检测需要按频点分别处理，因为不同频点的频率和阈值可能不同
// =========================================================================
void SpoofingDoa::getSatelliteDataByType(const std::vector<SatelliteDataPhaseDiffA> &dataA, std::map<int, std::vector<SatelliteDataPhaseDiffA>> &dataT){
    int typeInt = 0;
    SatelliteDataPhaseDiffA tp;
    vector<SatelliteDataPhaseDiffA> tpVec;
    for (unsigned int i = 0; i < dataA.size(); i++){

        tp = dataA[i];
        typeInt = TypeInt(tp.i_Sys, tp.i_Type);

        if (dataT.find(typeInt) == dataT.end()){ // map中find()函数,表示没有找到(该频点首次出现)
            tpVec.clear();
            tpVec.push_back(tp);
            dataT[typeInt] = tpVec;
        }
        else{
            dataT[typeInt].push_back(tp);  // 追加到已有频点分组
        }

    }
}

// =========================================================================
// 清空SatelliteDataPhaseDiffB结构(重置所有字段为默认值)
// 为什么要清空: B对象在复用前需要清零，否则数组中的旧数据会污染新数据
// 注: 固定数组大小100，对应最多100个切刀位置
// =========================================================================
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

// =============================================================================
// == 原 Corrected.cpp —— 通道相位校正实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Corrected.cpp
// 功能描述: 校正数据处理模块
// 系统角色: 本文件负责通道相位校正，消除不同接收通道之间的相位不一致性。
//           由于两个接收通道(端口0和端口1)的硬件特性不完全一致，会引入
//           固有的相位偏差(通道间相位误差)，需要通过校正消除。
//
// 校正原理: 在切刀顺序中，当天线对的两个天线相同时(如{1,1})，两个通道
//           接收的是同一天线的信号(通过功分器分配)，理论上相位差应为0。
//           实际测得的相位差即为通道间的固有相位误差。
//           将后续所有相位差测量值减去该校正值，即可消除通道误差。
//
// 关键功能:
//   1. setCorrectionData - 从同天线自校准刀({1,1}=code=0)收集校正数据
//   2. calCorrectionOffset - 计算校正偏移(对应 Python compute_calibration:
//      非 GLONASS 按频点圆形均值统一偏移, GLONASS 逐卫星偏移)
//   3. getCorrectedGnssData - 对所有相位差数据应用校正
//   4. calCorrecteData - 对单个卫星数据减去校正偏移(对应 Python offset_fn)
// =============================================================================
// 校正数据的相关操作

// =========================================================================
// 设置校正数据(主入口)
// 功能: 从同天线自校准刀({1,1} = Python code=0)提取校正信息，
//       计算校正偏移后对所有输入数据应用校正。
// =========================================================================
void SpoofingDoa::setCorrectionData(const vector<vector<SatelliteDataPhaseDiffA>> dataA)
{
    // 数据长度必须与切刀序列一致
    if (dataA.size() != m_cutSequence.size())
    {
        return;
    }

    // 收集同天线自校准刀的数据(天线对相同的刀，如 {1,1})
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

// =========================================================================
// 由同天线校正刀数据计算校正偏移 —— 对应 Python detection_lib.py compute_calibration
// 流程:
//   1. 数据筛选(对应 Python compute_calibration):
//      - 均匀分布:     有效采样数 >= min(校正刀内帧数, MIN_STABLE_SAMPLES)
//        (每刀多帧的稳定性过滤与覆盖判定已由上游 getSmoothData/calSmoothData 完成，
//         不满足的卫星载噪比已置 0，此处按 <1e-3 跳过；
//         不再要求信号从该刀第一帧(第一秒)就存在 —— Python 亦不要求);
//      - 相位差稳定:   最小覆盖弧 < STABILITY_RANGE_DEG(15°);
//   2. 偏移组合:
//      - 非 GLONASS: 每个(系统,频点)取各稳定卫星偏移的圆形均值, 存入 prn=-1;
//      - GLONASS(FDMA): 每颗卫星逐星偏移, 存入对应 prn;
//   3. 仅当本轮 {1,1} 校正刀能算出有效偏移时才重建 m_CorrectionData(与 Python 每周期
//      重算 cal_rx/cal_glo 一致); 无 {1,1} 刀或算不出时保留上一次校正, 沿用其校正。
// =========================================================================
void SpoofingDoa::calCorrectionOffset(const vector<vector<SatelliteDataPhaseDiffA>> &calCuts)
{
    // samples: typeInt -> prn -> [相位差(度), ...]
    map<int, map<int, vector<double>>> samples;  // 相位差样本(度)
    map<int, map<int, double>> sumSnr;           // 载噪比累加(仅用于日志)
    int nCalCuts = 0;

    for (unsigned int c = 0; c < calCuts.size(); ++c)
    {
        ++nCalCuts;
        for (const auto &sat : calCuts[c])
        {
            // 载噪比有效性检查(数据完整性判断，非质量门限)
            // 不满足「从该刀第一帧就存在」的卫星已在上游被置 0，此处自动跳过
            if (sat.i_Snr1 < 1e-3 || sat.i_Snr2 < 1e-3)
            {
                continue;
            }
            int typeInt = TypeInt(sat.i_Sys, sat.i_Type);
            int prn = sat.i_Prn;
            samples[typeInt][prn].emplace_back(sat.i_phase_diff * 360.0);  // 周->度
            sumSnr[typeInt][prn] += (sat.i_Snr1 + sat.i_Snr2) / 2.0;
        }
    }

    // 本轮没有 {1,1} 校正刀数据: 不重建, 沿用上一次校正继续校正
    if (nCalCuts <= 0)
    {
        return;
    }
    int required = (nCalCuts < MIN_STABLE_SAMPLES) ? nCalCuts : MIN_STABLE_SAMPLES;

    // 先写入临时容器, 仅当本轮 {1,1} 刀确实算出有效新校正时才整体替换 m_CorrectionData;
    // 否则(无有效星/全部不稳定)不清空不覆盖, 保留上一次校正值。
    map<int, map<int, SatelliteDataPhaseDiffA>> newCorrection;
    map<int, vector<double>> freqVals;  // 非 GLONASS: typeInt -> [稳定卫星偏移(度)...]

    for (auto &tkv : samples)
    {
        int typeInt = tkv.first;
        int sys = typeInt / 100;
        for (auto &skv : tkv.second)
        {
            int prn = skv.first;
            const vector<double> &samp = skv.second;

            // 1. 相位差稳定(最小覆盖弧 < STABILITY_RANGE_DEG=15°)
            if (circularSpanDeg(samp) >= STABILITY_RANGE_DEG)
            {
                continue;
            }
            // 2. 均匀分布(有效采样覆盖足够帧数)
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
                cyc += 1.0;  // 归一化到 [0,1) 周
            }
            tp.i_phase_diff = cyc;
            tp.i_Snr1 = meanSnr;
            tp.i_Snr2 = meanSnr;

            if (sys == 1)
            {
                // GLONASS(FDMA): 逐卫星偏移
                newCorrection[typeInt][prn] = tp;
            }
            else
            {
                // 非 GLONASS: 收集用于频点级圆形均值
                freqVals[typeInt].emplace_back(meanDeg);
            }
        }
    }

    // 非 GLONASS: 每(系统,频点)统一偏移 = 稳定卫星偏移的圆形均值
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

    // {1,1} 校正刀没能算出任何稳定有效偏移: 保留上一次校正(不清空不覆盖), 沿用其校正。
    if (newCorrection.empty())
    {
        return;
    }

    // 整体替换为本轮新校正(与 Python 每周期重算 cal_rx/cal_glo 一致)。
    m_CorrectionData.swap(newCorrection);

    // 日志输出校正数据
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

// =========================================================================
// 对所有输入数据进行相位校正
// 遍历所有切刀的所有卫星数据，分别调用calCorrecteData进行校正
// =========================================================================
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

// =========================================================================
// 对单个卫星数据进行相位差校正 —— 对应 Python offset_fn
// 校正策略(与 Python compute_calibration / offset_fn 语义一致):
//   - GLONASS(FDMA): 逐卫星偏移(该卫星在校正刀有偏移则减之, 否则偏移0不校正);
//   - 非 GLONASS:    统一频点偏移(prn=-1 综合值, 无则偏移0不校正);
//   无该频点校正数据时: 一律偏移 0（保持原值），对应 Python offset_fn 返回 0；
//   注: 原「测向模式无该频点校正则载噪比置 0 不进入测向」已移除，改为与 Python 一致
//       (仅当某 (系统,频点) 完全无稳定校正星时才按原相位差继续，Python 同样如此)。
// 公式: 校正后相位差 = 原始相位差 - 校正偏移
// =========================================================================
void SpoofingDoa::calCorrecteData(SatelliteDataPhaseDiffA &dataA)
{
    int typeInt = TypeInt(dataA.i_Sys, dataA.i_Type);
    int prn = dataA.i_Prn;

    // 载噪比无效
    if (dataA.i_Snr1 < 1e-3 || dataA.i_Snr2 < 1e-3)
    {
        return;
    }

    auto it = m_CorrectionData.find(typeInt);
    if (it == m_CorrectionData.end())
    {
        return;  // 不存在该频点校正数据：偏移0，保持原值（对齐 Python offset_fn）
    }

    const map<int, SatelliteDataPhaseDiffA> &tp_prnData = it->second;
    const SatelliteDataPhaseDiffA *off = nullptr;
    if (dataA.i_Sys == 1)
    {
        // GLONASS(FDMA): 逐卫星偏移; 无则该星偏移0不校正(对应 Python offset_fn 返回 0)
        auto ps = tp_prnData.find(prn);
        if (ps != tp_prnData.end())
        {
            off = &(ps->second);
        }
    }
    else
    {
        // 非 GLONASS: 统一频点偏移(prn=-1 综合值)
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

// =============================================================================
// == 原 Interf.cpp —— 相关干涉仪测向实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Interf.cpp
// 功能描述: 相关干涉仪测向模块
// 系统角色: 实现基于相关干涉仪(Correlative Interferometer)的DOA算法。
//           相关干涉仪是欺骗测向系统的主要DOA方法，其原理是:
//           1. 预先计算360个角度上的理论相位差(形成"指纹库")
//           2. 将实测相位差与每个角度的理论相位差进行相关匹配
//           3. 相关性最高的角度即为估计的到达角
//
// DOA算法选择(通过m_Doa_Arithmetic配置):
//   1=相关干涉仪(使用理论相位差模板) - 基于阵列几何参数计算
//   2=幅相法(使用仿真阵列流型) - 基于电磁仿真数据
//   3=相关干涉仪+阵列仿真数据(使用仿真相位差) - 基于仿真提取的相位差
//
// 关键功能:
//   1. getResultInterferDoa - 干涉仪测向调度(全向/定向天线分派)
//   2. initTheory - 初始化理论相位差模板(基于阵列几何)
//   3. initTheoryBySimulatePhase - 初始化仿真相位差模板(基于仿真数据)
// =============================================================================
// 相关干涉仪测向

// =========================================================================
// 相关干涉仪测向调度函数
// 根据天线类型调用对应的数据处理函数:
//   全向天线(m_antnenaType=0): 调用setInterferInfoDataOmni
//   定向天线(m_antnenaType=1): 调用setInterferInfoDataDirect
// 构建好InterferInfo数据后，统一调用calAngle进行测向计算
// =========================================================================
void SpoofingDoa::getResultInterferDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    string nowT = getNowTime();
    std::map<int, std::map<int, InterferInfo>> inferInfoData;
    if (0 == m_antnenaType)
    {
        PublicSpace::Log("set Interfer Info Data Omni   %s\n", nowT.c_str());
        setInterferInfoDataOmni(dataB, inferInfoData);    // 全向天线
    }
    else
    {
        nowT = getNowTime();
        PublicSpace::Log("set Interfer Info Data Direct   %s\n", nowT.c_str());
        setInterferInfoDataDirect(dataB, inferInfoData);  // 定向天线
    }
    nowT = getNowTime();
    PublicSpace::Log("cal angle    %s\n", nowT.c_str());
    calAngle(inferInfoData);  // 执行相关干涉仪角度计算
}


// =========================================================================
// 初始化理论相位差模板(基于阵列几何参数)
// 原理: 对于均匀圆阵(UCA), 天线i与天线j之间的理论相位差为:
//       phi = 2*pi*r/lambda * (sin(theta)*cos(alpha_i-phi0) - sin(theta)*cos(alpha_j-phi0))
//       其中: r=阵列半径, lambda=波长=c/f, theta=仰角, alpha=天线方位角
//
// 流程:
//   1. 遍历所有工作频点(m_F)
//   2. 获取该频点的阵列半径(m_R)
//   3. 调用ArithmeticDoa::calPhaseTheory计算360个角度的理论相位差
//   4. 可选: 如启用虚拟阵列(m_Virtual_Flag=1)，使用getVirtualTheory扩展
//   5. 结果存入m_Theory[typeInt]
// =========================================================================
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
        f = it->second;           // 频率(Hz)
        r = m_R[typeInt];         // 阵列半径(米)
        ArithmeticDoa::calPhaseTheory(f, r, m_AntennaNum, tp_theory); // 计算理论相位差

        theory = tp_theory;
        // 虚拟阵列扩展: 通过数学变换构造虚拟天线对，增加相位差数据量
        if (1 == m_Virtual_Flag)
        {
            ArithmeticDoa::getVirtualTheory(m_AntennaNum, tp_theory, m_Virtual_Multiple, theory);
        }

        m_Theory[typeInt] = theory; // 保存各个频点的理论相位差模板
    }
}


// =========================================================================
// 初始化基于阵列仿真相位差的理论模板
// 与initTheory的区别: 不使用几何公式计算理论相位，而是从电磁仿真数据文件中
// 读取预先仿真好的相位差数据。仿真数据比理论公式更精确，能反映实际天线耦合、
// 多径反射等非理想因素。
//
// 流程:
//   1. 从仿真数据文件加载各频率的相位差模板
//   2. 对于每个工作频点，在仿真数据中寻找频率最接近的模板
//   3. 可选: 如启用虚拟阵列，使用getVirtualTheory扩展
//   4. 如果没有任何仿真数据可用，自动切换到理论相位差模板(initTheory)
//   5. 结果存入m_Theory[typeInt]
// =========================================================================
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
        // 从仿真数据文件读取该频率的相位差模板
        flg = ArithmeticDoa::getSimulatePhase(m_Simulate_Data_file, f, tp_theory);

        if (-1 == flg){
            continue;  // 该频率无仿真数据
        }
        erse_index.emplace_back(f);
        tp_theory3 = tp_theory;

        // 虚拟阵列扩展
        if (1 == m_Virtual_Flag){
            ArithmeticDoa::getVirtualTheory(m_AntennaNum, tp_theory, m_Virtual_Multiple, tp_theory3);
        }

        tp_theory2[f] = tp_theory3;
    }
    // 如果没有任何仿真数据可用，自动切换到理论相位差模板
    if (0 == tp_theory2.size()){
        m_Doa_Arithmetic = 1; // 若没有阵列仿真数据，则自动转成使用相关干涉仪算法
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

    // 为每个工作频点匹配最接近的仿真频率
    for (auto it = m_F.begin(); it != m_F.end(); ++it){
        typeInt = it->first;
        f = it->second;  // 工作频点实际频率
        tp_f_min = 99999e8;

        for (int i = 0; i < (int)m_All_Simulate_data_Fre.size(); i++){
            tp_f2 = m_All_Simulate_data_Fre[i];
            tp_diff = abs(tp_f2 - f);  // 频率差

            if (tp_f_min > tp_diff){
                tp_f_min = tp_diff;
                f2 = tp_f2;  // 记录最接近的仿真频率
            }

        }

        m_Theory[typeInt] = tp_theory2[f2]; // 保存各个频点的理论相位差模板(使用最接近频率的仿真数据)
    }

}

// =============================================================================
// == 原 AmpPhase.cpp —— 幅相法测向实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: AmpPhase.cpp
// 功能描述: 幅相法(Amplitude-Phase)测向模块
// 系统角色: 实现基于仿真阵列流型的幅相法DOA。与相关干涉仪不同，
//           幅相法不仅利用相位差信息，还利用幅度差(载噪比)信息进行测向。
//           仿真阵列流型(SimulateA)通过电磁仿真软件(如HFSS/CST)生成，
//           包含每个角度上的复数导向矢量(幅度和相位响应)。
//
// 核心算法: 将实测幅度和相位与仿真阵列流型进行匹配，计算相关系数，
//           相关性最高的角度即为估计的到达角。
//
// 关键功能:
//   1. getResultAmpPhaseDoa - 幅相法测向主函数(对各卫星逐星计算)
//   2. initSimulateA - 初始化仿真阵列流型模板
// =============================================================================
// 幅相法测向

// =========================================================================
// 幅相法测向主函数
// 算法流程(对每颗卫星):
//   1. 获取该频点的仿真阵列流型(m_SimulateA)
//   2. 构建幅度向量(ampsnr)和相位差向量(phase_diff)
//   3. 填充InterferInfo结构(包含幅度和相位差)
//   4. 调用ArithmeticDoa::calAmpPhase进行幅相法匹配
//   5. 记录匹配结果(角度、质量)到m_AngleResultData
//
// 幅度处理: 载噪比(SNR)通过10^(SNR/20)转换为线性幅度
// 相位处理: 相位差(周)乘以2π转换为弧度
// 参考通道(第0刀): 幅度取平均载噪比，相位差设为0
// =========================================================================
void SpoofingDoa::getResultAmpPhaseDoa(vector<SatelliteDataPhaseDiffB> dataB)
{
    // 清空上一轮的结果
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
    data.i_End = 359;  // 全方向搜索
    map<int, vector<double>> tp_peseudo;

    // 设置天线对序列(与切刀顺序一致)
    for (int i = 0; i < num1; i++)
    {
        data.i_AntennaSq[i][0] = m_cutSequence[i][0];
        data.i_AntennaSq[i][1] = m_cutSequence[i][1];
    }
    data.i_Phase_Len = num1;

    // 对每颗卫星独立进行幅相法测向
    for (int i = 0; i < size; i++)
    {
        tp_alarms.clear();
        tpB = dataB[i];
        typeInt = TypeInt(tpB.i_Sys, tpB.i_Type);
        tp_SimulateA.clear();
        tp_SimulateA = m_SimulateA[typeInt];  // 获取该频点的仿真阵列流型
        double sum_snr = 0.0;
        int num2 = tpB.i_diffLen;
        int prn = dataB[i].i_Prn;
        tp_peseudo.clear();

        // 构建幅度向量和相位差向量(从第1刀开始，第0刀为参考)
        for (int j = 1; j < num2; j++)
        {
            sum_snr = sum_snr + tpB.i_Snr1[j];
            // 载噪比(dB)转换为线性幅度: 10^(SNR/20)
            amp_snr[j] = pow(10, (tpB.i_Snr2[j] / 20));
            // 相位差(周)转换为弧度: 相位差 * 2π
            phase_diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
            data.i_Amp[j] = pow(10, (tpB.i_Snr2[j] / 20));
            data.i_Phase_Diff[j] = (tpB.i_phase_diff[j]) * 2 * 4 * atan(1);
        }
        // 参考通道(第0刀): 相位差=0，幅度为平均载噪比
        tmp2 = sum_snr / (tpB.i_diffLen - 1);
        tmp1 = tmp2;
        amp_snr[0] = pow(10, (tmp1 / 20));
        phase_diff[0] = 0;
        data.i_Amp[0] = pow(10, (tmp1 / 20));
        data.i_Phase_Diff[0] = 0;

        double angle;
        double quality;
        diff.clear();

        // 调用底层幅相法匹配算法
        ArithmeticDoa::calAmpPhase(tp_SimulateA, data, angle, quality, diff);

        // 记录最大载噪比
        m_Max_Snr[typeInt][tpB.i_Prn] = tmp2;

        // 构建告警/测向结果
        tp_alarm.i_Prn = tpB.i_Prn;
        tp_alarm.i_Angle = (int)angle;
        tp_alarm.i_Quality = quality;

        // 按频点汇总结果到m_AngleResultData
        if (m_AngleResultData.find(typeInt) == m_AngleResultData.end())
        {
            tp_alarms.emplace_back(tp_alarm);
            if (1 == m_PseudoSpectrum_Flag)
            {
                tp_peseudo[prn] = diff;  // 保存伪谱值
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


// =========================================================================
// 初始化幅相法理论模版(仿真阵列流型)
// 流程:
//   1. 从仿真数据文件加载各频率的阵列流型数据
//   2. 对于每个工作的GNSS频点，在仿真数据中寻找频率最接近的模板
//      注: 仿真数据可能只有有限几个频率点，需要匹配最近的频率
//   3. 如果没有任何仿真数据可用，自动切换到相关干涉仪算法(m_Doa_Arithmetic=1)
// 结果: 存放在m_SimulateA中，按频点编码索引
// =========================================================================
void SpoofingDoa::initSimulateA(void){
    int typeInt = 0;
    double f = 0;
    std::map<double, std::vector<std::vector<complex<double>>>> tp_SimulateA;
    tp_SimulateA.clear();
    vector<vector<complex<double>>> simulateA;
    int num = m_All_Simulate_data_Fre.size();
    vector<double> erse_index;
    erse_index.clear();

    // 加载所有仿真频率的阵列流型
    for (int i = 0; i < num; i++){
        f = m_All_Simulate_data_Fre[i];
        simulateA.clear();
        ArithmeticDoa::calAmpPhaseSimulateA(m_Simulate_Data_file, f, simulateA);  // 从文件读取
        if (0 == simulateA.size()){
            continue;  // 该频率无仿真数据
        }
        erse_index.emplace_back(f);
        tp_SimulateA[f] = simulateA;
    }

    // 如果没有加载到任何仿真数据，自动切换到相关干涉仪算法
    if (0 == tp_SimulateA.size()){
        m_Doa_Arithmetic = 1; // 若没有阵列仿真数据，则自动转成使用相关干涉仪算法
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

    // 为每个工作频点匹配最接近的仿真频率
    for (auto it = m_F.begin(); it != m_F.end(); ++it){

        typeInt = it->first;
        f = it->second;  // 工作频点实际频率
        tp_f_min = 999999e8;
        for (int i = 0; i < (int)m_All_Simulate_data_Fre.size(); i++){

            tp_f2 = m_All_Simulate_data_Fre[i];
            tp_diff = abs(tp_f2 - f);  // 频率差
            if (tp_f_min > tp_diff){
                tp_f_min = tp_diff;
                f2 = tp_f2;  // 记录最接近的仿真频率
            }

        }

        PublicSpace::Log("Fre = %.1f,f= %.1f\n", f, f2);
        m_SimulateA[typeInt] = tp_SimulateA[f2]; // 保存各个频点的仿真阵列流型(使用最接近频率的模板)
    }
}

// =============================================================================
// == 原 Omni.cpp —— 全向天线测向数据实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Omni.cpp
// 功能描述: 全向天线测向数据处理模块
// 系统角色: 处理全向天线(omni-directional antenna)模式下的干涉仪测向准备。
//           全向天线在所有方向上具有基本一致的增益，信号到达角范围为0-359度。
//           与定向天线不同，全向天线不需要根据信号强度限制搜索扇区。
//
// 关键功能:
//   setInterferInfoDataOmni - 构建全向天线模式下的干涉仪输入数据
//                             所有天线对参与测向，搜索范围为全向(0-359度)
// =============================================================================
// 全向天线测向的相关处理

// =========================================================================
// 全向天线: 构建干涉仪测向输入数据
// 与定向天线(Directed.cpp)的区别:
//   1. 全向天线搜索范围为0-359度(全向)，定向天线限制搜索扇区
//   2. 全向天线不需要根据载噪比选择天线扇区
//   3. 全向天线所有方向增益一致，不会因方向不同导致信号强度差异
// 流程:
//   1. 遍历所有卫星的相位差数据
//   2. 调用calAngleUseAntenna验证和构建InterferInfo结构
//   3. 设置搜索范围为全向(0-359度)
//   4. 将构建好的InterferInfo存入inferInfoData(map<频点编码, map<卫星号, InterferInfo>>)
// =========================================================================
void SpoofingDoa::setInterferInfoDataOmni(const std::vector<SatelliteDataPhaseDiffB> &dataB, std::map<int, std::map<int, InterferInfo>> &inferInfoData)
{
    int size = (int)dataB.size();
    int typInt = 0;
    int prn = 0;
    SatelliteDataPhaseDiffB tp1;
    int startAngle = 0;
    int endAngle = 359;  // 全向天线: 全方向搜索
    for (int i = 0; i < size; i++)
    {
        tp1 = dataB[i];
        typInt = TypeInt(tp1.i_Sys, tp1.i_Type);
        prn = tp1.i_Prn;

        InterferInfo tp2;
        int flg;
        // 根据天线关系和相位差构建干涉仪输入数据
        calAngleUseAntenna(tp1, tp2, flg);
        if (0 == flg)
        {
            continue;  // 有效天线对数量不足，跳过此卫星
        }
        // 设置搜索范围(全向天线无方向限制)
        tp2.i_Start = startAngle;
        tp2.i_End = endAngle;
        // 组织到按频点+卫星号索引的map中
        inferInfoData[typInt][prn] = tp2;
    }
    Log("set Interfer Info Data sucess....\n");
}

// =============================================================================
// == 原 Directed.cpp —— 定向天线测向数据实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Directed.cpp
// 功能描述: 定向天线测向数据处理模块
// 系统角色: 处理定向天线(directional antenna)模式下的干涉仪测向准备。
//           定向天线在不同方向具有不同增益，因此可以利用载噪比信息
//           缩小到达角搜索范围，提高测向效率和精度。
//
// 定向天线测向特点:
//   1. 每个天线阵元有独立的扇区覆盖范围(如7阵元各覆盖约51度)
//   2. 通过比较各天线的载噪比可确定信号大致的到达扇区
//   3. 搜索范围可缩小到特定扇区(startAngle ~ endAngle)，减少计算量
//   4. 与全向天线测向的主要区别在于搜索范围约束
//
// 关键功能:
//   1. setInterferInfoDataDirect - 定向天线模式的干涉仪数据构建
//   2. getUseAntennaBySnr - 利用载噪比选择测向用的天线和搜索扇区
// =============================================================================
// 定向天线的相关操作

// =========================================================================
// 定向天线: 构建干涉仪测向输入数据
// 与全向天线的区别: 需要根据天线方向特性确定搜索范围(startAngle, endAngle)
// 两种天线选择模式:
//   模式1 (m_getUseAntennaBySnr_Flag=1): 利用载噪比智能选择天线和扇区
//   模式2 (m_getUseAntennaBySnr_Flag=0): 按固定扇区划分
//                                       每个天线覆盖 360/m_AntennaNum 度的扇区
//                                       基于校正数据所在的天线确定主天线和扇区
// =========================================================================
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
            // 模式1: 由载噪比确定用于测向的天线和搜索扇区
            getUseAntennaBySnr(tp1, startAngle, endAngle);
        }
        else
        {
            // 模式2: 按固定扇区划分(每个天线覆盖360/7≈51度)
            int per = (int)360 / m_AntennaNum;  // 每个天线的扇区角度

            int len = m_cutSequence.size();
            int doaNum = 0;
            vector<int> correction_index; // 校正数据的切刀序号(天线对相同)
            vector<int> doa_index;        // 测向数据的切刀序号(天线对不同)

            // 分离校正刀和测向刀
            for (int k1 = 0; k1 < len; k1++)
            {

                if (m_cutSequence[k1][0] == m_cutSequence[k1][1])
                {
                    // 同天线自检刀(用于校正数据)
                    correction_index.emplace_back(k1);
                    tp1.i_Snr1[k1] = 0.0;
                    tp1.i_Snr2[k1] = 0.0;
                }
                else
                {
                    // 不同天线对(用于测向数据)
                    doa_index.emplace_back(k1);
                    if (tp1.i_Snr1[k1] < 1e-6 || tp1.i_Snr2[k1] < 1e-6)
                    {
                        // 某个天线对数据无效时，相邻天线对也可能受影响
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

            // 确定主天线(校正数据对应的天线)
            int mainAnten = m_cutSequence[correction_index[0]][0];
            // 计算扇区范围
            startAngle = (mainAnten - 2) * per;
            endAngle = (mainAnten)*per;
            // 比较首尾天线对的载噪比，确定信号来向在扇区的前半部还是后半部
            int doaNum2 = (int)doa_index.size();
            double tp_snr1 = tp1.i_Snr2[doa_index[0]];
            double tp_snr2 = tp1.i_Snr2[doa_index[doaNum2 - 1]];
            if (tp_snr1 < tp_snr2)
            {
                // 尾部载噪比更大，信号来自扇区后半部
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
                // 头部载噪比更大，信号来自扇区前半部
                if (doaNum > m_Doa_Cut_Num)
                {
                    tp1.i_Snr1[doa_index[doaNum - 1]] = 0.0;
                    tp1.i_Snr2[doa_index[doaNum - 1]] = 0.0;
                }
                startAngle = startAngle - 30;
                endAngle = endAngle - 30;
            }
        }

        // 构建InterferInfo并进行搜索范围约束
        InterferInfo tp2;
        int flg;
        calAngleUseAntenna(tp1, tp2, flg);
        if (0 == flg)
        {
            continue;
        }
        // 设置定向天线的搜索范围约束
        tp2.i_Start = startAngle;
        tp2.i_End = endAngle;
        inferInfoData[typInt][prn] = tp2;
    }
}

// =========================================================================
// 利用载噪比确定用于测向的天线序号和搜索扇区
// 核心思想: 定向天线各阵元在不同方向具有不同的增益，
//           载噪比最高的天线说明信号来自该天线的覆盖方向。
// 算法流程:
//   1. 分离校正刀(天线对相同)和测向刀(天线对不同)
//   2. 计算各天线对两个通道的平均载噪比
//   3. 按载噪比从高到低排序
//   4. 从载噪比最高的天线开始，向两侧扩展连续的相邻天线
//   5. 直到收集到足够的连续天线(>=m_Doa_Cut_Num)
//   6. 调用ArithmeticDoa::calAngleSerchRange确定搜索扇区范围
// =========================================================================
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

    vector<int> correction_index; // 校正数据的切刀序号(天线对相同)
    vector<int> doa_index;        // 测向数据的切刀序号(天线对不同)
    int doaNum = 0;
    for (int i = 0; i < len; i++)
    {

        if (m_cutSequence[i][0] == m_cutSequence[i][1])
        {
            correction_index.emplace_back(i);  // 校正刀
        }
        else
        {
            doa_index.emplace_back(i);  // 测向刀
            tp_Snr[doaNum] = (dataB.i_Snr1[i] + dataB.i_Snr2[i]) / 2;  // 两通道平均载噪比
            tp_Snr1[doaNum] = dataB.i_Snr1[i];
            tp_Snr2[doaNum] = dataB.i_Snr2[i];
            index.emplace_back(doaNum);
            doaNum++;
        }
        // 先清空所有天线对的载噪比，后续只恢复选中的
        dataB.i_Snr1[i] = 0.0;
        dataB.i_Snr2[i] = 0.0;
    }
    // 按载噪比从高到低排序(降序)
    std::sort(index.begin(), index.end(), [&](int i, int j)
              { return tp_Snr[i] > tp_Snr[j]; });

    // 从载噪比最高的天线(index[0])开始扩展
    int max_index = index[0];  // 载噪比最高的天线对索引
    int last_index = 0;
    int next_index = 0;
    vector<int> tp_index;
    tp_index.emplace_back(max_index);
    int size = 1;
    int flg = 1;
    int num = 0;

    // 检查天线对是否构成闭合环路(首尾天线对的编号连续)
    // 例如: {1,2},{2,3},{3,4},...,{7,1} 构成闭合环
    if (m_cutSequence[doa_index[0]][0] == m_cutSequence[doa_index[doaNum - 1]][1] || m_cutSequence[doa_index[0]][1] == m_cutSequence[doa_index[doaNum - 1]][0])
    {

        // 从最高载噪比的天线向两侧扩展，直到收集到足够的连续天线
        while (size < m_Doa_Cut_Num && flg > 0 && num < 20)
        {

            last_index = (tp_index[0] - 1 + doaNum) % doaNum;      // 向左侧扩展
            next_index = (tp_index[size - 1] + 1) % doaNum;        // 向右侧扩展
            flg = 0;
            for (int i = 1; i < doaNum; i++)
            {
                if (tp_Snr1[index[i]] < 1e-6 || tp_Snr2[index[i]] < 1e-6)
                {
                    break;  // 载噪比无效
                }
                if (index[i] == last_index)
                {
                    tp_index.insert(tp_index.begin() + 0, index[i]);  // 插入到左侧
                    flg = 1;
                    break;
                }
                if (index[i] == next_index)
                {
                    tp_index.emplace_back(index[i]);  // 追加到右侧
                    flg = 1;
                    break;
                }
            }
            size = (int)tp_index.size();
            num++;
        }
        // 重新组织选中的天线对数据
        vector<int>().swap(index);
        index.resize(size);
        double maxSnr2 = -99;
        for (int i = 0; i < size; i++)
        {
            int tp_index2 = doa_index[tp_index[i]];
            // 恢复选中的天线对的载噪比(其他保持为0)
            dataB.i_Snr1[tp_index2] = tp_Snr1[tp_index[i]];
            dataB.i_Snr2[tp_index2] = tp_Snr2[tp_index[i]];
            index[i] = tp_index2;
            if (maxSnr2 < tp_Snr[tp_index[i]])
            {
                maxSnr2 = tp_Snr[tp_index[i]];
                max_index = tp_index2;
            }
        }
        // 根据选中的天线确定搜索扇区范围
        ArithmeticDoa::calAngleSerchRange(index, m_cutSequence, m_AntennaNum, max_index, startAngle, endAngle);
    }
}

// =============================================================================
// == 原 Alarm.cpp —— 欺骗检测与告警实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: Alarm.cpp
// 功能描述: 欺骗信号检测与告警模块
// 系统角色: 本文件实现了欺骗信号检测方法，是欺骗测向系统的检测前端:
//           1. 相位差法检测(calAlarmByPhaseDiff，载噪比≥35dB 作为数据质量门限)
// 检测原理: 欺骗信号由同一干扰源发射，因此多颗卫星的相位差相近(来自同一方向)
// =============================================================================

// =========================================================================
// 欺骗检测参数（对应 Python detection_lib.py）
// =========================================================================
const double SpoofingDoa::CNR_MIN_DB = 35.0;           // 载噪比质量门限：两端口都需 ≥35dB
const double SpoofingDoa::STABILITY_RANGE_DEG = 15.0;  // 稳定性阈值：最小覆盖弧 < 15° 判为稳定
                                                       // （对齐 Python detection_lib.STABILITY_RANGE_DEG=15，2026-09 版）
const int SpoofingDoa::MIN_STABLE_SAMPLES = 3;         // 每刀至少需要的有效采样帧数（对齐 Python MIN_STABLE_SAMPLES=3）

// =========================================================================
// 归一化角度到 [-180°, 180°)，对应 Python detection_lib.normalize_angle_180
// =========================================================================
double SpoofingDoa::normalizeAngle180(double deg)
{
    double r = fmod(deg + 180.0, 360.0);
    if (r < 0)
    {
        r += 360.0;
    }
    return r - 180.0;
}

// =========================================================================
// 角度圆形均值(度)，对应 Python circular_mean
// =========================================================================
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

// =========================================================================
// 最小覆盖弧跨度(度)，对应 Python circular_span
// =========================================================================
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

// =========================================================================
// 折叠到 [0°, 180°)，消除跳半周(±180°)歧义，对应 Python fold_half_cycle
// =========================================================================
double SpoofingDoa::foldHalfCycle(double deg)
{
    double r = fmod(normalizeAngle180(deg), 180.0);
    if (r < 0)
    {
        r += 180.0;
    }
    return r;
}

// =========================================================================
// 180° 半周圆上的最小覆盖弧跨度(度)，对应 Python circular_span_180
// =========================================================================
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

// =========================================================================
// 利用相位差计算是否告警(相位差法欺骗检测)
// 核心思想: 真实卫星来自不同方向，相位差各不相同；
//           欺骗信号来自同一干扰源，相位差应当相近(差异在阈值内)
// 检测方法对应 Python detection_lib.py 的 cluster_satellites：
//   1. 数据筛选：两端口载噪比均 ≥ 35dB 的观测才参与检测（载噪比质量门限）；
//   2. 相位差由「周」转「度」并归一化到 [-180°, 180°)，消除 0/360 边界歧义；
//   3. 按相位差排序后，用滑动窗口找「跨度 < 相位差阈值」的最大卫星集合
//      （比原「以某星为参考数邻近星」更准确：聚类跨度直接受阈值约束）；
//   4. 最大聚类卫星数超过检测阈值(严格大于)则判为欺骗，与 Python 的 cnt > count_thr 一致。
// 输出:
//   alarmSatelliteData: 判定为欺骗的卫星相位差数据列表
//   alarm: 1=检测到欺骗, 0=未检测到
// =========================================================================
void SpoofingDoa::calAlarmByPhaseDiff(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm) // 利用相位差计算是否告警
{
    alarmSatelliteData.clear();
    alarm = 0;

    double phsThresholdDeg = m_Detection_PhsThreshold[typeInt] * 360.0;  // 相位差阈值：周 -> 度

    // 1. 数据筛选 + 相位差归一化：valid 存 (校正相位差[度], dataA 索引)
    vector<pair<double, int>> valid;
    for (unsigned int i = 0; i < dataA.size(); ++i)
    {
        if (dataA[i].i_Snr1 < CNR_MIN_DB || dataA[i].i_Snr2 < CNR_MIN_DB)
        {
            continue;  // 载噪比不达标，不使用该观测
        }
        double ph = normalizeAngle180(dataA[i].i_phase_diff * 360.0);  // [0,360) -> [-180,180)
        valid.emplace_back(ph, (int)i);
    }

    int n = (int)valid.size();
    if (n < 2)
    {
        return;  // 少于 2 颗有效卫星，无法形成聚集
    }

    // 2. 排序 + 复制一份(整体 +360°) 解决簇跨越 0/360 边界的问题
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

    // 3. 滑动窗口：窗口内相位差跨度 < 阈值，找最大聚类
    int bestCount = 0;
    vector<int> bestIds;
    int left = 0;
    for (int right = 0; right < (int)ext.size(); ++right)
    {
        while (ext[right].first - ext[left].first >= phsThresholdDeg)
        {
            ++left;
        }
        // 窗口最多包含 n 颗星，防止同一颗星(复制份)被重复计数
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

    // 4. 判定：最大聚类卫星数超过阈值（严格大于，与 Python 的 cnt > count_thr 一致）
    if (bestCount > m_Detection_Threshold[typeInt])
    {
        alarm = 1;
        for (int idx : bestIds)
        {
            alarmSatelliteData.emplace_back(dataA[idx]);
        }
    }
}


// =============================================================================
// == 原 SpectrumDesity.cpp —— 伪谱积分实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: SpectrumDesity.cpp
// 功能描述: 伪谱积分(Pseudo-spectrum Integration)模块
// 系统角色: 对多帧测向结果进行累积和加权，提高测向稳定性和精度。
//           伪谱积分是一种多帧融合技术，类似于图像处理中的帧累积。
//
// 工作原理:
//   1. 每帧测向得到每个角度(0-359度)的伪谱值(相关系数)
//   2. 将各卫星的伪谱按类型加权合并(欺骗信号获得更高权重)
//   3. 对合并后的伪谱进行多帧积分累积(m_Angle_Accumulate)
//   4. 每帧累积后乘以衰减因子(m_Accumulate_multiplier)，实现指数衰减
//   5. 累积器中的峰值角度即为最终的稳定测向结果
//
// 关键功能:
//   1. setSpoofingResultPseudoSpectrumDesity - 伪谱积分主函数
//   2. getPseudoSpectrumType - 按类型计算合并伪谱(加权)
//   3. getPseudoSpectrumDesity - 多星伪谱合成求角度
// =============================================================================
// 伪谱积分

// =========================================================================
// 伪谱积分主函数
// 流程:
//   1. 对每个频点的所有卫星伪谱进行加权合并(getPseudoSpectrumType)
//   2. 对各频点的合并伪谱再进行合成求角度(getPseudoSpectrumDesity)
//   3. 将角度结果累积到m_Angle_Accumulate积分器中
//   4. 在峰值角度附近施加邻域扩散(5度范围内的角度也获得递减的积分)
//   5. 选择累积值最高的角度作为最终结果
//   6. 所有角度累积值乘以衰减因子(m_Accumulate_multiplier)实现指数衰减
// 注: 最终结果会添加微小随机扰动(0.1-1.0度)，避免完全相同的输出
// =========================================================================
void SpoofingDoa::setSpoofingResultPseudoSpectrumDesity(SpoofingResult &result)
{
    int count = result.i_Count;
    SatelliteAngle tp_Satellite;
    AlarmData tp_alarm;
    int prnNum = 0;
    int typeInt = 0;
    int prn = 0;
    vector<vector<double>> pseudoS;       // 多星伪谱: [卫星数][360角度]
    vector<double> tp_pseudo;             // 合并后的伪谱(360维)
    vector<vector<double>> tp_pseudoS;    // 多频点合并伪谱: [频点数][360角度]
    vector<double> tp_pseudo2;
    int tp_Alarm_num = 0;
    for (int i = 0; i < count; i++)
    {

        tp_Satellite = result.i_SatelliteAngle[i];
        typeInt = TypeInt(tp_Satellite.i_Sys, tp_Satellite.i_Type);
        prnNum = tp_Satellite.i_Count;
        pseudoS.clear();
        // 收集该频点所有卫星的伪谱
        for (int j = 0; j < prnNum; j++)
        {
            tp_pseudo2.clear();
            prn = tp_Satellite.i_AlarmData[j].i_Prn;
            tp_pseudo2 = m_Pseudo_Spectrum_Value[typeInt][prn];  // 从保存的伪谱数据中取出
            pseudoS.emplace_back(tp_pseudo2);
        }
        tp_Alarm_num = tp_Satellite.i_Alarm;
        tp_pseudo.clear();
        PublicSpace::Log("Sys=%d,Type=%d\n", tp_Satellite.i_Sys, tp_Satellite.i_Type);
        getPseudoSpectrumType(pseudoS, tp_pseudo, tp_Alarm_num);  // 加权合并伪谱
        tp_pseudoS.emplace_back(tp_pseudo);
    }

    if (count > 0)
    {
        double tp_angle = 0;
        getPseudoSpectrumDesity(tp_pseudoS, tp_angle);  // 多频点合成求角度
        int tp_angle_diff = 0;
        int tp_angle_diff2 = 0;

        // 伪谱累积: 主角度获得权重50
        m_Angle_Accumulate[tp_angle] += 50;
        double angle2 = tp_angle;
        double max_accu = m_Angle_Accumulate[tp_angle];
        // 邻域扩散: 主角度±5度范围内获得递减的积分权重
        for (int i = 1; i < 6; i++)
        {
            tp_angle_diff = Round360((tp_angle - i));
            tp_angle_diff2 = Round360((tp_angle + i));
            m_Angle_Accumulate[tp_angle_diff] += (50 - i);   // 权重递减
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

        // 添加微小随机扰动(0.1-1.0度)，避免完全相同的输出
        std::random_device rd;
        std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(1, 10);
        for (int i = 0; i < count; i++)
        {
            int rand = dis(gen);
            result.i_SatelliteAngle[i].i_Angle = angle2 + rand / 10.0;  // 添加0.1-1.0度随机扰动
            result.i_SatelliteAngle[i].i_Alarm = 1;
        }
    }
    // 日志输出累积值，并应用衰减因子
    PublicSpace::Log("Accumulate:\n");
    for (int i = 0; i < 360; i++)
    {
        PublicSpace::Log("%.2f,", m_Angle_Accumulate[i]);
        // 指数衰减: 每帧累积值乘以衰减因子(0-1之间)
        // 衰减因子为0则不清零(保留历史)，因子<1则历史影响逐渐减小
        m_Angle_Accumulate[i] *= m_Accumulate_multiplier;
    }
    PublicSpace::Log("\n");
}

// =========================================================================
// 多频点伪谱合成求角度
// 功能: 将多个频点的合并伪谱叠加(各角度值相加)
//       找到叠加后伪谱值最大的角度作为估计的到达角
// =========================================================================
void SpoofingDoa::getPseudoSpectrumDesity(vector<vector<double>> diff, double &angle)
{
    int size = (int)diff.size();  // 频点数
    double max_diff = -9999.0;
    int max_angle = 0;
    PublicSpace::Log("diff sum all:\n");
    double tp_sum = 0.0;
    for (int k1 = 0; k1 < 360; k1++)
    {
        tp_sum = 0.0;
        // 将所有频点的伪谱在该角度上的值相加
        for (int j = 0; j < size; j++)
        {

            tp_sum = tp_sum + diff[j][k1];
        }
        if (tp_sum > max_diff)
        {
            max_diff = tp_sum;
            max_angle = k1;  // 峰值角度
        }
        PublicSpace::Log("%.4f,", tp_sum);
    }
    PublicSpace::Log("\n");
    angle = max_angle;
}

// =========================================================================
// 按类型计算合并伪谱(加权)
// 功能: 对同一频点的多颗卫星的伪谱进行合并
// 加权策略:
//   - Alarm=1(欺骗信号): 将合并后的伪谱乘以3(提高权重)
//     因为欺骗信号被认为来自同一方向，其伪谱应当一致
//   - Alarm=0(非欺骗): 保持原权重
// 输出: meanDiff[360] 该频点的合并伪谱(每个角度的均值/加权值)
// =========================================================================
void SpoofingDoa::getPseudoSpectrumType(const vector<vector<double>> diff, vector<double> &meanDiff, int Alarm)
{

    meanDiff.clear();
    meanDiff.resize(360);
    int size = (int)diff.size();  // 该频点的卫星数
    vector<double> sum_diff;
    sum_diff.clear();
    sum_diff.resize(360);
    for (int k1 = 0; k1 < 360; k1++)
    {
        sum_diff[k1] = 0.0;
    }
    // 各角度伪谱求和
    for (int j = 0; j < size; j++)
    {
        for (int k1 = 0; k1 < 360; k1++)
        {
            sum_diff[k1] = sum_diff[k1] + diff[j][k1];
            PublicSpace::Log("%.4f,", diff[j][k1]);
        }
        PublicSpace::Log("\n");
    }

    // 计算均值并应用加权
    for (int k1 = 0; k1 < 360; k1++)
    {

        meanDiff[k1] = sum_diff[k1] / size;  // 均值归一化
        if (1 == Alarm)
        {
            meanDiff[k1] = meanDiff[k1] * 3;  // 欺骗信号的伪谱权重乘以3
        }
        PublicSpace::Log("%.4f,", meanDiff[k1]);
    }
    PublicSpace::Log("\n");
}

// =============================================================================
// == 原 WriteLog.cpp —— 日志输出实现 =================================================
// =============================================================================
// =============================================================================
// 文件名: WriteLog.cpp
// 功能描述: 日志输出模块
// 系统角色: 提供各类数据结构的格式化日志输出函数，用于:
//           1. 调试和问题定位
//           2. 数据流程跟踪
//           3. 测向结果记录和分析
// 日志内容: 测向结果(SpoofingResult)、原始GNSS数据、相位差A/B格式数据
// 注: 日志输出通过PublicSpace::Log()函数实现，最终写入日志文件
// =============================================================================
// 编写日志的相关函数

// =========================================================================
// 输出欺骗测向结果日志
// 格式: 时间戳 -> 频点数 -> 每个频点的系统、频点、卫星数、到达角
//       -> 每颗卫星的PRN、载噪比、测向角度、测向质量
// =========================================================================
void SpoofingDoa::LogSpoofingResult(const SpoofingResult result)
{ // 输出检测结果或测向结果
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

// =========================================================================
// 输出原始GNSS观测数据日志
// 包含两个通道(端口0和端口1)的所有卫星信息:
//   卫星系统、频点、PRN、伪距、载噪比、载波相位、多普勒
// =========================================================================
void SpoofingDoa::LogGNSSData(const GNSSData data, int n)
{
    SatelliteData tp;
    // 原始数据(仅在启用原始数据保存时输出)
    if (0 != m_Save_Original_Flg)
    {
        // 端口0(第一通道)
        PublicSpace::Log("%d-1,i_PortOneNum:%d\n", n, data.i_PortOneNum);
        for (int i = 0; i < data.i_PortOneNum; i++)
        {
            tp = data.i_PortOne[i];
            PublicSpace::Log("%d-1,Sys=%d,Type=%d,Prn=%d,Psr=%.5f,Snr=%.1f,Phase=%.5f,Dop=%.5f\n", n, tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Psr, tp.i_Snr, tp.i_Phase, tp.i_Dop);
        }
        // 端口1(第二通道)
        PublicSpace::Log("%d-2,i_PortTwoNum:%d\n", n, data.i_PortTwoNum);
        for (int i = 0; i < data.i_PortTwoNum; i++)
        {
            tp = data.i_PortTwo[i];
            PublicSpace::Log("%d-2,Sys=%d,Type=%d,Prn=%d,Psr=%.5f,Snr=%.1f,Phase=%.5f,Dop=%.5f\n", n, tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Psr, tp.i_Snr, tp.i_Phase, tp.i_Dop);
        }
    }
}

// =========================================================================
// 输出单颗卫星的B格式相位差数据日志
// 格式: 卫星信息(系统/频点/PRN) -> snr1数组 -> snr2数组 -> 相位差数组(度)
// 注: 日志中相位差已转换为度(原始数据为周，乘以360)
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffB(const SatelliteDataPhaseDiffB tp)
{
    PublicSpace::Log("Sys=%d,Type=%d,Prn=%d\n",
                     tp.i_Sys, tp.i_Type, tp.i_Prn);
    // 第一通道载噪比
    PublicSpace::Log("       snr1=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_Snr1[j]);
    }
    PublicSpace::Log("]\n");
    // 第二通道载噪比
    PublicSpace::Log("       snr2=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_Snr2[j]);
    }
    PublicSpace::Log("]\n");
    // 载波相位差(度)
    PublicSpace::Log("  phasediff=[");
    for (int j = 0; j < tp.i_diffLen; j++)
    {
        PublicSpace::Log("%.2f,", tp.i_phase_diff[j] * 360);
    }
    PublicSpace::Log("]\n");
}

// =========================================================================
// 输出多颗卫星的B格式相位差数据日志(批量)
// =========================================================================
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

// =========================================================================
// 输出多颗卫星的A格式相位差数据日志(批量)
// A格式: 每颗卫星一行，包含系统/频点/PRN/两个SNR/相位差(度)
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffA(const vector<SatelliteDataPhaseDiffA> &dataA)
{

    for (unsigned int i = 0; i < dataA.size(); i++)
    {
        SatelliteDataPhaseDiffA tp = dataA[i];
        PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,snr1=%.2f,snr2=%.2f,phasediff=%.2f\n",
                         tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Snr1, tp.i_Snr2, tp.i_phase_diff * 360);
    }
}

// =========================================================================
// 输出单颗卫星的A格式相位差数据日志
// =========================================================================
void SpoofingDoa::LogSatelliteDataPhaseDiffA(const SatelliteDataPhaseDiffA dataA)
{

    SatelliteDataPhaseDiffA tp = dataA;
    PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,snr1=%.2f,snr2=%.2f,phasediff=%.2f\n",
                     tp.i_Sys, tp.i_Type, tp.i_Prn, tp.i_Snr1, tp.i_Snr2, tp.i_phase_diff * 360);
}

// =========================================================================
// 输出按频点分类的相位差数据日志
// 按频点分组输出该频点下所有卫星的A格式相位差数据
// =========================================================================
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

#include <vector>
#include <map>

using namespace std;

// =============================================================================
// 每个 GN902 实例的内部状态(引擎 + 整轮帧缓冲)。
// 以对象地址索引存放在文件内容器中，避免改动 GN902.h 中已有的类定义/成员。
//   eng       : SpoofingDoa 引擎。构造时即 initProject()+Init()(运行参数内置在引擎代码中，
//               不读取外部配置文件)，并按 Python 流程阈值初始化各频点)。引擎内部持跨轮(跨周期)状态:
//               m_ConsecutiveAlarm / m_Tracking / m_Baselines，在本类多轮调用间持续累积。
//   buf/cuts  : 当前轮逐帧送来的 GNSSData 与对应 cutIdx(通道2天线号 1..7)。
//   calFrames : 已储存的校正刀(cutIdx=1 / 天线对{1,1})数据，每轮出现校正刀则整体刷新。
//   result    : 最近一轮喂入引擎后取出的测向结果(供 GetResult 返回)。
// =============================================================================
namespace {
struct GN902State
{
    SpoofingDoa *eng;                // 欺骗检测+测向引擎(循环切刀)
    std::vector<GNSSData> buf;       // 当前轮待处理帧(原始到达顺序)
    std::vector<int> cuts;           // 每帧对应 cutIdx
    std::vector<GNSSData> calFrames; // 储存的校正刀数据
    SpoofingResult result;           // 最近一轮测向结果

    GN902State() : eng(0) { clearResult(); }

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

GN902::GN902(){
    GN902State *st = new GN902State();
    // SpoofingDoa 构造即 initProject()+Init()：运行参数已内置在引擎 Init() 中(不读配置文件)，
    // 并按 Python 流程对齐各频点阈值。
    st->eng = new SpoofingDoa();
    if (st->eng != 0)
    {
        // GN902 运行参数：循环切刀检测开、全向半径 0.1865m(重建理论模板)、
        // 暂不平滑(整轮喂帧时按每刀实际帧数自适应)。
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
// @param cutCountThreshold 通道2对应天线阈值
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
// @param cutIdx 通道二天线索引
// @param endFlag 结束标志
void GN902::SetData(const GNSSData* data, int cutIdx, int endFlag){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        return;
    }
    GN902State *st = it->second;
    // 帧空或 cutIdx 非法(1=校正刀, 2..7=六测向刀)则忽略该帧，不打断整轮累积
    if (data == 0 || cutIdx < 1 || cutIdx > 7)
    {
        return;
    }
    // 非最后一帧数据，保存数据
    if (!endFlag){
        // 保存数据
        st->buf.push_back(*data);
        st->cuts.push_back(cutIdx);
        return;
    }
    // 最后一帧数据，先保存后整轮处理：
    // Detect() 组批喂入引擎(更新告警flag/跟踪)，Doa() 判断是否测向并取出结果
    else{
        st->buf.push_back(*data);
        st->cuts.push_back(cutIdx);
        Detect();
        Doa();
    }
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
    frameCuts.swap(st->cuts);

    // 拆帧：
    //   cutIdx=1  -> 校正刀(code=0 / 天线对1-1)，不参与测向，先存为校正数据；
    //   cutIdx=2..7 -> 六刀测向刀(对应 code 9,57,17,25,33,1 / {1,2}..{1,7})。
    std::vector<GNSSData> calNew;
    std::vector<std::vector<int> > detByCut(6); // index = cutIdx-2 (0..5)
    for (size_t i = 0; i < frameCuts.size(); ++i)
    {
        int c = frameCuts[i];
        if (c == 1)
        {
            calNew.push_back(frameData[i]);
        }
        else
        {
            detByCut[c - 2].push_back((int)i); // c 已保证 2..7
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

    // 按引擎行序组批：row0 = 校正(cutIdx1)，row1..6 = 六刀测向(cutIdx2..7)
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
