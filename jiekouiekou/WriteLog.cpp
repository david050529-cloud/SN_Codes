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
#include "SpoofingDoa.h"
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
