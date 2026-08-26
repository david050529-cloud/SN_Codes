// =============================================================================
// 文件名: Alarm.cpp
// 功能描述: 欺骗信号检测与告警模块
// 系统角色: 本文件实现了欺骗信号检测方法，是欺骗测向系统的检测前端:
//           1. 相位差法检测(calAlarmByPhaseDiff，载噪比≥35dB 作为数据质量门限)
// 检测原理: 欺骗信号由同一干扰源发射，因此多颗卫星的相位差相近(来自同一方向)
// =============================================================================
#include "pch.h"
#include "SpoofingDoa.h"

// =========================================================================
// 欺骗检测参数（对应 Python cyclic_phase_detection.py）
// =========================================================================
const double SpoofingDoa::CNR_MIN_DB = 35.0;           // 载噪比质量门限：两端口都需 ≥35dB
const double SpoofingDoa::STABILITY_RANGE_DEG = 5.0;   // 稳定性阈值：最小覆盖弧 < 5° 判为稳定
const int SpoofingDoa::MIN_STABLE_SAMPLES = 5;         // 每刀至少需要的有效采样帧数

// =========================================================================
// 归一化角度到 [-180°, 180°)，对应 Python simplified_detection.normalize_angle_180
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
// 检测方法对应 Python cyclic_phase_detection.py 的 cluster_satellites：
//   1. 数据筛选：两端口载噪比均 ≥ 35dB 的观测才参与检测（载噪比质量门限）；
//   2. 相位差由「周」转「度」并归一化到 [-180°, 180°)，消除 0/360 边界歧义；
//   3. 按相位差排序后，用滑动窗口找「跨度 < 相位差阈值」的最大卫星集合
//      （比原「以某星为参考数邻近星」更准确：聚类跨度直接受阈值约束）；
//   4. 最大聚类卫星数达到检测阈值则判为欺骗。
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

    // 4. 判定：最大聚类卫星数达到阈值
    if (bestCount >= m_Detection_Threshold[typeInt])
    {
        alarm = 1;
        for (int idx : bestIds)
        {
            alarmSatelliteData.emplace_back(dataA[idx]);
        }
    }
}

