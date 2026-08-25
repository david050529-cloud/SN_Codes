// =============================================================================
// 文件名: Alarm.cpp
// 功能描述: 欺骗信号检测与告警模块
// 系统角色: 本文件实现了三种欺骗信号检测方法，是欺骗测向系统的检测前端:
//           1. 相位差法检测(calAlarmByPhaseDiff)
//           2. 角度法检测(calAlarmByAngle)
//           3. 载噪比法检测(calAlarmBySnr)
//           4. 滑动窗口确认(setDetectionRecords)
// 检测原理: 欺骗信号由同一干扰源发射，因此:
//           - 多颗卫星的相位差相近(来自同一方向)
//           - 多颗卫星的测向角度相近(到达角聚类)
//           - 多颗卫星的载噪比相近(同一发射功率)
// =============================================================================
#include "pch.h"
#include "SpoofingDoa.h"

#include <set>

// =========================================================================
// 欺骗检测参数（对应 Python cyclic_phase_detection.py 的数据质量门限）
// =========================================================================
static const double CNR_MIN_DB = 35.0;  // 载噪比质量门限：两端口载噪比都需 ≥35dB

// =========================================================================
// 归一化角度到 [-180°, 180°)，对应 Python simplified_detection.normalize_angle_180
// 把 [0°, 360°) 的相位差折叠到以 0° 为中心的主值区间，消除 0/360 边界歧义
// =========================================================================
static double normalizeAngle180(double deg)
{
    double r = fmod(deg + 180.0, 360.0);
    if (r < 0)
    {
        r += 360.0;
    }
    return r - 180.0;
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

// =========================================================================
// 利用示向度(到达角)计算是否为告警(角度法欺骗检测)
// 核心思想: 真实卫星来自不同方向，到达角各不相同；
//           欺骗信号来自同一干扰源，到达角应当相近
// 算法流程:
//   1. 遍历0-359度，统计每个角度附近(m_Angle_Threshold范围内)的卫星数
//   2. 找到聚集卫星数最多的角度(如果有多个角度聚集数相同，选质量最高的)
//   3. 如果最大聚集卫星数 >= 检测阈值，判定为欺骗
//   4. 在最大聚集中找出最频繁出现的具体角度作为最终到达角
// =========================================================================
void SpoofingDoa::calAlarmByAngle(int typeInt, const std::vector<AlarmData> &data, std::vector<int> &alarmIndex, int &alarm, double &angle) // 利用示向度计算是否为告警
{
    PublicSpace::Log("Doa result gather:\n");
    for (unsigned int i = 0; i < data.size(); i++)
    {
        PublicSpace::Log("Sys=%d,Type=%d,Prn=%d,Angle=%d,Qulity=%.2f\n", typeInt / 100, typeInt % 100, data[i].i_Prn, data[i].i_Angle, data[i].i_Quality);
    }

    // angles[ang]: 存储所有到达角在ang附近的卫星索引列表
    vector<vector<int>> angles(360);
    double diff;
    int maxSize = -1;
    int maxAng = -1;
    vector<AlarmData> localAlarmData;
    vector<AlarmData> tp;
    for (int ang = 0; ang < 360; ++ang)
    {
        angles[ang].clear();
        tp.clear();
        for (unsigned int i = 0; i < data.size(); ++i)
        {
            diff = fabs(ang - data[i].i_Angle);
            if (diff >= 180)
            {
                diff = 360 - diff;  // 角度差值考虑360度循环
            }
            if (diff < m_Angle_Threshold) // 示向度在±m_Angle_Threshold度之内算同一方向
            {
                angles[ang].push_back(i);
                tp.emplace_back(data[i]);
            }
        }

        int angles_size = (int)angles[ang].size();
        // 如果与当前最大聚集数相同，比较质量
        if (angles_size == maxSize)
        {
            double sum_qulity_tp1 = 0.0;
            double sum_qulity_t2 = 0.0;
            for (int k2 = 0; k2 < maxSize; k2++)
            {
                sum_qulity_tp1 = sum_qulity_tp1 + localAlarmData[k2].i_Quality;
                sum_qulity_t2 = sum_qulity_t2 + tp[k2].i_Quality;
            }

            // 质量更高的组胜出
            if ((sum_qulity_tp1) < (sum_qulity_t2))
            {
                maxAng = ang;
                alarmIndex = angles[ang];
                localAlarmData = tp;
            }
        }
        if (angles_size > maxSize) // 找到最大的那个数据
        {
            maxSize = angles_size;
            maxAng = ang;
            alarmIndex = angles[ang];
            localAlarmData = tp;
        }
    }
    // 聚集卫星数不足阈值，不判定为欺骗
    if (maxSize < m_Detection_Threshold[typeInt])
    {
        alarm = 0;
        angle = -1;
        alarmIndex.clear();
        localAlarmData.clear();
        return;
    }

    alarm = 1;
    int count = 0;
    int maxCount = -1;
    // 在最大聚集中，找出最频繁出现的具体角度作为最终到达角
    for (int ang = 0; ang < 360; ++ang)
    {
        count = 0;
        for (unsigned int i = 0; i < angles[maxAng].size(); ++i)
        {
            if (ang == data[angles[maxAng][i]].i_Angle)
            {
                ++count;
            }
        }
        if (count > maxCount)
        {
            maxCount = count;
            angle = ang;
        }
    }
    angle = Round360(angle);
}

// =========================================================================
// 利用载噪比检测是否为欺骗信号(载噪比法欺骗检测)
// 核心思想: 欺骗信号由同一干扰源发射，相同传播路径，
//           到达接收机的各颗卫星载噪比应相近(差异<=1dB)
// 算法流程:
//   1. 对第一通道和第二通道分别进行载噪比聚类分析
//   2. 统计载噪比相近(差值<=1dB)的卫星数量
//   3. 两个通道各自计算最大聚类，取两者中更大的
//   4. 如果最大聚类数量 >= 检测阈值，判定为欺骗
// =========================================================================
void SpoofingDoa::calAlarmBySnr(int typeInt, const std::vector<SatelliteDataPhaseDiffA> &dataA, std::vector<SatelliteDataPhaseDiffA> &alarmSatelliteData, int &alarm)
{ // 由载噪比检测是否未欺骗信号
    alarmSatelliteData.clear();
    int maxCount = -1;
    int count = 0;
    int count2 = 0;
    int size = (int)dataA.size();
    vector<SatelliteDataPhaseDiffA> localMaxSatelliteData;
    vector<SatelliteDataPhaseDiffA> localMaxSatelliteData2;
    double tp1, tp2, diff;
    double tp21, tp22, diff2;
    for (int i = 0; i < size; ++i)
    {
        count = 0;
        count2 = 0;
        localMaxSatelliteData.clear();
        localMaxSatelliteData2.clear();
        tp1 = dataA[i].i_Snr1;   // 第一通道载噪比参考值
        tp21 = dataA[i].i_Snr2;  // 第二通道载噪比参考值
        if (tp1 < 1e-3)
        {
            tp1 = -30;  // 无效载噪比设为极低值
        }
        if (tp21 < 1e-3)
        {
            tp2 = -30;
        }
        for (int j = 0; j < size; ++j)
        {
            tp2 = dataA[j].i_Snr1;
            tp22 = dataA[j].i_Snr2;
            diff = fabs(tp1 - tp2);
            diff2 = fabs(tp21 - tp22);
            if (diff <= 1) // 若载噪比的差值小于1dB，认为来自同一发射源
            {
                ++count;
                localMaxSatelliteData.emplace_back(dataA[j]);
            }
            if (diff2 <= 1) // 若载噪比的差值小于1dB
            {
                ++count2;
                localMaxSatelliteData2.emplace_back(dataA[j]);
            }
        }
        // 更新第一通道最大聚类
        if (count > maxCount)
        {
            maxCount = count;
            alarmSatelliteData = localMaxSatelliteData;
        }
        // 更新第二通道最大聚类
        if (count2 > maxCount)
        {
            maxCount = count2;
            alarmSatelliteData = localMaxSatelliteData2;
        }
    }

    alarm = (maxCount >= (m_Detection_Threshold[typeInt]) ? 1 : 0);
    if (0 == alarm)
    {
        alarmSatelliteData.clear();
    }
}


// =========================================================================
// 设置欺骗检测记录(滑动窗口确认机制)
// 原理: 使用固定长度的队列维护最近N帧的检测结果
//       N = m_Detection_Recodds_Num
//       只有连续N帧全部检测为欺骗(1)时，才最终判定为欺骗
// 作用: 避免单帧检测错误导致的虚警，提高检测可靠性
// 注: 如果该频点是首次检测，先用0填充前面N-1位置
// =========================================================================
void SpoofingDoa::setDetectionRecords(int typeInt, int &alarm)
{

    deque<int> dq;
    if (m_Detection_Records.find(typeInt) == m_Detection_Records.end())
    {
        // 首次检测: 创建长度为N的队列，前面填0，最后一位置当前结果
        for (int i = 0; i < m_Detection_Recodds_Num - 1; i++)
        {
            dq.push_back(0);
        }
        dq.push_back(alarm);
        m_Detection_Records[typeInt] = dq;
    }
    else
    {
        Log("typeInt=%d:", typeInt);

        dq = m_Detection_Records[typeInt];
        dq.pop_front();   // 移除最旧的一帧
        dq.push_back(alarm);  // 加入最新的一帧
        int count = 0;
        for (auto it = dq.begin(); it != dq.end(); ++it)
        {
            count = count + *it;  // 累计欺骗检测次数
            Log("records=%d,", *it);
        }
        Log("\n");
        m_Detection_Records[typeInt] = dq;
        // 计算欺骗检测概率(检测为欺骗的帧比例)
        double probability = count / m_Detection_Recodds_Num;
        if (probability < 1)
        {
            alarm = 0;  // 不是连续N帧都检测为欺骗，重置告警
        }
    }
}
