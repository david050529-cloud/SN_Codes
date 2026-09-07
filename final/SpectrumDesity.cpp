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
#include "SpoofingDoa.h"
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
