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
#include "SpoofingDoa.h"
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
