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
#include "SpoofingDoa.h"
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
