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
#include "SpoofingDoa.h"
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
