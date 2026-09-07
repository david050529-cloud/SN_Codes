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
#include "SpoofingDoa.h"
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
