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
#include "SpoofingDoa.h"


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
        m_Doa_Cut_Num = 7; // 用于测向的刀数
        m_Doa_Arithmetic = 1;  // 使用相关干涉仪算法
        m_Doa_Cut_min_Num = 6; // 用于测向的相位差的最少数量
        m_omni_R = 0.1865;    // 全向天线阵列半径(米)
        m_antnenaType = 0;    // 全向天线
        m_cutSequence.clear();
        // 切刀顺序: {7,7}同天线功分(校正), {1,2}~{1,7}天线1与其他天线组成基线
        m_cutSequence = {{7, 7}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
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
        m_cutSequence = {{7, 7}, {1, 2}, {1, 3}, {1, 4}, {7, 7}, {1, 5}, {1, 6}, {1, 7}};
        break;
    case GN930U:
        m_AntennaNum = 7;
        m_Doa_Cut_Num = 7;
        m_Doa_Arithmetic = 1;
        m_Doa_Cut_min_Num = 6;
        m_omni_R = 0.2;      // 比GN902/GN930的半径(0.1865m)略大
        m_antnenaType = 0;
        m_cutSequence.clear();
        m_cutSequence = {{7, 7}, {1, 2}, {1, 3}, {1, 4}, {1, 5}, {1, 6}, {1, 7}};
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
// 设置连续欺骗检测记录数(滑动窗口长度)
// 取值范围: 1-10, 超出范围默认设为1
// 作用: 修改后需要重新初始化记录队列
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

    initRecords();  // 重新初始化历史记录队列
}


// =========================================================================
// 初始化历史记录(为每个频点创建固定长度的滑动窗口队列)
// 队列初始值全部为0(表示无欺骗)，长度为m_Detection_Recodds_Num
// 作用: 每次有新检测结果时，push_back新值,pop_front旧值
//       当队列中所有值都为1时才最终判定为欺骗
// =========================================================================
void SpoofingDoa::initRecords(void)
{
    m_Detection_Records.clear();
    deque<int> qu;

    // 创建初始全0队列
    for (int i = 0; i < m_Detection_Recodds_Num; ++i)
    {
        qu.push_back(0);
    }

    // 为每个频点分配独立的队列
    for (auto it = m_F.begin(); it != m_F.end(); ++it)
    {
        m_Detection_Records[it->first] = qu;
    }
}
