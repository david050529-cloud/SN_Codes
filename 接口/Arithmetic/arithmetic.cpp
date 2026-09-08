/**
 * @file arithmetic.cpp
 * @brief 测向算法核心实现 -- 相关干涉仪、幅相法、虚拟阵列扩展等
 *
 * 本文件实现了 ArithmeticDoa 类中声明的所有测向算法方法，包括：
 *
 * ---- 相关干涉仪测向 (Correlative Interferometer DOA) ----
 *   核心思想：将实测相位差向量与每个候选角度的理论相位差向量做相关匹配，
 *   相关度最高的角度即为来波方向。使用余弦函数作为相关度量，天然处理相位周期性。
 *   相关公式：correlation(ang) = sum_i cos( theory_phase_diff_i(ang) - actual_phase_diff_i )
 *   优点：计算简单、速度快、对噪声有一定鲁棒性。
 *   局限：仅使用相位信息，在阵元间距大于半波长时可能出现相位模糊（跳半周）。
 *
 * ---- 幅相法测向 (Amplitude-Phase DOA) ----
 *   核心思想：同时利用幅度和相位信息，通过复数阵列流型的内积匹配进行测向。
 *   阵列流型匹配公式：similarity(ang) = |sum_i conj(simulate_A[ang][i]) * actual_A[i]| / (norm(simulate_A) * norm(actual))
 *   即归一化复数内积的模，取值范围 [0, 1]，1 表示完全匹配。
 *   优点：同时利用幅度信息、对不同天线增益差异敏感、适合非理想天线方向图。
 *   局限：依赖仿真/校准数据、对天线方向图精度要求较高。
 *
 * ---- 虚拟阵列扩展 (Virtual Array Expansion) ----
 *   核心思想：将实测的相位差按虚拟倍率 virMultiple 缩放，生成等效的新"虚拟阵元"和对应的相位差，
 *   从而在不增加物理天线的条件下改变等效基线长度。
 *   - virMultiple < 1：缩短等效基线，用于消除相位模糊（解决"跳半周"问题）
 *   - virMultiple > 1：增加等效孔径，用于提高角度分辨率
 *
 * ---- 测向质量 (DOA Quality) ----
 *   基于归一化互相关系数计算：quality = (corr + 1) / 2 * 100
 *   corr = sum(diffTheory[i] * diff[i]) / (sqrt(sum(diffTheory^2)) * sqrt(sum(diff^2)))
 *   quality 的取值范围为 [0, 100]，越高表示测向结果越可靠。
 *
 * ---- 关键物理常数（定义于 publicFunctionDoa.h） ----
 *   m_PI = 3.14159265358979323846  -- 圆周率
 *   m_C  = 3e8                     -- 光速 (m/s)，用于相位-距离转换
 *
 * ---- 坐标系约定 ----
 *   - 角度以度为单位，范围 [0, 360)，0 度 = 正北/正前方（取决于阵列安装方向）
 *   - 相位以弧度为单位，范围通常为 [-pi, pi]（实际测量）或任意值（理论计算）
 *   - 天线从 1 开始编号，内部计算时转换为 0-based 索引
 */

#include "arithmetic.h"
using namespace std;
// using namespace PublicSpace;
ArithmeticDoa::ArithmeticDoa()
{
    // m_CurrentPath = getCurrentExecutablePath();
}

ArithmeticDoa::~ArithmeticDoa()
{
}

/**
 * @brief 虚拟阵列扩展 -- 根据虚拟倍率生成虚拟阵元及对应的相位差
 *
 * 算法原理（逐步说明）：
 *   对于每一对原始天线 (A, B) 及其相位差 diff_AB = phase(A) - phase(B)：
 *
 *   步骤 1 -- 正向虚拟扩展：
 *     - 生成虚拟阵元编号：virtualAnt = getVirtualAntNum(B, A) = B*10 + A
 *       含义：以天线 B 为基准，向 A 的方向外推一个虚拟阵元
 *     - 虚拟相位差：virtualDiff = diff_AB * virMultiple
 *       含义：将原始 AB 之间的相位差按倍率缩放，相当于改变了等效基线长度
 *     - 添加天线对 (B, virtualAnt)，相位差为 virtualDiff
 *       含义：虚拟阵元的相位 = 天线 B 的相位 + virtualDiff
 *
 *   步骤 2 -- 反向虚拟扩展：
 *     - 生成对称的虚拟阵元编号：virtualAnt = getVirtualAntNum(A, B) = A*10 + B
 *     - 虚拟相位差：virtualDiff = -diff_AB * virMultiple
 *     - 添加天线对 (A, virtualAnt)，相位差为 -virtualDiff
 *       含义：另一个方向的虚拟扩展，保证对称性
 *
 * 虚拟阵列的物理意义：
 *   如果 virMultiple = 0.5，等效于将基线长度缩小一半，可用于消除大孔径阵列的相位模糊。
 *   如果 virMultiple = 2.0，等效于将基线长度扩大一倍，可提高角度分辨率。
 *   每个原始天线对产生 2 个虚拟天线对，总天线对数从 N 变为 3N。
 *
 * @param virMultiple 虚拟倍率，乘以原始相位差得到虚拟相位差
 * @param antnna 天线对列表 [输入] 原始天线对 / [输出] 原始+虚拟天线对
 * @param phase_diff 相位差数组 [输入] 原始相位差 / [输出] 原始+虚拟相位差
 */
void ArithmeticDoa::getVirtual(const double virMultiple, vector<vector<int>> &antnna, vector<double> &phase_diff)
{
    // 保存原始天线对和相位差的副本，因为后续 emplace_back 会修改容器
    vector<vector<int>> tp_antnna;
    tp_antnna = antnna;
    vector<double> tp_diff;
    tp_diff = phase_diff;
    int count = tp_antnna.size(); // 原始天线对数量，循环只遍历原始数据
    int virtualAnt = 0;
    double virtualDiff = 0.0;
    vector<int> tp;
    tp.resize(2);
    for (int i = 0; i < count; i++)
    {
        // ---- 正向虚拟扩展：以天线对中的第二个天线为基准，向第一个天线的方向外推 ----
        virtualAnt = getVirtualAntNum(tp_antnna[i][0], tp_antnna[i][1]); // 虚拟天线编号 = 天线1*10 + 天线2
        virtualDiff = tp_diff[i] * virMultiple;                          // 按倍率缩放相位差
        tp[0] = tp_antnna[i][1];                                         // 新天线对：天线B
        tp[1] = virtualAnt;                                              // 新天线对：虚拟天线
        tp_antnna.emplace_back(tp);                                      // 追加到天线对列表
        tp_diff.emplace_back(virtualDiff);                               // 追加对应的虚拟相位差

        // ---- 反向虚拟扩展：对称方向的外推 ----
        virtualAnt = getVirtualAntNum(tp_antnna[i][1], tp_antnna[i][0]); // 对称虚拟天线编号
        virtualDiff = -tp_diff[i] * virMultiple;                         // 取反的缩放相位差
        tp[0] = tp_antnna[i][0];                                         // 新天线对：天线A
        tp[1] = virtualAnt;                                              // 新天线对：虚拟天线
        tp_antnna.emplace_back(tp);                                      // 追加到天线对列表
        tp_diff.emplace_back(virtualDiff);                               // 追加对应的虚拟相位差
    }

    // 用扩展后的数据替换原始数据
    antnna.clear();
    phase_diff.clear();
    antnna = tp_antnna;
    phase_diff = tp_diff;
}

/**
 * @brief 根据虚拟倍率扩展理论相位矩阵，生成虚拟阵元的理论相位
 *
 * 算法原理：
 *   对于每一对不同的天线 (i, j)（i != j）：
 *     1. 计算虚拟天线编号 index = getVirtualAntNum(i+1, j+1) - 1，作为虚拟阵元在线性化矩阵中的列偏移
 *        (i+1, j+1 是因为内部循环使用 0-based 索引，而虚拟天线编号从 1 开始)
 *     2. 计算缩放的相位差：tp_diff = virMultiple * (theory[ang][i] - theory[ang][j])
 *        含义：将天线 i 和 j 之间的相位差按虚拟倍率缩放
 *     3. 计算虚拟阵元的理论相位：tp_diff2 = theory[ang][j] - tp_diff
 *        含义：从天线 j 的实际相位出发，减去（按倍率缩放的）相位差，得到虚拟阵元的理论相位
 *
 * 虚拟理论相位矩阵的结构：
 *   virtualTheory 的列数为 antnnaNum*10 + antnnaNum，这是因为虚拟天线编号 i*10+j 的最大值
 *   对于 antnnaNum 个天线，最大编号为 (antnnaNum-1)*10 + antnnaNum = antnnaNum*10
 *   （实际上不会用到所有位置，仅 index = i*10+j 的位置有值，其余为 0）
 *
 * @param antnnaNum 实际物理天线数量
 * @param tp_theory 原始理论相位矩阵，tp_theory[ang][i] = 角度 ang 处天线 i 的理论相位
 * @param virMultiple 虚拟倍率
 * @param virtualTheory 输出的虚拟理论相位矩阵，virtualTheory[ang][index] = 角度 ang 处虚拟阵元 index 的理论相位
 */
void ArithmeticDoa::getVirtualTheory(const int antnnaNum, const std::vector<std::vector<double>> tp_theory, const double virMultiple, std::vector<std::vector<double>> &virtualTheory) // 获得虚拟矩阵的理论相位
{
    // 虚拟阵元的数量上限：最大编号 = (antnnaNum-1)*10 + antnnaNum，实际使用 antnnaNum*10 个位置
    int num = antnnaNum * 10 + antnnaNum;
    int index = 0;
    virtualTheory.clear();
    virtualTheory.resize(360);
    // 遍历所有 360 个角度
    for (int ang = 0; ang < 360; ang++)
    {
        virtualTheory[ang].resize(num);
        // 遍历所有天线对 (i, j)，i != j
        for (int i = 0; i < antnnaNum; i++)
        {
            for (int j = 0; j < antnnaNum; j++)
            {
                if (i == j)
                {
                    continue; // 跳过同一天线自身，自身的相位差为 0，无意义
                }
                // 计算虚拟天线在线性化矩阵中的索引 (编码规则: ant1*10+ant2)
                index = getVirtualAntNum(i + 1, j + 1) - 1;
                // 缩放的相位差：virMultiple * (相位i - 相位j)
                double tp_diff = virMultiple * (tp_theory[ang][i] - tp_theory[ang][j]);
                // 虚拟阵元的理论相位 = 天线j的实际相位 - (缩放后的相位差)
                // 物理含义：从天线 j 的方向反向外推，得到虚拟阵元位置的理论相位
                double tp_diff2 = tp_theory[ang][j] - tp_diff;
                virtualTheory[ang][index] = tp_diff2;
            }
        }
    }
}

/**
 * @brief 计算定向天线的角度搜索范围
 *
 * 算法原理：
 *   定向天线（如对数周期天线）在不同方向上有不同的增益，信号只能从特定角度范围入射。
 *   因此需要根据信号最强的天线位置，限定DOA搜索范围以提高测向效率和精度。
 *
 * 步骤：
 *   1. 计算每个天线对应的角度跨度：perAngle = 360 / AntennaNum
 *   2. 判断信号最强的天线对（max_index 对应的天线对）中哪一端朝向信号源：
 *      - 天线编号差值 antenna_diff = ant[0] - ant[1]
 *      - 若差值绝对值 > AntennaNum/2，说明跨越了 0 度（360度）边界，需要特殊处理
 *      - 根据差值符号确定 startAntenna（信号来向起始天线）和 endAntenna（信号来向终止天线）
 *   3. 根据 max_index 在 index 中的位置（首/尾/中间），确定搜索范围：
 *      - 若是首元素：搜索范围 = [start角度-10, end角度]
 *      - 若是尾元素：搜索范围 = [start角度-1间距, end角度-1间距+10]
 *      - 若是中间元素：搜索范围 = [start角度-半间距, end角度+半间距]
 *   4. 若起始角度大于结束角度（跨越0度边界），将起始角度减去360度
 *
 * @param index 参与测向的天线对序号列表（按某种顺序排列，如载噪比排序）
 * @param cutSequence 天线对序列（切刀顺序），cutSequence[i] = {天线1编号, 天线2编号}
 * @param AntennaNum 天线总数
 * @param max_index 载噪比最大的天线对在 cutSequence 中的索引
 * @param startAngle 输出的搜索起始角度（度），可能为负值（表示从某角度到0度）
 * @param endAngle 输出的搜索结束角度（度）
 */
void ArithmeticDoa::calAngleSerchRange(const vector<int> index, const vector<vector<int>> cutSequence, const int AntennaNum, int max_index, int &startAngle, int &endAngle) // 计算角度搜索范围，主要用于定向天线
{                                                                                                                                                                           // cutSequence 切刀顺序
    // AntennaNum 天线数
    // index 用于测向的切刀序号
    // max_index 载噪比最大的序号
    int size = (int)index.size();
    int perAngle = (int)(360 / AntennaNum); // 每个天线所占的角度宽度
    int startAntenna = -1;
    int endAntenna = -1;
    float th = AntennaNum / 2.0; // 判断跨越 0 度的阈值
    int antenna_diff = cutSequence[max_index][0] - cutSequence[max_index][1];

    // 根据天线编号差值判断信号来向，确定起始天线和终止天线
    // 四种情况分别处理，考虑了圆周上天线编号的循环性
    if (antenna_diff < (0 - th))  // 差值小于 -N/2：天线1在0度附近，天线2在其后
    {
        startAntenna = cutSequence[max_index][1];
        endAntenna = cutSequence[max_index][0];
    }
    if (antenna_diff > th)  // 差值大于 N/2：跨越 360 度边界
    {
        startAntenna = cutSequence[max_index][0];
        endAntenna = cutSequence[max_index][1];
    }
    if (-th < antenna_diff && antenna_diff < 0)  // 差值在 (-N/2, 0) 区间：天线1在前面
    {
        startAntenna = cutSequence[max_index][0];
        endAntenna = cutSequence[max_index][1];
    }
    if (antenna_diff > 0 && antenna_diff < th)  // 差值在 (0, N/2) 区间：天线2在前面
    {
        startAntenna = cutSequence[max_index][1];
        endAntenna = cutSequence[max_index][0];
    }
    // 根据最强信号天线对在列表中的位置，微调搜索范围
    if (index[0] == max_index)  // 最强对是第一个：搜索范围向前扩展10度
    {
        startAngle = (startAntenna - 1) * perAngle - 10;
        endAngle = endAntenna * perAngle;
    }
    else if (index[size - 1] == max_index)  // 最强对是最后一个：搜索范围向后扩展10度
    {
        startAngle = (startAntenna - 2) * perAngle;
        endAngle = (endAntenna - 1) * perAngle + 10;
    }
    else  // 最强对在中间：搜索范围为所覆盖角度的中心区域
    {
        startAngle = (startAntenna - 1) * perAngle - perAngle / 2;
        endAngle = (endAntenna - 1) * perAngle + perAngle / 2;
    }
    // 处理跨越 0 度边界的情况：起始角度大于结束角度时，减去 360 使搜索范围变为 [start-360, end]
    if (startAngle > endAngle)
    {
        startAngle = startAngle - 360;
    }
}

/**
 * @brief 定向天线模式下的天线选择与搜索范围计算
 *
 * 算法目的：
 *   在定向天线系统中，由于天线具有方向性，信号往往只被其中几个天线接收到。
 *   此函数根据各天线的接收幅度，自动选择用于测向的天线子集，并计算搜索角度范围。
 *
 * 算法原理（逐步说明）：
 *   1. 对所有天线按幅度降序排序，找出信号最强的天线 max_index
 *   2. 以最强天线为中心，逐步向两侧扩展，选取相邻天线加入测向子集：
 *      - 每轮在选中的天线两侧各尝试扩展一个（优先扩展幅度更大的那一侧）
 *      - 持续扩展直到选中天线数达到 use_cut_num + 1
 *      - 该策略考虑了"饱和情况"：当天线饱和时，幅度最大的天线不一定正对信号源，
 *        通过选取两侧相邻天线来覆盖信号源可能的入射方向
 *   3. 根据选中的天线位置计算角度搜索范围：
 *      - 奇数个天线：以中间天线为中心，向两侧各延伸一个天线间距
 *      - 偶数个天线：以中间两个天线之间的位置为中心
 *   4. 搜索范围外扩 10 度作为安全余量
 *
 * @param A 各天线的接收幅度数组（线性值或相对值），A[i] 越大表示天线 i 接收到的信号越强
 * @param use_cut_num 期望选用的测向天线数（注意：实际选 use_cut_num + 1 个天线）
 * @param use_cut 输出的选中天线序号列表（从 1 开始编号）
 * @param startAngle 输出的角度搜索起始值（度）
 * @param endAngle 输出的角度搜索结束值（度）
 */
void ArithmeticDoa::getUseAntennaByADirect(const vector<double> A, const int use_cut_num, vector<int> &use_cut, int &startAngle, int &endAngle)
{
    // use_cut_num 用于测向的天线个数，给出的天线个数要+1
    // 理论上只需要找到最大值，然后再在两边选择几个天线即可
    // 但是，若存在饱和现象，则最大值对应的天线就不一定是信号源正对的天线
    if (use_cut_num < 2)
    {
        return;
    }
    int size = A.size();
    vector<int> index;
    index.clear();
    index.resize(size);
    for (int i = 0; i < size; i++)
    {
        index[i] = i;
    }
    // 按幅度从大到小排序，index 存储排序后的天线序号（0-based）
    std::sort(index.begin(), index.end(), [&](int i, int j)
              { return A[i] > A[j]; }); // 排序，从大到小
    int max_index = index[0];           // 幅度最大的天线序号
    int last_index = 0;
    int next_index = 0;
    vector<int> tp_index;
    tp_index.emplace_back(max_index); // 从最强天线开始构建选中列表
    int size2 = 1;
    int num = 0;
    Log("cal use ant:\n");
    // 迭代扩展直到选中足够多的天线或超过 20 次迭代（防止死循环）
    while (size2 < use_cut_num + 1 && num < 20)
    {
        // 计算当前选中天线范围左侧和右侧的第一个未选中天线
        last_index = (tp_index[0] - 1 + size) % size;              // 左侧扩展候选
        next_index = (tp_index[size2 - 1] + 1) % size;             // 右侧扩展候选
        // 在已排序的列表中查找，优先选择幅度更大的那一侧
        for (int i = 1; i < size; i++)
        {
            if (index[i] == last_index)  // 左侧天线幅度更大
            {
                tp_index.insert(tp_index.begin() + 0, index[i]); // 插入到列表头部
                break;
            }
            if (index[i] == next_index)  // 右侧天线幅度更大
            {
                tp_index.emplace_back(index[i]); // 追加到列表尾部
                break;
            }
        }

        size2 = tp_index.size();
        num++;
        for (int i = 0; i < size2; i++)
        {
            Log("%d,", tp_index[i]);
        }
        Log("\n");
    }
    // use_cut = tp_index;

    int mid = (int)use_cut_num / 2;
    int per = 360 / size; // 每个天线覆盖的角度宽度
    // 根据选中天线数量奇偶性计算搜索范围
    if ((use_cut_num + 1) % 2 == 1)  // 选中奇数个天线
    {
        // int midAnten = tp_index[mid];
        //  cout << "midAnten=" << midAnten << endl;
        startAngle = (tp_index[mid - 1]) * per;    // 中间左侧天线的起始角度
        endAngle = (tp_index[mid + 1]) * per;      // 中间右侧天线的结束角度
        endAngle = Round360(endAngle);
    }
    else  // 选中偶数个天线
    {
        int first = tp_index[mid];                 // 中间偏左的天线
        if (mid + 1 > use_cut_num)
        {
            mid = size2 - 2;
        }
        int sec = tp_index[mid + 1];               // 中间偏右的天线
        startAngle = first * per - per / 2;         // 两天线中间位置 - 半间距
        endAngle = sec * per + per / 2;             // 两天线中间位置 + 半间距
    }
    // 跨越 0 度边界处理
    if (startAngle > endAngle)
    {
        startAngle = startAngle - 360;
    }
    // 留出 10 度安全余量，防止边界误差
    startAngle = startAngle + 10;
    endAngle = endAngle - 10;
    use_cut.resize(size2);
    Log("direct ant use ant:{");
    for (int i = 0; i < (int)tp_index.size(); i++)
    {
        use_cut[i] = tp_index[i] + 1; // 转换为 1-based 天线编号
        Log("%d,  ", use_cut[i]);
    }
    Log("}");
    Log("  startAngle=%d,endAngle=%d\n", startAngle, endAngle);
}

// ==================== 以下为当前项目组(Spoofing)与Suppress组均未使用的函数，暂时注释保留 ====================
// /**
//  * @brief 从相位差数据还原各天线的绝对相位（按天线对顺序推导版本）
//  *
//  * 注意：假设天线对按顺序排列（如 [1,2], [2,3], [3,4], ...），便于顺序推导。
//  * 假设第一个天线对中的第一个天线的相位为 0（参考天线），然后按顺序推导其他天线的相位。
//  *
//  * 推导规则：
//  *   若 ant2 不是参考天线（first_antenna），则：
//  *     phase[ant2] = phase[ant1] - diff(ant1, ant2)
//  *   因为 diff(ant1, ant2) = phase(ant1) - phase(ant2)，所以 phase(ant2) = phase(ant1) - diff
//  *
//  * 潜在问题：
//  *   - 未处理切刀顺序最大序号大于天线个数的异常
//  *   - 未处理切刀顺序不是按天线序号连续的情况（如 [1 2], [3 4] 跳过了天线 2->3 的关系）
//  *
//  * @param cutSequence 天线对序列
//  * @param phaseDiff 相位差数组 [输入] 天线对间差值 / [输出] 各天线的绝对相位（弧度）
//  * @param antennNUm 天线总数
//  */
// void ArithmeticDoa::getAntennaPhase(vector<vector<int>> &cutSequence, vector<double> &phaseDiff, int antennNUm)
// { // 注意：异常未处理：切刀顺序最大序号大于天线个数；切刀顺序不是按天线序号的顺序来，例如【1 2】【3 4】
//     vector<double> tp_phs;
//     tp_phs.clear();
//     tp_phs.resize(antennNUm);
//     // 选取第一个天线对中的第一个天线作为参考天线，设其相位为 0
//     int first_antenna = cutSequence[0][0] - 1;
//     tp_phs[first_antenna] = 0.0;
//     int ant1 = 0;
//     int ant2 = 0;
//     for (int i = 0; i < (int)phaseDiff.size(); i++)
//     {
//         ant1 = cutSequence[i][0] - 1; // 天线对的第一个天线（0-based）
//         ant2 = cutSequence[i][1] - 1; // 天线对的第二个天线（0-based）
//         if (ant2 != first_antenna) // 参考天线本身不需要重新计算
//         {
//             // 从已知的 ant1 相位推导 ant2 的相位
//             // diff = phase(ant1) - phase(ant2)  =>  phase(ant2) = phase(ant1) - diff
//             tp_phs[ant2] = tp_phs[ant1] - phaseDiff[i];
//         }
//
//         Log("ant1=%d,ant2=%d,phs=%.2f\n", ant1 + 1, ant2 + 1, tp_phs[ant2] * 180 / m_PI);
//     }
//     phaseDiff.clear();
//     phaseDiff = tp_phs;
//     Log("get all per antenna phase sucess...\n");
// }
/**
 * @brief 从相位差数据还原指定天线的绝对相位（图连通性推导版本）
 *
 * 算法原理：
 *   将天线视为图中的节点，天线对之间的相位差视为有向边。
 *   以 use_ant[0] 为根节点（相位设为 0），使用广度优先搜索（BFS）的方式，
 *   逐步推导其余天线的绝对相位。
 *
 * 推导规则：
 *   - 若在 cutSequence 中找到 (target, known) 天线对，且已知 known 的相位：
 *     target_phase = known_phase + diff(target, known)
 *     因为 diff(target, known) = phase(target) - phase(known)
 *   - 若在 cutSequence 中找到 (known, target) 天线对：
 *     target_phase = known_phase - diff(known, target)
 *     因为 diff(known, target) = phase(known) - phase(target)
 *
 * 该版本适用于任意天线对排列（不要求连续），只要从参考天线出发能通过
 * 已知相位差关系到达目标天线即可。若某天线无法从参考天线推导到（图不连通），
 * 则返回 -1 表示失败。
 *
 * @param use_ant 需要求解相位的天线序号列表（从 1 开始编号），use_ant[0] 为参考天线
 * @param antennNUm 天线总数
 * @param cutSequence 天线对序列（所有可用的相位差关系）
 * @param phaseDiff 相位差数组 [输入] 各天线对间差值 / [输出] 各天线绝对相位（弧度）
 * @return 0 成功，所有天线相位均已推导出来；-1 失败，存在孤立天线
 */
int ArithmeticDoa::getAntennaPhase(const vector<int> use_ant, const int antennNUm, vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
{ // use_ant 用于测向的天线  cutSequence相位差对应的天线序号,该函数只用于相关干涉仪算法，所以每个相位差只涉及两个天线
    Log("use doa antenna:{");
    for (size_t i = 0; i < use_ant.size(); i++)
    {
        Log("%d,", use_ant[i]);
    }
    Log("}\n");
    int firstA = use_ant[0] - 1; // 参考天线（0-based），设其相位为 0
    // int antennNUm = (int)use_ant.size();
    vector<double> tp_phs;
    tp_phs.clear();
    tp_phs.resize(antennNUm);
    tp_phs[firstA] = 0; // 参考天线相位 = 0
    bool flg = false;
    int ant = 0;
    double tp_diff = 0.0;
    int tp_ant = 0;
    // 对于待求相位的每个天线（跳过参考天线 use_ant[0]）
    for (size_t j = 1; j < use_ant.size(); j++)
    {
        tp_ant = use_ant[j]; // 当前需要求解的天线编号
        flg = false;
        // 在已求解的天线中查找是否有连接关系
        for (size_t k = 0; k < j; k++)
        {
            ant = use_ant[k]; // 已求解相位的天线编号
            // 遍历所有天线对，寻找连接 tp_ant 和 ant 的相位差关系
            for (size_t i = 0; i < cutSequence.size(); i++)
            {
                // 情况 1：cutSequence 顺序为 (tp_ant, ant)，即 diff = phase(tp_ant) - phase(ant)
                if (tp_ant == cutSequence[i][0] && ant == cutSequence[i][1])
                {
                    flg = true;
                    tp_diff = tp_phs[ant - 1] + phaseDiff[i]; // phase(tp_ant) = phase(ant) + diff
                    break;
                }
                // 情况 2：cutSequence 顺序为 (ant, tp_ant)，即 diff = phase(ant) - phase(tp_ant)
                if (tp_ant == cutSequence[i][1] && ant == cutSequence[i][0])
                {
                    flg = true;
                    tp_diff = tp_phs[ant - 1] - phaseDiff[i]; // phase(tp_ant) = phase(ant) - diff
                    break;
                }
            }
        }

        if (flg)
        {
            tp_phs[tp_ant - 1] = tp_diff; // 记录推导出的相位
        }
        else
        {
            return -1; // 图不连通：当前天线无法从已知天线推导得到
        }
    }
    phaseDiff.clear();
    phaseDiff = tp_phs; // 返回各天线的绝对相位
    Log("per antenna phase:\n");
    for (int i = 0; i < phaseDiff.size(); i++)
    {
        double tp2 = phaseDiff[i] * 180 / m_PI;
        Log("---ant = %d,  phse=%.2f\n", i + 1, Round3600(tp2));
    }
    return 0;
}
/**
 * @brief 根据指定天线子集，生成所有天线对之间的相位差（C(N,2) 组合）
 *
 * 从各天线的绝对相位出发（phaseDiff 输入应为绝对相位而非相位差），
 * 对 use_cut 中指定的天线子集，生成全部两两组合的相位差：
 *   diff(ant_i, ant_j) = phase(ant_i) - phase(ant_j)
 * 组合数为 C(N,2) = N*(N-1)/2，其中 N = use_cut 的大小。
 *
 * @param use_cut 参与的天线序号列表（1-based）
 * @param cutSequence 输出的天线对序列（所有两两组合）
 * @param phaseDiff [输入] 各天线的绝对相位 / [输出] 天线对间的相位差
 */
void ArithmeticDoa::getUsePhaseDiffAll(const vector<int> use_cut, vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
{

    vector<double> result_phs;
    result_phs.clear();

    int size = (int)use_cut.size();
    int num = size * (size - 1) / 2; // 组合数 C(N,2)
    vectorResize(cutSequence, num, 2);
    result_phs.resize(num);
    int num1 = 0;
    int num2 = 0;
    num = 0;
    // 双重循环生成所有无重复的天线对 (i, j)，其中 i < j
    for (int i = 0; i < size - 1; i++)
    {
        num1 = use_cut[i];
        for (int j = i + 1; j < size; j++)
        {

            num2 = use_cut[j];
            cutSequence[num][0] = num1;        // 天线对中的第一个天线
            cutSequence[num][1] = num2;        // 天线对中的第二个天线
            result_phs[num] = phaseDiff[num1 - 1] - phaseDiff[num2 - 1]; // diff = phase(i) - phase(j)
            num++;
        }
    }
    phaseDiff.clear();
    phaseDiff = result_phs;
    for (int i = 0; i < phaseDiff.size(); i++)
    {
        double tp2 = phaseDiff[i] * 180 / m_PI;
        Log("---ant = %d a1=%d,a2=%d, phse=%.2f\n", i + 1, cutSequence[i][0], cutSequence[i][1], Round3600(tp2));
    }
}

// /**
//  * @brief 对所有天线，生成全部两两之间的相位差（全排列版本）
//  *
//  * 与 getUsePhaseDiffAll 的区别：本函数使用所有天线（默认从 1 到 N），
//  * phaseDiff 输入为各天线的绝对相位（按天线编号 1..N 排列），
//  * 输出为全部 C(N,2) 对天线之间的相位差。
//  *
//  * @param cutSequence 输出的天线对序列（全部 C(N,2) 组合）
//  * @param phaseDiff [输入] 所有天线的绝对相位 / [输出] 全部天线对间的相位差
//  */
// void ArithmeticDoa::getPhaseDiffAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
// {
//     vector<double> result_phs;
//     result_phs.clear();
//
//     int size = (int)phaseDiff.size(); // 天线总数
//     int num = size * (size - 1) / 2;  // 全部组合数 C(N,2)
//     vectorResize(cutSequence, num, 2);
//     result_phs.resize(num);
//     int num1 = 0;
//     int num2 = 0;
//     num = 0;
//     for (int i = 0; i < size - 1; i++)
//     {
//         num1 = i + 1; // 天线 i+1 的编号
//         for (int j = i + 1; j < size; j++)
//         {
//
//             num2 = j + 1; // 天线 j+1 的编号
//             cutSequence[num][0] = num1;
//             cutSequence[num][1] = num2;
//             result_phs[num] = phaseDiff[i] - phaseDiff[j]; // diff = phase(i) - phase(j)
//             num++;
//         }
//     }
//     phaseDiff.clear();
//     phaseDiff = result_phs;
//     Log("get all phase diff...\n ");
// }

// void ArithmeticDoa::getUsePhaseDiffAll(const vector<int> use_cut, vector<vector<int>> &cutSequence, vector<double> &phaseDiff)
// {
//     vector<double> tp_phs_diff;
//     vector<vector<int>> tp_cutSequence;
//     int size = (int)use_cut.size();
//     int num = size * (size - 1) / 2;

//     vectorResize(tp_cutSequence, num, 2);
//     tp_phs_diff.resize(num);
//     int num1 = 0;
//     int num2 = 0;
//     bool flg1 = false;
//     bool flg2 = false;
//     num = 0;
//     for (int i = 0; i < (int)phaseDiff.size(); i++)
//     {
//         flg1 = false;
//         flg2 = false;
//         num1 = cutSequence[i][0];
//         num2 = cutSequence[i][1];
//         for (int j = 0; j < size; j++)
//         {
//             if (num1 == use_cut[j])
//             {
//                 flg1 = true;
//             }
//             if (num2 == use_cut[j])
//             {
//                 flg2 = true;
//             }
//         }
//         if (flg1 && flg2)
//         {
//             tp_cutSequence[num][0] = num1;
//             tp_cutSequence[num][1] = num2;
//             tp_phs_diff[num] = phaseDiff[i];
//             num++;
//         }
//     }
//     phaseDiff.clear();
//     phaseDiff = tp_phs_diff;
//     cutSequence.clear();
//     cutSequence = tp_cutSequence;
//     Log("get use doa phase diff:\n");
//     for (size_t i = 0; i < phaseDiff.size(); i++)
//     {
//         Log("%d,%d: phase diff= %.2f\n", cutSequence[i][0], cutSequence[i][1], phaseDiff[i]);
//     }
// }

/**
 * @brief 利用传递性原理，从已知相位差组合生成所有可能的相位差
 *
 * 算法原理（相位差传递性 -- 图的传递闭包）：
 *   将天线视为图中的节点，相位差视为有向边上的权重。
 *   利用以下传递规则，从已知的边推导出新的边：
 *
 *   规则 1 -- 路径连接 (A->B->C)：
 *     diff(A,C) = diff(A,B) + diff(B,C)
 *     对应：cutSequence[i][1] == cutSequence[j][0] (B==B)
 *
 *   规则 2 -- 反向路径连接 (C->B->A)：
 *     diff(C,A) = diff(C,B) + diff(B,A)
 *     对应：cutSequence[i][0] == cutSequence[j][1] (B==B)
 *
 *   规则 3 -- 同起点 (A->B, A->C)：
 *     diff(B,C) = diff(A,C) - diff(A,B)
 *     对应：cutSequence[i][0] == cutSequence[j][0] (A==A)
 *
 *   规则 4 -- 同终点 (B->A, C->A)：
 *     diff(B,C) = diff(B,A) - diff(C,A)
 *     对应：cutSequence[i][1] == cutSequence[j][1] (A==A)
 *
 * 算法采用迭代逼近策略：每次迭代扫描当前所有边对，尝试推导新边，
 * 若成功推导出新边则加入图中，继续下一轮迭代，直到无新边可推导或达到最大迭代次数。
 * 类比于 Floyd-Warshall 算法的思想，通过局部传递性逐步构建全连通图。
 *
 * 该函数增强了算法的鲁棒性：即使数据缺失了某些天线对的相位差，
 * 也能通过已知相位差推导出来，确保有足够的相位差信息用于测向。
 *
 * @param cutSequence [输入] 已知天线对序列 / [输出] 扩展后的天线对序列（包含推导出的新对）
 * @param phaseDiff [输入] 已知相位差数组 / [输出] 扩展后的相位差数组
 */
void ArithmeticDoa::setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff) // 根据天线序号，组合相位差
{
    // 使用嵌套 map 存储天线对有向图的邻接表：tp_antenna_map[ant1][ant2] = diff(ant1, ant2)
    map<int, map<int, double>> tp_antenna_map;
    tp_antenna_map.clear();
    map<int, double> tp1;
    // 初始化邻接表：将输入的已知天线对及其相位差加入图中
    for (int i = 0; i < (int)phaseDiff.size(); i++)
    {
        tp1.clear();
        int ant1 = cutSequence[i][0];
        if (tp_antenna_map.find(ant1) != tp_antenna_map.end())
        {
            tp1 = tp_antenna_map[ant1];
        }
        tp1[cutSequence[i][1]] = phaseDiff[i];
        tp_antenna_map[ant1] = tp1;
    }
    int count = 1; // 本轮新推导出的边数
    int size = 0;
    int start_index = 0;
    int tp_ant1 = 0;
    int tp_ant2 = 0;
    double tp2_diff = 0.0;
    // 迭代推导新边，直到无新边可推导或超过最大迭代次数（50 次防止异常情况死循环）
    while (0 != count && count < 50)
    {
        count = 0;
        size = (int)phaseDiff.size();
        // 双重循环：遍历所有已存在的边对 (i, j)，尝试推导新边
        for (int i = 0; i < size; i++)
        {
            if (start_index <= i)
            {
                start_index = i + 1;
            }

            for (int j = start_index; j < size; j++)
            {
                tp1.clear();
                // 规则 1：A->B 和 B->C => A->C
                if (cutSequence[i][1] == cutSequence[j][0])
                {
                    tp_ant1 = cutSequence[i][0];  // A
                    tp_ant2 = cutSequence[j][1];  // C
                    tp2_diff = phaseDiff[i] + phaseDiff[j]; // diff(A,C) = diff(A,B) + diff(B,C)
                }
                // 规则 2：B->C 和 A->B => C->A (实际上是 C->B + B->A = C->A 的另一个方向)
                if (cutSequence[i][0] == cutSequence[j][1])
                {
                    tp_ant1 = cutSequence[j][0];  // C
                    tp_ant2 = cutSequence[i][1];  // A
                    tp2_diff = phaseDiff[i] + phaseDiff[j]; // diff(C,A) = diff(C,B) + diff(B,A)
                }
                // 规则 3：A->B 和 A->C => B->C
                if (cutSequence[i][0] == cutSequence[j][0])
                {
                    tp_ant1 = cutSequence[i][1];  // B
                    tp_ant2 = cutSequence[j][1];  // C
                    tp2_diff = phaseDiff[j] - phaseDiff[i]; // diff(B,C) = diff(A,C) - diff(A,B)
                }
                // 规则 4：B->A 和 C->A => B->C
                if (cutSequence[i][1] == cutSequence[j][1])
                {
                    tp_ant1 = cutSequence[i][0];  // B
                    tp_ant2 = cutSequence[j][0];  // C
                    tp2_diff = phaseDiff[i] - phaseDiff[j]; // diff(B,C) = diff(B,A) - diff(C,A)
                }
                // 跳过自环边（起点和终点相同）
                if (tp_ant1 == tp_ant2)
                {
                    continue;
                }
                // 检查是否已经存在 (tp_ant2, tp_ant1) 的反向边（邻接表中有记录），若存在则跳过
                if (tp_antenna_map.find(tp_ant2) != tp_antenna_map.end())
                {
                    tp1 = tp_antenna_map[tp_ant2];
                    if (tp1.find(tp_ant1) != tp1.end())
                    {
                        continue;
                    }
                    tp1.clear();
                }
                // 检查是否已经存在 (tp_ant1, tp_ant2) 的正向边，若存在则跳过
                if (tp_antenna_map.find(tp_ant1) != tp_antenna_map.end())
                {
                    tp1 = tp_antenna_map[tp_ant1];
                    if (tp1.find(tp_ant2) != tp1.end())
                    {
                        continue;
                    }
                }

                // 新边不存在于邻接表中，添加新边
                tp1[tp_ant2] = tp2_diff;
                tp_antenna_map[tp_ant1] = tp1;
                vector<int> tp2;
                tp2.clear();
                tp2.resize(2);

                tp2[0] = tp_ant1;
                tp2[1] = tp_ant2;
                cutSequence.emplace_back(tp2);   // 添加新天线对
                phaseDiff.emplace_back(tp2_diff); // 添加对应的相位差
                ++count;
            }
        }
        start_index = size; // 下一轮扫描从上一轮结束位置开始，避免重复检查已检查过的对
    }
}


/**
 * @brief 从配置文件中读取频率-阵列半径映射数据
 *
 * 文件格式（每两行为一组）：
 *   第 1 行：频率区间，格式为 "起始MHz-结束MHz" 或 "起始MHz~结束MHz"
 *   第 2 行：对应的阵列半径（米）
 * 例如：
 *   100-200
 *   0.15
 *   200-400
 *   0.12
 * 表示 100~200MHz 频段使用 0.15m 阵列半径，200~400MHz 使用 0.12m 阵列半径。
 *
 * 不同频率对应不同阵列半径的原因：天线阵列通常在多个频段工作，
 * 各频段的天线互耦效应不同，导致等效阵列孔径（半径）有所变化。
 *
 * @param adr 配置文件路径
 * @param mR 输出的频率-半径映射数组（每个元素记录一个频段及其对应的半径）
 * @return 0 成功；-1 文件读取失败；-2 文件格式错误（频率行无法按 '-' 或 '~' 分割）
 */
int ArithmeticDoa::getRData(const string adr, vector<Rs> &mR){
    vector<Rs>().swap(mR);
    vector<string> tpVec;
    int n = readFile(adr, tpVec);

    if (0 != n){
        Log("***error:read r adr is flase-----%s\n", adr.c_str());
        return -1;
    }

    // 每次读取两行：第一行为频率区间，第二行为半径值
    for (int i = 0; i < tpVec.size(); i += 2){

        string &a = tpVec[i];
        vector<string> a2;

        // 尝试用 '-' 分割频率区间（格式：起始MHz-结束MHz）
        split(a2, a, '-');

        if (a2.size() == 1){
            a2.clear();
            // 尝试用 '~' 分割频率区间（格式：起始MHz~结束MHz）
            split(a2, a, '~');
        }

        if (a2.size() == 1){
            return -2; // 无法分割频率区间，格式错误
        }

        // 将 MHz 转换为 Hz（乘以 1e6）
        double starF1 = stod(a2[0]) * 1e6; // 起始频率（Hz）
        double endF1 = stod(a2[1]) * 1e6;  // 结束频率（Hz）

        Rs tmpR;
        tmpR.i_starF = starF1;
        tmpR.i_endF = endF1;
        tmpR.i_r = stod(tpVec[i + 1]); // 阵列半径（米）
        mR.emplace_back(tmpR);
    }

    tpVec.clear();
    vector<string>().swap(tpVec);
    Log("set fre and R is sucess...\n");
    return 0;
}

/**
 * @brief 利用相关干涉仪 + 虚拟阵元进行二次测向，解决相位模糊（"跳半周"）问题
 *
 * ===== 问题背景 =====
 * 当阵元间距 d > lambda/2 时，相位差可能超过 [-pi, pi] 的范围。
 * 由于实际测量只能得到 [-pi, pi] 范围内的相位（相位缠绕，Phase Wrapping），
 * 导致理论相位差和实测相位差的匹配出现"多峰"现象：在伪谱中出现多个峰值，
 * 难以判断哪个峰值对应真实来波方向。
 *
 * 例如：实际相位差为 200 度（超过 180 度），测量值会被折叠为 -160 度，
 * 从而与某个错误角度的理论值匹配，产生"镜像峰"。
 *
 * ===== 算法策略 =====
 * 1. 先用相关干涉仪粗测向，得到伪谱 diff 及其多个峰
 * 2. 对各候选峰，选取区分度最好的天线对（cos 值最小的，即理论相位差差异最大的）
 * 3. 用虚拟阵列扩展缩短等效基线，消除相位模糊
 * 4. 对比实际相位差与各候选角度下虚拟理论相位差的相关度，选出最佳匹配
 *
 * ===== 详细步骤 =====
 *
 * 步骤 1 -- 伪谱预处理（降噪）：
 *   将伪谱中小于 90 的值置零，过滤低相关噪声，仅保留显著的峰。
 *
 * 步骤 2 -- 寻峰：
 *   调用 findPeaks 在伪谱中寻找峰值。若只有 0 或 1 个峰，说明无模糊问题，
 *   直接返回（第一次测向结果即可信）。
 *
 * 步骤 3 -- 提取候选角度的理论相位差：
 *   取出前两个峰值角度 index[0] 和 index[1]（通常最强的两个候选），
 *   分别计算这两个角度下各天线对的理论相位差 tp_theory_diff1 和 tp_theory_diff2。
 *
 * 步骤 4 -- 选择鉴别基（Discriminative Baselines）：
 *   对每个天线对 j，计算两个候选角度下理论相位差的余弦值：
 *     cos_diff[j] = cos(tp_theory_diff1[j] - tp_theory_diff2[j])
 *   cos 值越小，说明该天线对在两个候选角度下的相位差差异越大，
 *   对该天线对而言两个角度更容易区分（鉴别力更强）。
 *   选取 cos 值最小的 num=3 个天线对作为"鉴别基"。
 *
 * 步骤 5 -- 虚拟阵列扩展：
 *   对选出的鉴别基天线对和对应的相位差，分别进行三次虚拟阵列扩展：
 *     - 用实际测量相位差进行虚拟扩展 (antnna, phase_diff)
 *     - 用候选角度1的理论相位差进行虚拟扩展 (antnna1, tp_theory_diff3)
 *     - 用候选角度2的理论相位差进行虚拟扩展 (antnna2, tp_theory_diff4)
 *   虚拟扩展改变了等效基线长度，使得原本模糊的相位差变得可区分。
 *
 * 步骤 6 -- 对比判定：
 *   计算实际虚拟相位差与两个候选角度下虚拟理论相位差的余弦相关度之和：
 *     sum1 = sum_i cos(phase_diff[i] - tp_theory_diff3[i])  // 候选角度1
 *     sum2 = sum_i cos(phase_diff[i] - tp_theory_diff4[i])  // 候选角度2
 *   取 sum 值更大的角度作为最终结果（相关度更高 = 匹配更好）。
 *
 * 步骤 7 -- 质量评估：
 *   基于最终选定的角度重新计算理论伪谱和测向质量。
 *
 * @param phaseTheory 理论相位矩阵
 * @param virMultiple 虚拟倍率（通常 < 1，用于缩短等效基线以消除模糊）
 * @param diff 第一次粗测向得到的伪谱数组（360 点），作为输入同时也会被修改（< 90 的值被清零）
 * @param data 实测数据（天线对和相位差）
 * @param angle 输出的最终测向角度（度）
 * @param quality 输出的测向质量（0-100）
 */
void ArithmeticDoa::calSecondDoaByVirInterf(const vector<vector<double>> phaseTheory, const double virMultiple, vector<double> diff, InterferInfo data, double &angle, double &quality) // 利用相关干涉仪+虚拟矩阵进行二次测向
{

    // ===== 步骤 1：伪谱预处理，滤除低相关噪声 =====
    for (int i = 0; i < (int)diff.size(); i++)
    {
        if (diff[i] < 90) // 相关度小于 90 的角度视为噪声
        {
            diff[i] = 0.0;
        }
    }
    // ===== 步骤 2：在伪谱中寻峰 =====
    vector<int> index;
    vector<double> vaules;
    findPeaks(diff, index, vaules); // 获取伪谱的峰值位置
    if (index.empty() || index.size() == 1) // 无峰或单峰，无需解模糊
    {
        return;
    }

    int size = data.i_Phase_Len;
    vector<double> cos_diff;
    vector<int> diff_index;
    cos_diff.resize(size);
    diff_index.resize(size);

    vector<double> tp_theory_diff1; // 候选角度 1 下各天线对的理论相位差
    vector<double> tp_theory_diff2; // 候选角度 2 下各天线对的理论相位差
    tp_theory_diff1.resize(size);
    tp_theory_diff2.resize(size);
    // ===== 步骤 3：计算两个候选角度下的理论相位差 =====
    // Log("angle1=%d,angle2=%d\n", index[0], index[1]);
    for (int j = 0; j < size; j++)
    {
        // 理论相位差 = 天线对中天线1的理论相位 - 天线对中天线2的理论相位
        tp_theory_diff1[j] = phaseTheory[index[0]][data.i_AntennaSq[j][0] - 1] - phaseTheory[index[0]][data.i_AntennaSq[j][1] - 1];
        tp_theory_diff2[j] = phaseTheory[index[1]][data.i_AntennaSq[j][0] - 1] - phaseTheory[index[1]][data.i_AntennaSq[j][1] - 1];
        // 计算两个候选角度下理论相位差的余弦值（cos 值越小，差异越大，区分度越好）
        cos_diff[j] = cos(tp_theory_diff1[j] - tp_theory_diff2[j]);
        // Log("cos_diff:i=%d,diff=%.2f\n", j, cos_diff[j]);
        diff_index[j] = j;
    }
    // ===== 步骤 4：选择鉴别力最强的天线对（鉴别基） =====
    // 按 cos_diff 从小到大排序，cos 值小的天线对鉴别力最强
    std::sort(diff_index.begin(), diff_index.end(), [&](int i, int j)
              { return cos_diff[i] < cos_diff[j]; });

    int num = 3; // 选择 3 个最具鉴别力的天线对
    //(int)size / 2;
    vector<vector<int>> antnna;
    antnna.resize(num);
    vector<double> phase_diff;
    phase_diff.resize(num);
    vector<double> tp_theory_diff3; // 鉴别基在候选角度 1 下的理论相位差
    vector<double> tp_theory_diff4; // 鉴别基在候选角度 2 下的理论相位差
    tp_theory_diff3.resize(num);
    tp_theory_diff4.resize(num);
    for (int i = 0; i < num; i++)
    {
        antnna[i].resize(2);
        antnna[i][0] = data.i_AntennaSq[diff_index[i]][0]; // 天线对中的天线1
        antnna[i][1] = data.i_AntennaSq[diff_index[i]][1]; // 天线对中的天线2
        phase_diff[i] = data.i_Phase_Diff[diff_index[i]];  // 对应的实测相位差
        tp_theory_diff3[i] = tp_theory_diff1[diff_index[i]]; // 候选角度1下的理论相位差
        tp_theory_diff4[i] = tp_theory_diff2[diff_index[i]]; // 候选角度2下的理论相位差
        //   Log("sort cos_diff:diff_index=%d,antanna1 = %d,antanna2=%d,phase_diff = %.2f\n", diff_index[i], antnna[i][0], antnna[i][1], phase_diff[i]);
    }
    // ===== 步骤 5：虚拟阵列扩展（三次并行扩展） =====
    vector<vector<int>> antnna1;
    vector<vector<int>> antnna2;
    antnna1 = antnna; // 用于候选角度 1 的虚拟扩展
    antnna2 = antnna; // 用于候选角度 2 的虚拟扩展
    getVirtual(virMultiple, antnna, phase_diff);      // 实测数据的虚拟扩展
    getVirtual(virMultiple, antnna1, tp_theory_diff3); // 候选角度 1 理论值的虚拟扩展
    getVirtual(virMultiple, antnna2, tp_theory_diff4); // 候选角度 2 理论值的虚拟扩展
    // ===== 步骤 6：对比相关度，确定最终角度 =====
    double sum1 = 0.0;
    double sum2 = 0.0;
    for (int i = 0; i < (int)antnna.size(); i++)
    {
        // 计算实测相位差与候选角度 1 虚拟理论相位差的余弦相关度
        sum1 = sum1 + cos(phase_diff[i] - tp_theory_diff3[i]);
        // 计算实测相位差与候选角度 2 虚拟理论相位差的余弦相关度
        sum2 = sum2 + cos(phase_diff[i] - tp_theory_diff4[i]);
        // Log("virtual:phase_diff = %.2f,tp_theory_diff3 = %.2f,tp_theory_diff4 = %.2f\n", phase_diff[i], tp_theory_diff3[i], tp_theory_diff4[i]);
    }
    // 选择相关度更高的角度
    if (sum1 < sum2) // 候选角度 2 匹配更好
    {
        angle = index[1];
        // ===== 步骤 7：用选定的角度重新计算质量 =====
        for (int j = 0; j < size; j++)
        {
            data.i_Phase_Diff[j] = tp_theory_diff2[j]; // 更新相位差为选定角度的理论值
        }
        vector<double> tp_theory_Doa_diff;
        calPseudoByInterfer(phaseTheory, data, tp_theory_Doa_diff); // 计算理论伪谱
        quality = getDoaMass(tp_theory_Doa_diff, diff);            // 计算测向质量
    }
}
/**
 * @brief 计算理论伪谱（相关干涉仪版本）-- 用于测向质量评估
 *
 * 伪谱定义：
 *   对于搜索范围内的每个角度 ang，计算该角度下各天线对的理论相位差与
 *   "已知最优匹配相位差"（存储在 data.i_Phase_Diff 中）的余弦相关度之和。
 *   diff[ang] = sum_i cos(theoryPhaseDiff_i(ang) - data.i_Phase_Diff[i])
 *
 * 与 calInterfer 的区别：
 *   - calInterfer 遍历所有角度找出最佳匹配，其过程产生"实际伪谱"
 *   - calPseudoByInterfer 使用已确定的最佳匹配相位差作为"理论值"，
 *     重新计算一遍伪谱，得到的称为"理论伪谱"
 *   - 这两个伪谱的相关系数即为测向质量（getDoaMass）
 *
 * 为什么要分别计算两个伪谱：
 *   实际伪谱反映的是"实测数据与各角度理论值的匹配程度"，
 *   理论伪谱反映的是"最佳角度理论值与各角度理论值的匹配程度"（理想情况）。
 *   两者越相似，说明实测数据越接近理论模型，测向结果越可靠。
 *
 * @param phaseTheory 理论相位矩阵
 * @param data 测向数据，其中 i_Phase_Diff 应已填充为最佳匹配角度下的理论相位差
 * @param diff 输出的理论伪谱数组（360 点），diff[ang] = 角度 ang 处的理论相关度累加值
 */
void ArithmeticDoa::calPseudoByInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, vector<double> &diff)
{
    int startAngle = (int)data.i_Start;
    int endAngle = (int)data.i_End;
    int size = data.i_Phase_Len;
    diff.clear();
    vector<double>().swap(diff);
    diff.resize(360);
    std::fill(diff.begin(), diff.end(), 0.0);
    int tp_ang = 0;
    // 在指定的搜索范围内遍历所有角度
    for (int ang = startAngle; ang < endAngle + 1; ang++)
    {
        double sumDiff = 0.0;
        tp_ang = Round360(ang); // 将角度归一化到 [0, 360)
        // 计算该角度下各天线对的余弦相关度
        for (int i = 0; i < size; i++)
        {
            int antn1 = data.i_AntennaSq[i][0];
            int antn2 = data.i_AntennaSq[i][1];
            // 该角度下天线对的理论相位差
            double theoryPhase = phaseTheory[tp_ang][antn1 - 1] - phaseTheory[tp_ang][antn2 - 1];
            // 余弦相关度：cos(0)=1 表示完美匹配，cos(pi)=-1 表示反相
            sumDiff = sumDiff + cos(theoryPhase - data.i_Phase_Diff[i]);
        }
        // 存储该角度的相关度累加值
        diff[tp_ang] = sumDiff;
    }
}

/**
 * @brief 相关干涉仪测向主算法 -- 通过相位差余弦相关匹配确定来波方向
 *
 * ===== 算法原理 =====
 * 相关干涉仪测向法是一种经典的无线电测向算法，其核心思想是：
 * 将实测的天线阵列相位差向量与预先计算的理论相位差向量进行相关匹配，
 * 相关度最高的角度即为来波方向的估计。
 *
 * ===== 匹配度量 =====
 * 使用余弦相关度作为匹配度量函数：
 *   correlation(ang) = sum_i cos( theory_phase_diff_i(ang) - actual_phase_diff_i )
 *
 * 使用余弦函数而非直接相减的原因：
 *   1. 相位具有 2*pi 周期性，直接相减无法区分相差 2*pi 整数倍的情况
 *   2. cos(delta) 对相位差 delta 是周期函数，天然处理了相位缠绕问题
 *   3. cos(0) = 1（完全匹配），cos(pi) = -1（完全反相），cos(pi/2) = 0（无相关）
 *   4. cos 函数在 [-pi/2, pi/2] 区间是单调递减的，具有良好的梯度特性
 *
 * ===== 算法步骤 =====
 *
 * 步骤 1 -- 遍历搜索：
 *   在数据指定的搜索范围 [startAngle, endAngle] 内，以 1 度为步长遍历所有候选角度。
 *   对每个候选角度 ang，计算该角度下各天线对的理论相位差。
 *
 * 步骤 2 -- 计算相关度：
 *   sumDiff = sum_i cos(theoryPhaseDiff_i(ang) - actualPhaseDiff_i)
 *   sumDiff 最大值为 N（天线对数），此时所有天线对的实测值与理论值完全一致。
 *   sumDiff 最小值为 -N，此时所有天线对完全反相。
 *
 * 步骤 3 -- 峰值搜索：
 *   在所有候选角度中，找到 sumDiff 最大的角度 ang_val 作为 DOA 估计结果。
 *
 * 步骤 4 -- 质量评估：
 *   基于最优角度的理论相位差，构造"理论伪谱"；基于实测值的遍历结果构造"实际伪谱"。
 *   计算两个伪谱的归一化互相关系数作为测向质量：
 *     quality = (corr + 1) / 2 * 100，其中 corr 为互相关系数。
 *   quality 反映最优角度的理想匹配曲线与实际匹配曲线的吻合程度。
 *
 * 步骤 5 -- 输出伪谱：
 *   diff2 输出加权伪谱：diff2[ang] = ((sumDiff/N + 1) / 2) * quality
 *   归一化到 [0, 1] 后再乘以质量值，用于后续的二次测向（解模糊）等处理。
 *
 * ===== 参数说明 =====
 * @param phaseTheory 理论相位矩阵，phaseTheory[ang][i] = 角度 ang（0-359度）处天线 i 的理论相位（弧度）
 *                    该矩阵通常由 calPhaseTheory 函数根据阵列参数预计算得到
 * @param data 实测数据，包含：
 *             - i_Start / i_End：角度搜索范围
 *             - i_Phase_Len：有效天线对数
 *             - i_AntennaSq：天线对序号（天线从 1 开始编号）
 *             - i_Phase_Diff：实测相位差（弧度）
 * @param angle 输出的测向角度（度），已归一化到 [0, 360) 范围
 * @param quality 输出的测向质量（0-100），反映结果可靠性
 * @param diff2 输出的加权伪谱数组（360 点），diff2[ang] = 归一化相关度 * quality
 */
void ArithmeticDoa::calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2) // 利用相关干涉仪测向
{
    diff2.resize(360);
    vector<double> diff; // 实际伪谱（用于质量评估）
    int startAngle = (int)data.i_Start;
    int endAngle = (int)data.i_End;
    int size = data.i_Phase_Len; // 天线对数
    double max_val = -9999.9;    // 初始化最大相关度为极小值
    int ang_val = 0;             // 记录最大相关度对应的角度
    diff.clear();
    vector<double>().swap(diff);
    diff.resize(360);
    int tp_ang = 0;
    InterferInfo tp_data = data;     // 副本，用于存储最佳匹配的理论相位差
    double tp_Phase_theory[200];     // 临时存储当前角度的理论相位差数组
    // ===== 步骤 1-3：遍历搜索 + 计算相关度 + 峰值搜索 =====
    for (int ang = startAngle; ang < endAngle + 1; ang++)
    {
        double sumDiff = 0.0;
        tp_ang = Round360(ang); // 角度归一化
        // 计算当前候选角度 ang 与实测值的余弦相关度
        for (int i = 0; i < size; i++)
        {
            int antn1 = data.i_AntennaSq[i][0]; // 天线对中第一个天线的编号（1-based）
            int antn2 = data.i_AntennaSq[i][1]; // 天线对中第二个天线的编号（1-based）
            // 当前角度下该天线对的理论相位差 = 天线1的理论相位 - 天线2的理论相位
            double theoryPhase = phaseTheory[tp_ang][antn1 - 1] - phaseTheory[tp_ang][antn2 - 1];
            tp_Phase_theory[i] = theoryPhase;

            // 余弦相关度累加：cos(理论相位差 - 实测相位差)
            // cos(0)=1 表示完美匹配，cos(pi)=-1 表示完全反相
            sumDiff = sumDiff + cos(theoryPhase - data.i_Phase_Diff[i]);
        }
        //  sumDiff = (sumDiff / size + 1) / 2 * 100;
        // sumDiff = sumDiff / size;
        // 更新最大相关度及其对应的角度
        if (sumDiff > max_val)
        {
            max_val = sumDiff;
            ang_val = tp_ang; // 记录最佳匹配角度
            // 保存最佳匹配角度下的各理论相位差，用于后续质量评估
            for (int j = 0; j < data.i_Phase_Len; j++)
            {
                tp_data.i_Phase_Diff[j] = tp_Phase_theory[j];
            }
        }
        diff[tp_ang] = sumDiff; // 实际伪谱：每个角度的相关度值
        // 输出归一化伪谱：将 sumDiff 映射到 [0, 1] 区间
        // (sumDiff/size + 1) / 2 将 [-1, 1] 的余弦均值映射到 [0, 1]
        diff2[tp_ang] = (sumDiff / size + 1) / 2;
    }

    // ===== 步骤 4：计算测向质量 =====
    angle = Round360(ang_val); // 确保角度在 [0, 360) 范围内
    vector<double> tp_theory_diff;
    // 以最佳匹配角度下的理论相位差作为"理论值"，重新计算一条理论伪谱
    calPseudoByInterfer(phaseTheory, tp_data, tp_theory_diff);
    // 计算理论伪谱与实际伪谱的相关系数，得到测向质量
    quality = getDoaMass(tp_theory_diff, diff, startAngle, endAngle); // 计算测向质量
    // ===== 步骤 5：将归一化伪谱乘以质量值，得到加权伪谱 =====
    for (int i = 0; i < 359; i++)
    {
        diff2[i] *= quality; // diff2[ang] = 归一化相关度 * 测向质量
    }
}


/**
 * @brief 计算均匀圆阵的理论相位（以阵列中心为参考点）
 *
 * ===== 物理模型 =====
 * 对于 N 元均匀圆阵（Uniform Circular Array, UCA）：
 *   - 天线均匀分布在半径为 r 的圆周上
 *   - 第 i 个天线（i 从 0 开始）的方位角为：theta_i = i * 2*pi / N
 *   - 以圆心为相位参考点，入射平面波来自角度 ang（度）
 *
 * ===== 相位公式推导 =====
 * 远场条件下，平面波到达天线 i 与到达圆心的波程差为：
 *   delta_d = r * cos(theta_i - alpha)
 *   其中 alpha = ang * pi / 180 是入射角的弧度值
 *
 * 对应的相位差为：
 *   phase_i = 2*pi * delta_d / lambda
 *           = 2*pi * f * r / c * cos(i * 2*pi/N - ang * pi/180)
 *
 * 其中：
 *   - f: 信号频率（Hz）
 *   - r: 阵列半径（m）
 *   - c: 光速 3e8（m/s），定义于 m_C
 *   - lambda = c/f: 波长（m）
 *
 * ===== 相位范围 =====
 * 由于 cos 值域为 [-1, 1]，相位绝对值最大为 2*pi*f*r/c = 2*pi*r/lambda。
 * 当 r > lambda/2 时，相位绝对值超过 pi，可能出现"跳半周"相位模糊。
 * 这就是为什么大孔径阵列需要虚拟阵列解模糊的原因。
 *
 * ===== 参数说明 =====
 * @param f 信号频率（Hz），用于计算波长
 * @param r 阵列半径（米），圆阵的物理半径
 * @param antnnaNum 天线阵元数量
 * @param theory 输出的理论相位矩阵：
 *               theory[ang][i] 表示来自角度 ang（0-359度）的信号
 *               到达天线 i（0-based）时相对于圆心的相位延迟（弧度）
 *               theory 的大小为 360 x antnnaNum
 */
void ArithmeticDoa::calPhaseTheory(const double f, const double r, const int antnnaNum, std::vector<std::vector<double>> &theory) // 计算理论相位差
{

    // 相邻天线之间的角度间隔（弧度）
    double perAngel = 2 * m_PI / antnnaNum;
    theory.resize(360);
    // 遍历所有 360 个角度（0-359 度，步长 1 度）
    for (int ang = 0; ang < 360; ang++)
    {
        theory[ang].resize(antnnaNum);
        // 计算每个天线在角度 ang 处的理论相位
        for (int i = 0; i < antnnaNum; i++)
        {
            // 天线 i 的角位置 = i * 2*pi/N
            // 入射波方向 = ang * pi/180
            // 相位 = 2*pi*f*r/c * cos(天线角位置 - 入射方向)
            double theoryPhase = 2 * m_PI * f * r / m_C * cos(i * perAngel - ang * m_PI / 180);
            theory[ang][i] = theoryPhase;
        }
    }
    // Log("get theory phase:f=%.2f, r=%.4f, antennaNum=%d\n ", f, r, antnnaNum);
}

/**
 * @brief 计算幅相法的理论伪谱（用于测向质量评估）
 *
 * 伪谱定义：
 *   对每个角度 ang，计算该角度下仿真阵列流型 simulateA[ang] 与
 *   最优匹配阵列流型 theory_A 的归一化内积（复数余弦相似度）：
 *     similarity = |sum(conj(sim_A[i]) * theory_A[i])| / (norm(sim_A) * norm(theory_A))
 *   similarity 取值范围 [0, 1]，1 表示完全匹配，0 表示完全不匹配。
 *
 * 归一化内积的物理意义：
 *   two 复数向量 A 和 B 的内积 conj(A).B 的模反映了两个向量在复数空间中的夹角，
 *   除以各自的模长后得到的值即为余弦相似度。
 *
 * 与相关干涉仪伪谱的类比：
 *   - 相关干涉仪：伪谱 = sum(cos(理论相位差 - 实际相位差))
 *   - 幅相法：伪谱 = 阵列流型的复数余弦相似度
 *   两者本质上都是衡量"理论值"与"实际值"相似度的方法，只是度量空间不同。
 *
 * @param simulateA 仿真阵列流型矩阵（360 x 天线数，复数）
 * @param theory_A 最优匹配角度下的阵列流型向量（复数），作为"理论值"
 * @param diff 输出的理论伪谱数组（360 点），diff[ang] = 归一化内积 * 100
 */
void ArithmeticDoa::calPseudoByAmpPhase(const vector<vector<complex<double>>> simulateA, vector<complex<double>> theory_A, vector<double> &diff)
{
    double tp_ActualNorm = getNorm(theory_A); // 理论向量的 L2 范数
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    diff.clear();
    diff.resize(360);
    vector<complex<double>> tp_A;
    // 遍历所有 360 个角度
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        tp_A = simulateA[ang];         // 当前角度的仿真阵列流型
        double tp_ANorm = getNorm(tp_A); // 当前角度阵列流型的 L2 范数
        // 计算共轭内积：sum(conj(sim_A[i]) * theory_A[i])
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * theory_A[i];
        }
        // 归一化内积的模（余弦相似度），映射到 [0, 100]
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm);
        diff[ang] = tp_diff * 100;
    }
}

/**
 * @brief 幅相法测向主算法（版本一：通过 InterferInfo 传入实测数据）
 *
 * ===== 算法原理 =====
 * 幅相法测向同时利用天线阵列的幅度和相位信息，通过复数阵列流型的内积匹配进行测向。
 * 相比于相关干涉仪法（仅使用相位），幅相法对非理想天线方向图有更好的适应性。
 *
 * ===== 算法步骤 =====
 *
 * 步骤 1 -- 构造实测阵列流型：
 *   从 InterferInfo 中提取幅度（i_Amp，线性值）和相位差（i_Phase_Diff，弧度），
 *   调用 getA 构造复数形式的实测阵列流型向量 actual：
 *     actual[k] = amp[k] * exp(j * phase_diff[k])
 *   注意：这里使用的是相位"差"而非绝对相位，因此阵列流型是"比值"形式。
 *
 * 步骤 2 -- 构造各角度的仿真阵列流型（比值形式）：
 *   对每个候选角度 ang，计算各天线对的复数比值：
 *     tp_A[k] = simulateA[ang][ant1-1] / simulateA[ang][ant2-1]
 *   使用比值而非绝对值的原因是：实测数据也是相位差（比值）形式，
 *   这样可以消除接收机通道间的不一致性。
 *
 * 步骤 3 -- 复数内积匹配：
 *   计算仿真阵列流型 tp_A 与实测阵列流型 actual 的归一化内积（余弦相似度）：
 *     similarity = |sum(conj(tp_A[i]) * actual[i])| / (norm(tp_A) * norm(actual))
 *   取 similarity 最大的角度作为 DOA 估计。
 *
 * 步骤 4 -- 质量评估：
 *   基于最优角度的仿真阵列流型，重新计算理论伪谱，并求其与实际伪谱的相关系数。
 *
 * ===== 与相关干涉仪方法的对比 =====
 *   - 相关干涉仪：correlation = sum(cos(理论相位差 - 实际相位差))
 *     等价于复数内积的实部（忽略幅度后）
 *   - 幅相法：correlation = |sum(conj(理论_A) * 实际_A)| / (norm * norm)
 *     同时考虑了幅度和相位，且使用复数内积的模，统一处理了相位差和幅度比
 *
 * @param simulateA 仿真阵列流型矩阵（360 x 天线数，复数），simulateA[ang][i] = 天线 i 在角度 ang 的复数响应
 * @param data 实测数据，包含幅度（i_Amp）、相位差（i_Phase_Diff）和天线对序号（i_AntennaSq）
 * @param angle 输出的测向角度（度，0-359）
 * @param quality 输出的测向质量（0-100）
 * @param diff 输出的归一化内积相似度曲线（360 点），diff[ang] = 相似度 * 100
 */
void ArithmeticDoa::calAmpPhase(vector<vector<complex<double>>> simulateA, const InterferInfo data, double &angle, double &quality, vector<double> &diff) // 幅相法测向
{
    int num = data.i_Phase_Len;
    vector<double> amp;
    amp.resize(num);
    vector<double> phase;
    phase.resize(num);
    // ===== 步骤 1：提取实测幅度和相位差，构造实测阵列流型 =====
    for (int i = 0; i < num; i++)
    {
        amp[i] = data.i_Amp[i];               // 线性幅度值
        phase[i] = data.i_Phase_Diff[i];      // 相位差（弧度）
    }
    vector<complex<double>> actual;
    getA(amp, phase, actual); // 组合为复数阵列流型：actual[k] = amp * exp(j*phase)
    double tp_ActualNorm = getNorm(actual); // 实测向量的 L2 范数

    // ===== 步骤 2-3：遍历搜索 + 复数内积匹配 =====
    double tp_max = -999.0;       // 最大相似度初始值
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    vector<complex<double>> tp_A;
    tp_A.clear();
    tp_A.resize(num);
    vector<complex<double>> tp_A2; // 存储最佳匹配角度的阵列流型
    tp_A2.clear();

    diff.clear();
    diff.resize(360);
    vector<vector<complex<double>>> tp_simulateA; // 存储所有角度的比值形式阵列流型，用于质量评估
    tp_simulateA.resize(360);
    // 遍历所有 360 个角度
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        // 对每个天线对 k，计算仿真阵列流型的比值
        // tp_A[k] = 仿真阵列中天线对第一个天线的响应 / 仿真阵列中天线对第二个天线的响应
        for (int k = 0; k < num; k++)
        {
            tp_A[k] = simulateA[ang][data.i_AntennaSq[k][0] - 1] / simulateA[ang][data.i_AntennaSq[k][1] - 1];
        }
        tp_simulateA[ang] = tp_A; // 保存当前角度的比值形式阵列流型

        double tp_ANorm = getNorm(tp_A); // 仿真向量的 L2 范数
        // 计算共轭内积并归一化
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * actual[i];
        }
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm); // 归一化内积的模
        // 更新最大相似度
        if (tp_max < tp_diff)
        {
            tp_max = tp_diff;
            angle = ang;      // 记录最佳匹配角度
            tp_A2.clear();
            tp_A2 = tp_A;     // 记录最佳匹配角度下的阵列流型
        }
        diff[ang] = tp_diff * 100; // 存储相似度曲线（百分比）
    }
    // ===== 步骤 4：计算测向质量 =====
    vector<double> tp_theory_diff;
    // 以最佳匹配角度下的阵列流型作为"理论值"，重新计算理论伪谱
    calPseudoByAmpPhase(tp_simulateA, tp_A2, tp_theory_diff);
    // 计算理论伪谱与实际伪谱的相关系数作为测向质量
    quality = getDoaMass(tp_theory_diff, diff, 0, 360);
    // quality = tp_max * 100; //(tp_max + 1) / 2 * 100;
}

/**
 * @brief 幅相法测向主算法（版本二：直接传入复数阵列流型向量）
 *
 * 与版本一的核心算法相同，区别在于：
 *   1. 输入直接是完整的复数阵列流型向量 actualA（已预处理为复数形式），
 *      不需要从 InterferInfo 中提取幅度和相位再组合
 *   2. 仿真阵列流型直接使用 simulateA[ang]（不需要计算天线对的比值）
 *   3. 角度输出取负值：angle = -ang（坐标系约定可能与版本一相反，取决于
 *      仿真数据的角度定义方式）
 *
 * ===== 适用场景 =====
 * 版本二适用于已预先将实测数据转换为复数阵列流型的场景，
 * 例如已经提取了各天线的绝对幅度和相位（而非相位差），
 * 并通过 getA 转换为复数形式。
 *
 * @param simulateA 仿真阵列流型矩阵（360 x 天线数，复数）
 * @param actualA 实测阵列流型向量（复数），各天线独立的复数响应
 * @param angle 输出的测向角度（度），注意此处取负值：angle = -ang
 * @param quality 输出的测向质量（0-100）
 * @param diff 输出的相似度曲线（360 点），diff[ang] = 相似度 * 100
 */
void ArithmeticDoa::calAmpPhase(const vector<vector<complex<double>>> simulateA, const vector<complex<double>> actualA, double& angle, double& quality, vector<double>& diff)
{
    int num = actualA.size();                              // 天线数量
    double tp_ActualNorm = getNorm(actualA);               // 实测向量的 L2 范数
    double tp_max = -999.0;                                // 最大相似度
    complex<double> tp_sumComplex(0.0, 0.0);
    double tp_diff = 0.0;
    diff.clear();
    diff.resize(360);
    vector<vector<complex<double>>> tp_simulateA;          // 保存所有角度的仿真阵列流型
    tp_simulateA.resize(360);

    vector<complex<double>> tp_A;
    tp_A.clear();
    tp_A.resize(num);
    vector<complex<double>> tp_A2;                         // 最佳匹配角度的阵列流型
    tp_A2.clear();
    // 遍历所有 360 个角度，计算相似度
    for (int ang = 0; ang < 360; ang++)
    {
        tp_sumComplex.imag(0.0);
        tp_sumComplex.real(0.0);
        tp_A = simulateA[ang];                             // 当前角度的仿真阵列流型
        tp_simulateA[ang] = simulateA[ang];                // 保存用于后续质量评估

        double tp_ANorm = getNorm(tp_A);                   // 仿真向量的 L2 范数
        // 计算共轭内积
        for (int i = 0; i < (int)tp_A.size(); i++)
        {
            tp_sumComplex = tp_sumComplex + conj(tp_A[i]) * actualA[i];
           // cout << conj(tp_A[i]) <<  endl;
           // cout << conj(tp_A[i]) * actualA[i] << endl;
        }
        tp_diff = abs(tp_sumComplex) / (tp_ActualNorm * tp_ANorm); // 归一化内积的模
        // 更新最大相似度及其对应角度
        if (tp_max < tp_diff)
        {
            tp_max = tp_diff;
            angle = Round3600(-ang);                       // 注意：取负角度（坐标系转换）
            tp_A2.clear();
            tp_A2 = tp_A;
        }
        diff[ang] = tp_diff * 100;                         // 相似度百分比

    }
    // 计算测向质量
    vector<double> tp_theory_diff;
    calPseudoByAmpPhase(tp_simulateA, tp_A2, tp_theory_diff);
    quality = getDoaMass(tp_theory_diff, diff, 0, 360);
}
/**
 * @brief 检查指定频率的幅度-相位仿真数据文件是否存在
 *
 * 仿真数据文件的命名规则为：
 *   - 幅度文件：{path}A-{频率Hz}.csv
 *   - 相位文件：{path}P-{频率Hz}.csv
 *
 * 此函数检查幅度文件是否存在（幅度和相位文件通常成对出现，
 * 通过检查幅度文件即可判断两者是否都存在）。
 *
 * @param path 仿真数据文件的目录路径
 * @param f 信号频率（Hz），用于构造文件名
 * @return true 文件存在并可打开，false 文件不存在
 */
bool ArithmeticDoa::existAmpPhsFile(const string path, const double f)
{
        bool tp_f = true;
        std::stringstream ss;
        ss << std::fixed << std::setprecision(0); // 频率整数化显示
        ss << f;
        string path1 = ss.str();
        string csv = ".csv";
        string Amp_File_Path = "";

        Amp_File_Path.append(path);
        Amp_File_Path.append("A-");
        Amp_File_Path.append(path1);
        Amp_File_Path.append(csv);
        ifstream in(Amp_File_Path);
       // tp_f = in.is_open();
        return in.is_open();

}
/**
 * @brief 读取仿真数据并构建完整的复数阵列流型矩阵
 *
 * 处理流程：
 *   1. 查找最接近目标频率 f 的仿真数据文件：
 *      - 仿真数据通常以特定频率间隔存储（如每 10kHz 一组），
 *        不太可能恰好有目标频率的数据文件
 *      - 从目标频率开始，以 10kHz 为步长向外搜索，找到最近的存在文件
 *      - 最大搜索 10000 次，对应约 100MHz 范围
 *
 *   2. 读取找到的幅度数据（dB 值）和相位数据（度值）
 *
 *   3. 数据格式转换：
 *      - 幅度：dB 转线性值 -- linear = 10^(dB/20)
 *      - 相位：取反（适配坐标系约定，仿真数据的相位定义为入射方向）
 *
 *   4. 对每个角度（0-359 度），调用 getA 组合幅度和相位为复数阵列流型
 *
 * @param path 仿真数据文件目录路径
 * @param f 目标信号频率（Hz）
 * @param simulateA 输出的仿真阵列流型矩阵（360 x 天线数，复数）
 */
void ArithmeticDoa::calAmpPhaseSimulateA(const string path, const double f, vector<vector<complex<double>>> &simulateA) // 计算仿真阵列流型
{
    int num1 = 0;
    bool tp_bool = true;
    double tp_f1 = f;
    double tp_f = f;
    int diff = 10e3; // 频率搜索步长 10kHz
    // 搜索最接近目标频率的仿真数据文件
    while (num1<10000 ) // 最多搜索 10000 步
    {
        tp_f1 = f + diff*num1; // 向上搜索：f + 10kHz*num1
        tp_bool = existAmpPhsFile(path, tp_f1);
        if (tp_bool)
        {
            tp_f = tp_f1;
            break;
        }
        tp_f1 = f - diff * num1; // 向下搜索：f - 10kHz*num1
        tp_bool = existAmpPhsFile(path, tp_f1);
        if (tp_bool)
        {
            tp_f = tp_f1;
            break;
        }
        num1++;
    }
    simulateA.clear();
    // 读取幅度数据（dB 值）
    vector<vector<double>> AmpData;
    int flg_amp = getSimulateAmp(path, tp_f, AmpData);
    // 读取相位数据（度值）
    vector<vector<double>> PhsData;
    int flg_phase = getSimulatePhase(path, tp_f, PhsData);
    if (0 != flg_amp || 0 != flg_phase)
    {
        Log("*** error:simulate data is error!!!\n");
        return;
    }
    // 检查幅度和相位数据的维度是否一致
    if (PhsData.size() != AmpData.size() || PhsData[0].size() != AmpData[0].size())
    {
        Log("*** error:simulate data is error!!!\n");
        return;
    }
    int num = PhsData[0].size(); // 天线数量
    vector<double> tp_amp;
    vector<double> tp_phase;
    vector<complex<double>> tp_A;
    tp_amp.resize(num);
    tp_phase.resize(num);
    simulateA.resize(360);
    // 对每个角度，组合复数阵列流型
    for (int i = 0; i < (int)PhsData.size(); i++)
    {
        for (int j = 0; j < num; j++)
        {
            // 幅度转换：dB -> 线性值（10^(dB/20)）
            tp_amp[j] = pow(10, (AmpData[i][j] / 20));
            // tp_amp[j] = AmpData[i][j];
            // tp_phase[j] = PhsData[i][j] * PI / 180;
            // 相位取反：仿真数据中相位定义为入射方向，阵列流型中需要取反
            tp_phase[j] = -PhsData[i][j];
        }
        tp_A.clear();
        getA(tp_amp, tp_phase, tp_A); // 组合幅度和相位为复数
        simulateA[i] = tp_A;          // 存储当前角度的阵列流型
    }
}

/**
 * @brief 将幅度和相位组合为复数形式的阵列流型
 *
 * 阵列流型（Array Manifold）描述了天线阵列在不同来波方向下的复数响应。
 * 每个元素为：A[i] = amp[i] * exp(j * phase[i])
 *
 * 物理含义：A[i] 的模 = 天线 i 的增益（幅度响应），
 *          A[i] 的幅角 = 天线 i 的相位延迟（相位响应）。
 *
 * @param amp 幅度数组（线性值，非 dB）
 * @param phase 相位数组（弧度）
 * @param A 输出的复数阵列流型向量
 */
void ArithmeticDoa::getA(const vector<double> amp, const vector<double> phase, vector<complex<double>> &A) // 计算阵列流型
{
    int size = amp.size();
    A.clear();
    A.resize(size);
    for (int i = 0; i < size; i++)
    {
        // 利用欧拉公式构造复数：amp * (cos + j*sin) = amp * exp(j*phase)
        complex<double> tp(cos(phase[i]), sin(phase[i]));
        A[i] = amp[i] * tp;
    }
}

/**
 * @brief 生成虚拟阵元的一维编号（二维编码的哈希映射）
 *
 * 编码规则：virtualAntNum = ant1 * 10 + ant2
 * 含义：将天线对 (ant1, ant2) 编码为一个整型编号，用作虚拟理论相位矩阵的列索引。
 * 例如：天线对 (3, 5) -> 编号 35；天线对 (5, 3) -> 编号 53。
 * 注意此编码不是对称的，(3,5) 和 (5,3) 的编号不同，这是有意为之，
 * 因为 (A,B) 和 (B,A) 对应不同的虚拟阵元位置和相位关系。
 *
 * @param ant1 第一天线编号（从 1 开始）
 * @param ant2 第二天线编号（从 1 开始）
 * @return 虚拟阵元编号，用于线性化存储
 */
int ArithmeticDoa::getVirtualAntNum(const int ant1, const int ant2) // 生成虚拟阵元的编号
{
    return ant1 * 10 + ant2;
}

/**
 * @brief 从仿真数据文件中读取指定频率的相位数据
 *
 * 文件名格式：{path}P-{频率Hz}.csv
 * 文件内容：每行以角度（度）开头，后跟各天线的相位值（度），以逗号分隔。
 * 读取后自动将相位从度转换为弧度。
 *
 * @param path 仿真数据文件目录路径
 * @param f 信号频率（Hz），用于构造文件名
 * @param phase 输出的相位矩阵，phase[ang][i] = 角度 ang 处天线 i 的相位（弧度）
 * @return 0 成功，-1 文件读取或格式错误
 */
int ArithmeticDoa::getSimulatePhase(const string path, const double f, std::vector<std::vector<double>> &phase) // 获得阵列仿真数据的相位
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(0);
    ss << f;
    string path1 = ss.str();
    string csv = ".csv";
    //  string path = "/simulateData/";
    string Phase_File_Path = ""; // m_CurrentPath;
    Phase_File_Path.append(path);
    Phase_File_Path.append("P-");
    Phase_File_Path.append(path1);
    Phase_File_Path.append(csv);
    // vector<vector<double>> PhsData;
    // const char *phase_path = Phase_File_Path.c_str();
    phase.clear();
    int flg = getModeData(Phase_File_Path, phase); // 读取 CSV 文件
    if (0 != flg)
    {
        Log("***error:get simulate phase data  is error:%s\n", Phase_File_Path.c_str());
        cout << "***error:get simulate phase data is error!!!" << endl;
        phase.clear();
        return -1;
    }
    // 将相位从度转换为弧度
    for (int i = 0; i < (int)phase.size(); i++)
    {
        for (int j = 0; j < (int)phase[i].size(); j++)
        {
            phase[i][j] = phase[i][j] * m_PI / 180; // 度转弧度
        }
    }
    // phase = PhsData;
    return 0;
}

/**
 * @brief 从仿真数据文件中读取指定频率的幅度数据
 *
 * 文件名格式：{path}A-{频率Hz}.csv
 * 文件内容：每行以角度（度）开头，后跟各天线的幅度值（dB），以逗号分隔。
 * 注意：读取的幅度为 dB 值，需在调用方通过 10^(dB/20) 转换为线性值后使用。
 *
 * @param path 仿真数据文件目录路径
 * @param f 信号频率（Hz），用于构造文件名
 * @param amp 输出的幅度矩阵，amp[ang][i] = 角度 ang 处天线 i 的幅度（dB）
 * @return 0 成功，-1 文件读取或格式错误
 */
int ArithmeticDoa::getSimulateAmp(const string path, const double f, std::vector<std::vector<double>> &amp) // 获得阵列仿真数据的幅度
{
    std::stringstream ss;
    ss << std::fixed << std::setprecision(0);
    ss << f;
    string path1 = ss.str();
    string csv = ".csv";
    string Amp_File_Path = "";

    Amp_File_Path.append(path);
    Amp_File_Path.append("A-");
    Amp_File_Path.append(path1);
    Amp_File_Path.append(csv);

    int flg = getModeData(Amp_File_Path, amp); // 读取 CSV 文件
    if (flg != 0)
    {
        Log(" error:get simulate amp data  is error!!!\n");
        cout << "error:get simulate amp data  is error!!!" << endl;
        return -1;
    }
    return 0;
}

/**
 * @brief 从 CSV 文件中读取模板（仿真）数据
 *
 * 文件格式：每行一个角度，格式为 "角度, 值1, 值2, ..."
 * 例如 "0, -1.2, 0.5, 2.1" 表示在角度 0 度时，各天线/参数的值分别为 -1.2, 0.5, 2.1。
 *
 * 角度列用于确定该行数据对应的方向（支持任意角度，不要求按顺序排列），
 * 通过 Round360 归一化到 [0, 360) 范围后进行索引。
 *
 * @param path 模板数据文件的完整路径
 * @param data 输出的模板数据矩阵，data[ang][j] = 角度 ang 处的第 j 个参数值
 * @return 0 成功，非 0 文件读取失败
 */
int ArithmeticDoa::getModeData(const string path, vector<vector<double>> &data) // 获取模板数据
{
    vector<string> tpVec;
    tpVec.clear();
    int flg = 0;
    flg = readFile(path, tpVec); // 读取文件内容到字符串数组

    if (0 != flg)
    {
        Log("***error:get sigle channel error paranmeter adr is false!!!\n");
        cout << "***error:get sigle channel error paranmeter adr is false!!!" << endl;
        return flg;
    }
    int num = tpVec.size();
    data.clear();
    data.resize(360); // 预分配 360 个角度的空间
    for (int i = 0; i < num; i++)
    {
        string &a1 = tpVec[i];
        vector<string> a2;
        a2.clear();
        vector<string>().swap(a2);
        split(a2, a1, ','); // 按逗号分割行
        int ang = Round360(stoi(a2[0])); // 第一列为角度（度），归一化到 [0, 360)
        for (int j = 1; j < (int)a2.size(); j++)
        {
            data[ang].emplace_back(stod(a2[j])); // 后续列为数据值
        }
    }
    return 0;
}

/**
 * @brief 计算测向质量（区域版本）-- 基于两个伪谱的归一化互相关系数
 *
 * 测向质量计算公式：
 *   corr = sum(diffTheory[i] * diff[i]) / (sqrt(sum(diffTheory^2)) * sqrt(sum(diff^2)))
 *   quality = (corr + 1) / 2 * 100
 *
 * corr 为两个向量的归一化互相关系数（皮尔逊相关系数的简化形式，假设无 DC 偏移），
 * 取值范围 [-1, 1]。映射到 [0, 100] 后：
 *   - quality = 100：两个伪谱完全正相关（形状完全一致，最可靠）
 *   - quality = 50： 两个伪谱无相关性（随机匹配）
 *   - quality = 0：  两个伪谱完全负相关（形状完全相反，最不可靠）
 *
 * @param diffTheory 理论伪谱（最优匹配角度下的理论值构成的伪谱）
 * @param diff 实际伪谱（实测数据与各角度理论值匹配的伪谱）
 * @param starAngle 搜索起始角度（度）
 * @param endAngle 搜索结束角度（度）
 * @return 测向质量值（0-100）
 */
double ArithmeticDoa::getDoaMass(const vector<double> diffTheory, const vector<double> diff, int starAngle, int endAngle)
{
    double quality;
    int size = (int)diffTheory.size();
    double tp_sum = 0.0;        // sum(diffTheory * diff) -- 叉积和
    double tp_theory_sum = 0.0; // sum(diffTheory^2)   -- 理论伪谱的能量
    double tp_diff_sum = 0.0;   // sum(diff^2)         -- 实际伪谱的能量
    int tp_ang = 0;
    // 在指定的角度范围内累加
    for (int i = starAngle; i < endAngle + 1; i++)
    {
        tp_ang = Round360(i);
        tp_sum = tp_sum + diffTheory[tp_ang] * diff[tp_ang];
        tp_theory_sum = tp_theory_sum + diffTheory[tp_ang] * diffTheory[tp_ang];
        tp_diff_sum = tp_diff_sum + diff[tp_ang] * diff[tp_ang];
    }
    // 归一化因子 = sqrt(sum(x^2)) * sqrt(sum(y^2))
    double tp_mode_sum = sqrt(tp_theory_sum) * sqrt(tp_diff_sum);
    if (tp_mode_sum == 0) // 防止除零
    {
        quality = 0;
    }
    else
    {
        // corr = tp_sum / tp_mode_sum，取值范围 [-1, 1]
        // quality = (corr + 1) / 2 * 100，映射到 [0, 100]
        quality = (tp_sum / tp_mode_sum + 1) / 2 * 100;
    }

    return quality;
}

/**
 * @brief 计算测向质量（全局版本）-- 在全部角度范围内计算相关系数
 *
 * 与区域版本的算法完全相同，区别仅在于累加角度范围为 [0, 359]。
 *
 * @param diffTheory 理论伪谱（360 点）
 * @param diff 实际伪谱（360 点）
 * @return 测向质量值（0-100）
 */
double ArithmeticDoa::getDoaMass(const vector<double> diffTheory, const vector<double> diff) // 获得测向质量
{
    double quality;
    int size = (int)diffTheory.size();
    double tp_sum = 0.0;        // sum(diffTheory * diff)
    double tp_theory_sum = 0.0; // sum(diffTheory^2)
    double tp_diff_sum = 0.0;   // sum(diff^2)
    // 在全部 360 个角度范围内累加
    for (int i = 0; i < size; i++)
    {
        tp_sum = tp_sum + diffTheory[i] * diff[i];
        tp_theory_sum = tp_theory_sum + diffTheory[i] * diffTheory[i];
        tp_diff_sum = tp_diff_sum + diff[i] * diff[i];
    }
    // 计算归一化互相关系数
    double tp_mode_sum = sqrt(tp_theory_sum) * sqrt(tp_diff_sum);
    if (tp_mode_sum == 0) // 防止除零
    {
        quality = 0;
    }
    else
    {
        quality = (tp_sum / tp_mode_sum + 1) / 2 * 100; // 映射到 [0, 100]
    }

    return quality;
}
