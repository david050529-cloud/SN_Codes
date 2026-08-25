/**
 * @file arithmetic.h
 * @brief 测向算法核心类声明 -- 相关干涉仪与幅相法测向
 *
 * 本文件定义了 ArithmeticDoa 类，封装了两种主要的无线电测向（Direction Of Arrival, DOA）算法：
 *   1. 相关干涉仪测向（Correlative Interferometer）：
 *      - 利用天线阵列中各阵元间的理论相位差与实际测量相位差进行相关匹配
 *      - 通过计算理论伪谱与实际伪谱的相似度来确定来波方向
 *      - 核心方法：calPhaseTheory（理论相位计算）、calInterfer（相关干涉仪主算法）
 *   2. 幅相法测向（Amplitude-Phase DOA）：
 *      - 同时利用阵列的幅度和相位信息，通过阵列流型（Array Manifold）匹配进行测向
 *      - 适用于天线方向图非均匀的场景，利用幅度差异增强分辨能力
 *      - 核心方法：calAmpPhase（幅相法主算法）
 *
 * 此外还包含虚拟阵列扩展（Virtual Array Expansion）功能：
 *      - 通过数学变换生成虚拟阵元，等效扩大阵列孔径，提高测向精度
 *      - 利用虚拟阵元进行二次测向以解决相位模糊（"跳半周"问题）
 *
 * 代码中的关键物理概念：
 *   - 阵列流型(Array Manifold)：天线阵列在不同来波方向下的复数响应向量，包含幅度和相位信息
 *   - 伪谱(Pseudo-spectrum)：由理论值（或仿真数据）与实测数据的相关度构成的角度函数，峰值对应DOA估计
 *   - 测向质量(DOA Quality)：量化测向结果可信度的指标，基于理论伪谱与实际伪谱的相关系数计算
 *   - 跳半周(Half-Cycle Slip)：当阵元间距大于半波长时，相位差可能超出[-pi, pi]范围导致的角度模糊问题
 *   - 虚拟阵列(Virtual Array)：利用已有阵元相位差进行外推/内插，构造等效的新阵元，提升孔径和分辨率
 */

#include "../publicFunctionDoa/publicFunctionDoa.h"
using namespace PublicSpace;
#pragma pack(1)

/**
 * @struct InterferInfo
 * @brief 测向输入数据结构体，封装一次测向所需的全部原始信息
 *
 * 该结构体作为 calInterfer / calAmpPhase 等核心测向函数的输入参数，
 * 包含了实测数据的相位差、幅度以及搜索范围等关键信息。
 *
 * 天线序号约定：天线从 1 开始编号（与用户侧一致），内部计算时减 1 转为 0-based 索引。
 * 相位差定义：i_Phase_Diff[k] = 天线 i_AntennaSq[k][0] 的相位 减去 天线 i_AntennaSq[k][1] 的相位
 */
struct InterferInfo // 用于测向的相关信息
{
    int i_Start;              ///< 搜索范围开始角度（度），闭区间，用于限定相关峰搜索的范围以提高效率或适应定向天线
    int i_End;                ///< 搜索范围结束角度（度），闭区间
    int i_Phase_Len;          ///< 有效的相位差/幅度数据条数（即 i_AntennaSq、i_Phase_Diff、i_Amp 的有效长度）
    int i_AntennaSq[200][2];  ///< 相位差对应的天线对序号，相位差 = 天线[0]的相位 - 天线[1]的相位，天线序号从 1 算起
    double i_Phase_Diff[200]; ///< 实测相位差数组（弧度），与 i_AntennaSq 一一对应
    double i_Amp[200];        ///< 实测幅度数组（线性值），幅相法测向时使用，与 i_AntennaSq 一一对应
};

/**
 * @struct Rs
 * @brief 频率-阵列半径对应关系，存储某一频率区间内对应的阵列孔径（半径）
 *
 * 天线阵列的有效半径（孔径）可能随频率变化（例如不同频段使用不同的阵列校准数据），
 * 该结构体记录了一个频率区间 [i_starF, i_endF] 与其对应的阵列半径 i_r 的映射关系。
 * 频率单位为 Hz，半径单位为米。
 */
struct Rs
{
    double i_starF; ///< 起始频率（Hz）
    double i_endF;  ///< 结束频率（Hz）
    double i_r;     ///< 该频率区间对应的阵列半径（米），用于理论相位差计算
};


#pragma pack()
/**
 * @class ArithmeticDoa
 * @brief 测向算法核心类，封装相关干涉仪、幅相法、虚拟阵列扩展等测向方法
 *
 * 该类不包含数据成员（无状态设计），所有方法均为算法工具函数。
 * 调用流程通常为：
 *   1. 构造 InterferInfo 结构体，填入实测相位差/幅度数据
 *   2. 调用 calPhaseTheory 计算理论相位差，或调用 calAmpPhaseSimulateA 读取仿真阵列流型
 *   3. 调用 calInterfer（相关干涉仪）或 calAmpPhase（幅相法）执行测向
 *   4. 获取测向角度（angle）和测向质量（quality）
 *
 * 两种测向方法的核心差异：
 *   - 相关干涉仪：仅使用相位差信息，通过余弦相关度匹配，计算速度快，适用于均匀圆阵
 *   - 幅相法：同时使用幅度和相位信息，通过复数阵列流型内积匹配，对非理想天线方向图有更好的适应性
 *
 * 虚拟阵列扩展（getVirtual / calSecondDoaByVirInterf）：
 *   用于解决大孔径阵列中的相位模糊问题。当阵元间距超过半波长时，相位差可能发生"跳半周"，
 *   导致在伪谱中出现多个峰值。通过构造虚拟阵元缩短基线长度，可消除模糊得到真实角度。
 */
class ArithmeticDoa
{
public:
    ArithmeticDoa(void);
    ~ArithmeticDoa(void);

protected:
    /*==================相关干涉仪测向==================*/
    /**
     * @brief 计算均匀圆阵的理论相位（以阵列中心为参考点）
     *
     * 对于 N 元均匀圆阵，第 i 个阵元（从1开始编号）在角度 ang 处的理论相位为：
     *   phase[i] = 2*pi*f*r/c * cos(i*2*pi/N - ang*pi/180)
     * 其中 f 为频率，r 为阵列半径，c 为光速，ang 为来波方向角（度）。
     * 该公式基于平面波假设和远场条件推导。
     *
     * @param f 信号频率（Hz）
     * @param r 阵列半径（米），即圆阵的物理半径
     * @param antnnaNum 天线阵元数量
     * @param theory 输出的理论相位矩阵，theory[ang][i] 表示角度 ang（0-359度）处第 i 个阵元的理论相位（弧度）
     */
    void calPhaseTheory(const double f, const double r, const int antnnaNum, std::vector<std::vector<double>> &theory);

    /**
     * @brief 计算理论伪谱（仅用于测向质量评估，不用于角度搜索）
     *
     * 对搜索范围内的每个角度，计算该角度下的理论相位差与实际相位差的余弦相关度之和，
     * 形成一条 360 点（覆盖 0-359 度）的伪谱曲线。
     * 伪谱值越高，表示该角度的理论值与实测值越匹配。
     *
     * @param phaseTheory 理论相位差矩阵
     * @param data 实测相位差数据（相位差在 i_Phase_Diff 中，天线对在 i_AntennaSq 中）
     * @param diff 输出的理论伪谱数组，diff[ang] 存储角度 ang 处的相关度累加值
     */
    void calPseudoByInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, vector<double> &diff);

    /**
     * @brief 相关干涉仪测向主算法 -- 通过相位差相关匹配确定来波方向
     *
     * 算法原理（逐步说明）：
     *   1. 在搜索范围 [startAngle, endAngle] 内遍历每个候选角度 ang
     *   2. 对每个候选角度，根据理论相位差矩阵 phaseTheory 计算各天线对的理论相位差
     *   3. 计算理论相位差与实际相位差的余弦相关度之和：sumDiff = sum(cos(theory - actual))
     *      -- 使用余弦函数而非直接做差，天然处理了相位的 2*pi 周期性
     *   4. 找到 sumDiff 最大值对应的角度，即为 DOA 估计结果
     *   5. 基于最优角度的理论伪谱与实际伪谱计算测向质量（相关系数）
     *
     * 余弦相关度的物理意义：当理论值与实测值完全一致时，cos(0)=1，sumDiff 取最大值 N；
     * 当两者相差 180 度时，cos(pi)=-1，sumDiff 取最小值 -N。
     *
     * @param phaseTheory 理论相位矩阵，phaseTheory[ang][i] = 角度 ang 处天线 i 的理论相位（弧度）
     * @param data 实测数据，包含相位差数组、天线对序号、搜索范围等
     * @param angle 输出的测向角度（度），取余弦相关度最大处的角度值
     * @param quality 输出的测向质量（0-100），反映结果的可靠性
     * @param diff2 输出的加权伪谱数组，diff2[ang] = (sumDiff/N+1)/2 * quality
     */
    void calInterfer(const vector<vector<double>> phaseTheory, const InterferInfo data, double &angle, double &quality, vector<double> &diff2);

    /*==================幅相法测向==================*/
    /**
     * @brief 从仿真数据文件中读取指定频率的相位数据
     * @param path 仿真数据文件目录路径（末尾带路径分隔符）
     * @param f 目标频率（Hz），用于构造文件名
     * @param phase 输出的相位矩阵，phase[ang][i] = 角度 ang 处天线 i 的相位（弧度）
     * @return 0 成功，非 0 失败（文件不存在或格式错误）
     */
    int getSimulatePhase(const string path, const double f, std::vector<std::vector<double>> &phase);

    /**
     * @brief 从仿真数据文件中读取指定频率的幅度数据
     * @param path 仿真数据文件目录路径（末尾带路径分隔符）
     * @param f 目标频率（Hz），用于构造文件名
     * @param amp 输出的幅度矩阵，amp[ang][i] = 角度 ang 处天线 i 的幅度（dB）
     * @return 0 成功，非 0 失败
     */
    int getSimulateAmp(const string path, const double f, std::vector<std::vector<double>> &amp);

    /**
     * @brief 结合幅度和相位仿真数据，构建完整的阵列流型矩阵（复数形式）
     *
     * 处理流程：
     *   1. 根据频率 f 查找最近的可用的仿真数据文件（通过 existAmpPhsFile 查找）
     *   2. 读取幅度数据（dB）并转换为线性值：linear = 10^(dB/20)
     *   3. 读取相位数据（度）并转换为弧度（取反以适配坐标系约定）
     *   4. 调用 getA 将幅度和相位组合为复数阵列流型：A = amplitude * exp(j*phase)
     *
     * @param path 仿真数据文件目录路径
     * @param f 目标频率（Hz）
     * @param simulateA 输出的阵列流型矩阵，simulateA[ang][i] 表示角度 ang 处第 i 个天线的复数响应
     */
    void calAmpPhaseSimulateA(const string path, const double f, vector<vector<complex<double>>>& simulateA);

    /**
     * @brief 幅相法测向主算法（版本一：通过 InterferInfo 传入实测数据）
     *
     * 算法原理：
     *   1. 从 InterferInfo 中提取实测幅度和相位差，构造实测阵列流型向量 actual
     *   2. 对每个候选角度 ang，从仿真阵列流型中提取各天线对的比值作为该角度下的理论阵列流型：
     *      tp_A[k] = simulateA[ang][ant1-1] / simulateA[ang][ant2-1]
     *      -- 使用天线对的比值而非绝对值，消除参考相位不一致的影响
     *   3. 计算理论阵列流型与实际阵列流型的归一化内积（余弦相似度）：
     *      diff = |sum(conj(tp_A[i]) * actual[i])| / (norm(tp_A) * norm(actual))
     *   4. 取相似度最大的角度作为 DOA 估计
     *   5. 基于最优角度的理论伪谱与实际伪谱计算测向质量
     *
     * 相比于相关干涉仪法的优势：同时利用了幅度信息，对不同天线的增益差异更敏感，
     * 在非理想天线方向图场景下有更好的角度分辨能力。
     *
     * @param simulateA 仿真阵列流型矩阵（复数），simulateA[ang][i] = 天线 i 在角度 ang 的复数响应
     * @param data 实测数据，包含幅度（i_Amp）和相位差（i_Phase_Diff）
     * @param angle 输出的测向角度（度）
     * @param quality 输出的测向质量（0-100）
     * @param diff 输出的相似度曲线，diff[ang] = 角度 ang 处的归一化内积 * 100
     */
    void calAmpPhase(const vector<vector<complex<double>>> simulateA, const InterferInfo data, double &angle, double &quality, vector<double> &diff);

    /**
     * @brief 幅相法测向主算法（版本二：直接传入复数阵列流型向量）
     *
     * 与版本一的核心算法相同，但输入直接是复数阵列流型 actualA，适用于已预处理好的场景。
     * 注意：此版本中角度输出取负值（angle = -ang），与版本一的坐标系约定可能不同。
     *
     * @param simulateA 仿真阵列流型矩阵
     * @param actualA 实测阵列流型向量（复数）
     * @param angle 输出的测向角度（度），注意此处取负
     * @param quality 输出的测向质量（0-100）
     * @param diff 输出的相似度曲线
     */
    void calAmpPhase(const vector<vector<complex<double>>> simulateA, const vector<complex<double>> actualA, double& angle, double& quality, vector<double>& diff);

    /**
     * @brief 将幅度和相位组合为复数形式的阵列流型
     *
     * A[i] = amp[i] * exp(j * phase[i]) = amp[i] * (cos(phase[i]) + j*sin(phase[i]))
     *
     * @param amp 幅度数组（线性值，非 dB）
     * @param phase 相位数组（弧度）
     * @param A 输出的复数阵列流型向量，长度为天线数
     */
    void getA(const vector<double> amp, const vector<double> phase, vector<complex<double>> &A);

    /**
     * @brief 从 CSV 文件中读取模板（仿真）数据
     *
     * 文件格式：每行以角度（度）开头，后跟各天线的参数值，以逗号分隔。
     * 例如：0, -1.2, 0.5, 2.1 表示角度 0 度时，天线 1、2、3 的值分别为 -1.2, 0.5, 2.1。
     *
     * @param path 模板数据文件的完整路径
     * @param data 输出的模板数据矩阵，data[ang][i] = 角度 ang 处天线 i 的值
     * @return 0 成功，非 0 失败
     */
    int getModeData(const string path, vector<vector<double>> &data);

    /**
     * @brief 根据幅相法计算理论伪谱（用于测向质量评估）
     *
     * 对每个角度 ang，计算仿真阵列流型 simulateA[ang] 与最优匹配阵列流型 theory_A 的
     * 归一化内积（复数余弦相似度），形成伪谱曲线。该伪谱用于后续的测向质量计算。
     *
     * @param simulateA 仿真阵列流型矩阵（360 x 天线数）
     * @param theory_A 最优匹配角度对应的阵列流型向量（作为"理论值"）
     * @param diff 输出的理论伪谱，diff[ang] = 内积相似度 * 100
     */
    void calPseudoByAmpPhase(const vector<vector<complex<double>>> simulateA, vector<complex<double>> theory_A, vector<double> &diff);

    /**
     * @brief 检查指定路径和频率的仿真数据文件是否存在
     * @param path 仿真数据目录路径
     * @param f 频率（Hz）
     * @return true 存在，false 不存在
     */
    bool existAmpPhsFile(const string path, const double f);

    /*==================虚拟阵列扩展（Virtual Array）==================*/
    /**
     * @brief 根据虚拟倍数扩展虚拟阵元及其相位差
     *
     * 虚拟阵列原理：
     *   对于每一对实际天线 (ant_a, ant_b) 及其相位差 diff = phase(a) - phase(b)：
     *   - 生成虚拟天线 ant_virtual = getVirtualAntNum(ant_b, ant_a)，即编号为 b*10 + a
     *   - 虚拟相位差 = diff * virMultiple，即按倍率缩放相位差
     *   - 添加天线对 (ant_b, ant_virtual) 和对应的缩放后相位差
     *   - 同时生成对称的虚拟天线和负相位差
     *
     * 这样，原有的 N 对天线扩展后产生更多的天线对，等效于扩大了阵列孔径。
     * virMultiple < 1 时可用于缩小基线（解模糊），virMultiple > 1 时可用于扩展基线（提高精度）。
     *
     * @param virMultiple 虚拟倍率，实际相位差乘以该倍率得到虚拟相位差
     * @param antnna 输入/输出的天线对列表，输入为原始天线对，输出包含原始+虚拟天线对
     * @param phase_diff 输入/输出的相位差数组，与 antnna 同步更新
     */
    void getVirtual(const double virMultiple, vector<vector<int>> &antnna, vector<double> &phase_diff);

    /**
     * @brief 根据虚拟倍率扩展理论相位矩阵，生成虚拟阵元的理论相位
     *
     * 对每个角度，遍历所有天线对 (i, j)，生成虚拟天线编号，并计算虚拟理论相位：
     *   tp_diff = virMultiple * (theory[i] - theory[j])       // 缩放的相位差
     *   virtualTheory[ang][virtualIndex] = theory[j] - tp_diff  // 虚拟阵元的理论相位
     *
     * @param antnnaNum 实际天线数量
     * @param tp_theory 原始理论相位矩阵
     * @param virMultiple 虚拟倍率
     * @param virtualTheory 输出的虚拟理论相位矩阵，列数为 antnnaNum*10 + antnnaNum
     */
    void getVirtualTheory(const int antnnaNum, const std::vector<std::vector<double>> tp_theory, const double virMultiple, std::vector<std::vector<double>> &virtualTheory);

    /**
     * @brief 生成虚拟阵元的编号
     *
     * 编码规则：virtualAntNum = ant1 * 10 + ant2
     * 这是一种将二维天线对索引编码为一维编号的简单哈希方式，
     * 编码结果用作虚拟理论相位矩阵的列索引。
     *
     * @param ant1 第一天线编号（从 1 开始）
     * @param ant2 第二天线编号（从 1 开始）
     * @return 虚拟阵元编号
     */
    int getVirtualAntNum(const int ant1, const int ant2);

    /**
     * @brief 利用相关干涉仪 + 虚拟阵元进行二次测向，解决相位模糊（"跳半周"）问题
     *
     * 算法流程（逐步说明）：
     *   1. 预处理：将伪谱 diff 中小于 90 的值置零，滤除噪声
     *   2. 寻峰：在伪谱中搜索峰值，若只有 0 或 1 个峰值则直接返回（无模糊问题）
     *   3. 选基：从天线对中选择余弦差异最大的若干对作为"鉴别基"，这些天线对在候选角度间有最好的区分度
     *   4. 虚拟扩展：分别对两个候选峰值角度，用鉴别基的天线对进行虚拟阵列扩展
     *   5. 对比判定：计算实际相位差与两个候选角度下虚拟理论相位差的余弦相关度之和
     *      - 选择相关度更高的角度作为最终结果
     *   6. 质量评估：基于最终选定的角度重新计算测向质量
     *
     * "跳半周"问题的本质：
     *   当阵元间距 d > lambda/2 时，相位差 delta_phi = 2*pi*d*sin(theta)/lambda 的绝对值
     *   可能超过 pi，导致实际测量的相位差被截断到 [-pi, pi] 范围内（相位缠绕），
     *   造成理论值与实测值的匹配出现多个候选角度。虚拟阵列通过缩短等效基线来消除模糊。
     *
     * @param phaseTheory 理论相位矩阵
     * @param virMultiple 虚拟倍率（通常 < 1，用于缩短等效基线）
     * @param diff 第一次测向得到的伪谱数组（360 点）
     * @param data 实测数据（天线对和相位差信息）
     * @param angle 输出的最终测向角度（度）
     * @param quality 输出的测向质量（0-100）
     */
    void calSecondDoaByVirInterf(const vector<vector<double>> phaseTheory, const double virMultiple, vector<double> diff, InterferInfo data, double &angle, double &quality);

    /*==================天线选择与相位处理工具方法==================*/
    /**
     * @brief 计算定向天线的角度搜索范围
     *
     * 定向天线需要根据最大信号天线位置限制搜索范围，避免在不可能的方向上浪费时间。
     * 根据天线间差值判断信号指向，结合 perAngle = 360/antennaNum 计算角度区间。
     *
     * @param index 用于测向的天线序号列表
     * @param cutSequence 天线对序列（切刀顺序）
     * @param AntennaNum 总天线数
     * @param max_index 载噪比最大的天线对索引
     * @param startAngle 输出的搜索起始角度（度）
     * @param endAngle 输出的搜索结束角度（度）
     */
    void calAngleSerchRange(const vector<int> index, const vector<vector<int>> cutSequence, const int AntennaNum, int max_index, int &startAngle, int &endAngle);

    /**
     * @brief 根据天线序号组合生成所有可能的相位差对
     *
     * 利用传递性原理，从已知相位差推导出未知的相位差组合：
     *   - 若已知 diff(A,B) 和 diff(B,C)，则 diff(A,C) = diff(A,B) + diff(B,C)
     *   - 若已知 diff(A,B) 和 diff(A,C)，则 diff(B,C) = diff(A,C) - diff(A,B)
     *
     * 此方法迭代地扩展相位差集合，直到无法生成新的天线对为止。
     *
     * @param cutSequence 输入/输出的天线对序列
     * @param phaseDiff 输入/输出的相位差数组
     */
    void setUseAntennaAndPhaseAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    // ==================== 以下为当前项目组(Spoofing)与Suppress组均未使用的函数，暂时注释保留 ====================
    // /**
    //  * @brief 从相位差数据还原各天线的绝对相位（假设第一个天线的相位为 0）
    //  *
    //  * 适用于天线对按顺序排列（相邻天线对）的场景。
    //  *
    //  * @param cutSequence 天线对序列
    //  * @param phaseDiff 相位差数组（输入为天线对间差值，输出为各天线绝对相位）
    //  * @param antennNUm 天线总数
    //  */
    // void getAntennaPhase(vector<vector<int>> &cutSequence, vector<double> &phaseDiff, int antennNUm);

    /**
     * @brief 从相位差数据还原各天线的绝对相位（指定天线顺序版本）
     *
     * 适用于任意天线对排列的场景。以 use_ant 中第一个天线为参考（相位设为 0），
     * 逐步推导其余天线的绝对相位。若某一对天线间无直接或间接相位差关系，则返回失败。
     *
     * @param use_ant 参与测向的天线序号列表
     * @param antennNUm 天线总数
     * @param cutSequence 天线对序列
     * @param phaseDiff 相位差数组（输入为天线对间差值，输出为各天线绝对相位）
     * @return 0 成功，-1 失败（天线对关系不连通）
     */
    int getAntennaPhase(const vector<int> use_ant, const int antennNUm, vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    // /**
    //  * @brief 生成所有天线对两两之间的相位差（全排列）
    //  *
    //  * 从各天线的绝对相位出发，计算所有 C(N,2) 对天线之间的相位差：
    //  *   diff[i][j] = phase[i] - phase[j]
    //  *
    //  * @param cutSequence 输出的天线对序列（全排列，共 N*(N-1)/2 对）
    //  * @param phaseDiff 输入的各天线绝对相位，输出为对应的相位差数组
    //  */
    // void getPhaseDiffAll(vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    /**
     * @brief 生成指定天线之间所有可能的相位差组合
     *
     * 与 getPhaseDiffAll 类似，但仅针对 use_cut 中指定的天线子集。
     *
     * @param use_cut 参与的天线序号列表
     * @param cutSequence 输出的天线对序列
     * @param phaseDiff 输入的绝对相位数组，输出为指定天线对之间的相位差
     */
    void getUsePhaseDiffAll(const vector<int> use_cut, vector<vector<int>> &cutSequence, vector<double> &phaseDiff);

    /**
     * @brief 定向天线模式：根据幅度信息选择用于测向的天线并计算搜索范围
     *
     * 处理流程：
     *   1. 按幅度从大到小对天线排序，找出信号最强的天线
     *   2. 以最强天线为中心，向两侧选取相邻天线，构成测向天线子集
     *   3. 根据选中的天线位置计算定向搜索的角度范围
     *
     * 特别考虑了饱和情况：当天线存在饱和时，幅度最大的天线不一定是正对信号源的天线，
     * 因此采用逐步扩展的策略，覆盖信号源可能的方向。
     *
     * @param A 各天线的幅度数组
     * @param use_cut_num 需要选择的测向天线数
     * @param use_cut 输出的选中天线序号列表
     * @param startAngle 输出的搜索起始角度
     * @param endAngle 输出的搜索结束角度
     */
    void getUseAntennaByADirect(const vector<double> A, const int use_cut_num, vector<int> &use_cut, int &startAngle, int &endAngle);

    /**
     * @brief 读取频率-半径配置文件，获取各频段对应的阵列半径
     *
     * 文件格式：每两行为一组，第一行为频率区间（如 "100-200" 或 "100~200"，单位 MHz），
     * 第二行为对应的阵列半径（单位米）。
     *
     * @param adr 配置文件路径
     * @param mR 输出的频率-半径映射数组
     * @return 0 成功，-1 文件读取失败，-2 格式错误
     */
    int getRData(const string adr, vector<Rs> &mR);

    /*==================测向质量评估==================*/
    /**
     * @brief 计算测向质量（全局版本），基于理论伪谱与实际伪谱的相关系数
     *
     * 测向质量定义：quality = (corr + 1) / 2 * 100
     * 其中 corr = sum(diffTheory[i] * diff[i]) / (sqrt(sum(diffTheory^2)) * sqrt(sum(diff^2)))
     * 即归一化互相关系数（余弦相似度），取值范围 [-1, 1]，映射到 [0, 100]。
     *
     * 质量值越高，说明最优角度的理论伪谱与整体伪谱的一致性越好，测向结果越可靠。
     *
     * @param diffTheory 理论伪谱（最优匹配角度下的理论值构成的伪谱）
     * @param diff 实际伪谱（实测数据与各角度理论值匹配得到的伪谱）
     * @return 测向质量值（0-100）
     */
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff);

    /**
     * @brief 计算测向质量（区域版本），仅在指定角度范围内计算相关系数
     *
     * 与全局版本逻辑相同，但限定在 [starAngle, endAngle] 范围内计算。
     * 适用于定向天线场景，仅关注信号可能出现的角度区间。
     *
     * @param diffTheory 理论伪谱
     * @param diff 实际伪谱
     * @param starAngle 搜索起始角度（度，闭区间）
     * @param endAngle 搜索结束角度（度，闭区间）
     * @return 测向质量值（0-100）
     */
    double getDoaMass(const vector<double> diffTheory, const vector<double> diff, int starAngle, int endAngle);

};
