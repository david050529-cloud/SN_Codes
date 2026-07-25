// =============================================================================
// 文件名: jointdiag.cpp
// 功能描述: 联合对角化(Joint Diagonalization)算法实现
//   用于同频多信号盲源分离(BSS)，即多个同频率信号同时到达阵列时的测向。
//
// =============================================================================
// 算法原理概述:
//
//   问题: 阵列接收信号 X = A*S + N
//     - X: MxN 阵列接收数据 (M天线, N采样点)
//     - A: Mxd 阵列流型(导向矢量矩阵) (d个信号)
//     - S: dxN 信号源矩阵
//     - N: 噪声
//
//   目标: 从X中分离出每个信号的导向矢量a(theta_k)
//
//   传统MUSIC/ESPRIT依赖信号子空间分解，但需要信号数已知且不相关。
//   联合对角化方法利用信号的统计独立性，构造多个延迟协方差矩阵，
//   通过同时对角化这些矩阵来估计解混矩阵W。
//
//   关键步骤:
//     1. getRAll(): 构造多个伪协方差矩阵 R(k), k=0,1,...,K-1
//        R(k) = X(:, 1:N-k) * X(:, 1+k:N)^H  (延迟相关)
//        构造K个不同的协方差矩阵用于联合对角化
//
//     2. getJointA(): 提取d个信号对应的阵列流型
//        a. 特征分解: R(0) = U*Lambda*U^H, 取前d个特征向量
//        b. 白化压缩: 计算中间矩阵 multiD = U^H * R(k) * U
//        c. Uwedge(): 联合对角化求解W，使得所有 R(k) 被同时对角化
//        d. 阵列流型: A = R(0) * U * W (分离后的导向矢量)
//
//     3. 对每个分离出的信号，使用相关干涉仪或幅相法进行DOA估计
//
// =============================================================================
// 伪协方差矩阵说明:
//   区别于常规协方差 E[x(t)*x(t)^H]，伪协方差使用时间延迟相关:
//     R(k) = E[x(t) * x(t+k)^H]
//   不同延迟k对应不同的相位信息，联合对角化利用这些冗余信息
//   来提高分离精度。矩阵数量K=m_RAll_Num，延迟间隔=m_RAll_PointNum。
// =============================================================================
#include "SuppressDoa.h"
// 同频多信号测向
// 同频信号分离

// =============================================================================
// getJointdiagByInterf(): 联合对角化 + 相关干涉仪测向（默认版本）
// 流程:
//   1. getRAll(RAll): 从m_Data构建伪协方差矩阵集合
//   2. getJointA(RAll, A, sigNum): 联合对角化获得分离后的阵列流型A
//   3. 对每个分离信号: 从A提取相位差 -> 相关干涉仪匹配测向
// =============================================================================
void SuppressDoa::getJointdiagByInterf()
{
    vector<MatrixXcd> RAll;
    MatrixXd Actual;
    double tp_phs;
    getRAll(RAll); // 构建m_RAll_Num个伪协方差矩阵
    MatrixXcd A;
    getJointA(RAll, A, m_signalNum); // 联合对角化获得分离后的阵列流型

    InterferInfo data;
    data.i_Start = 0;
    data.i_End = 359;
    int num = 0;
    vector<double> diff;
    // 对每个分离出的信号分别进行相关干涉仪测向
    for (int sig = 0; sig < m_signalNum; ++sig)
    {
        num = 0;

        // 从分离后的阵列流型A提取各天线对的相位差
        for (int i = 0; i < m_AntennaNum - 1; ++i)
        {
            for (int j = i + 1; j < m_AntennaNum; ++j)
            {
                tp_phs = arg(A(j, sig) / A(i, sig)); // 相位差 = arg(aj/ai)
                data.i_Phase_Diff[num] = tp_phs;
                data.i_AntennaSq[num][0] = j + 1;
                data.i_AntennaSq[num][1] = i + 1;
                num++;
            }
        }
        diff.clear();
        data.i_Phase_Len = num;

        double tp_angle;
        double tp_quality;
        calInterfer(m_Theory, data, tp_angle, tp_quality, diff); // 利用相关干涉仪测向
        m_angles[sig] = tp_angle;
        m_qualities[sig] = tp_quality;
    }
}

// =============================================================================
// getJointdiagByAP(): 联合对角化 + 幅相法测向
// 与上述流程类似，但最后使用幅相法(calAmpPhase)而非相关干涉仪进行角度匹配
// 适用于有仿真阵列流型数据的场景（如GN560的DML测向模式）
// 参数:
//   data: 外部提供的三维IQ数据
//   CutSequence: 对应的切刀序列
// =============================================================================
void SuppressDoa::getJointdiagByAP(const std::vector<std::vector<std::vector<std::complex<double>>>> data,
    const vector<vector<int>> CutSequence)//联合对角化+幅相法
{
    vector<MatrixXcd> RAll;
    getRAll(RAll,data,CutSequence); // 使用外部数据构建伪协方差矩阵
    MatrixXcd A;
    getJointA(RAll, A, m_signalNum); // 联合对角化分离信号
    int num = 0;
    vector<double> diff;

    double tp_phs;
    vector<complex<double>> actualA;
    for (int sig = 0; sig < m_signalNum; ++sig)
    {
        num = 0;
        actualA.clear();
        actualA.resize(m_AntennaNum);
        for (int i = 0; i < m_AntennaNum ; ++i)
        {
            actualA[i] = -A(i, sig); // 取反（适配幅相法匹配方向约定）
        }

        double tp_angle;
        double tp_quality;
       // calInterfer(m_Theory, data, tp_angle, tp_quality, diff); // 利用相关干涉仪测向
        calAmpPhase(m_SimulateA,actualA, tp_angle, tp_quality, diff); // 幅相法测向

        m_angles[sig] = tp_angle;
        m_qualities[sig] = tp_quality;
    }

}


// =============================================================================
// getJointdiagByInterf(angles, qualities): 重载版本，返回vector而非写入成员变量
// 与无参数版本功能相同，但输出通过参数返回
// =============================================================================
void SuppressDoa::getJointdiagByInterf(vector<double> &angles, vector<double> &qualities)
{
    vector<MatrixXcd> RAll;
    MatrixXd Actual;
    double tp_phs;
    getRAll(RAll);
    MatrixXcd A;
    getJointA(RAll, A, m_signalNum);

    InterferInfo data;
    data.i_Start = 0;
    data.i_End = 359;
    int num = 0;
    vector<double> diff;
    angles.resize(m_signalNum);
    qualities.resize(m_signalNum);
    for (int sig = 0; sig < m_signalNum; ++sig)
    {
        num = 0;

        for (int i = 0; i < m_AntennaNum - 1; ++i)
        {
            for (int j = i + 1; j < m_AntennaNum; ++j)
            {
                tp_phs = arg(A(j, sig) / A(i, sig));
                data.i_Phase_Diff[num] = tp_phs;
                data.i_AntennaSq[num][0] = j + 1;
                data.i_AntennaSq[num][1] = i + 1;
                num++;
            }
        }
        diff.clear();
        data.i_Phase_Len = num;

        double tp_angle;
        double tp_quality;
        calInterfer(m_Theory, data, tp_angle, tp_quality, diff); // 利用相关干涉仪测向
        angles[sig] = tp_angle;
        qualities[sig] = tp_quality;
    }
}

// =============================================================================
// getRAll: 构造伪协方差矩阵集合（使用成员变量m_Data）
//
// 伪协方差矩阵定义:
//   R(k) = X * X^H_k
//   其中 X_k = X(:, 1+k : end)  即延迟k个采样点的数据矩阵
//
// 对于多切刀场景，每次切刀只采集部分天线的数据，
// 需要将各切刀的子协方差矩阵按天线序号填充到完整的MxM矩阵中。
//
// 延迟参数: start = count (count=0,1,...,m_RAll_Num-1)
//   延迟从0开始递增，0延迟即常规协方差矩阵
//
// 最终: RAll[count] = R + R^H (强制Hermitian对称)
//   保证协方差矩阵的共轭对称性
// =============================================================================
void SuppressDoa::getRAll(vector<MatrixXcd> &RAll)
{
    // MatrixXcd  X代表动态，c代表复数，d代表双精度double
    RAll.resize(m_RAll_Num); // K个伪协方差矩阵
    MatrixXcd R(m_AntennaNum, m_AntennaNum);
    int num1, num2;

    for (int count = 0; count < m_RAll_Num; ++count)
    {
        int start = count; // 延迟量=count个采样点
        for (int cut = 0; cut < m_cutNum; ++cut)
        {
            // tp1: 当前切刀的未延迟数据矩阵 (channelNum x (N-start))
            // tp2: 当前切刀的延迟数据矩阵 (channelNum x (N-start))
            MatrixXcd tp1(m_channelNum, m_usePointNum - start);
            MatrixXcd tp2(m_channelNum, m_usePointNum - start);
            for (int i = 0; i < m_channelNum; ++i)
            {
                for (int j = 0; j < m_usePointNum - start; ++j)
                {
                    tp1(i, j) = m_Data[cut][i][j];        // x(t)
                    tp2(i, j) = m_Data[cut][i][j + start]; // x(t+start)
                }
            }

            //    cout << "cut=" << cut << "  ,tp1->" << tp1(0, 0) << "  ,tp2->" << tp2(0, 0) << "  ,tp1->" << tp1(2, m_usePointNum - start - 1) << "  ,tp2->" << tp2(2, m_usePointNum - start - 1) << endl;
            // localR = tp1 * tp2^H: 通道间的延迟协方差
            MatrixXcd localR = tp1 * tp2.adjoint(); // 共轭转置c
            // 将子协方差按实际天线序号填充到完整矩阵的对应位置
            for (int i = 0; i < m_channelNum; ++i)
            {
                for (int j = 0; j < m_channelNum; ++j)
                {
                    num1 = m_cutSequence[cut][i] - 1; // 转为0-based
                    num2 = m_cutSequence[cut][j] - 1;
                    R(num1, num2) = localR(i, j);
                }
            }
        }

        // 强制Hermitian: R_all = R + R^H，确保协方差矩阵共轭对称
        RAll[count] = R + R.adjoint();
    }
}

// =============================================================================
// getRAll: 构造伪协方差矩阵集合（使用外部数据，重载版本）
// 与上述方法逻辑相同，但数据源为外部提供的data参数
// 用于单通道多信号FFT处理后的数据
// =============================================================================
void SuppressDoa::getRAll(vector<MatrixXcd> &RAll,
                          const std::vector<std::vector<std::vector<std::complex<double>>>> data,
                          const vector<vector<int>> CutSequence)
{
    // MatrixXcd  X代表动态，c代表复数，d代表双精度double
    RAll.resize(m_RAll_Num);
    MatrixXcd R(m_AntennaNum, m_AntennaNum);
    int num1, num2;

    int channel = data[0].size(); // 通道数/天线数
    for (int count = 0; count < m_RAll_Num; ++count)
    {
        int start = count;
        for (int cut = 0; cut < data.size(); ++cut) // 切刀次数，若是全通道或单通道则应该是1刀
        {
            MatrixXcd tp1(channel, m_usePointNum - start);
            MatrixXcd tp2(channel, m_usePointNum - start);
            for (int i = 0; i < channel; ++i) // 通道数
            {
                for (int j = 0; j < data[cut][i].size() - start; ++j)
                {
                    tp1(i, j) = data[cut][i][j];
                    tp2(i, j) = data[cut][i][j + start];
                }
            }

            //    cout << "cut=" << cut << "  ,tp1->" << tp1(0, 0) << "  ,tp2->" << tp2(0, 0) << "  ,tp1->" << tp1(2, m_usePointNum - start - 1) << "  ,tp2->" << tp2(2, m_usePointNum - start - 1) << endl;
            MatrixXcd localR = tp1 * tp2.adjoint(); // 共轭转置c

            for (int i = 0; i < CutSequence[cut].size(); ++i)
            {
                for (int j = 0; j < CutSequence[cut].size(); ++j)
                {
                    num1 = CutSequence[cut][i] - 1;
                    num2 = CutSequence[cut][j] - 1;
                    R(num1, num2) = localR(i, j);
                }
            }
        }

        RAll[count] = R + R.adjoint(); // 强制Hermitian
    }
}

// =============================================================================
// getJointA: 联合对角化提取分离后的阵列流型
//
// 算法步骤:
//   1. 特征分解: RAll[0] = V * D * V^{-1} (复数特征分解)
//      取最大的sigNum个特征值对应的特征向量组成U矩阵
//      (特征值按升序排列，取最后sigNum个 = start = M-1, ..., M-sigNum)
//
//   2. 白化投影: 将所有RAll[i]投影到信号子空间
//      matrixTp = U^H * RAll[i] * U   (sigNum x sigNum)
//      将结果堆叠成 multiD = [M1, M2, ..., MK]
//      其中 M_i = U^H * RAll[i] * U
//
//   3. Uwedge: 联合对角化迭代求解解混矩阵W
//      目标: 找W使得 W^H*M_i*W 对每个i都是(近似)对角阵
//      即同时对角化所有中间矩阵
//
//   4. 阵列流型: A = RAll[0] * U * W
//      A的每一列对应一个分离信号的导向矢量
// =============================================================================
void SuppressDoa::getJointA(const vector<MatrixXcd> &RAll, MatrixXcd &A, int sigNum)
{
    // 步骤1: 对第一个伪协方差矩阵进行特征分解
    ComplexEigenSolver<MatrixXcd> ces;
    ces.compute(RAll[0]);

    MatrixXcd matrixTp = ces.eigenvectors(); // 特征向量矩阵
    MatrixXcd U(m_AntennaNum, sigNum);       // 信号子空间基 (M x d)
    int start = m_AntennaNum - 1;
    // 取最大的d个特征值对应的特征向量（Eigen的特征值按从小到大排列）
    for (int i = 0; i < m_AntennaNum; ++i)
    {
        for (int j = 0; j < sigNum; ++j)
        {
            U(i, j) = matrixTp(i, start - j); // 从大到小选择列
        }
    }

    // 步骤2: 白化投影，构造联合对角化输入矩阵 multiD
    MatrixXcd multiD(sigNum, sigNum * RAll.size()); // 水平堆叠: [M_0, M_1, ..., M_{K-1}]
    for (int i = 0; i < RAll.size(); ++i)
    {
        matrixTp = U.adjoint() * RAll[i] * U; // sigNum x sigNum 白化后的协方差
        for (int j = 0; j < sigNum; ++j)
        {
            for (int k = 0; k < sigNum; ++k)
            {
                multiD(j, i * sigNum + k) = matrixTp(j, k);
            }
        }
    }

    // 步骤3: 联合对角化，求解解混矩阵W
    MatrixXcd W;
    Uwedge(multiD, W); // 核心迭代算法

    // 步骤4: 重建阵列流型
    // A = RAll[0] * U * W，每一列对应一个分离信号的导向矢量
    A = RAll[0] * (U * W);
}

// =============================================================================
// Uwedge: 联合对角化核心算法 (Joint Diagonalization by Unitary Wedges)
//
// 算法目标:
//   寻找酉矩阵 W，使得 W^H * M_k * W 对所有的 k=1,...,K 尽可能接近对角矩阵。
//   即同时对角化多个矩阵(水平堆叠在M中)。
//
// 输入:
//   M (d x Kd): 水平堆叠的对角化目标矩阵集合 M = [M_0 | M_1 | ... | M_{K-1}]
//   其中 d = sigNum (信号数), K = L (矩阵个数)
//
// 输出:
//   W (d x d): 解混矩阵, 满足 W^H * M_k * W 近似对角
//
// 算法详述 (基于Newton-Raphson迭代):
//
//  1. 初始化:
//     - 取M的前d列作为M0 (第一个矩阵)
//     - 特征分解 M0 = H * E * H^H
//     - 初始解混矩阵 W = E^(-1/2) * H^H  (白化)
//     - 变换 Ms = W * M * W^H  (白化后的堆叠矩阵)
//     - 提取 Rs = diag(Ms_k) for each k  (每个子矩阵的对角线)
//
//  2. 代价函数:
//     crit = sum(|Ms_ij|^2) - sum(|Rs_i|^2)
//     即所有元素平方和 减去 对角线元素平方和
//     crit = 非对角元素的Frobenius范数平方，越小越好
//
//  3. 迭代优化 (while crit > eps && iter < 100):
//     a. 计算相关矩阵 B = Rs * Rs^H (实部)
//     b. 计算Q = diag(B) (各行的对角权重)
//     c. 计算C1矩阵: 衡量非对角元素的影响
//        利用元素乘积 Ms(i, id + k*d) * conj(Rs(id, k)) 计算梯度
//     d. 构造 Hessian 矩阵 D0 = 2*(B.*B - Q*Q') + I
//     e. 牛顿更新 A0 = I + (C1^H.*B - diag(Q)*C1) ./ D0
//        (逐元素除法 ./ 即cwiseQuotient)
//     f. 伪逆更新 W = pinv(A0) * W
//        (由于A0可能奇异，使用最小二乘伪逆)
//     g. 重归一化: diag = 1/sqrt(|diag(W*M0*W^H)|)
//        (保证输出的模值归一化)
//     h. 更新 Ms = W * M * W^H
//        更新 Rs = diag(Ms_k)
//     i. 收敛检查: crit = (sum|A0| - d) / d^2
//
//  4. 输出: W = W^H (转置共轭, 还原到原始空间的解混矩阵)
//
// 数学背景:
//   联合对角化是盲源分离中的核心技术。
//   对于独立信号源，不同的延迟协方差矩阵具有相同的特征向量结构(阵列流型)，
//   只是特征值(信号功率)不同。通过联合对角化多个协方差矩阵，
//   可以比单一矩阵的特征分解更精确地估计出解混矩阵。
//
//   本算法属于 "非正交联合对角化" (Non-Orthogonal Joint Diagonalization)，
//   不要求W是酉矩阵，因此适用于非白化场景。
// =============================================================================
void SuppressDoa::Uwedge(const MatrixXcd &M, MatrixXcd &W)
{
    int d = M.rows();     // 信号数 = sigNum
    int Md = M.cols();    // 总列数 = sigNum * 矩阵个数
    int L = Md / d;       // 矩阵个数K
    int iter = 0;          // 迭代计数器
    double eps = 0.1;     // 收敛阈值

    // ---- 初始化: 取第一个矩阵M0进行特征分解 ----
    Eigen::MatrixXcd EM0(d, d);
    MatrixXcd M0(d, d);
    for (int i = 0; i < d; ++i)
    {
        for (int j = 0; j < d; ++j)
        {
            EM0(i, j) = M(i, j); // 第一个d×d子矩阵
            M0(i, j) = M(i, j);  // 保存副本
        }
    }
    Eigen::ComplexEigenSolver<Eigen::MatrixXcd> eig; // 特征分解
    eig.compute(EM0);
    Eigen::VectorXcd e = eig.eigenvalues();    // 特征值(复数)
    Eigen::MatrixXcd h = eig.eigenvectors();   // 特征向量矩阵
    MatrixXcd H(d, d);
    int tp_s = d - 1;
    // 特征值和特征向量按从大到小排列
    for (int i = 0; i < d; ++i)
    {
        for (int j = 0; j < d; ++j)
        {
            H(i, j) = h(i, tp_s - j); // 从大到小选择列（大特征值排前面）
        }
    }
    // 构造白化矩阵: E = diag(1/sqrt(|e_1|), ..., 1/sqrt(|e_d|))
    MatrixXcd E(d, d);
    for (int i = 0; i < d; ++i)
    {
        for (int j = 0; j < d; ++j)
        {
            if (i == j)
            {
                E(i, j) = 1 / sqrt(abs(e(tp_s - i))); // 特征值取模的平方根倒数
            }
            else
            {
                E(i, j) = 0;
            }
        }
    }
    // 初始解混矩阵: W = E * H^H (白化矩阵)
    W = E * H.adjoint();

    // ---- 白化变换: Ms = W * M, 然后对每个子矩阵再次变换 ----
    MatrixXcd Ms(d, Md);
    MatrixXcd Ms_trans(d, Md);
    MatrixXcd Rs(d, L);
    MatrixXcd tpM(d, d);
    MatrixXcd tpMs(d, d);
    int start;
    for (int k = 0; k < L; ++k)
    {
        start = k * d;
        for (int i = 0; i < d; ++i)
        {
            for (int j = 0; j < d; ++j)
            {
                tpM(i, j) = M(i, start + j); // 提取第k个子矩阵
            }
        }
        tpMs = W * tpM * W.adjoint(); // 双线性变换
        for (int i = 0; i < d; ++i)
        {
            for (int j = 0; j < d; ++j)
            {
                Ms(i, start + j) = tpMs(i, j);       // 保存变换结果
                // Ms_trans(i, start + j) = conj(tpMs(i, j));
                Ms_trans(i, start + j) = tpMs(j, i);  // 转置(非共轭)
            }
        }
        // Rs的第k列 = diag(第k个变换后矩阵)
        for (int i = 0; i < d; ++i)
        {
            Rs(i, k) = Ms(i, start + i); // 对角线元素
        }
    }

    // ---- 计算初始代价函数值 ----
    double sum1 = 0;
    double sum2 = 0;
    for (int i = 0; i < d; ++i)
    {
        for (int j = 0; j < Md; ++j)
        {
            sum1 += abs(Ms(i, j)) * abs(Ms(i, j)); // 所有元素的Frobenius范数平方
        }
        for (int j = 0; j < L; ++j)
        {
            sum2 += abs(Rs(i, j)) * abs(Rs(i, j)); // 对角线元素的模平方和
        }
    }
    double crit = sum1 - sum2; // 非对角元素能量 = 总能量 - 对角能量

    // 辅助变量
    MatrixXcd tp1, tp2, tp3;
    MatrixXcd A0;
    MatrixXcd D0;
    MatrixXcd ones(d, 1); // 全1列向量
    MatrixXcd Raux;
    for (int i = 0; i < d; ++i)
    {
        ones(i, 0) = 1;
    }
    MatrixXcd eyed(d, d); // 单位矩阵
    for (int i = 0; i < d; ++i)
    {
        for (int j = 0; j < d; ++j)
        {
            if (i == j)
            {
                eyed(i, j) = 1;
            }
            else
            {
                eyed(i, j) = 0;
            }
        }
    }
    Eigen::VectorXd diag;

    // ---- Newton-Raphson 迭代主循环 ----
    while (crit > eps && iter < 100)
    {
        // 计算 B = Rs * Rs^H (实部)，衡量对角线元素之间的相关性
        MatrixXcd tp = Rs * Rs.adjoint();
        MatrixXd B(d, d);
        for (int i = 0; i < d; ++i)
        {
            for (int j = 0; j < d; ++j)
            {
                B(i, j) = tp(i, j).real();
            }
        }
        // Q = diag(B)，即每行的对角权重
        MatrixXd Q(d, 1);
        for (int i = 0; i < d; ++i)
        {
            Q(i, 0) = B(i, i);
        }

        // ---- 计算C1矩阵: 梯度的关键组成部分 ----
        // C1(i, id) = sum_k[ Ms(i, id+k*d) * conj(Rs(id, k)) * conj(Rs(i, k)) ]
        //            + conj(sum_k[ Ms_trans(i, id+k*d) * conj(Rs(id, k)) * conj(Rs(i, k)) ])
        // 对应论文中的梯度项
        MatrixXcd C1(d, d);
        for (int id = 0; id < d; ++id)
        {
            MatrixXcd RsTp(1, L);
            for (int i = 0; i < L; ++i)
            {
                RsTp(0, i) = conj(Rs(id, i)); // Rs的第id行取共轭
            }

            tp1 = ones * RsTp; // d×K 矩阵，每行是 Rs(id,:) 的共轭

            tp2.resize(d, L);
            tp3.resize(d, L);
            for (int i = 0; i < d; ++i)
            {
                for (int j = 0; j < L; ++j)
                {
                    // tp2(i,j) = Ms(i, id + j*d) * conj(Rs(id, j))
                    tp2(i, j) = Ms(i, id + j * d) * tp1(i, j);
                    // tp3(i,j) = Ms_trans(i, id + j*d) * conj(Rs(id, j))
                    tp3(i, j) = Ms_trans(i, id + j * d) * tp1(i, j);
                }
            }

            for (int i = 0; i < d; ++i)
            {
                complex<double> sum1(0, 0);
                complex<double> sum2(0, 0);
                for (int j = 0; j < L; ++j)
                {
                    sum1 += tp2(i, j); // 沿k求和
                    sum2 += tp3(i, j);
                }
                C1(i, id) = sum1 + conj(sum2); // 复合梯度项
            }
        }

        // 构造对角矩阵 tp1 = diag(Q)
        tp1.resize(d, d);
        for (int i = 0; i < d; ++i)
        {
            for (int j = 0; j < d; ++j)
            {
                if (i == j)
                {
                    tp1(i, j) = Q(i, 0);
                }
                else
                {
                    tp1(i, j) = 0;
                }
            }
        }

        // ---- 计算 Hessian 和 Newton 更新 ----
        // D0 = 2*(B.*B - Q*Q') + I  (近似Hessian矩阵)
        MatrixXcd D0 = 2 * (B.cwiseProduct(B) - Q * Q.adjoint()) + MatrixXd::Identity(d, d);

        // A0 = I + (C1^H.*B - diag(Q)*C1) ./ D0  (逐元素除法)
        // 这是 Newton-Raphson 更新的核心: delta_W = inv(Hessian) * gradient
        A0 = eyed + ((C1.adjoint()).cwiseProduct(B) - tp1 * C1).cwiseQuotient(D0);
        // cwiseQuotient 逐元素相除
        //  A0 = complexMatrix(eye(d, 1.)) + elemDivd(elemMult(trH(C1), complexMatrix(B)) - tp1 * C1,
        //                                            complexMatrix(2.0 * (elemMult(B, B) - multTr(Q, Q)) + eye(d, 1.0)));

        // 伪逆更新 W = pinv(A0) * W
        // 使用完全正交分解的伪逆，处理A0可能奇异的情况
        W = A0.completeOrthogonalDecomposition().pseudoInverse() * W; // 伪逆矩阵

        // ---- 重归一化: 保证解混矩阵的尺度 ----
        // diag = 1/sqrt(|diag(W*M0*W^H)|)
        Raux = W * M0 * W.adjoint();

        diag = 1.0 / Raux.diagonal().cwiseAbs().array().sqrt();
        W = diag.asDiagonal() * W; // W = diag(1/sqrt(|diag|)) * W

        // ---- 更新 Ms 和 Rs ----
        Ms = W * M;
        for (int k = 0; k < L; ++k)
        {
            int ini = k * d;
            for (int i = ini; i < ini + d; ++i)
            {
                tpMs.col(i - ini) = Ms.col(i);
                // tpMs.setColumn(Ms.getColumn(i), i - ini);
            }
            tpMs = tpMs * W.adjoint();
            for (int i = 0; i < d; ++i)
            {
                Ms.col(i + ini) = tpMs.col(i);
                // Ms.setColumn(tpMs.getColumn(i), i + ini);
            }
            // 更新Rs对角线
            for (int i = 0; i < d; i++)
            {
                Rs(i, k) = tpMs(i, i);
            }

            tpMs = tpMs.transpose().eval();

            for (int i = 0; i < d; ++i)
            {
                Ms_trans.col(i + ini) = tpMs.col(i);
            }
        }

        // 收敛检查: crit = (sum|A0_ij| - d) / (d*d)
        // 当A0接近单位矩阵时(sum|A0| ≈ d)，crit趋近于0
        crit = ((A0.cwiseAbs()).sum() - d) / (d * d);
        ++iter;
    }
    // crit = 0;
    // MatrixXd tp(Ms.rows(), Ms.cols());
    // tp = Ms.cwiseAbs();
    // tp *= tp;
    // crit += tp.sum();
    // tp.resize(Rs.rows(), Rs.cols());
    // tp = Rs.cwiseAbs();
    // tp *= tp;
    // crit -= tp.sum();

    // 输出: W = W^H (转换回原始空间的解混矩阵)
    W = W.adjoint().eval();
}