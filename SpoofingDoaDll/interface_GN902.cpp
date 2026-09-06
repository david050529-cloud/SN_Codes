// =============================================================================
// 文件名: interface_GN902.cpp
// 功能描述: GN902 欺骗检测/测向 DLL 接口实现（对应 接口.md 的 GN902 接口）
//
// 分析结论(为什么需要新增这层接口):
//   现有通用接口(interface.h)是「一次传一整轮数组 + 内部固定切刀分帧」的批量模型，
//   且结果返回丰富 SpoofingResult；而 GN902 主控是按「逐帧流式 + cutIdx(通道2天线)
//   + endFlag(整轮结束)」送数、Init 用数组配置阈值、GetResult 只要单个角度。
//   二者无法直接对接，故新增本层 wrapper：
//     GNSSData(逐帧) -> 校正刀(cutIdx=1/code0)单独存为校正数据；测向轮由六刀
//     (cutIdx=2..7) 组成 -> endFlag=1 触发：用(储存或本轮)校正帧 + 六刀
//     SpoofingDoa::setGNSSData 走循环切刀「检测+测向」，与当前 Python 流程一致。
//
// 约定与假设(接口.md 未文档化，按本项目 Python 流程对齐，需主控侧核对)：
//   1. cutIdx = 通道2 对应天线号(1..7)：
//        cutIdx=1 -> 第一刀校正刀，天线对 {1,1}(=Python code=0)，**不参与测向**，
//                    传入的该刀数据先储存为校正数据(每轮出现则刷新储存)；
//        cutIdx=2..7 -> 六刀测向/检测刀 {1,2}..{1,7}(=Python code 9,57,17,25,33,1)，
//                    测向只用这六刀。
//      引擎切刀行 j = cutIdx-1：row0=校正(cutIdx1)，row1..6=六测向刀(cutIdx2..7)。
//   2. Init_GN902 数组按下面「阈值表」顺序(长度>=12)提供；未列频点用默认
//      (卫星数阈值 2、相位差阈值 5°，严格大于触发)。
//   3. cutCountThreshold[0] = 连续确认刀数 p(对应 ALARM_CONSECUTIVE_P，建议 2)。
//   4. 测向轮六刀每刀帧数一致(且与校正帧数一致)时按 Python 方式做
//      「稳定性过滤+圆形均值」(与 dat 8 帧/刀一致)；不一致时退化为每刀取末尾
//      (最稳定)一帧喂入。
//   5. GetResult_GN902 返回该轮后各频点测向角度的圆均值(0..359)；无有效结果 angle=-1。
// =============================================================================
#include "interface_GN902.h"
#include "../publicFunctionDoa/publicFunctionDoa.h"

#include <vector>
#include <map>
#include <cmath>

using namespace PublicSpace;

// =============================================================================
// 版本号（独立于 interface.h 的 Version，避免重复定义）
// =============================================================================
#ifdef _WIN32
char GN902Version[] = "V1.0.0.20260906_GN902_X86_WIN64";
#else
char GN902Version[] = "V1.0.0.20260906_GN902_X86_LIN64";
#endif

// =============================================================================
// GN902 阈值表（顺序固定，与 Python detection_lib.ORIGINAL_CONFIG_TEXT 对齐）
// 数组下标 = 本表顺序；主控传入的 satelliteCountThreshold[]/phsDiffThreshold[]
// 长度需 >= GN902_THRESH_CNT(=12)。
// =============================================================================
namespace {
const int GN902_THRESH_CNT = 12;
const int GN902_THRESH[GN902_THRESH_CNT][2] = {
    {0, 2},   // GPS     L5C   (Sys=0,Type=2)
    {1, 0},   // GLONASS G1    (Sys=1,Type=0)
    {1, 1},   // GLONASS G2    (Sys=1,Type=1)
    {3, 2},   // Galileo E1C   (Sys=3,Type=2)
    {3, 12},  // Galileo E5a   (Sys=3,Type=12)
    {3, 17},  // Galileo E5b   (Sys=3,Type=17)
    {4, 0},   // BDS     B1I   (Sys=4,Type=0)
    {4, 2},   // BDS     B3I   (Sys=4,Type=2)
    {4, 8},   // BDS     B1C   (Sys=4,Type=8)
    {4, 17},  // BDS     B2I   (Sys=4,Type=17)
    {4, 19},  // BDS     B2b   (Sys=4,Type=19)
    {4, 34},  // BDS     B1X   (Sys=4,Type=34)
};
} // namespace

// =============================================================================
// 对象上下文：引擎实例 + 整轮帧缓冲
// =============================================================================
struct GN902Ctx
{
    SpoofingDoa *eng = nullptr;        // SpoofingDoa 引擎（循环切刀检测 + 相关干涉仪测向）
    std::vector<GNSSData> buf;         // 当前待处理帧(原始到达顺序，含校正刀与测向刀)
    std::vector<int> cuts;             // 每帧对应的 cutIdx
    std::vector<GNSSData> calFrames;   // 已储存的校正刀(code=0/cutIdx=1)数据(每轮有则刷新)
};

namespace {
std::map<int, GN902Ctx *> g_gn902;     // 实例容器
int g_num = 0;                         // 实例 id 计数器

GN902Ctx *findCtx(int id)
{
    auto it = g_gn902.find(id);
    return (it == g_gn902.end()) ? nullptr : it->second;
}

// 角度圆均值并归一化到 [0,360)
double circularMeanWrap(const std::vector<double> &degs)
{
    if (degs.empty())
    {
        return -1.0;
    }
    double s = 0.0;
    double c = 0.0;
    for (double d : degs)
    {
        double rad = d * PI / 180.0;
        s += std::sin(rad);
        c += std::cos(rad);
    }
    double mean = std::atan2(s, c) * 180.0 / PI;
    if (mean < 0)
    {
        mean += 360.0;
    }
    return mean;
}
} // namespace

// =============================================================================
// 创建 GN902 对象
// 注意：SpoofingDoa 构造即读取 ./DoaBSpoofingConfig.txt 完成频率/理论模板等初始化；
//       此处再按 GN902 覆盖运行参数(半径 0.1865、循环检测开、切刀顺序 {1,1}..{1,7})。
// =============================================================================
int Create_GN902(int &id)
{
    while (g_gn902.find(g_num) != g_gn902.end())
    {
        ++g_num;
    }
    id = g_num;

    GN902Ctx *ctx = new GN902Ctx();
    ctx->eng = new SpoofingDoa();
    if (ctx->eng == nullptr)
    {
        delete ctx;
        return 1;
    }

    // GN902 运行参数：循环切刀检测开、半径 0.1865m（重建理论模板）、暂不平滑(整轮喂帧时自适应)
    ctx->eng->configCyclicRuntime(true, 0, false, 0.1865);
    // 切刀顺序 = {1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7}（对齐 Python code 0/9/57/17/25/33/1）
    const int cutSeq[14] = {1, 1, 1, 2, 1, 3, 1, 4, 1, 5, 1, 6, 1, 7};
    ctx->eng->setCutSquence(14, cutSeq);

    g_gn902[g_num] = ctx;
    ++g_num;
    return 0;
}

// =============================================================================
// 初始化 GN902 对象：配置阈值 + 连续确认刀数
// 数组约定见文件头注释；任一数组传 NULL 则该项保持 DLL 默认。
// =============================================================================
int Init_GN902(int id, double *phsDiffThreshold, double *satelliteCountThreshold,
               double *cutCountThreshold)
{
    getLogCont(id);
    GN902Ctx *ctx = findCtx(id);
    if (ctx == nullptr)
    {
        printf("Init_GN902: invalid id %d\n", id);
        return 1;
    }
    SpoofingDoa *target = ctx->eng;

    if (satelliteCountThreshold != nullptr)
    {
        for (int i = 0; i < GN902_THRESH_CNT; ++i)
        {
            int sys = GN902_THRESH[i][0];
            int type = GN902_THRESH[i][1];
            double phs = (phsDiffThreshold != nullptr) ? phsDiffThreshold[i] : -1.0;
            target->setThresholdDetectionDoa((int)satelliteCountThreshold[i], phs, sys, type);
        }
    }
    if (cutCountThreshold != nullptr && cutCountThreshold[0] > 0)
    {
        target->setDetectionRecordNum((int)cutCountThreshold[0]);
    }
    PublicSpace::Log("Init_GN902 ok, id=%d\n", id);
    return 0;
}

// =============================================================================
// 逐帧送入：整轮缓冲，endFlag=1 触发一次完整循环切刀「检测+测向」
// =============================================================================
int SetData_GN902(int id, const GNSSData *data, int cutIdx, int endFlag)
{
    getLogCont(id);
    GN902Ctx *ctx = findCtx(id);
    if (ctx == nullptr)
    {
        printf("SetData_GN902: invalid id %d\n", id);
        return 1;
    }
    if (data == nullptr)
    {
        return 1;
    }
    if (cutIdx < 1 || cutIdx > 7)
    {
        PublicSpace::Log("SetData_GN902: ignore frame with invalid cutIdx=%d\n", cutIdx);
        return 0; // 非法切刀标识不缓存；不打断整轮
    }

    ctx->buf.push_back(*data);
    ctx->cuts.push_back(cutIdx);

    if (endFlag == 0)
    {
        return 0; // 继续累积整轮数据
    }

    // ---------- 端到端处理(整轮结束)：取出缓冲（避免清空后引用失效） ----------
    std::vector<GNSSData> frameData;
    std::vector<int> frameCuts;
    frameData.swap(ctx->buf);
    frameCuts.swap(ctx->cuts);

    // 拆帧：
    //   cutIdx=1  -> 校正刀(code=0 / 天线对1-1)，不参与测向，先存为校正数据；
    //   cutIdx=2..7 -> 六刀测向刀(对应 code 9,57,17,25,33,1 / {1,2}..{1,7})。
    std::vector<GNSSData> calNew;
    std::vector<std::vector<int>> detByCut(6);  // index = cutIdx-2 (0..5)
    for (size_t i = 0; i < frameCuts.size(); ++i)
    {
        int c = frameCuts[i];
        if (c == 1)
        {
            calNew.push_back(frameData[i]);
        }
        else
        {
            detByCut[c - 2].push_back((int)i);  // c 已保证 2..7
        }
    }

    // 本轮出现了校正刀数据 -> 刷新储存的校正数据(替换旧校正，取本轮为准)
    if (!calNew.empty())
    {
        ctx->calFrames = calNew;
    }
    if (ctx->calFrames.empty())
    {
        PublicSpace::Log("SetData_GN902: no calibration(code0) data stored yet -> drop this round\n");
        return 0;
    }

    // 测向轮需六刀齐全；缺刀则丢弃本轮(跨轮连续/跟踪状态不变)
    bool complete = true;
    for (int r = 0; r < 6; ++r)
    {
        if (detByCut[r].empty())
        {
            complete = false;
            break;
        }
    }
    if (!complete)
    {
        PublicSpace::Log("SetData_GN902: six-cut round incomplete at endFlag -> dropped\n");
        return 0;
    }

    // 六测向刀每刀帧数是否一致
    int C = (int)detByCut[0].size();
    bool detUniform = true;
    for (int r = 1; r < 6; ++r)
    {
        if ((int)detByCut[r].size() != C)
        {
            detUniform = false;
            break;
        }
    }

    // 校正行(引擎 row0)：六刀帧数统一为 C 且校正帧数 >= C 时，取校正帧末尾 C 帧
    // (最稳定尾段)以便整体平滑；否则使用全部校正帧(将落入“每刀取末帧”路径)。
    std::vector<GNSSData> calRow;
    if (detUniform && (int)ctx->calFrames.size() >= C)
    {
        calRow.assign(ctx->calFrames.end() - C, ctx->calFrames.end());
    }
    else
    {
        calRow = ctx->calFrames;
    }

    bool uniform = detUniform && ((int)calRow.size() == C) && C > 0;
    int oneCutFrams = uniform ? C : 1;
    bool smooth = uniform;

    // 按引擎行序组批：row0 = 校正(cutIdx1)，row1..6 = 六刀测向(cutIdx2..7)
    std::vector<GNSSData> batch;
    if (uniform)
    {
        batch.reserve((size_t)7 * C);
        for (int k = 0; k < C; ++k)
        {
            batch.push_back(calRow[k]);
        }
        for (int r = 0; r < 6; ++r)
        {
            for (int k = 0; k < C; ++k)
            {
                batch.push_back(frameData[detByCut[r][k]]);
            }
        }
    }
    else
    {
        // 帧数不一致(或校正帧不足) -> 每行取末尾(最稳定)一帧
        batch.reserve(7);
        batch.push_back(calRow.back());
        for (int r = 0; r < 6; ++r)
        {
            batch.push_back(frameData[detByCut[r].back()]);
        }
    }

    // 运行一轮：引擎 row0 校正刀用于通道校正，六刀测向刀做检测+测向；
    // 跨轮 m_ConsecutiveAlarm / m_Tracking 在引擎内持续累积。
    ctx->eng->configCyclicRuntime(true, oneCutFrams, smooth, 0.0);
    ctx->eng->setGNSSData(batch.data(), (int)batch.size());
    return 0;
}

// =============================================================================
// 获取测向结果（单角 = 各频点测向角的圆均值；无有效结果 angle=-1）
// =============================================================================
int GetResult_GN902(int id, double &angle)
{
    getLogCont(id);
    GN902Ctx *ctx = findCtx(id);
    if (ctx == nullptr)
    {
        printf("GetResult_GN902: invalid id %d\n", id);
        return 1;
    }
    angle = -1.0;
    SpoofingResult result;
    ctx->eng->getAngleSpoofingDoa(result);
    std::vector<double> doas;
    for (int i = 0; i < result.i_Count; ++i)
    {
        int a = result.i_SatelliteAngle[i].i_Angle;
        if (a >= 0)
        {
            doas.push_back((double)a);
        }
    }
    angle = circularMeanWrap(doas);
    return 0;
}

// =============================================================================
// 释放 GN902 对象
// =============================================================================
int Release_GN902(int id)
{
    GN902Ctx *ctx = findCtx(id);
    if (ctx == nullptr)
    {
        printf("Release_GN902: invalid id %d\n", id);
        return 1;
    }
    if (ctx->eng != nullptr)
    {
        delete ctx->eng;
    }
    delete ctx;
    g_gn902.erase(id);
    return 0;
}

char *GetALGVersionGN902(void)
{
    return GN902Version;
}
