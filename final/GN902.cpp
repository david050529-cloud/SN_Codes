#include "GN902.h"
#include "SpoofingDoa.h"

#include <vector>
#include <map>

using namespace std;

// =============================================================================
// 每个 GN902 实例的内部状态(引擎 + 整轮帧缓冲)。
// 以对象地址索引存放在文件内容器中，避免改动 GN902.h 中已有的类定义/成员。
//   eng       : SpoofingDoa 引擎。构造时即 initProject()+Init()(运行参数内置在引擎代码中，
//               不读取外部配置文件)，并按 Python 流程阈值初始化各频点)。引擎内部持跨轮(跨周期)状态:
//               m_ConsecutiveAlarm / m_Tracking / m_Baselines，在本类多轮调用间持续累积。
//   buf/cuts  : 当前轮逐帧送来的 GNSSData 与对应 cutIdx(通道2天线号 1..7)。
//   calFrames : 已储存的校正刀(cutIdx=1 / 天线对{1,1})数据，每轮出现校正刀则整体刷新。
//   result    : 最近一轮喂入引擎后取出的测向结果(供 GetResult 返回)。
// =============================================================================
namespace {
struct GN902State
{
    SpoofingDoa *eng;                // 欺骗检测+测向引擎(循环切刀)
    std::vector<GNSSData> buf;       // 当前轮待处理帧(原始到达顺序)
    std::vector<int> cuts;           // 每帧对应 cutIdx
    std::vector<GNSSData> calFrames; // 储存的校正刀数据
    SpoofingResult result;           // 最近一轮测向结果

    GN902State() : eng(0) { clearResult(); }

    ~GN902State()
    {
        if (eng)
        {
            delete eng;
            eng = 0;
        }
    }

    void clearResult()
    {
        result.i_Count = 0;
        for (int i = 0; i < 24; ++i)
        {
            result.i_SatelliteAngle[i].i_Sys = 0;
            result.i_SatelliteAngle[i].i_Type = 0;
            result.i_SatelliteAngle[i].i_Alarm = 0;
            result.i_SatelliteAngle[i].i_Angle = -1.0;
            result.i_SatelliteAngle[i].i_Count = 0;
        }
    }
};

// 实例容器: 对象地址 -> 状态(不改动 GN902.h 的既有类定义/成员)
std::map<const GN902 *, GN902State *> g_gn902State;
} // namespace

// GN902 实例容器(声明于 interface.h)，供 Create_GN902/Release_GN902 等 C 接口使用
std::vector<GN902 *> GN902Container;

GN902::GN902(){
    GN902State *st = new GN902State();
    // SpoofingDoa 构造即 initProject()+Init()：运行参数已内置在引擎 Init() 中(不读配置文件)，
    // 并按 Python 流程对齐各频点阈值。
    st->eng = new SpoofingDoa();
    if (st->eng != 0)
    {
        // GN902 运行参数：循环切刀检测开、全向半径 0.1865m(重建理论模板)、
        // 暂不平滑(整轮喂帧时按每刀实际帧数自适应)。
        st->eng->configCyclicRuntime(true, 0, false, 0.1865);
        // 切刀顺序 = {1,1},{1,2},{1,3},{1,4},{1,5},{1,6},{1,7}
        // (7 组天线对 = 校正刀 + 六刀测向，对齐 Python code 0/9/57/17/25/33/1)
        const int cutSeq[14] = {1, 1, 1, 2, 1, 3, 1, 4, 1, 5, 1, 6, 1, 7};
        st->eng->setCutSquence(14, cutSeq);
    }
    g_gn902State[this] = st;
}

GN902::~GN902(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it != g_gn902State.end())
    {
        delete it->second; // 内部释放引擎
        g_gn902State.erase(it);
    }
}


// 设置阈值检测参数
// @param phsDiffThreshold 位相差阈值
// @param satelliteCountThreshold 卫星数阈值
// @param cutCountThreshold 通道2对应天线阈值
// @param sysEnum 系统类型
// @param typeEnum 类型
void GN902::SetThresholdDetection(double phsDiffThreshold, double satelliteCountThreshold, double cutCountThreshold, int sysEnum, int typeEnum){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    SpoofingDoa *eng = it->second->eng;
    // 参数顺序: (系统sysEnum, 频点typeEnum, 卫星数阈值satelliteCountThreshold, 相位差阈值phsDiffThreshold)
    eng->setThresholdDetectionDoa(sysEnum, typeEnum, (int)satelliteCountThreshold, phsDiffThreshold);
    // cutCountThreshold = 连续确认刀数 p(对应 ALARM_CONSECUTIVE_P)，>0 时生效
    if (cutCountThreshold > 0)
    {
        eng->setDetectionRecordNum((int)cutCountThreshold);
    }
    return;
}

// 设置数据
// @param data 数据指针
// @param cutIdx 通道二天线索引
// @param endFlag 结束标志
void GN902::SetData(const GNSSData* data, int cutIdx, int endFlag){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        return;
    }
    GN902State *st = it->second;
    // 帧空或 cutIdx 非法(1=校正刀, 2..7=六测向刀)则忽略该帧，不打断整轮累积
    if (data == 0 || cutIdx < 1 || cutIdx > 7)
    {
        return;
    }
    // 非最后一帧数据，保存数据
    if (!endFlag){
        // 保存数据
        st->buf.push_back(*data);
        st->cuts.push_back(cutIdx);
        return;
    }
    // 最后一帧数据，先保存后整轮处理：
    // Detect() 组批喂入引擎(更新告警flag/跟踪)，Doa() 判断是否测向并取出结果
    else{
        st->buf.push_back(*data);
        st->cuts.push_back(cutIdx);
        Detect();
        Doa();
    }
    return;
}

// 获取结果
// @param result 结果结构体
void GN902::GetResult(SpoofingResult& result){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end())
    {
        result.i_Count = 0;
        return;
    }
    // 取最近一轮(喂入引擎后)的测向结果
    result = it->second->result;
    return;
}

// 检测
// 整轮组批并喂入引擎，完成循环切刀欺骗检测与跟踪(跨轮状态在引擎内连续累积)。
void GN902::Detect(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    GN902State *st = it->second;

    // 取出本轮缓冲(swap 后引用安全)
    std::vector<GNSSData> frameData;
    std::vector<int> frameCuts;
    frameData.swap(st->buf);
    frameCuts.swap(st->cuts);

    // 拆帧：
    //   cutIdx=1  -> 校正刀(code=0 / 天线对1-1)，不参与测向，先存为校正数据；
    //   cutIdx=2..7 -> 六刀测向刀(对应 code 9,57,17,25,33,1 / {1,2}..{1,7})。
    std::vector<GNSSData> calNew;
    std::vector<std::vector<int> > detByCut(6); // index = cutIdx-2 (0..5)
    for (size_t i = 0; i < frameCuts.size(); ++i)
    {
        int c = frameCuts[i];
        if (c == 1)
        {
            calNew.push_back(frameData[i]);
        }
        else
        {
            detByCut[c - 2].push_back((int)i); // c 已保证 2..7
        }
    }

    // 本轮出现了校正刀数据 -> 刷新储存的校正数据(替换旧校正，取本轮为准)
    if (!calNew.empty())
    {
        st->calFrames = calNew;
    }
    if (st->calFrames.empty())
    {
        return; // 尚无校正数据则丢弃本轮(跨轮连续/跟踪状态不变)
    }

    // 测向轮需六刀齐全；缺刀则丢弃本轮
    for (int r = 0; r < 6; ++r)
    {
        if (detByCut[r].empty())
        {
            return;
        }
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
    if (detUniform && (int)st->calFrames.size() >= C)
    {
        calRow.assign(st->calFrames.end() - C, st->calFrames.end());
    }
    else
    {
        calRow = st->calFrames;
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
    // 跨轮 m_ConsecutiveAlarm / m_Tracking / m_Baselines 在引擎内持续累积，缺刀位
    // 用上一周期相位差补齐六条基线。
    st->eng->configCyclicRuntime(true, oneCutFrams, smooth, 0.0);
    st->eng->setGNSSData(batch.data(), (int)batch.size());
    return;
}

// 测向
// 从引擎取出本轮测向结果，存入内部 result 供 GetResult 返回。
void GN902::Doa(){
    std::map<const GN902 *, GN902State *>::iterator it = g_gn902State.find(this);
    if (it == g_gn902State.end() || it->second->eng == 0)
    {
        return;
    }
    GN902State *st = it->second;
    st->clearResult();
    st->eng->getAngleSpoofingDoa(st->result);
    return;
}
