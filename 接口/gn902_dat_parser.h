#pragma once
// =============================================================================
// 文件名: gn902_dat_parser.h
// 功能描述: GN902 欺骗检测/测向 测试数据解析工具（自包含、仅依赖 C++ 标准库）
//
// 用于 main902 测试案例：解析 K827 双端口 NovAtel Msg43 原始二进制 dat 与配套
// debug 切刀时刻表，构造与欺骗检测/测向流程传入数据结构一致的 GNSSData，并
// 流式喂入 GN902 引擎（Create_GN902 / SetData_GN902 / GetResult_GN902）。
//
// 解析口径与 Python 流程 detection_lib.py 逐字段对齐：
//   - dat 帧: sync(3) + 消息头(25) + numObs(4) + n×44 字节观测记录
//   - 观测记录: satId(u32) 伪距(f64) 载波相位(f64) 多普勒(f32) 载噪比(f32) status(u32)
//   - status 解码: sys=(st>>16)&0x0f, band=(st>>16)&0xf0, sig=(st>>24)&0xff
//   - 板卡卫星号 → PRN: QZSS +62, BDS -140, GLONASS (satId&0xff)-37, 其余不变
//   - debug: range时间 锚点(本地时间↔GPS秒) + OpenAntenna code 标记 → 切刀时刻表
// =============================================================================

#include "GN902.h"   // SatelliteData / GNSSData 结构体定义

#include <vector>
#include <map>
#include <string>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <algorithm>
#include <fstream>
#include <iterator>
#include <filesystem>

namespace gn902test {

// ---------------------------------------------------------------------------
// 小端读取（x86/x64 本机字节序即小端，直接 memcpy 即可）
// ---------------------------------------------------------------------------
static inline uint32_t rdU32(const char* p) { uint32_t v; std::memcpy(&v, p, 4); return v; }
static inline double   rdF64(const char* p) { double   v; std::memcpy(&v, p, 8); return v; }
static inline float    rdF32(const char* p) { float    v; std::memcpy(&v, p, 4); return v; }

// ---------------------------------------------------------------------------
// Msg43 status 字段 → (i_Sys, i_Type)
// 三元组 (sys, band, sig) 与 Python detection_lib.SIGNAL_TABLE + FREQ_TO_SYS_TYPE
// 对齐（band 为高 4 位，保留原值 0x00/0x20/0x40/0x60/0x80/0xc0，不右移）。
// ---------------------------------------------------------------------------
static inline bool decodeSignal(uint32_t status, int& outSys, int& outType)
{
    uint32_t sys  = (status >> 16) & 0x0f;
    uint32_t band = (status >> 16) & 0xf0;
    uint32_t sig  = (status >> 24) & 0xff;

    // 组合键: sys(20..) | band(12..) | sig(0..8)
    uint32_t key = (sys << 20) | (band << 12) | (sig & 0xff);

    switch (key)
    {
    // ---- GPS ----
    case (0x0u << 20) | (0x00u << 12) | 0x08: outSys = 0; outType = 0;  return true; // L1C -> L1_CA
    case (0x0u << 20) | (0x40u << 12) | 0x08: outSys = 0; outType = 2;  return true; // L5C
    case (0x0u << 20) | (0x20u << 12) | 0x09: outSys = 0; outType = 9;  return true; // L2P -> L2_P_codeless
    case (0x0u << 20) | (0x20u << 12) | 0x0a: outSys = 0; outType = 17; return true; // L2C -> L2_C
    // ---- GLONASS ----
    case (0x1u << 20) | (0x00u << 12) | 0x08: outSys = 1; outType = 0;  return true; // G1C -> G1
    case (0x1u << 20) | (0x20u << 12) | 0x08: outSys = 1; outType = 1;  return true; // G2
    case (0x1u << 20) | (0xc0u << 12) | 0x08: outSys = 1; outType = 6;  return true; // G3
    // ---- Galileo ----
    case (0x3u << 20) | (0x40u << 12) | 0x08: outSys = 3; outType = 2;  return true; // E1C
    case (0x3u << 20) | (0x80u << 12) | 0x09: outSys = 3; outType = 12; return true; // E5A -> E5a_Q
    case (0x3u << 20) | (0x20u << 12) | 0x0a: outSys = 3; outType = 17; return true; // E5B -> E5b_Q
    // ---- BDS ----
    case (0x4u << 20) | (0x00u << 12) | 0x08: outSys = 4; outType = 0;  return true; // B1I
    case (0x4u << 20) | (0x40u << 12) | 0x08: outSys = 4; outType = 2;  return true; // B3I
    case (0x4u << 20) | (0x00u << 12) | 0x09: outSys = 4; outType = 8;  return true; // B1C
    case (0x4u << 20) | (0x20u << 12) | 0x0a: outSys = 4; outType = 17; return true; // B2I
    case (0x4u << 20) | (0x60u << 12) | 0x0a: outSys = 4; outType = 19; return true; // B2B -> B2b
    case (0x4u << 20) | (0x80u << 12) | 0x09: outSys = 4; outType = 12; return true; // B2A -> B2a
    // ---- QZSS ----
    case (0x5u << 20) | (0x00u << 12) | 0x08: outSys = 5; outType = 0;  return true; // L1C -> L1_CA
    case (0x5u << 20) | (0x20u << 12) | 0x0a: outSys = 5; outType = 17; return true; // L2C -> L2_C
    case (0x5u << 20) | (0xc0u << 12) | 0x09: outSys = 5; outType = 14; return true; // L5_Q
    default: return false;   // 未标定信号，跳过
    }
}

// ---------------------------------------------------------------------------
// 板卡内部卫星号 → PRN（与 debug 观测对齐）
//   QZSS      : PRN = satId + 62
//   BDS       : PRN = satId - 140
//   GLONASS   : PRN = (satId & 0xff) - 37
//   GPS/Galileo: PRN = satId
// ---------------------------------------------------------------------------
static inline int satIdToPrn(uint32_t satId, int sys)
{
    switch (sys)
    {
    case 5: return (int)satId + 62;                 // QZSS
    case 4: return (int)satId - 140;                // BDS
    case 1: return (int)(satId & 0xff) - 37;        // GLONASS
    default: return (int)satId;                     // GPS / Galileo
    }
}

// ---------------------------------------------------------------------------
// 读取整个文件为字节向量
// ---------------------------------------------------------------------------
static inline std::vector<char> readBinaryFile(const std::string& path)
{
    std::ifstream in(path, std::ios::binary);
    std::vector<char> data;
    if (!in) return data;
    data.assign((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    return data;
}

// ---------------------------------------------------------------------------
// 解析单个 K827 Msg43 dat 文件 → { gps_sec : [SatelliteData, ...] }
// 载波相位取 f64 字段（周，全精度）。
// ---------------------------------------------------------------------------
static inline std::map<double, std::vector<SatelliteData>> parseDatPort(const std::string& path)
{
    std::map<double, std::vector<SatelliteData>> frames;
    std::vector<char> data = readBinaryFile(path);
    if (data.empty())
    {
        std::fprintf(stderr, "[GN902] 无法打开 dat: %s\n", path.c_str());
        return frames;
    }

    static const char SYNC[3] = { (char)0xaa, (char)0x44, (char)0x12 };
    size_t n = data.size();
    size_t off = 0;

    while (off + 32 <= n)
    {
        // 查找同步字 aa 44 12
        size_t s = n;
        for (size_t i = off; i + 3 <= n; ++i)
        {
            if (data[i] == SYNC[0] && data[i + 1] == SYNC[1] && data[i + 2] == SYNC[2])
            {
                s = i;
                break;
            }
        }
        if (s == n) break;          // 无更多帧
        if (s + 32 > n) break;

        double    gpsSec = rdU32(&data[s + 16]) / 1000.0;   // GPS 毫秒 → 秒
        uint32_t  numObs = rdU32(&data[s + 28]);
        size_t    base   = s + 32;

        for (uint32_t i = 0; i < numObs; ++i)
        {
            size_t r = base + (size_t)i * 44;
            if (r + 44 > n) break;

            uint32_t status = rdU32(&data[r + 40]);
            int sys, type;
            if (!decodeSignal(status, sys, type)) continue;

            SatelliteData sd;
            sd.i_Prn          = satIdToPrn(rdU32(&data[r]), sys);
            sd.i_Sys          = sys;
            sd.i_Type         = type;
            sd.i_Psr          = rdF64(&data[r + 4]);    // 伪距(m)
            sd.i_Phase        = rdF64(&data[r + 16]);   // 载波相位(周)
            sd.i_Dop          = rdF32(&data[r + 28]);   // 多普勒(Hz)
            sd.i_Snr          = rdF32(&data[r + 32]);   // 载噪比(dB-Hz)
            sd.i_SpoofingFlag = 0;                      // 0=正常
            frames[gpsSec].push_back(sd);
        }
        off = base + (size_t)numObs * 44;   // 跳过观测区，继续找下一帧同步字
    }
    return frames;
}

// ---------------------------------------------------------------------------
// debug 切刀时刻表解析 → { (gps_sec, code), ... } 按 gps_sec 升序
//   range时间 锚点: 本地时间 ↔ GPS秒(gps_ms/1000)，单调化后线性插值；
//   OpenAntenna code 标记: 本地时刻 → 插值成 GPS 秒的切换边界。
// ---------------------------------------------------------------------------
static inline std::vector<std::pair<double, int>> parseDebugSchedule(const std::string& path)
{
    std::vector<std::pair<double, int>> schedule;
    std::vector<std::pair<double, double>> anchors;   // (localSec, gpsSec)
    std::vector<std::pair<double, int>> codes;        // (localSec, code)

    // "range时间:" 的 UTF-8 字节序列（range + 时(E6 97 B6) + 间(E9 97 B4) + ':'）
    // 用字节序列避免源码编码对中文字符串字面量的影响。
    static const char RANGE_TIME_MARK[] = "range\xE6\x97\xB6\xE9\x97\xB4:";

    std::ifstream in(path);
    if (!in)
    {
        std::fprintf(stderr, "[GN902] 无法打开 debug: %s\n", path.c_str());
        return schedule;
    }

    std::string line;
    while (std::getline(in, line))
    {
        // 行首时间戳 YYYY-MM-DD HH:MM:SS.mmm
        int Y = 0, Mo = 0, D = 0, H = 0, Mi = 0, S = 0, ms = 0;
        if (std::sscanf(line.c_str(), "%d-%d-%d %d:%d:%d.%d",
                        &Y, &Mo, &D, &H, &Mi, &S, &ms) != 7)
            continue;

        std::tm t = {};
        t.tm_year = Y - 1900; t.tm_mon = Mo - 1; t.tm_mday = D;
        t.tm_hour = H; t.tm_min = Mi; t.tm_sec = S;
        double localSec = (double)std::mktime(&t) + ms / 1000.0;

        // range时间 锚点: "range时间:<week> <gps_ms>"
        const char* p = std::strstr(line.c_str(), RANGE_TIME_MARK);
        if (p)
        {
            unsigned int week = 0, gpsms = 0;
            if (std::sscanf(p + (std::strlen(RANGE_TIME_MARK)), "%u%u", &week, &gpsms) == 2)
            {
                anchors.push_back({ localSec, gpsms / 1000.0 });
            }
        }

        // OpenAntenna 切换标记: "OpenAntenna, code = X"
        p = std::strstr(line.c_str(), "OpenAntenna");
        if (p)
        {
            const char* q = std::strstr(line.c_str(), "code =");
            if (q)
            {
                int code = 0;
                if (std::sscanf(q, "code = %d", &code) == 1)
                {
                    codes.push_back({ localSec, code });
                }
            }
        }
    }
    in.close();

    if (anchors.empty() || codes.empty()) return schedule;

    // 锚点单调化：按本地时间升序，仅保留 GPS 秒严格递增的（消除重复/回退）
    std::sort(anchors.begin(), anchors.end());
    std::vector<std::pair<double, double>> mono;
    double lastGps = -1e300;
    for (auto& a : anchors)
    {
        if (a.second > lastGps)
        {
            mono.push_back(a);
            lastGps = a.second;
        }
    }
    if (mono.empty()) return schedule;

    // 本地时间 → GPS 秒 线性插值（与 Python parse_code_schedule.interp 一致）
    auto interp = [&](double t) -> double
    {
        if (t <= mono.front().first) return mono.front().second - (mono.front().first - t);
        if (t >= mono.back().first)  return mono.back().second  + (t - mono.back().first);
        for (size_t i = 1; i < mono.size(); ++i)
        {
            if (t <= mono[i].first)
            {
                double t0 = mono[i - 1].first, t1 = mono[i].first;
                double g0 = mono[i - 1].second, g1 = mono[i].second;
                return g0 + (g1 - g0) * (t - t0) / (t1 - t0);
            }
        }
        return mono.back().second;
    };

    for (auto& c : codes)
    {
        schedule.push_back({ interp(c.first), c.second });
    }
    std::sort(schedule.begin(), schedule.end());
    return schedule;
}

// ---------------------------------------------------------------------------
// gps_sec 时刻有效的切刀 code（最后一个 ≤ gps_sec 的切换边界；早于首个切换返回 -1）
// ---------------------------------------------------------------------------
static inline int activeCode(double gpsSec, const std::vector<std::pair<double, int>>& schedule)
{
    int best = -1;
    for (auto& sc : schedule)
    {
        if (sc.first <= gpsSec) best = sc.second;
        else break;
    }
    return best;
}

// ---------------------------------------------------------------------------
// 在目录中查找端口0/端口1 dat 文件（K827Data_0_*.dat / K827Data_1_*.dat）。
// 返回是否找到，找到后写入 out0 / out1（各取字典序第一个）。
// ---------------------------------------------------------------------------
static inline bool findPortDatFiles(const std::string& dir, std::string& out0, std::string& out1)
{
    namespace fs = std::filesystem;
    std::error_code ec;
    if (!fs::is_directory(dir, ec)) return false;

    std::vector<std::string> f0, f1;
    for (fs::directory_iterator it(dir, ec), end; !ec && it != end; it.increment(ec))
    {
        if (!it->is_regular_file(ec)) continue;
        std::string name = it->path().filename().string();
        std::string full = it->path().string();
        if (name.rfind("K827Data_0_", 0) == 0 && name.size() >= 4 &&
            name.compare(name.size() - 4, 4, ".dat") == 0)
            f0.push_back(full);
        else if (name.rfind("K827Data_1_", 0) == 0 && name.size() >= 4 &&
                 name.compare(name.size() - 4, 4, ".dat") == 0)
            f1.push_back(full);
    }
    std::sort(f0.begin(), f0.end());
    std::sort(f1.begin(), f1.end());
    if (f0.empty() || f1.empty()) return false;
    out0 = f0.front();
    out1 = f1.front();
    return true;
}

// ---------------------------------------------------------------------------
// code → 通道2天线索引（cutIdx_2，即 pair_index）
//   0→1(校正) 9→2 57→3 17→4 25→5 33→6 1→7
// ---------------------------------------------------------------------------
static inline int codeToPair(int code)
{
    switch (code)
    {
    case 0:  return 1;
    case 9:  return 2;
    case 57: return 3;
    case 17: return 4;
    case 25: return 5;
    case 33: return 6;
    case 1:  return 7;
    default: return -1;
    }
}

// ---------------------------------------------------------------------------
// 由同一 GPS 秒的两端口卫星数据构造 GNSSData（与检测/测向流程输入结构一致）。
// port0 → i_PortOne(通道1/参考天线1)，port1 → i_PortTwo(通道2/切换天线)。
// 载波相位原样传入，引擎内部自行计算两端口相位差。
// ---------------------------------------------------------------------------
static inline GNSSData buildGnssData(const std::vector<SatelliteData>& p0,
                                     const std::vector<SatelliteData>& p1)
{
    GNSSData d;
    std::memset(&d, 0, sizeof(d));

    int n0 = (int)std::min<size_t>(p0.size(), 500);
    int n1 = (int)std::min<size_t>(p1.size(), 500);
    d.i_PortOneNum = n0;
    for (int i = 0; i < n0; ++i) d.i_PortOne[i] = p0[i];
    d.i_PortTwoNum = n1;
    for (int i = 0; i < n1; ++i) d.i_PortTwo[i] = p1[i];
    return d;
}

} // namespace gn902test
