# Python ↔ C++ 欺骗检测/测向流程一致性对比与修改记录（2026-09-06）

> 分析对象：
> - Python 参考：`jiance_flow/detection_main.py` + `jiance_flow/detection_lib.py`（2026-09-06 版，git HEAD 一致）
> - C++ 设备代码：`SN_Codes/SpoofingDoaDll/*`（Alarm.cpp / Corrected.cpp / SpoofingDoa.cpp / SpoofingDoa.h / InitData.cpp / PreparationData.cpp / Interf.cpp / Omni.cpp / DoaBSpoofingConfig.txt）
>
> 结论：总体流程框架一致（逐刀相位差 → 稳定行向量 → 校正 → 排序+滑动窗聚类 → 跨刀连续确认 → 持续跟踪测向）。
> 本记录列出**本次已按当前 Python 对齐的差异**与**确认保留/仍存在的差异**。

---

## 1. 逐环节流程对应关系

| 环节 | Python（detection_lib / detection_main） | C++（SpoofingDoaDll） | 一致性 |
|---|---|---|---|
| 天线对/切刀 | code 0 校正 + code 9,57,17,25,33,1 测向（pair 2..7） | cutSequence 第 0 刀 {1,1} 校正 + {1,2}..{1,7} 测向 | 一致（顺序相同） |
| 相位差计算 | `(p2−p1) % 360`（端口2−端口1，度） | `frac(port1.phase − port2.phase)`（端口1−端口2，周） | 符号相反，**已知保留项**（各自内部自洽） |
| 数据质量门限 | 两端口载噪比均 ≥ 35dB 才纳入 | 匹配对两端口均 ≥ m_Snr_Threshold(=35) 才产生 | 一致 |
| 刀内多帧平滑 | `build_vectors_and_detect` / `compute_calibration`：稳定采样数 ≥ min(刀内帧数, MIN_STABLE_SAMPLES=3)，最小覆盖弧 < STABILITY_RANGE_DEG=15°，取圆形均值 | `getSmoothData` + `calSmoothData` | **已对齐**（见 §2.1–2.3） |
| 跳半周 | `check_stability`：360° 跨度大时折叠 [0,180) 再看；HALF_CYCLE_CORRECT=False → 不修正、按不稳定丢弃 | `calSmoothData` 折叠后仍不稳定 → 载噪比置 0 丢弃 | 一致 |
| 通道校正 | 校正刀稳定卫星圆形均值；GLONASS 逐星、其余按频点统一 | `calCorrectionOffset` + `calCorrecteData` | **已对齐**（见 §2.4） |
| 欺骗聚类 | `cluster_satellites`：排序+复制 +360°、滑动窗找跨度<阈值最大集合，**cnt > count_thr** | `calAlarmByPhaseDiff`：同上逻辑，**bestCount > 阈值** | 一致 |
| 连续确认 | `consecutive` 连续 p=ALARM_CONSECUTIVE_P=2 次确认 | `m_ConsecutiveAlarm` ≥ m_Detection_Recodds_Num(=2) | 一致 |
| 持续跟踪 | 确认后 cluster_sats 累积、测向跟踪、消失输出最近 DOA | `m_Tracking` + setDataAngle 步骤 8 | 一致（候选集已对齐见 §2.5） |
| 测向候选星 | 只对 tracking.cluster_sats 的卫星测向；要求 6 个测向 code 齐全 | 循环流程只保留确认频点 | **已对齐到 cluster_sats** |

---

## 2. 本次已修改项（按当前 Python 对齐）

### 2.1 稳定性/平滑常量（Alarm.cpp:15,17）
- 旧：`STABILITY_RANGE_DEG = 5.0`、`MIN_STABLE_SAMPLES = 5`
- 新：`STABILITY_RANGE_DEG = 15.0`、`MIN_STABLE_SAMPLES = 3`
- 依据：`detection_lib.py:68`（=15）、`detection_lib.py:179`（=3）。
- 影响：放宽每刀保留卫星的条件 → 更多稳定卫星进入行向量、聚类与校正偏移，检出结果向当前 Python 看齐。

### 2.2 检测阈值表（SpoofingDoa.cpp 构造 24–39）
按 `detection_lib.ORIGINAL_CONFIG_TEXT` 核对，修正 3 处：
- GLONASS G1 `(Sys=1,Type=0)`：3 → **2**（触发≥3）
- GLONASS G2 `(Sys=1,Type=1)`：4 → **3**（触发≥4）
- BDS B1C `(Sys=4,Type=8)`：未设（落默认）→ 显式 **2**（触发≥3）
其余条目已与 Python 一致，未动。

### 2.3 默认颗数阈值（SpoofingDoa.h:508、DoaBSpoofingConfig.txt:35、InitData initDetectionThreshold）
- 默认兜底 countThreshold：3 → **2**（对应 Python 默认规则 `Sys=-1,Type=-1,countThreshold=2`）。
- 注：未在构造表中显式列出的频点（GPS L1/L1C/L2C、GAL E1B/E6C 等）回退到该默认值。

### 2.4 校正“从该刀第一帧就存在”门槛移除（SpoofingDoa.cpp `calSmoothData`、`getSmoothData`）
- 旧：校正刀要求卫星从该刀第 0 帧(第 1 秒)就存在（`requireFromStart`）。
- Python `compute_calibration` 明确“不要求信号从该刀第一秒就存在”，只要求覆盖 min(刀内帧数, MIN_STABLE_SAMPLES) 帧。
- 已删除 `requireFromStart` 参数及相关逻辑。

### 2.5 平滑帧聚合不再“要求全部帧有效”（SpoofingDoa.cpp `getSmoothData:902-905`）
- 旧：刀内 8 帧汇总复用了 `getSatelliteDataPhaseDiffB`，受 `deletePrnFlag=1` 影响 → 缺任一帧的卫星被整体丢弃（比 Python 严苛太多）。
- 新：按帧聚合时临时 `m_Delete_Prn_Flag = 0`，保留缺帧卫星，交给 `calSmoothData` 的覆盖判定 min(8, MIN_STABLE_SAMPLES)=3 取舍，与 Python `build_vectors_and_detect`/`compute_calibration` 一致。

### 2.6 无该频点校正数据时保持原值（Corrected.cpp `calCorrecteData`）
- 旧：测向模式(m_Detection_Tag=0)下，某 (系统,频点) 若完全无校正星 → 载噪比置 0、不参与后续（Python 会偏移 0 继续用原始相位差）。
- 新：一律偏移 0、保持原值，与 Python `offset_fn` 返回 0 一致。

### 2.7 测向候选星限定为已确认 cluster_sats（SpoofingDoa.cpp `getCyclicDetectionData:1353-1370`）
- 旧：只按“已确认频点 typeInt”过滤，会给该频点**全部**卫星测向。
- 新：过滤到 `m_Tracking[typeInt].cluster_sats`，与 Python `run_doa_one`（只对 tracking.cluster_sats 测向）一致。

---

## 3. 确认保留 / 仍存在的差异（本次不改）

| # | 差异 | 说明与影响 | 处置 |
|---|---|---|---|
| 1 | 相位差符号 `port1−port2` vs Python `port2−port1` | 聚类/校正/测向在 C++ 内部自洽；若与 Python 模板同套用会导致角度镜像。改动会翻转设备现测向方向，属高风险 | 保留（同既有文档） |
| 2 | 校正刀载噪比过滤 | C++ 自校准刀同样经 ≥35dB 门限（Python compute_calibration 不过滤） | 保留（C++ 更严、影响小） |
| 3 | DOA 数值口径 | Python `correlative_doa`（6 基线余弦均值、quality=(corr+1)/2×100、阈值 10、多星圆均值）；C++ 用 `Arithmetic::calInterfer`（getDoaMass 质量、伪谱归一化）+ calAngle 内恒做 `getDetection180` 半周复测 | 保留（本次范围“检测链路为主”，未动共享 Arithmetic） |
| 4 | DOA 数据门控粒度 | C++ 测向输入 `dataB` 走 `deletePrnFlag=1`（要求全部 7 个 dataA 槽含校正刀都有数据）；Python 只要求 6 个测向 code 基线齐全、不含 code0。仅在“6 个测向位齐全但自校准刀缺失”的边缘卫星上有差异 | 保留，已知限制 |
| 5 | “消失输出最近 DOA”时机 | C++ 在一轮(整周期)结束且末刀未报警时输出；Python 在流结束时统一输出。均保留跟踪，行为接近 | 保留 |
| 6 | 卫星随刀短暂缺失的跨周期基线 | Python 的 code 基线仅在“报警刀”上累积；C++ 经数据路径一次取整周期各刀有效相位。两者对“某卫星在部分刀仍被保留并测向”的门控略有差异（见 #4） | 保留，已在 §4.4 说明 |

---

## 4. 修改文件与关键行索引

| 文件 | 修改点 | 位置 |
|---|---|---|
| `SpoofingDoaDll/Alarm.cpp` | 稳定性/平滑常量 | :15, :17 |
| `SpoofingDoaDll/SpoofingDoa.cpp` | 检测阈值表（G1/G2/B1C） | 构造 24–39（:28,:29,:36） |
| 〃 | `getSmoothData` 帧聚合放宽 | :897–:905 |
| 〃 | `calSmoothData` 移除 requireFromStart | :933（函数） |
| 〃 | `getCyclicDetectionData` 候选星=cluster_sats | :1353–:1370 |
| `SpoofingDoaDll/SpoofingDoa.h` | `calSmoothData` 签名、`m_Detection_Threshold_Num` 默认 | :305–:307, :508 |
| `SpoofingDoaDll/Corrected.cpp` | `calCorrecteData` 无校正即保持原值 | :205 起 |
| `SpoofingDoaDll/DoaBSpoofingConfig.txt` | `detectionNum` 默认 3→2 | :35 |

> 依赖：需要真实设备/录波数据做回归对比。改动为数值与门控口径对齐，建议用同一批 debug/dat 数据在改前/改后对比确认检出集合变化符合预期（尤其稳定性阈值 5°→15° 会扩大保留卫星集）。

---

## 5. 对照 Python 常量速查

| 常量/规则 | Python | C++（本次对齐后） |
|---|---|---|
| CNR_MIN_DB | 35.0 | 35.0 |
| STABILITY_RANGE_DEG | 15 | 15 |
| MIN_STABLE_SAMPLES | 3 | 3 |
| HALF_CYCLE_CORRECT | False（跳半周按不稳定丢弃） | 折叠后跨度不够 → 丢弃 |
| ALARM_CONSECUTIVE_P / p | 2 | m_Detection_Recodds_Num=2 |
| 默认 countThreshold（未列频点） | 2 | 2 |
| GLO G1 / G2 count | 2 / 3 | 2 / 3 |
| BDS B1C count | 2 | 2 |
