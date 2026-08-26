# 欺骗检测流程改造变更记录（6e7f43f → HEAD）

> 基线提交：`6e7f43f`（分析源代码）
> 当前提交：`2189795`（HEAD）
> 生成日期：2026-08-26

## 1. 变更范围总览

### 1.1 期间提交

| 提交 | 说明 |
|------|------|
| ba9ab93 | 映射 |
| a9ac980 | 文档 |
| fcb0b25 | 将 calAlarmByPhaseDiff 改写为 cluster_satellites「排序+滑动窗口跨度聚类」 |
| c877736 | 注释掉角度聚类/载噪比聚类、补移植四步 |
| 2d4e205 | 校正「从起始帧存在+均匀分布」、配置文件、消失时输出最终 DOA |
| f090134 | 修改配置文件，注释不使用的函数 |
| 2305714 | code=0 ↔ 同天线切刀 {1,1}（注释确认）、校正策略改写、旧路径清理 |
| 2189795 | 修正 setDataAlarm 注释（getAlarm 仅剩相位差法） |

### 1.2 文件清单

**与欺骗检测流程相关（7 个）：**

| 文件 | 变动量 | 性质 |
|------|--------|------|
| `SpoofingDoaDll/Alarm.cpp` | 452 行变动 | 检测算法整体替换 |
| `SpoofingDoaDll/Corrected.cpp` | 288 行变动 | 校正策略改写 |
| `SpoofingDoaDll/SpoofingDoa.cpp` | 436 行变动 | 主流程新增循环检测链路 |
| `SpoofingDoaDll/SpoofingDoa.h` | 73 行变动 | 成员/声明增删 |
| `SpoofingDoaDll/InitData.cpp` | 33 行变动 | 删除 initRecords |
| `SpoofingDoaDll/interface.h` | 2 行变动 | 注释更新 |
| `SpoofingDoaDll/DoaBSpoofingConfig.txt` | 新增 103 行 | 配置文件（baseline 无） |

**与流程无关（4 类）：**

| 文件 | 说明 |
|------|------|
| `Arithmetic/*`、`publicFunctionDoa/*` | 仅注释掉两个项目组均未使用的函数（f090134） |
| `.vscode/*` | 构建调试配置 |
| 根目录 3 个 `.md` | 卫星频点映射、阈值配置分析、虚假警告说明文档 |
| `SpoofingDoaGN560Dll/`、`SuppressDoa*Dll/` | 未改动 |

---

## 2. Alarm.cpp —— 检测算法整体替换

### 2.1 删除的三个旧方法（死代码 / 已不参与流程）

| 函数 | 原职责 | 删除原因 |
|------|--------|----------|
| `calAlarmByAngle` | 角度法：统计到达角相近卫星数 | 无调用方，角度聚类已从新流程移除 |
| `calAlarmBySnr` | 载噪比法：统计载噪比≤1dB 相近卫星数 | 无调用方，载噪比仅作质量门限 |
| `setDetectionRecords` | 滑动窗口：连续 N 帧才最终判定 | 旧路径确认机制，循环流程改用 `m_ConsecutiveAlarm` |

### 2.2 新增常量（对应 Python）

```cpp
const double SpoofingDoa::CNR_MIN_DB = 35.0;           // 载噪比质量门限：两端口都需 ≥35dB
const double SpoofingDoa::STABILITY_RANGE_DEG = 5.0;   // 稳定性阈值：最小覆盖弧 < 5° 判为稳定
const int SpoofingDoa::MIN_STABLE_SAMPLES = 5;         // 每刀至少需要的有效采样帧数
```

### 2.3 新增圆形统计工具（对应 Python simplified_detection）

| 函数 | 对应 Python | 说明 |
|------|-------------|------|
| `normalizeAngle180` | `normalize_angle_180` | 归一化到 [-180°, 180°) |
| `circularMeanDeg` | `circular_mean` | 角度圆形均值(度) |
| `circularSpanDeg` | `circular_span` | 360° 圆最小覆盖弧跨度(度) |
| `foldHalfCycle` | `fold_half_cycle` | 折叠到 [0°, 180°)，消除跳半周歧义 |
| `circularSpan180Deg` | `circular_span_180` | 180° 半周圆上的最小覆盖弧跨度(度) |

### 2.4 `calAlarmByPhaseDiff` 重写

**旧算法**：
1. 以每颗星为参考，数相位差差值 `< 阈值` 的邻近星，取最大聚类
2. 若最大聚类数 = 阈值-1，用「组内载噪比差 ≤1dB」做双重确认
3. 以周为单位的相位差直接做差取绝对值（存在 0/360 边界歧义）

**新算法**（Python `cluster_satellites`）：
1. 数据筛选：两端口载噪比均 ≥35dB
2. 相位差周→度并归一化到 [-180°, 180°)
3. 按相位差排序后复制一份整体 +360°，用滑动窗口找「跨度 < 阈值」的最大卫星集合（窗口限长 n 防同一星重复计数）
4. 最大聚类卫星数 ≥ 阈值判为欺骗

---

## 3. Corrected.cpp —— 校正策略改写为 Python compute_calibration

### 3.1 `setCorrectionData` 改为瘦包装

- **旧**：筛选同天线刀 → 逐星调 `calCorrectionData`（高载噪比保留 + `|ΔCNR|<10dB` 过滤）→ 多星平滑出综合值(prn=-1)
- **新**：筛选 `{1,1}` 同天线自校准刀（= Python code=0）→ 调 `calCorrectionOffset`

### 3.2 新增 `calCorrectionOffset`（对应 Python compute_calibration）

筛选规则：
1. 载噪比有效性检查（`<1e-3` 跳过，数据完整性判断，非质量门限）
2. **从起始帧存在**：每次切刀持续 8s(=8 帧)，卫星须从该刀**第一帧（第一秒）**就存在 ——
   该过滤由上游 `getSmoothData/calSmoothData` 的 `requireFromStart` 完成
   （不满足的卫星载噪比已置 0，此处按 `<1e-3` 跳过），本函数**不再按「第一个校正刀」过滤**
3. **相位差稳定**：`circularSpanDeg < STABILITY_RANGE_DEG(5°)`
4. **均匀分布**：有效采样数 ≥ `min(校正刀数, MIN_STABLE_SAMPLES)`

偏移组合：
- **非 GLONASS**：每 (系统,频点) 取各稳定卫星偏移的**圆形均值**，存入 `prn=-1`
- **GLONASS(FDMA)**：**逐卫星**偏移，存入对应 prn
- 每周期重建 `m_CorrectionData`（与 Python 每周期重算 cal_rx/cal_glo 一致）

### 3.3 删除 `calCorrectionData`（高载噪比保留逻辑）

### 3.4 `calCorrecteData` 重写（对应 Python offset_fn）

- **旧**：有该星校正值→逐星减；无则回退频点综合值 prn=-1
- **新**：
  - GLONASS：逐星查偏移（无则偏移 0 不校正）
  - 非 GLONASS：查 `prn=-1` 频点综合偏移（无则偏移 0 不校正）
  - 无该频点校正数据：检测模式保持原值；测向模式载噪比置 0（不进入测向）
- 公式：校正后相位差 = 原始相位差 − 校正偏移

---

## 4. SpoofingDoa.cpp —— 主流程新增循环检测链路

### 4.1 新增循环切刀检测（对应 Python 主流程）

**`getCyclicDetectionData`**：
1. 跳过同天线自校准刀（`{1,1}`，仅用于校正）
2. 每刀按频点 `calAlarmByPhaseDiff` 聚类，记录报警卫星号集合
3. 跨刀连续确认：本刀报警 `m_ConsecutiveAlarm` +1，未报警清零；达到 `m_Detection_Recodds_Num` 次进入跟踪 `m_Tracking`
4. 只保留已确认（跟踪中）频点的卫星数据，供后续相关干涉仪测向

**`resetCyclicDetection`**：清空 `m_ConsecutiveAlarm` 与 `m_Tracking`（Init 时调用，替代原 `initRecords`）

**步骤 8（setDataAngle 末尾）**：更新跟踪中频点的最近一次 DOA（各星 DOA 圆形均值）；若本轮消失则把最近一次 DOA 作为报警结果保留在 `m_AngleResultData`（对应 Python 消失时输出最终 DOA）

### 4.2 流程入口分派（步骤 5）

```cpp
if (m_Cyclic_Detection_Flag != 0)       // 新增：循环切刀检测
    getCyclicDetectionData(dataA);
else if (m_Doa_Detection_Flag != 0)     // 原有：旧检测流程
    getSpoofingDetectionData(dataA);
```

### 4.3 `calSmoothData` / `getSmoothData` 重写

- **旧**：以参考刀数邻近帧（差<阈值），一致性最高组转复数取均值
- **新**（Python `check_stability` / `fold_half_cycle`）：
  1. 收集载噪比有效样本（周→度）
  2. `requireFromStart=true`（校正刀）要求卫星从该刀第一帧（第一秒）就存在
     （每次切刀持续 8s=8 帧；并非「出现在第一个校正刀」）
  3. 均匀分布：有效采样数 ≥ min(总帧数, MIN_STABLE_SAMPLES)
  4. 360° 覆盖弧 < 5° → 稳定，取圆形均值
  5. 折叠 180° 后跨度 < 5° → 跳半周，按波动大处理（载噪比置 0）
  6. 其余不稳定样本丢弃（载噪比置 0）
- `getSmoothData` 对同天线自校准刀传入 `requireFromStart=true`

### 4.4 `setSpoofingResult`（测向结果组装）

- **旧**：测向模式用 `calAlarmByAngle` 角度聚类确定到达角
- **新**：直接输出每颗卫星 DOA，频点角度取各星 DOA 的**圆形均值**（归一化到 [0°,360°)）

### 4.5 `getAlarm`（旧路径重构）

- 删除：`calAlarmBySnr` 后备检测、`setDetectionRecords` 滑动窗口确认
- 保留：`calAlarmByPhaseDiff`（单帧判定）
- 说明：本函数仅被旧路径（`getSpoofingDetectionData`、dataLen=2 的 `setCorrectDetectionDataAlarm`）调用；循环路径的连续确认由 `getCyclicDetectionData` 承担

### 4.6 `setCorrectDetectionDataAlarm`（dataLen=2 旧校正路径）

- **旧**：内联 `calCorrectionData` + 多星平滑综合值 + 日志循环
- **新**：`GNSSData[0]` 作为单个校正刀调 `calCorrectionOffset`

### 4.7 `setConfigData` 新增配置键

```cpp
getMapData(configMap, "cyclicDetectionFlag", m_Cyclic_Detection_Flag);
```

---

## 5. SpoofingDoa.h —— 成员与声明增删

**删除**：
- `initRecords` 声明
- `calAlarmByAngle` / `calAlarmBySnr` / `setDetectionRecords` 声明
- `calCorrectionData` 声明
- `m_Detection_Records` 成员（滑动窗口队列）

**新增**：
- `#include <set>`
- 静态角度工具：`normalizeAngle180` / `circularMeanDeg` / `circularSpanDeg` / `foldHalfCycle` / `circularSpan180Deg`
- 常量：`CNR_MIN_DB` / `STABILITY_RANGE_DEG` / `MIN_STABLE_SAMPLES`
- 循环检测：`resetCyclicDetection` / `getCyclicDetectionData` 声明，`m_ConsecutiveAlarm`、`m_Tracking`（含 `TrackingInfo` 结构）、`m_Cyclic_Detection_Flag`
- `calSmoothData` 增加参数 `bool requireFromStart = false`

---

## 6. InitData.cpp / interface.h

### 6.1 InitData.cpp

- `setDetectionRecordNum`：去掉 `initRecords()` 调用，保留 `m_Detection_Recodds_Num` 赋值（现为循环流程跨刀连续确认阈值）
- 删除 `initRecords` 函数本体

### 6.2 interface.h

- 仅更新 `setDetectionRecordNum` 注释：`设置连续检测记录数` → `设置跨刀连续确认刀数(循环切刀检测)`
- **DLL 导出接口签名全部未变**

---

## 7. DoaBSpoofingConfig.txt（新增配置文件）

关键键值：

| 键 | 值 | 说明 |
|----|-----|------|
| `cyclicDetectionFlag` | 1 | 循环切刀检测开关（对应 Python 流程） |
| `detectionRecoddsNum` | 2 | 跨刀连续确认的 p（Python ALARM_CONSECUTIVE_P） |
| `detectionNum` | 4 | 欺骗检测卫星颗数阈值 |
| `phaseDiffThreshold` | 5.0 | 相位差阈值(度) |
| `cutFrams` | 8 | 一刀帧数 |
| `smoothFlg` | 1 | 平滑（稳定性过滤+圆形均值） |
| `cutThw` | `{{1,1},{1,2},...{1,7}}` | 切刀顺序，`{1,1}` 为同天线自校准刀 |

---

## 8. 输入 / 输出影响

### 8.1 输入内容格式：**未改变**

- DLL 导出接口签名全部原样（`setSpoofingDoa` / `getAngleSpoofingDoa` / `setDetectionRecordNum` 等）
- 数据结构（`GNSSData` / `SatelliteDataPhaseDiffA/B` / `SpoofingResult` / `AlarmData`）零改动
- 三种 dataLen 模式不变：1=纯检测、2=带校正检测、>2=完整测向
- 新增配置键 `cyclicDetectionFlag`（默认由配置文件控制，向后兼容）

### 8.2 输出内容

| 层面 | 是否改变 | 说明 |
|------|----------|------|
| 输出结构/格式 | **不变** | SpoofingResult、告警标志、DOA 字段与原来一致 |
| 校正数值 | **改变（预期内）** | 校正偏移改按 Python compute_calibration 计算 → 校正后相位差、聚类、最终 DOA 数值与旧版不同 |
| 循环主流程逻辑 | **不变** | 每刀聚类+跨刀连续确认+跟踪结构，确认阈值仍取 m_Detection_Recodds_Num |
| 旧路径判定时序 | **改变** | getAlarm 从「连续 N 帧确认」变为「单帧判定」 |
| 死代码删除 | **无影响** | calAlarmByAngle / calAlarmBySnr 原无调用方 |

---

## 9. 与 Python 脚本对应关系

| C++ | Python |
|-----|--------|
| `calAlarmByPhaseDiff` | `cluster_satellites`（排序+滑动窗口聚类） |
| `calCorrectionOffset` | `compute_calibration` |
| `calCorrecteData` | `offset_fn` |
| `calSmoothData` | `check_stability` / `fold_half_cycle` / `circular_mean` |
| `getCyclicDetectionData` | 主流程：逐刀聚类 + `consecutive` + `tracking` |
| `m_ConsecutiveAlarm` / `m_Detection_Recodds_Num` | `consecutive`（连续 p 次确认） |
| `m_Tracking` / 消失时输出最终 DOA | `tracking` / `last_doa` |
| `{1,1}` 同天线自校准刀 | `code=0` |
| `CNR_MIN_DB`=35 | `CNR_MIN_DB` |
| `STABILITY_RANGE_DEG`=5 | `STABILITY_RANGE_DEG` |
| `MIN_STABLE_SAMPLES`=5 | `MIN_STABLE_SAMPLES` |

---

## 10. 构建验证

- `cmake --build cmake-build-debug --target SpoofingDoaDll` 编译链接通过（13 个 TU 全部编译，仅原有警告）
- grep 确认已删除符号仅剩注释提及，无残留代码引用
