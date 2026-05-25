# 双天线观测异常分析与解决方案汇报

**接收机型号**：Septentrio AsteRx-i3 D Pro+ (Firmware v1.5.2)
**实验日期**：2026-05-22 ~ 2026-05-25
**汇报对象**：导师
**记录人**：Lewin

---

## 0. TL;DR（一句话总结）

> 接收机在 `setGNSSAttitude=MultiAntenna` 模式下，**固件层面**会主动把 Aux1 天线的跟踪通道压缩到“仅供求解航向”的最小信号子集（GPS L1CA / GLO L1CA / GLO L2CA），导致 **Aux1 的原始观测量约为 Main 的 1/40，且不可通过用户命令解开**。这意味着 “主副天线各自独立 PPK，再算基线/航向” 的方案在当前硬件配置下**不可行**；后备方案是 (a) 把接收机内置的 `AttEuler` / `AuxAntPositions` 作为伪观测送进因子图，或 (b) 向 Septentrio 申请 “dual-rover” 许可。

---

## 1. 系统设置

| 项目 | 配置值 | 备注 |
|------|--------|------|
| 接收机 | AsteRx-i3 D Pro+ | Firmware v1.5.2 |
| 工作模式 | `setGNSSAttitude, MultiAntenna` | 双天线姿态模式 |
| 前端模式 | `setFrontendMode, Nominal` | 双天线均启用 |
| 信号许可 | All bands enabled | `getReceiverCapabilities` 已确认 |
| 传输 | TCP/IP（控制 28784，数据 28785）| 明文 |
| IMU 输出 | `ExtSensorMeas @ 200 Hz` | Stream 1 |
| 原始观测 | `MeasEpoch + MeasExtra @ 10 Hz` | Stream 2 |
| 姿态/基线 | `AuxAntPositions + AttEuler + AttCovEuler @ 10 Hz` | Stream 3 |
| 健康 | `ReceiverStatus + QualityInd + ReceiverTime @ 1 Hz` | Stream 4 |

录制 ~10 分钟的 `.sbf` 文件，使用 `scripts/split_recording.py` 拆分为 Main / Aux1 两个独立 SBF + IMU CSV。

---

## 2. 现状观测：双天线严重不对称

### 2.1 观测量计数（来自 `split_recording.py`）

| 项目 | Main 天线 (AntennaID=0) | Aux1 天线 (AntennaID=1) | 比值 |
|------|---|---|---|
| MeasEpoch Type1 子块总数 | **95,885** | **2,378** | **40.3 ×** |
| 跟踪到的 (signal, SV) 对 | 约 90 ~ 100 | **仅 3 ~ 4** | — |
| 平均 CN0 | 36.80 dBHz | 27.54 dBHz | — |

### 2.2 Aux1 跟踪到的信号种类（仅这三种）

```
GPS L1CA
GLO L1CA
GLO L2CA
```

→ 没有 GPS L2C / L5、没有 Galileo、没有 BeiDou、没有 GLO L3。

### 2.3 接收机自报的链路质量（QualityInd, block 4082）

| 指标 | 值 (0–10) |
|------|-----------|
| Type 1  — Main GNSS 信号质量 | **4.64** |
| Type 2  — Aux1 GNSS 信号质量 | **4.83** ← **比 Main 还略高** |
| Type 11 — Main RF 功率 | **10 / 10** |
| Type 12 — Aux1 RF 功率 | **10 / 10** |

> **关键结论**：Aux1 的 RF 链路完全健康（功率满分），自报的信号质量甚至略优于 Main，所以**问题不在线缆、不在天线、不在前端**，而在**接收机内部的通道分配策略**。

---

## 3. 获取证据所需的命令清单

将以下命令贴进汇报附录，或者现场让导师看实时输出。

### 3.1 接收机配置查询（在 Septentrio CLI 中执行）

```bash
# 1) 确认双天线模式开启
getGNSSAttitude
# 期望: getGNSSAttitude AuxAntenna1, MultiAntenna

# 2) 确认前端是双天线 Nominal，而非 SingleAnt
getFrontendMode
# 期望: getFrontendMode FrontendMode, Nominal

# 3) 确认许可的信号集合
getReceiverCapabilities
# 看 Signals 一行是否含有 GPSL2C / GAL_E1 / BDS_B1I 等

# 4) 查看当前各通道实际跟踪的卫星 / 信号
lstSatelliteTracking
lstSignalTracking
```

### 3.2 离线证据脚本（基于已录制的 `.sbf`）

```bash
# 假设录制文件: recordings/asterx-20260522-091500.sbf

# (a) 完整解码 & 校验 CRC
python3 scripts/parser.py recordings/asterx-20260522-091500.sbf

# (b) 按天线拆分输出 Main / Aux1 / IMU CSV
python3 scripts/split_recording.py recordings/asterx-20260522-091500.sbf

# (c) 输出每个天线的 (signal, SV) 列表 + CN0 直方图
python3 scripts/parser.py --dual-antenna-summary \
        recordings/asterx-20260522-091500.sbf

# (d) 输出 QualityInd 解码结果
python3 scripts/parser.py --quality-ind recordings/asterx-20260522-091500.sbf
```

### 3.3 进一步证据（推荐增补）

在 `src/receiver_config.hpp` 的默认 streams 里增加 `ChannelStatus`（block 4013），录制 60 秒后用 parser.py 解码，可以**逐通道**看到接收机把哪些通道分配给了 Aux1，从而**直接证明**“Aux1 只被允许跑指定子集” 这一假设。

```yaml
# config.yaml 示例
streams:
  - id: 5
    blocks: [ "ChannelStatus" ]
    interval: "sec1"
```

---

## 4. 根因分析（基于 Septentrio 手册）

依据：`docs/AsteRx-i3_D_Pro__Firmware_v1_5_2_Reference_Guide.pdf`

| 章节 | 内容 | 对本案的意义 |
|------|------|--------------|
| §1.8 Dual-antenna attitude setup | 描述双天线模式的硬件接线与触发条件 | 我们的接线、模式开关都正确 |
| §2.1 Channel Allocation and Signal Selection | **关键：固件保留一部分通道仅供 Aux1 用作航向解算** | **正是根因** |
| §3.2.4 `setChannelAllocation` / `setSatelliteTracking` / `setSignalTracking` | 用户可见的跟踪控制命令 | 这些命令**控制不到** Aux1 子集 |
| §3.2.8 `setGNSSAttitude` | 仅 `none` / `MultiAntenna` 两种值 | 没有 “dual-rover” 这样的第三种值 |
| §4.2.1 MeasEpoch / MeasExtra | AntennaID 在 Type 字段 bit 5..7 | 我们的解析已修正 |
| §4.2.11 AttEuler | **GNSS-only**；Mode 字段使用 RTK 浮点/固定语义 | **不是 IMU 融合后的姿态** |
| §4.2.10 INSNavCart / INSNavGeod | IMU + GNSS 紧组合解 | 目前**没有录制** |

> **手册没有公开 `setAux1ChannelAllocation` 之类的命令**——这是 Septentrio 的产品分级：完整 dual-rover 能力被锁在更高 SKU 或更高许可里。

---

## 5. 对“独立 PPK + 双天线”方案的精度影响

### 5.1 原计划

```
Main raw obs ──► PPK ──► r_main(t)
Aux1 raw obs ──► PPK ──► r_aux1(t)
                          ├── baseline b(t) = r_aux1 - r_main
                          └── heading ψ(t)
```

### 5.2 实测瓶颈

- PPK 双差固定解通常需要 **≥ 5 颗共视卫星**（更稳健是 7~8 颗），并且至少要有 L1+L2 两个频点用于电离层组合或宽巷模糊度。
- Aux1 当前**只有 3 ~ 4 个 (signal, SV)**，且 **只有 L1**（除了 GLO L2CA），完全无法构成可靠的 PPK 解。
- 即便能形成 single-frequency float 解，水平精度也会从 cm 级退化到 dm ~ m 级——这与项目对 “厘米/亚厘米级基线 → mrad 级航向” 的目标相去甚远。

### 5.3 接收机内部的 AttEuler / AuxAntPositions 替代方案

| 项目 | 数值（典型） |
|------|--------------|
| 双天线基线长 | ~1 m（取决于车顶布置）|
| 接收机航向精度（手册标称） | **0.3° (1σ)** for 1 m baseline，固定解 |
| 双天线基线 σ（AttCovEuler 推算） | mm ~ cm 级 |

但**重要事实**（§4.2.11）：

> AttEuler 来自接收机的 RTK 风格双差求解，**不是 IMU 紧组合**。IMU 融合后的姿态在 INSNavCart / INSNavGeod 里（我们当前未录制）。

---

## 6. 解决方案对比

| 方案 | 描述 | 优点 | 缺点 | 工作量 |
|------|------|------|------|--------|
| **A. 录 ChannelStatus 验证假设** | 增加 block 4013 录制，确认 Aux1 通道分配策略 | 拿到“接收机就是这么设计的”铁证；可写进论文 | 不解决问题，只是确认问题 | 半天，改一行 config |
| **B. 联系 Septentrio 申请 dual-rover license** | 询问是否有 SKU/许可可解锁 Aux1 全频点 | 一劳永逸；本质上还是同一台机器 | 不确定能否拿到；可能要 RMA；可能收费 | 联系 + 等回复 |
| **C. 接收机内置姿态作为伪观测进因子图** | 录制 `AttEuler + AttCovEuler + AuxAntPositions`，作为约束送入 RS-NZM-GPCT 因子图 | 即刻可行；精度满足 0.3° heading；与现有数据无缝衔接 | 失去 “完全用 raw obs 自闭环” 的方法论纯度；汇报时要解释为何不做独立 PPK | 0（数据已在录） |
| **D. 增录 INSNavCart 作为 IMU-fused 姿态参考** | 增加 block 4225，作为 ground-truth-like 参考 | 可与自己 RS-NZM-GPCT 估计结果对照 | INSNavCart 不是真正的 ground truth，只是接收机内部解 | 半天，改 config |

> **方案 A + D 可以并行做**——都是 `config.yaml` 加一行 streams、`receiver_config.hpp` 默认列表加一行。

---

## 7. 推荐路径

1. **本周内**：执行方案 A，把 ChannelStatus 加进默认流，再做一次 10 分钟录制，用 parser.py 解码，**把通道分配截图放进汇报**，作为第二个铁证。
2. **本周内**：执行方案 D，把 INSNavCart 加进默认流，留作日后与自己估计器的姿态做对比。
3. **本周内**：发邮件给 Septentrio 技术支持（方案 B），按合同/SKU 询问是否可解锁 Aux1 全频点；不阻塞其他工作。
4. **过渡方案（接受现实）**：在 RS-NZM-GPCT 因子图里增加两类约束：
   - **基线约束**：来自 `AuxAntPositions`，协方差从该 block 自带的不确定度字段读取。
   - **航向约束**：来自 `AttEuler.Heading`，协方差来自 `AttCovEuler`。
   两者均以**伪观测因子**的形式进入图，相当于把接收机视为一个“黑盒姿态传感器”。这与“纯 raw obs”路线相比是方法论上的妥协，但**论文里可以诚实地写明：受限于硬件的通道分配策略，副天线 raw obs 不足以独立 PPK；为保证项目进度，采用接收机内置姿态作为伪观测**。
5. **若 Septentrio 回应可解锁**：回到原计划——主副天线各自独立 PPK，再算基线 / 航向。这是最干净的方法论路径。

---

## 8. 风险与未决问题

- **Q1**：手册的 §2.1 是否真的是“硬性产品分级”，还是“可通过 admin/expert level 命令解开”？
  → 通过方案 B 与 Septentrio 直接确认。
- **Q2**：即便申请到 dual-rover，副天线只有 1 m 基线、CN0 较低（27 dBHz），独立 PPK 的固定率会不会依然偏低？
  → 拿到许可后做一次对比录制即可判断。
- **Q3**：作为伪观测的 AttEuler 协方差是否可信？
  → AttCovEuler 是接收机自报，不是真协方差；建议在 RS-NZM-GPCT 因子图里乘以一个保守的 inflation factor（如 ×2 ~ ×4），并预留章节做协方差一致性检验（NIS test）。

---

## 9. 附录：本汇报涉及的代码与脚本

| 文件 | 用途 |
|------|------|
| `src/sbf/frame_sync.{hpp,cpp}` | SBF 帧同步器（含 2026-05-22 修复的 deque/span UB） |
| `src/receiver_config.{hpp,cpp}` | 上电时下发给接收机的命令序列 |
| `src/main.cpp` | 录制主循环 |
| `scripts/parser.py` | 离线 SBF 解码、CRC 校验、双天线统计、QualityInd 解码 |
| `scripts/split_recording.py` | 把单个 `.sbf` 按 AntennaID 拆成 main / aux1 / imu.csv |
| `docs/architecture/03-sbf-parser-design.md` | SBF wire-format 参考 |
| `docs/AsteRx-i3_D_Pro__Firmware_v1_5_2_Reference_Guide.pdf` | 接收机权威手册（§1.8 / §2.1 / §3.2.4 / §3.2.8 / §4.2.1 / §4.2.10 / §4.2.11 / §4.2.16） |

---

## 10. 附录：建议的下一次录制 `config.yaml` 增量

```yaml
streams:
  # ...existing entries...
  - id: 5
    blocks: [ "ChannelStatus" ]
    interval: "sec1"
  - id: 6
    blocks: [ "INSNavCart" ]
    interval: "msec100"
```

对应 `src/receiver_config.hpp` 默认值也同步增加这两个 block，避免每次都依赖外部 yaml。
