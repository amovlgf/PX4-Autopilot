# hw_test 测试命令与硬件连接说明

本文档说明 `src/systemcmds/hw_test/` 下全部测试命令、产测接线要求及判定逻辑。
产测逻辑主要按 **Amovlab ICF6** 编写（见 `HW_TEST_FLOW.md`、各 `hw_test_*.cpp` 文件头注释）。
其他 H7 板（如 Flycore）若启用 `CONFIG_SYSTEMCMDS_HW_TEST=y`，命令语义相同，但 **UART 回环口列表、PWM GPIO 回退** 等可能需按板级串口映射调整。

---

## 1. 命令总览

| 命令 | 类型 | 作用 |
|------|------|------|
| `hw_test_all` | 全流程 | 按固定顺序跑 7 项，汇总 PASS/FAIL |
| `hw_test um982_cfg [dev] [baud]` | 单项 / 全流程第 1 项 | **TELEM2 / ttyS1** ↔ **UM982 COM2** 配置 |
| `hw_test can` / `hw_test can_ext` | 单项 / 全流程第 2 项 | CAN1 ↔ CAN2 双向互测（两者相同） |
| `hw_test can_tx <0\|1> [duration_s] [period_ms] [loopback]` | 调试 / 产测 | 单路 CAN 周期发帧（给分析仪） |
| `hw_test can_rx <0\|1> [duration_s]` | 调试 / 产测 | 单路 CAN 监听并打印帧 |
| `hw_test spi` | 单项 / 全流程第 3 项 | SPI 双 IMU + **FRAM 可访问性** |
| `hw_test i2c` | 单项 / 全流程第 4 项 | I2C1 / I2C4 探测同一颗 BMI088（并联夹具） |
| `hw_test sd` | 单项 / 全流程第 5 项 | SD 卡读写压测 |
| `hw_test pwm` | 单项 / 全流程第 6 项 | PWM 输出 + 蜂鸣器（ICF6 含 GPIO 回退） |
| `hw_test uart` | 单项 / 全流程第 7 项 | **5 路回环**（ttyS0/S3/S4/S5/S6）+ **GPS1 / ttyS2** ↔ **UM982 COM1** GPS 状态 |

### 1.1 `hw_test_all` 执行顺序

```text
um982_cfg → can → spi → i2c → sd → pwm → uart
```

任一子项失败会记录到失败列表，最后输出 `HW_TEST ALL: PASS/FAIL` 横幅。

### 1.2 编译开关

- Kconfig：`CONFIG_SYSTEMCMDS_HW_TEST=y`
- 当前在仓库中已启用示例：`boards/amovlab/icf6/default.px4board`、`boards/px4/fmu-v6c/default.px4board`
- Flycore default 若未打开该选项，NSH 中无 `hw_test` 命令

---

## 2. FRAM 测试说明

**有 FRAM 相关测试，但没有独立的 `hw_test fram` 命令。**

FRAM 合并在 **`hw_test spi`** 中，作为判定步骤之一。FRAM 硬件接在 **SPI4** 总线，软件通过 MTD 挂载为 `/fs/mtd_params`：

```text
SPI4 ──► FRAM ──► /fs/mtd_params
open("/fs/mtd_params", O_RDONLY) 成功 → FRAM/SPI4 路径可访问
```

失败日志示例：`FRAM open failed`。

### 2.1 `hw_test spi` 完整判定

1. 在 2 s 窗口内采集 `vehicle_imu`、`sensor_accel`、`sensor_gyro`
2. 必须识别四路：BMI088 accel/gyro、ICM42688P accel/gyro
3. 四路更新时间均 ≤ 200 ms（新鲜）
4. **打开 `/fs/mtd_params` 验证 FRAM 路径可访问**
5. 若 error_count 基线完整，窗口内增量必须为 0

### 2.2 手动辅助命令

```bash
hw_test spi
bmi088 -A status
bmi088 -G status
icm42688p status
ls /fs/mtd_params
```

---

## 3. 产测硬件接线总览

以下为 ICF6 产测夹具的典型接线方式。命令判定逻辑见第 4 节；本节只描述**物理连接**。

```text
                    ┌──────────────── 飞控板 ────────────────┐
                    │                                        │
  CAN1_H ───────────┼── CAN2_H   （H 对 H 短接）            │
  CAN1_L ───────────┼── CAN2_L   （L 对 L 短接）            │
                    │                                        │
  [板载] SPI1 ◄─────┤ BMI088（accel + gyro）                 │
  [板载] SPI2 ◄─────┤ ICM42688P                              │
  [板载] SPI4 ◄─────┤ FRAM（参数存储）                       │
                    │                                        │
  BMI088(I2C) ◄─────┤ I2C1 / I2C4                            │
  SD 卡 ────────────┤ SD 卡槽                                 │
  LED × N ◄─────────┤ PWM CH1~CH10                            │
                    │                                        │
  UM982 COM2 ◄─────┤ TELEM2  USART3 /dev/ttyS1 (um982_cfg 配置口) │
  UM982 COM1 ◄─────┤ GPS1    UART4  /dev/ttyS2 (uart GPS 状态检测) │
  TX-RX 短接 ◄──────┤ TELEM1(S0) TELEM3(S3) RC(S4)                   │
                    │        GPS2(S5) DEBUG(S6) (uart 回环)   │
                    └────────────────────────────────────────┘
```

| 测试项 | 硬件连接要点 | 外接线 |
|--------|--------------|--------|
| UM982 | **UM982 COM2** ↔ **TELEM2 / ttyS1**；**UM982 COM1** ↔ **GPS1 / ttyS2** | 是 |
| CAN | CAN1_H↔CAN2_H，CAN1_L↔CAN2_L | 是 |
| SPI / FRAM | SPI1=BMI088，SPI2=ICM42688P，SPI4=FRAM | 否（板载） |
| I2C | **一颗 BMI088** 同时接 **I2C1** 与 **I2C4**（产测夹具并联） | 是 |
| SD | 插入 SD 卡 | 是（插卡） |
| PWM | 各 PWM 口外接 LED | 是 |
| UART 回环 | **ttyS0/S3/S4/S5/S6**（TELEM1/TELEM3/RC/GPS2/DEBUG）TX-RX 短接 | 是 |
| UART GPS | **GPS1 / ttyS2** 接 **UM982 COM1**（经 `um982_cfg` 配置后输出 GNSS 数据） | 是 |

---

## 4. 各命令硬件连接与判定

### 4.1 `hw_test um982_cfg`

UM982 产测采用 **双串口接线**：**COM2** 接飞控 **TELEM2** 做配置，**COM1** 接飞控 **GPS1** 做 GNSS 数据输出（供后续 `hw_test uart` GPS 状态检测）。

#### UM982 产测接线拓扑

```text
                    ┌────────── UM982 模块 ──────────┐
                    │                                │
  TELEM2 (ttyS1) ◄──┤ COM2  配置口（115200）         │
  USART3           │       hw_test um982_cfg 走此路   │
                    │                                │
  GPS1   (ttyS2) ◄──┤ COM1  数据口（配置后 230400）   │
  UART4            │       hw_test uart GPS 检测走此路│
                    └────────────────────────────────┘
```

| 项目 | 说明 |
|------|------|
| 配置链路 | **UM982 COM2** ↔ **TELEM2**（**USART3 / `/dev/ttyS1`**） |
| 数据链路 | **UM982 COM1** ↔ **GPS1**（**UART4 / `/dev/ttyS2`**） |
| 配置口波特率 | 飞控 TELEM2 固定 **115200**，8N1 |
| 接线 | COM2：TELEM2 TX↔UM982 COM2 RX，TELEM2 RX↔UM982 COM2 TX；COM1：GPS1 TX↔UM982 COM1 RX，GPS1 RX↔UM982 COM1 TX；**GND 共地** |
| 测试内容 | 经 COM2 顺序发送 `FRESET`、`config com1 230400`、COM1 输出配置、`saveconfig` |
| 判定 | 每条命令应答含 `response: OK` 且 `$command,<命令名>` |
| 注意 | `config com1 230400` 只改 **UM982 COM1** 波特率，**不改** TELEM2 侧 115200；COM1 配置完成后从 GPS1 口输出 GNSS 数据 |

`hw_test um982_cfg` 使用 TELEM2（ttyS1），在 `hw_test uart` 中**不做回环**（`hw_test_all` 第 1 项，需在 uart 测试之前完成）。

常用命令：

```bash
hw_test um982_cfg
hw_test um982_cfg /dev/ttyS1 115200
```

---

### 4.2 `hw_test can` / `hw_test can_ext`

#### 硬件接线

CAN 测试采用 **CAN1 与 CAN2 对插互连**，形成一条短总线：

```text
  飞控 CAN1 口                    飞控 CAN2 口
  ┌─────────┐                    ┌─────────┐
  │  CAN1_H │────────────────────│  CAN2_H │   H 线对 H 线
  │  CAN1_L │────────────────────│  CAN2_L │   L 线对 L 线
  │   GND   │────────────────────│   GND   │   建议共地
  └─────────┘                    └─────────┘
```

| 步骤 | 操作 |
|------|------|
| 1 | 用杜邦线或产测夹具将 **CAN1_H 与 CAN2_H** 相连 |
| 2 | 将 **CAN1_L 与 CAN2_L** 相连 |
| 3 | **CAN1_GND 与 CAN2_GND** 共地（推荐） |
| 4 | 确认 CAN1、CAN2 接口处 **120 Ω 终端电阻已启用**（或总线两端各 120 Ω，等效约 60 Ω） |

> **注意：** 只短接 H-H、L-L，**不要** H 接 L，否则无法通信。

| 项目 | 说明 |
|------|------|
| 物理口 | CAN1 → `can0`（参数 **0**）；CAN2 → `can1`（参数 **1**） |
| 波特率 | 经典 CAN，仲裁段 **1 Mbps** |
| 帧类型 | 扩展帧 29 bit，ID = `0x001ABCDE`，DLC = 8 |
| 载荷 | can0→can1：`55 AA 12 34 7E 80 DE AD` |
| | can1→can0：`A5 5A 21 43 E7 08 ED DA` |
| 判定 | 两个方向均收到匹配帧才 PASS；每方向最多重试 3 次 |

常用命令：

```bash
hw_test can
hw_test can_ext    # 与 can 相同
```

典型失败：`recv timeout`（H/L 接反、未共地、终端电阻缺失）、`bind/ioctl failed`。

---

### 4.3 `hw_test can_tx` / `hw_test can_rx`（不在 `hw_test_all` 内）

用于 CAN 分析仪联调，详见 `hw_test_can.cpp` 文件头注释。

#### `can_tx`

| 项目 | 说明 |
|------|------|
| 用法 | `hw_test can_tx <0\|1> [duration_s] [period_ms] [loopback]` |
| 默认 | 30 s，每 100 ms 一帧 |
| 帧 | 扩展 ID `0x18DAF110`，载荷 `PX4T` + 小端递增序号 |
| 接线 | 分析仪接对应 CAN 口，**Normal 模式**（非 Listen-only），1 Mbps，共地 + 终端 |
| loopback=1 | Socket 回环，不依赖总线 ACK（线上可能无波形） |

```bash
hw_test can_tx 0
hw_test can_tx 1 60 200
hw_test can_tx 1 30 100 1
```

#### `can_rx`

| 项目 | 说明 |
|------|------|
| 用法 | `hw_test can_rx <0\|1> [duration_s]` |
| 默认 | 监听 30 s |
| 接线 | 分析仪向同一物理口发送经典 CAN 帧（ID 可自定，如 `0x100`） |

```bash
hw_test can_rx 0 60
hw_test can_rx 1
```

---

### 4.4 `hw_test spi`（含 FRAM）

#### 硬件拓扑（板载，无需外接线）

SPI 测试验证三条 SPI 总线上的器件是否正常通信：

```text
  SPI1 ──► BMI088（加速度计 + 陀螺仪，板载焊接）
  SPI2 ──► ICM42688P（加速度计 + 陀螺仪，板载焊接）
  SPI4 ──► FRAM（参数存储，挂载为 /fs/mtd_params）
```

| 总线 | 器件 | 产测接线 |
|------|------|----------|
| **SPI1** | BMI088 accel/gyro | 板载，**无需外接** |
| **SPI2** | ICM42688P accel/gyro | 板载，**无需外接** |
| **SPI4** | FRAM | 板载，**无需外接**；软件通过 `open("/fs/mtd_params")` 验证 |

| 项目 | 说明 |
|------|------|
| 前置条件 | 上电后 `rc.board_sensors` 已 start BMI088、ICM42688P |
| IMU 判定 | 2 s 内四路 uORB 数据新鲜（BMI088 a/g + ICM42688P a/g） |
| FRAM 判定 | `/fs/mtd_params` 打开成功 |
| 产测夹具 | 无额外 SPI 飞线；检查板载 IMU、FRAM 焊接与供电 |

```bash
hw_test spi
```

典型失败：`IMU status missing`（SPI1/SPI2 器件未识别）、`IMU stream stale`（SPI 通信异常）、`FRAM open failed`（SPI4/FRAM 路径不可访问）。

---

### 4.5 `hw_test i2c`

#### 硬件接线

I2C 测试使用 **一颗 BMI088（I2C 模式）**，通过产测夹具 **同时并联** 到 **I2C1** 与 **I2C4** 两路总线。软件分别对 bus1、bus4 发起探测，两路均应答成功即 PASS。

```text
  飞控 I2C1 口 ──┐
                 ├──► 一颗 BMI088（SCL、SDA、VCC、GND 并联）
  飞控 I2C4 口 ──┘
```

| 步骤 | 操作 |
|------|------|
| 1 | 产测夹具将 **同一颗 BMI088** 的 SCL、SDA、VCC、GND **并联** 到 I2C1、I2C4 两路接口 |
| 2 | 确认模块供电正常（通常 3.3 V），I2C 上拉已具备（板载或模块自带） |
| 3 | 运行 `hw_test i2c`，依次探测 bus1、bus4 |

| 项目 | 说明 |
|------|------|
| 编译依赖 | `CONFIG_DRIVERS_IMU_BOSCH_BMI088_I2C=y` |
| 传感器数量 | **1 颗** BMI088（I2C），**不是** I2C1/I2C4 各一颗 |
| 测试 bus | **I2C1**（bus 1）、**I2C4**（bus 4） |
| 方式 | 各执行 `bmi088_i2c -A -X -b <n> -R 4 start`，成功后 stop |
| 判定 | I2C1 与 I2C4 均 start 成功 → PASS |

```bash
hw_test i2c
```

未编译 I2C 驱动时直接返回 `-ENODEV`。典型失败：`I2C FAIL bus1` / `I2C FAIL bus4`（夹具未并联、某路 SDA/SCL 断线、地址冲突）。

---

### 4.6 `hw_test sd`

#### 硬件接线

SD 测试为 **直接检测 SD 卡**，无需额外飞线：

| 步骤 | 操作 |
|------|------|
| 1 | 将合格 SD 卡 **插入飞控 SD 卡槽** |
| 2 | 确认卡已识别（可先用 `ls /fs/microsd` 辅助确认） |
| 3 | 运行 `hw_test sd`，内部调用 `sd_bench` 做读写压测 |

| 项目 | 说明 |
|------|------|
| 接线 | **仅插 SD 卡**，无其它外设 |
| 内部调用 | `sd_bench -b 4096 -r 2 -d 1000` |
| 判定 | 返回 0 → PASS |

```bash
hw_test sd
```

典型失败：未插卡、卡接触不良、卡格式/分区异常导致 bench 失败。

---

### 4.7 `hw_test pwm`

#### 硬件接线

PWM 测试需在 **各 PWM 输出通道外接 LED**，通过肉眼观察 **1000 µs ↔ 2000 µs 交替闪烁** 判定通道正常：

```text
  PWM CH1 ──► LED ──► GND
  PWM CH2 ──► LED ──► GND
  ...
  PWM CH10 ──► LED ──► GND
```

| 步骤 | 操作 |
|------|------|
| 1 | 在产测夹具上为 **FMU PWM CH1~CH10** 各接一颗 LED（串限流电阻，如 330 Ω~1 kΩ） |
| 2 | LED 负极接 **GND**，正极经电阻接对应 PWM 信号脚 |
| 3 | 运行 `hw_test pwm`，观察各通道 LED **同步闪烁** |
| 4 | ICF6 可同时听 **蜂鸣器（PA15）** 是否有声（主路径或 GPIO 回退时拉高） |

| 项目 | 说明 |
|------|------|
| 主路径 | `up_pwm_servo_init` → 各通道 1000 µs ↔ 2000 µs 闪烁 |
| ICF6 通道 | FMU_CH1~10：PE9/11/13/14, PA7, PB0, PC9, PD13/14/15 |
| 产测目检 | **外接 LED 闪烁** 即表示该 PWM 通道输出正常 |
| 回退 1 | ICF6：`run_gpio_high_fallback()`（GPIO 点高 + 蜂鸣器） |
| 回退 2 | 其它板：`actuator_test` uORB 话题 |

```bash
hw_test pwm
```

典型失败：某路 LED 不亮（焊接/驱动/通道映射问题）；示波器可辅助定位，产测以 LED 目检为主。

---

### 4.8 `hw_test uart`

`hw_test uart` 覆盖 ICF6 上除 TELEM2（接 **UM982 COM2**）以外的 **6 路串口**：其中 **5 路做 TX-RX 回环**，**GPS1（接 UM982 COM1）做 GPS 状态检测**。
UM982 配置不在本命令内，见 **4.1 `hw_test um982_cfg`**（须先完成，`hw_test_all` 中 um982_cfg 在 uart 之前执行）。

#### 4.8.1 ICF6 串口映射总表

映射定义见 `boards/amovlab/icf6/nuttx-config/include/board.h`：

| 硬件 UART | 连接器 | 设备节点 | hw_test 用途 | 产测接线 |
|-----------|--------|----------|--------------|----------|
| **USART2** | **TELEM1** | **`/dev/ttyS0`** | `hw_test uart` **回环** | TX ↔ RX 短接 |
| **USART3** | **TELEM2** | **`/dev/ttyS1`** | `hw_test um982_cfg` **UM982 配置** | **UM982 COM2**（**不回环**） |
| **UART4** | **GPS1** | **`/dev/ttyS2`** | `hw_test uart` **GPS 状态检测** | **UM982 COM1**（**不回环**） |
| **UART5** | **TELEM3** | **`/dev/ttyS3`** | `hw_test uart` **回环** | TX ↔ RX 短接 |
| **USART6** | **RC** | **`/dev/ttyS4`** | `hw_test uart` **回环** | TX ↔ RX 短接 |
| **UART7** | **GPS2** | **`/dev/ttyS5`** | `hw_test uart` **回环** | TX ↔ RX 短接 |
| **UART8** | **DEBUG** | **`/dev/ttyS6`** | `hw_test uart` **回环** | TX ↔ RX 短接 |

#### 4.8.2 三类串口与测试命令对应

```text
  ┌──────────────────────────────────────────────────────────────────┐
  │ ① UM982 配置（hw_test um982_cfg，hw_test_all 第 1 项）           │
  │    TELEM2 / USART3 / ttyS1  ◄──COM2──►  UM982 模块              │
  ├──────────────────────────────────────────────────────────────────┤
  │ ② GPS 状态检测（hw_test uart 内，非回环）                        │
  │    GPS1   / UART4  / ttyS2  ◄──COM1──►  UM982 模块              │
  │    （需先完成 um982_cfg，COM1 输出 GNSS 数据）                   │
  ├──────────────────────────────────────────────────────────────────┤
  │ ③ 回环测试（hw_test uart 内，TX-RX 短接）                       │
  │    USART2 / TELEM1 / /dev/ttyS0  ──┐                             │
  │    UART5  / TELEM3 / /dev/ttyS3  ──┤                             │
  │    USART6 / RC     / /dev/ttyS4  ──┼── 各口 TX ↔ RX 短接         │
  │    UART7  / GPS2  / /dev/ttyS5  ──┤                             │
  │    UART8  / DEBUG / /dev/ttyS6  ──┘                             │
  └──────────────────────────────────────────────────────────────────┘
```

#### 4.8.3 回环口：ttyS0 / ttyS3 / ttyS4 / ttyS5 / ttyS6

产测夹具在下列 **5 路** 串口上将 **TX 与 RX 短接**（同口自环），115200 8N1 RAW，发送 32 字节固定 pattern，读回必须一致。每口最多重试 3 次。

| 硬件 UART | 连接器 | 设备节点 | 产测接线 |
|-----------|--------|----------|----------|
| USART2 | TELEM1 | `/dev/ttyS0` | **TX ↔ RX 短接** |
| UART5 | TELEM3 | `/dev/ttyS3` | **TX ↔ RX 短接** |
| USART6 | RC | `/dev/ttyS4` | **TX ↔ RX 短接**（FAT 产测固件勿在此口跑 rc_input） |
| UART7 | GPS2 | `/dev/ttyS5` | **TX ↔ RX 短接** |
| UART8 | DEBUG | `/dev/ttyS6` | **TX ↔ RX 短接** |

> **注意：** `/dev/ttyS1`（USART3 / TELEM2）接 **UM982 COM2**，**不在回环列表中**。

#### 4.8.4 GPS 状态口：GPS1 / ttyS2（接 UM982 COM1）

| 硬件 UART | 连接器 | 设备节点 | 产测接线 |
|-----------|--------|----------|----------|
| UART4 | GPS1 | `/dev/ttyS2` | **UM982 COM1**（GPS1 TX↔COM1 RX，GPS1 RX↔COM1 TX，GND 共地） |

- **不做 TX-RX 短接**；GPS1 口接 **UM982 COM1 数据口**，不是独立 GPS 模块
- **前置：** 先运行 `hw_test um982_cfg` 完成 COM1 配置（230400 及 NMEA 输出），UM982 才从 COM1 向 GPS1 口送 GNSS 数据
- 判定：订阅 `sensor_gps`，要求 `device_id` 为 SERIAL、**bus == 2**（对应 `/dev/ttyS2` / GPS1）、数据新鲜
- 日志：`UART /dev/ttyS2 gps status: OK` 或 `NOT OK`

```bash
hw_test uart
```

---

## 5. `hw_test_all` 产测接线检查清单（ICF6）

按 `hw_test_all` 顺序，上电前逐项确认：

| # | 测试项 | 硬件准备 |
|---|--------|----------|
| 1 | UM982 | **COM2** ↔ **TELEM2 / ttyS1**（`hw_test um982_cfg`）；**COM1** ↔ **GPS1 / ttyS2** |
| 2 | CAN | **CAN1_H ↔ CAN2_H**，**CAN1_L ↔ CAN2_L**，GND 共地，终端电阻就绪 |
| 3 | SPI / FRAM | 板载：**SPI1=BMI088**，**SPI2=ICM42688P**，**SPI4=FRAM**（无飞线） |
| 4 | I2C | **一颗 BMI088** 经产测夹具 **并联** 到 **I2C1 + I2C4** |
| 5 | SD | **插入 SD 卡** |
| 6 | PWM | **PWM CH1~10 外接 LED**，运行后目检闪烁 |
| 7 | UART | **回环：** ttyS0/S3/S4/S5/S6 各 TX-RX 短接；**GPS：** ttyS2(GPS1) 已接 UM982 COM1（依赖第 1 项 um982_cfg） |

推荐流程：

```bash
hw_test_all
```

---

## 6. Flycore 与其它板差异

| 项目 | 说明 |
|------|------|
| `CONFIG_SYSTEMCMDS_HW_TEST` | Flycore default 默认未开；需自行在 `default.px4board` 启用 |
| CAN | CAN1_H↔CAN2_H、CAN1_L↔CAN2_L 短接互测 |
| SPI / FRAM | SPI1=BMI088，SPI2=ICM42688P，SPI4=FRAM |
| I2C | **一颗 BMI088** 并联接 I2C1 + I2C4；需 `BMI088_I2C` 驱动 |
| UART | Flycore 串口映射见 `boards/amovlab/flycore/default.px4board`；回环口列表在 `hw_test_uart.cpp` 中按 ICF6 写死，**移植 Flycore 产测时需核对或修改** |
| PWM GPIO 回退 | 仅 `CONFIG_ARCH_BOARD_AMOVLAB_ICF6` 编译进专用 GPIO 路径 |

### Flycore 串口映射（参考）

| 功能 | 设备节点 |
|------|----------|
| TELEM1 | `/dev/ttyS0` |
| TELEM2 | `/dev/ttyS1` |
| GPS1 | `/dev/ttyS2` |
| TELEM3 | `/dev/ttyS3` |
| RC | `/dev/ttyS4` |
| GPS2 | `/dev/ttyS5` |

---

## 7. 输出与失败定位

| 输出 | 含义 |
|------|------|
| `[PASS] <TEST_NAME> ...` | 子项通过 |
| `[FAIL] <TEST_NAME> ...` | 子项失败 |
| `HW_TEST ALL: PASS/FAIL` | 全流程汇总 |

详细时序与 UM982 命令列表见同目录 **`HW_TEST_FLOW.md`**。

---

## 8. 相关源文件

| 文件 | 内容 |
|------|------|
| `hw_test_main.cpp` | 命令入口与 usage |
| `hw_test_runner.cpp` | 单项调度与 `hw_test_all` |
| `hw_test_can.cpp` | CAN 互测 / can_tx / can_rx |
| `hw_test_spi.cpp` | SPI IMU + FRAM |
| `hw_test_i2c.cpp` | I2C bus1/4 |
| `hw_test_uart.cpp` | 串口回环 + GPS 状态 |
| `hw_test_pwm.cpp` | PWM / 蜂鸣器 |
| `hw_test_sd.cpp` | SD bench |
| `hw_test_um982.cpp` | UM982 配置 |
| `HW_TEST_FLOW.md` | 测试流程与判定细则 |
