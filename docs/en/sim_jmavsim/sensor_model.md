# jMAVSim 传感器模型与手动定位来源配置

本分支提供用于测试 PX4 传感器输入的四旋翼仿真模型，直接生成飞控可用的测量结果。
模型包含两套独立 IMU、一个气压计、一个磁力计、一路带双天线航向的 RTK GNSS、一个下视测距仪，以及一路模拟视觉里程计。
视觉观测模拟的是定位算法的输出，不生成相机图像，也不运行实际的视觉惯性里程计（VIO）算法。

## 编译与启动

```sh
make px4_sitl_default jmavsim
```

启动目标会使用仓库中固定版本的 jMAVSim 源码，并应用本仓库保存的补丁进行编译。
应用补丁后的源码、Java 类文件和运行资源均生成在 PX4 构建目录中。
上游 jMAVSim 和 jMAVlib 子模块保持不变。
正常启动后会显示飞机窗口，使用标准 Iris 四旋翼外观和配置，机型编号为 `10017`。
退出 PX4 时也会停止本次会话的 Java 进程，重新启动同一目标时不会遗留占用端口的旧仿真进程。

将 *QGroundControl* 连接到 SITL，然后打开 **Analyze Tools > MAVLink Console**（分析工具 > MAVLink 控制台）。
下文中的命令也可以在本地 PX4 终端中使用。

## 传感器与观测

| 输入 | 数量 | 默认模型 |
| --- | --- | --- |
| IMU | 2 套 | 250 Hz；两套加速度和角速度测量的噪声相互独立 |
| 气压计 | 1 个 | 根据物理真值高度生成气压，不继承 GNSS 测量误差 |
| 磁力计 | 1 个 | 带测量噪声的机体系磁场 |
| GNSS | 1 路 | 20 Hz；RTK 固定解；延迟 200 ms；水平和垂直误差标准差分别为 0.02 m、0.03 m |
| GNSS 双天线航向 | 1 路 | 与 GNSS 使用相同的历史采样时刻和延迟；基线沿机体前向、长度 0.5 m；航向误差标准差 0.5° |
| 下视测距 | 1 个 | 20 Hz；量程 0.1–30 m；测量误差标准差 0.02 m |
| 视觉里程计 | 1 路 | 25 Hz；延迟 40 ms；位置和速度误差标准差分别为 0.05 m、0.05 m/s；姿态误差标准差 0.5° |

模型配置和随机种子保存在 `Tools/simulation/jmavsim/config/position_source_test.properties`。
修改模型配置后，需要重新构建并启动仿真。
`SENS_GNSS0_DELAY` 和 `EKF2_EV_DELAY` 应分别与模型的 GNSS 延迟和视觉延迟保持一致。
`SIM_GPS_VEL_ERR` 和 `SIM_GPS_YAW_ERR` 应分别与模型的速度误差和航向误差标准差保持一致。
模型中的航向基线长度应与 `SENS_GNSS0_AUXX` 保持一致，主天线位于机体原点。
这些误差参数描述的是模拟测量本身，不能直接视为 EKF 最终估计精度的保证。

测距仪沿机体向下的射线计算与平地的交点距离。
当射线朝向上方，或交点距离超出量程时，输出无效观测。
飞机倾斜时，输出沿射线方向的斜距。

视觉里程计使用固定的本地 NED（北、东、下）坐标系和 FRD（前、右、下）机体坐标系，并提供协方差与质量信息。
关闭后重新开启视觉输入，不会重置模型的本地原点。
所有测量均由仿真物理真值生成，而非由 PX4 的估计位置生成。

## 开启与关闭传感器输入

```sh
simulator_mavlink status
simulator_mavlink sensor imu1 off
simulator_mavlink sensor imu1 on
simulator_mavlink sensor imu2 off
simulator_mavlink sensor imu2 on
simulator_mavlink sensor baro off
simulator_mavlink sensor mag off
simulator_mavlink sensor gnss off
simulator_mavlink sensor gnss_yaw off
simulator_mavlink sensor range off
simulator_mavlink sensor vision off
simulator_mavlink sensor reset
```

将 `off` 改为 `on` 即可恢复对应输入。
`reset` 恢复全部模型输入。
GNSS 位置/速度和双天线航向虽然来自同一接收机，但可以分别关闭，互不连带。
关闭输入表示停止向 PX4 传感器链路发布对应测量，物理仿真和仿真时钟仍继续运行。

终端命令与参数界面共用以下位掩码，每一位控制一类输入：

| 参数 | 位编号及含义 | 本模型初始值 |
| --- | --- | --- |
| `SIM_MAV_IMU` | 位 0：IMU1；位 1：IMU2；位 2：IMU3，通用桥支持，但本模型未配置第三套 | `3` |
| `SIM_MAV_SENS` | 位 0：气压计；位 1：磁力计；位 2：GNSS 位置/速度；位 3：GNSS 航向；位 4：测距；位 5：视觉 | `63` |

例如，`param set SIM_MAV_SENS 31` 会关闭视觉输入，并保持其他五类观测开启。
开关变化在 1 秒内生效。
必须始终保留至少一套实际存在的 IMU。
终端会拒绝关闭最后一套可用 IMU；直接修改参数形成无效组合时，会恢复上一个有效值并输出提示。
关闭当前使用的 IMU 时，仿真桥可能短暂保持上一执行器指令，使仿真时间能够继续推进，直到原生 IMU 投票器完成超时判断。
现有投票器选择剩余输入后，恢复正常锁步运行。

每次重新启动 jMAVSim 会话，两个输入掩码都会恢复为 `3` 和 `63`。
EKF 定位来源参数则会保留，不会随传感器开关一起重置。
已有的 `failure` 故障注入与这些使能位相互独立；`sensor reset` 只恢复输入开关，不清除单独注入的故障。

## 配置估计器使用的定位来源

传感器是否提供数据，与估计器是否融合该数据，是两个独立的控制层。
例如，视觉里程计可以持续输出，但设置 `EKF2_EV_CTRL=0` 后，EKF 不会融合视觉观测。
本阶段支持在未解锁状态下配置定位来源，然后重启仿真并检查实际融合状态。
本阶段未新增自动故障切换策略，也不保证飞行中修改融合配置能够无中断切换。

下面每组示例都包含该组合依赖的完整来源配置。
输入一整组命令后，关闭 PX4，再重新启动相同的 jMAVSim 目标，使需要重启的参数生效。
开始配置前，先执行 `simulator_mavlink sensor reset`，确保模型所有输入均开启。

可按水平位置、速度、高度和航向分别组合来源：

| 约束 | GNSS（`EKF2_GPS_CTRL`） | 视觉（`EKF2_EV_CTRL`） | 其他来源 |
| --- | --- | --- | --- |
| 水平位置 | 将数值 `1` 加入掩码 | 将数值 `1` 加入掩码 | 气压计和测距仪都不提供水平位置 |
| 速度 | 将数值 `4` 加入掩码 | 将数值 `4` 加入掩码 | IMU 用于预测相邻观测之间的运动 |
| 高度 | 将数值 `2` 加入掩码；高度参考设为 `EKF2_HGT_REF=1` | 将数值 `2` 加入掩码；高度参考设为 `EKF2_HGT_REF=3` | 气压：`EKF2_BARO_CTRL=1`，高度参考为 `0`；测距：`EKF2_RNG_CTRL=2`，高度参考为 `2` |
| 航向 | 将数值 `8` 加入掩码 | 将数值 `8` 加入掩码 | 磁力计：`EKF2_MAG_TYPE=0`；设为 `5` 可关闭磁力计融合 |

同一个掩码内，将所需功能对应的数值相加即可。
高度参考参数只选择高度基准，本身不会开启相应的高度观测融合。
建议使用下面的完整示例，避免保留上一组配置中不再需要的来源。

### RTK 位置、高度与双天线航向（默认组合）

```sh
param set EKF2_GPS_CTRL 15
param set EKF2_EV_CTRL 0
param set EKF2_BARO_CTRL 0
param set EKF2_RNG_CTRL 0
param set EKF2_HGT_REF 1
param set EKF2_MAG_TYPE 5
```

模型将 GNSS 参数槽 0 配置为双天线接收机，基线沿机体前向。
其 `SENS_GNSS0_ID`、`SENS_GNSS0_HDG` 和天线位置偏移参数应与仿真接收机保持一致。
GNSS 航向需要通过 PX4 现有的初始化和质量检查后才能使用。

### 视觉位置、高度与航向

```sh
param set EKF2_GPS_CTRL 0
param set EKF2_EV_CTRL 15
param set EKF2_BARO_CTRL 0
param set EKF2_RNG_CTRL 0
param set EKF2_HGT_REF 3
param set EKF2_MAG_TYPE 5
```

### RTK 与视觉位置、速度联合融合

```sh
param set EKF2_GPS_CTRL 15
param set EKF2_EV_CTRL 5
param set EKF2_BARO_CTRL 0
param set EKF2_RNG_CTRL 0
param set EKF2_HGT_REF 1
param set EKF2_MAG_TYPE 5
```

RTK 提供高度和航向参考，同时由 RTK 与视觉共同约束水平位置和速度。

### GNSS 水平定位、气压高度与磁力计航向

```sh
param set EKF2_GPS_CTRL 5
param set EKF2_EV_CTRL 0
param set EKF2_BARO_CTRL 1
param set EKF2_RNG_CTRL 0
param set EKF2_HGT_REF 0
param set EKF2_MAG_TYPE 0
```

### 视觉水平定位与航向、下视测距高度

```sh
param set EKF2_GPS_CTRL 0
param set EKF2_EV_CTRL 13
param set EKF2_BARO_CTRL 0
param set EKF2_RNG_CTRL 2
param set EKF2_HGT_REF 2
param set EKF2_MAG_TYPE 5
```

这组测距高度配置适用于平地模型，并要求飞机处于测距仪的有效工作范围内。
测距仪和气压计都不能提供水平位置。
使用位置控制飞行时，需要同时配置有效的水平定位来源。

## 检查实际生效状态

```sh
simulator_mavlink status
ekf2 status
listener estimator_status_flags
listener vehicle_local_position
listener sensor_gnss
listener vehicle_gnss_heading
listener distance_sensor
listener vehicle_visual_odometry
```

检查仿真桥显示的接收频率、发布频率及各路输入的使能状态。
统计传感器数量时，应以实际持续更新的测量为准，而不是仅统计已预先注册的 uORB 实例。
通用仿真桥预留的传感器实例可能多于当前模型实际输出的数量。

RTK 组合应检查 `cs_gnss_pos`、`cs_gnss_vel`、`cs_gps_hgt` 和 `cs_gnss_yaw`。
视觉组合应检查 `cs_ev_pos`、`cs_ev_vel`、`cs_ev_hgt` 和 `cs_ev_yaw`。
气压高度和测距高度对应的融合标志分别为 `cs_baro_hgt` 和 `cs_rng_hgt`。
同时检查 `vehicle_local_position` 中对应的位置、速度有效标志是否保持为真。
输入已开启，并不代表 EKF 已接受和融合该输入。
相应的 `estimator_aid_src_*` 主题还会报告新观测是否实际参与融合，以及其创新量是否被拒绝。

## 复现自动化检查

不启动飞行，仅编译并测试实际应用补丁后的 Java 模型：

```sh
make px4_sitl_default test_jmavsim_sensors
```

启动 jMAVSim 并保持飞机未解锁，通过与 QGC 终端相同的 MAVLink `SERIAL_CONTROL` 协议逐项测试输入开关：

```sh
python3 Tools/simulation/jmavsim/tests/test_sitl_sensors.py
```

所有输入开启时，可添加 `--metadata-only`，在不改变输入掩码的情况下检查实例数量、接收的测量误差字段、坐标系、协方差和接收机延迟补偿。

运行时检查依赖 `pymavlink`，使用 UDP 端口 `14540`，请勿同时让其他客户端占用该端口。
QGC 使用独立端口，可以保持连接。

进行 RTK 飞行检查前，先在未解锁状态保存配置：

```sh
python3 Tools/simulation/jmavsim/tests/test_sitl_flight.py configure --profile rtk
```

然后执行 `shutdown` 关闭 PX4，重新运行 `make px4_sitl_default jmavsim`，再执行：

```sh
python3 Tools/simulation/jmavsim/tests/test_sitl_flight.py fly --profile rtk
```

将上述两步中的配置选项替换为 `--profile vision`，即可验证视觉位置、高度和航向组合。
飞行检查会先确认来源参数在重启后保留、传感器开关恢复初始状态，然后解锁 SITL，起飞至 3 m，悬停，降落并上锁。
悬停期间会持续检查观测是否被实际接受并融合，以及本地位置是否有效。
测试完成后，如果希望继续使用默认组合，请执行前面的 RTK 默认配置命令并重启。

相同的“配置后重启”流程也支持 `--profile mixed`、`--profile baro_mag` 和 `--profile vision_range`。
分别对应 RTK 与视觉联合融合、气压高度与磁力计航向、视觉定位与测距高度组合。
将 `fly` 操作替换为 `check`，即可在不解锁的情况下检查配置及实际融合状态。
飞机位于地面时，PX4 可能用其最小离地距离替代不可用的测距读数，因此地面测距高度融合检查不能验证空中的实际测距几何关系。
