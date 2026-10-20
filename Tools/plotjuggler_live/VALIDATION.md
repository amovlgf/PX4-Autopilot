# 实施与验收记录

日期：2026-10-19（Asia/Shanghai）。状态只使用“通过／失败／未执行／待手动验证”；运行证据保存在本机 `.state/`，不提交构建产物、图形认证文件和日志。

## 环境和保护范围

- 主机 Ubuntu 20.04；8 个逻辑处理器、约 7.8 GiB 内存；X11。
- 工程 `/home/amovlgf/PX4-Autopilot`，分支 `codex/position-source-switch-test`，基于 `7293640e6a` 的原有未提交工作区。
- 原启动命令 `unset HEADLESS; make px4_sitl_default jmavsim`，机型 `10017 / jmavsim_iris`；QGC 使用原 Debug 构建及包装脚本。
- 模型、传感器配置、启动文件和用户修改的基线差异及 25 个文件校验值位于 `.state/baseline/`。这些文件中只有计划内的 `dds_topics.yaml` 改动。
- Docker 由用户安装并授权组成员。未改全局代理、Docker daemon 配置、Shell ROS 环境、控制算法或飞行参数。
- 镜像 `px4-plotjuggler-live:humble-2.4.2`，ID `sha256:e9db5be659ae97461aaf5a0e2294c47110db6f60df1507dacd694f671557ad01`，约 2.07 GB。
- 镜像内 Ubuntu 22.04、ROS Humble、Agent 2.4.2、私有 Fast DDS 2.12.2、PlotJuggler 3.17.2、ROS 插件 2.3.1。当前 PX4 编译客户端为 XRCE v2 / 2.4.0。
- 完整软件包版本、Ubuntu rootfs 校验值、依赖源码提交及镜像 ID：`.state/image-build.tsv`；普通 UID、动态库、CLI 和插件验证：`.state/image-smoke.log`。

## 修改记录

| 范围 | 内容与原因 |
|---|---|
| `src/modules/uxrce_dds_client/dds_topics.yaml` | 增加实际角速度、期望角速度、积分状态、力矩指令、电机输出，设置 DDS 导出限频 |
| `src/modules/uxrce_dds_client/dds_topics.h.em` | 修复连接建立失败时误关闭未初始化订阅的缺陷，保证启动失败和重新接入不会破坏其他模块 |
| `src/modules/uxrce_dds_client/tests/test_dds_subscription_lifecycle.py` | 编译实际生成的生命周期代码，验证未初始化、部分失败、合法 fd 0、重复清理和重连 |
| `Tools/plotjuggler_live/` | 四个管理入口、隔离容器、消息同步、姿态转换、诊断、布局、中文文档和回归测试 |

联调时确认原 DDS 模板将未初始化的句柄置为 0，并在连接失败后无条件退订，可能误关闭 dataman 的订阅而导致 PX4 崩溃。修复仅调整 DDS 订阅的初始化与清理。六项针对性测试通过，其中三个关键用例在原模板上明确失败。真实启动失败、重新接入、停止监测测试随后通过。

ROS 监测范围根据现有客户端的 `Localhost only` 自动匹配。本机客户端为 `no`，监测进程使用 `ROS_LOCALHOST_ONLY=0`；强制设为 1 时无法发现 Agent 的发布者。没有更改客户端参数。

## 验收结果

| 项目 | 状态 | 实际证据或限制 |
|---|---|---|
| SITL 编译 | 通过 | 首轮 `.state/build-sitl.log`；修复后 `.state/build-sitl-dds-lifecycle.log` 记录重新生成、编译并链接成功 |
| 原模型回归 | 通过 | `.state/test-jmavsim.log`：几何、协方差、延迟、频率、独立噪声和 MAVLink 格式检查通过 |
| 模型及原有修改保护 | 通过 | 基线文件仅 DDS 配置变化，未改模型、传感器配置、原启动链 |
| 原启动流程及 QGC | 通过 | `.state/start-launch-verified.log`；原命令成功启动，QGC 复用；只读状态未解锁、预检通过、无 failsafe、QGC 连接正常 |
| 容器构建和普通用户运行 | 通过 | `.state/image-build.tsv`、`.state/image-smoke.log`；已授权但尚未重新登录的桌面也可直接运行，`.state/doctor-granted-group.log` |
| 消息同步与真实 ROS 编译 | 通过 | 272 个本地接口，工作区指纹 `16ed4e58a284fd4543c771e28bc97aea7e975b6d376decda71737925c2ea2210`；colcon 编译约 6 分 40 秒；重复 setup 成功复用，见 `.state/setup-repeat.log` |
| 数学、同步、诊断和脚本回归 | 通过 | 76 项测试通过，`.state/unit-tests-final.log`；Bash 语法、ShellCheck、差异空白检查通过 |
| DDS 订阅生命周期回归 | 通过 | `.state/dds-lifecycle-regression.log` 六项通过；`.state/dds-lifecycle-baseline-counterexample.log` 原模板反证有效 |
| 真实 DDS 接收与消息兼容 | 通过 | 全 11 类数据收到持续更新的有效样本；`.state/probe-cli-complete.json`、`.state/probe-reconnect.json`；位置有效标志均真，无 QoS 不匹配事件 |
| 姿态转换真实输出 | 通过 | `.state/derived-topic-check.json`：8 秒收到 198 个原始姿态及各 196 个转换输出；静止状态各轴最近样本差异小于 0.005° |
| 重复启动、停止和失败恢复 | 通过 | `.state/lifecycle-live.json`：重复启动复用同一容器；强制 1 秒启动失败后清理并重新接入；三类仿真进程 PID/启动时间保持不变 |
| 兼容外部 Agent 复用 | 通过 | `.state/external-agent-reuse.log`：验证 2.4.2、复用；stop 后外部 Agent 仍运行；测试结束仅删除本次创建的测试 Agent |
| 端口冲突、错误 Domain | 通过 | `.state/actual-fault-checks.json`：真实占用 8888、监测 Domain 1 均返回非零，未修改飞控 |
| 布局与消息版本一致性 | 通过 | 已监测 Topic 版本/命名空间变化会在打开 GUI 前明确报错；非 PX4 自定义 Topic 保留，9 项针对性回归与当前 5 份原生布局检查通过 |
| 缺失插件及图形连接故障 | 通过 | `.state/missing-plugin.log`、`.state/gui-unreachable.log`：隔离故障检查返回非零，运行中的监测不受影响 |
| 旧实例缺少 Topic 的诊断逻辑 | 通过 | 隔离回归验证基础通道通过但完整监测报告不完整 |
| 真实旧固件接入 | 未执行 | 未额外运行旧 PX4 固件实例，避免替换当前已验证构建 |
| PlotJuggler 原生布局保存、重载、更新 | 通过 | 四份独立布局及四页组合布局实际加载通过，共 15 图、29 曲线；暂停、叠加、缩放、平移、保存和重载通过；有限 60 秒缓存已验证。版本、文件校验值及截图见 `layouts/verification.json` |
| QGC 起飞、悬停、位置变化和降落 | 通过 | QGC 起飞约 3 m、悬停、向东移动约 5 m、降落自动锁定；真实控制数据与界面曲线有效，详见下方飞行记录 |
| 五分钟性能对比 | 通过 | 开启监测连续采样 300.44 秒，RTF 0.54850，对比 60.08 秒基线的 0.54043，未观察到仿真速度下降；CPU、RSS 和状态快照见下表 |

### 真实接收频率

以下为首次全通道通过的 10 秒接收窗口，按主机单调时钟测量；DDS 上限采用 PX4 时钟，仿真慢于实时会使墙钟接收频率相应降低。状态 Topic 还受源发布条件影响。

| 数据 | 实测 Hz（约） |
|---|---:|
| 姿态、本地位置、积分状态、电机输出 | 26.25 |
| 实际角速度、期望角速度、力矩指令、综合传感器、里程计 | 52.54 |
| 估计器状态标志 | 0.52 |
| 飞行状态 | 1.04 |

实际名称包含 `/fmu/out/vehicle_local_position_v1` 和 `/fmu/out/vehicle_status_v4`。此频率表统计地面未解锁状态；飞行期控制输出另见下节。

### QGC 飞行与动态数据

使用现有 QGC 界面正常起飞至约 3 m，悬停后通过 Go to location 向东移动约 5 m，再通过 Land 在当前位置降落。最终原生状态为 `arming_state=1`、`landed=True`、`at_rest=True`，位置 NED 约 `[-0.47, 5.42, -0.06] m`；未修改传感器或控制参数。

只读 ROS 记录中有 780 个解锁采样点，各点四电机指令均为有限值，角速度实际/期望、三轴积分和力矩指令同时存在，位置与速度有效标志均真。已观察到非零积分和动态控制输出。该记录含约 3.37 m 的高度范围；后续完整水平移动及降落由 QGC 截图、原生状态与独立落地后 ROS 采样补充验证。两段记录及最终快照均未发现 failsafe 或 GCS 丢链，最后丢链计数为 0。

证据：`.state/flight-acceptance-summary.json`、`flight-live.json`、`flight-landed.json`、`flight-final-vehicle_status.txt`、`flight-final-vehicle_land_detected.txt`、`flight-final-vehicle_local_position.txt`。实际界面截图包括 `flight-hover.png`、`flight-moved.png`、`flight-landed-qgc.png`、`flight-attitude-curves.png` 和 `flight-pid-curves.png`。

第一段观察器的 600 秒上限包含了界面准备时间，在完整移动和降落前结束；没有把它描述为整段飞行的连续记录。落地后的独立 20 秒 ROS 采样收到实际角速度 1087 条、期望角速度与力矩各 1086 条、位置/积分/电机各 543 条，确认监测持续工作。

### 五分钟性能对比

本机使用同一模型和原启动链，机体保持地面未解锁静止。无监测基线采样 **60.08 秒**；开启 Agent、姿态转换节点和 PlotJuggler 实时订阅后采样 **300.44 秒**，PlotJuggler 使用 60 秒数据缓存。两组中的 PX4、jMAVSim、QGC PID 均相同，分别为 111740、111939、47685。原始记录为 `.state/performance-baseline-final.json` 和 `.state/performance-monitor-300s.json`。

实时因子 RTF（PX4 姿态消息时间增量 / 主机单调时钟增量）从 **0.5404319** 变为 **0.5485036**，相对变化约 **+1.49%**。本次比较未观察到仿真速度下降；主机基线本身慢于实时。两组为不同时段、不同长度的单次采样，背景负载和图形绘制可能波动，结论限于本机此次静止场景。

约每秒读取一次进程累计 CPU 时间和 RSS，共获得 60 / 300 个样本。CPU 为首末样本间的平均占用，**100% 表示一个逻辑核**；RSS 为各窗口最后一个样本，单位 MiB。

| 进程 | 基线 CPU | 开启监测 CPU | 基线末 RSS MiB | 监测末 RSS MiB |
|---|---:|---:|---:|---:|
| PX4 | 8.65% | 9.76% | 8.05 | 8.05 |
| jMAVSim | 181.91% | 185.63% | 769.44 | 774.49 |
| QGC | 132.55% | 123.17% | 287.33 | 270.71 |
| DDS Agent | 未运行 | 3.83% | — | 24.27 |
| 姿态转换节点 | 未运行 | 3.40% | — | 69.03 |
| PlotJuggler | 未运行 | 67.78% | — | 189.67 |

新增三个监测进程平均 CPU 合计约 **75.01% 的单核**，各进程末次 RSS 数值合计约 **282.97 MiB**。RSS 包含共享页，多进程相加可能重复计数，且不是峰值、独占内存或系统净内存增量；未据此作内存泄漏判定。

`.state/performance-status-before.txt`、`.state/performance-status-after.txt` 两份状态快照均为未解锁，`failsafe=False`、`gcs_connection_lost=False`、`gcs_connection_lost_counter=0`。这些快照支持所检查时刻无失效保护、GCS 丢链计数未增加；它们不替代全程失效保护事件记录。

## 已知边界

- 当前数据提供已有积分状态、归一化力矩指令和电机输出，没有新增完整 P/I/D 分项，也不代表实测物理力矩。
- 停转电机和未使用通道的 NaN 保留；位置有效标志保留，诊断单独报告。
- 曲线采用接收时间，保留收到的 PX4 时间字段，不声称精确端到端传输延迟。慢速 SITL 与 Agent 墙钟不同步时，真实诊断观察到消息时间偶尔回退；姿态节点按约定重置连续航向段并记录日志，没有修改飞控时间同步参数。
- 原生插件首次恢复可能要求确认 Start Streaming 与 Topic 订阅选择。没有修改插件绕过确认。
- 首次连接需要 PX4 时间同步收敛，默认启动观察窗口 30 秒；后续诊断采样 10 秒。慢速机器可调整本地 `PJ_STARTUP_SECONDS`。
- 新增 Topic 必须重新编译并正常重启 PX4；attach 不会重编译、重启或更改飞控参数。
