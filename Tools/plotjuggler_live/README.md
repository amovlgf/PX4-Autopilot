# PX4 jMAVSim 实时监测

在现有 jMAVSim 仿真旁运行 PlotJuggler，实时查看 PX4 的姿态、角速度跟踪、位置、速度和已发布的控制器状态。飞行控制仍通过原有 QGroundControl 进行。

数据链路：`PX4 uORB → uXRCE-DDS Client → UDP → Micro XRCE-DDS Agent → ROS 2 → PlotJuggler`。

## 环境与首次部署

本次目标工程为 `/home/amovlgf/PX4-Autopilot`，主机为 Ubuntu 20.04。PX4、jMAVSim 和 QGC 保持主机原有运行方式；Ubuntu 22.04 容器提供 ROS 2 Humble、Agent 2.4.2、PlotJuggler ROS 插件。当前 PX4 使用 DDS v2，不应直接换用 Agent v3。

需要可用的 Docker 服务以及当前终端访问 Docker 的权限。主机安装命令：

```bash
sudo apt-get install -y docker.io curl
```

如当前账号尚未获得 Docker 权限，可由管理员将账号加入 docker 组。已获授权但桌面会话尚未刷新时，脚本会先核对系统组成员记录，再仅为 Docker 客户端子进程启用该权限，并恢复客户端原主组；当前会话无需重新登录。管理脚本自身的 UID、GID 保持不变，因此仍能核验已有 PX4 进程。此处理覆盖首次镜像构建，不请求密码，也不修改用户组、主机软件源或全局 Shell 环境。脚本还可复用已有的非交互 `sudo` 授权。

首次准备监测环境：

```bash
cd /home/amovlgf/PX4-Autopilot
./Tools/plotjuggler_live/setup.sh
```

首次构建默认从 Ubuntu 官方站点下载 Ubuntu-base 22.04.5 amd64 根文件系统，核对固定 SHA256 后导入本地基础镜像；此路径不依赖 Docker Hub。随后在容器内安装 ROS 2 和 PlotJuggler，并编译 Agent。主机系统保持 Ubuntu 20.04。下载缓存位于 `.state/ubuntu-base/`，成功构建的基础镜像标识、Agent 和软件包版本记录在 `.state/image-build.tsv`。

默认构建使用直接网络（`PJ_BUILD_PROXY=none`），仅对本次根文件系统下载及容器构建清除代理，不改变当前终端或系统代理。如所在网络必须使用当前终端已经配置好的代理，可执行：

```bash
PJ_BUILD_PROXY=inherit ./Tools/plotjuggler_live/setup.sh --rebuild-image
```

`--rebuild-image` 会重新执行镜像构建；已完成且配置未变化的构建层仍可复用。单独调用 `container/build.sh` 时，`--no-proxy`、`--use-proxy` 分别对应上述两种网络选项。需要自选已有 Ubuntu 22.04 基础镜像时，在工程根目录可使用 `./Tools/plotjuggler_live/container/build.sh --base-image <镜像名>`，再运行 `setup.sh`；该镜像本地不存在时会由 Docker 拉取，取决于 Docker 服务自身的网络配置。

镜像准备完成后，setup 从本工程的 `msg/*.msg`、`msg/versioned/*.msg`、`srv/*.srv`、`srv/versioned/*.srv` 生成独立消息工作区并编译。消息同步使用内容指纹，不借用或删除用户其他 ROS 工作区。只有构建成功才切换当前工作区。已有监测镜像会被复用；源消息变化只需重新运行 setup。

第一次补充 DDS 导出后，需要编译并在正常停机后重新启动一次 SITL：

```bash
cd /home/amovlgf/PX4-Autopilot
make -j2 px4_sitl_default
```

正在运行的旧 PX4 进程不会自动获得新导出列表。请先通过 QGC 正常降落，再在原 PX4 控制台退出仿真，用原有方式重新启动。日常 `--attach` 不编译、不重启 PX4；旧实例仍可先查看原来已经导出的姿态和位置。

## 三种日常使用场景

### A. jMAVSim 已经运行

在图形桌面的终端中执行：

```bash
/home/amovlgf/PX4-Autopilot/Tools/plotjuggler_live/start.sh --attach
```

脚本识别当前工程的 PX4，检查实际 DDS 端口和 Domain，复用兼容 Agent，启动缺少的监测组件并打开 PlotJuggler。默认先采样 30 秒，为 DDS 时间同步初始化及实际收数留出时间，再打开图形窗口；该等待由 `PJ_STARTUP_SECONDS` 设置。省略参数也表示 attach。

### B. 仿真尚未运行

```bash
/home/amovlgf/PX4-Autopilot/Tools/plotjuggler_live/start.sh --launch
```

脚本通过原命令 `unset HEADLESS; make px4_sitl_default jmavsim` 打开独立 PX4 控制台，由现有启动链加载 `jmavsim_iris`（10017）和定制传感器模型。QGC 使用已有包装脚本及 Debug 程序。已经运行的程序会被复用。

### C. 只关闭实时监测

```bash
/home/amovlgf/PX4-Autopilot/Tools/plotjuggler_live/stop.sh
```

只停止本工具创建的监测容器及其中的 Agent、转换节点、PlotJuggler。已有 Agent、PX4、jMAVSim 和 QGC 保持运行，包括通过 `--launch` 打开的仿真。脚本不使用按名称批量杀进程的方式。

无图形界面时，可先运行：

```bash
/home/amovlgf/PX4-Autopilot/Tools/plotjuggler_live/start.sh --attach --no-gui
/home/amovlgf/PX4-Autopilot/Tools/plotjuggler_live/doctor.sh
```

如需从无界面模式改为界面模式，先执行 `stop.sh`，再从桌面终端执行 `start.sh --attach`。

## PlotJuggler 操作和曲线含义

没有加载布局时，在 Streaming 区域选择 **ROS2 Topic Subscriber**，点击启动按钮并确认订阅列表。所有 PX4 类型必须在本工具的 ROS 工作区环境中加载。

加载包含订阅配置的原生布局时，官方程序可能依次出现两次确认：先在 **Start Streaming?** 中选择 **Yes**，再确认预选的 ROS Topic 列表。它们是正常的原生启动步骤。当前核对的版本是 PlotJuggler 3.17.2 与 ROS 插件 2.3.1；实际安装版本记录在构建清单中。带订阅配置的布局只通过 `--layout` 加载，不额外传入 `--start_streamer`，避免重复启动；旧版程序对后者的参数要求也不同。

从左侧数据列表把数值字段拖入绘图区；将第二条曲线拖入同一绘图区即可叠加。使用 PlotJuggler 原生缩放、平移、暂停及重新跟随实时数据功能。默认保留最近 60 秒数据，暂停显示不会暂停无人机。

按以下四组组织标签页，字段前缀使用 `doctor.sh` 输出的实际 Topic 名称：

| 标签页 | 字段 | 单位/坐标 |
|---|---|---|
| Attitude | `/px4_monitor/attitude_deg/vector/x`、`y`、`z` | Roll/Pitch/Yaw，度 |
| Attitude | `/px4_monitor/yaw_unwrapped_deg` 的 `data` | 连续航向，度 |
| Rate Control | `vehicle_rates_setpoint.roll/pitch/yaw` 与 `vehicle_angular_velocity.xyz[0/1/2]` | FRD 机体系，rad/s |
| Position | `vehicle_local_position.x/y/z`、`vx/vy/vz` | NED，m、m/s；Z 正方向向下 |
| PID Analysis | `rate_ctrl_status.rollspeed_integ/pitchspeed_integ/yawspeed_integ` | 归一化力矩中的积分贡献 |
| PID Analysis | `vehicle_torque_setpoint.xyz[0/1/2]` | 归一化控制器力矩指令，不是实测 N·m |
| PID Analysis | `actuator_motors.control[0..3]` | 当前四旋翼的归一化电机推力指令 |

例如分析俯仰角速度：把 `vehicle_rates_setpoint.pitch` 与 `vehicle_angular_velocity.xyz[1]` 拖入同一绘图区。两者均为 rad/s，可直接比较趋势、超调、振荡和跟踪误差。

PX4 的消息版本会影响 Topic 名称：当前 `VehicleLocalPosition` 的 `MESSAGE_VERSION=1`，对应 `/fmu/out/vehicle_local_position_v1`；版本 0 不追加后缀。不要仅按旧文档中不带后缀的名称查找。

姿态转换按 Hamilton 四元数 `w,x,y,z` 解释 FRD 到 NED 的旋转，归一化后生成欧拉角。航向展开在时间回退、姿态重置或重新启动时重新初始化；欧拉角在接近 ±90° 俯仰时具有固有奇异性，应结合原始四元数判断。

图中默认采用消息接收时间。原始 PX4 `timestamp`/`timestamp_sample` 字段以微秒表示，不能当作秒直接绘图，也不能仅用接收时间曲线声称测得精确传输延迟。

位置有效标志为假时，数值不能作为有效定位结果。未解锁或未使用电机通道出现 NaN 是消息语义的一部分，不应填成零。当前仅导出现有积分状态、力矩指令及电机输出，没有新增 P/D 分项、饱和状态或其他未发布的 PID 内部变量。

通过 PlotJuggler 的保存布局功能，把自己的布局保存到容器内 `/work/layouts/local/`（对应主机本目录 `.state/layouts/local/`）；目录不存在时可在保存对话框中新建。容器内 `/opt/px4-monitor` 是只读工具目录。自定义布局不会被 setup 覆盖，后续可在 GUI 中重新打开，也可在本地配置中把 `PJ_LAYOUT` 指向该文件的容器路径。启动时若存在 `/work/current/layouts/live.xml`，该工作区布局具有更高加载优先级。

本次已逐一实际加载验证的原生布局位于 `layouts/`：

| 文件 | 内容 |
|---|---|
| `live.xml` | 四个标签页，15个绘图区、29条曲线；默认打开 Attitude |
| `attitude.xml` | 姿态与连续航向，4条曲线 |
| `rate_control.xml` | 三轴角速度跟踪与陀螺数据，9条曲线 |
| `position.xml` | NED位置与速度，6条曲线 |
| `pid_analysis.xml` | 三轴积分、归一化力矩、四电机推力，10条曲线 |

在 GUI 中使用 Layout 的加载按钮，选择容器内 `/opt/px4-monitor/layouts/` 下的文件。已经处于实时订阅时，切换布局的 **Start Streaming?** 提示可选 **No**，继续使用当前订阅；首次启动则选 **Yes** 并确认 Topic。上述文件保留60秒缓存设置，启动脚本同时处理原生程序的缓存上限语义，界面应显示黑色 **60 sec**，而不是红色 `=inf`。

布局版本、文件校验值及实际GUI证据索引见 `layouts/verification.json`；本次布局验证使用未解锁的仿真实例，飞行动态验证另列于 `VALIDATION.md`。布局使用当前清单的Topic名称；消息版本或命名空间变化后应重新生成并加载核对。

源码生成器在容器内默认输出 `/work/layout-candidate.xml`（主机 `.state/layout-candidate.xml`）及其 `.sources.json` 来源记录，不会自动把新的候选布局标记为 GUI 验证通过。

打开 GUI 前，启动脚本会检查实际选中布局的 PX4 订阅名和曲线路径。已监测 Topic 的版本、命名空间或 DDS 导出与当前消息清单不符时会明确退出，并显示实际应使用的名称；诊断收到新版本消息不会掩盖旧布局空白。非 PX4 自定义 Topic 保留，清单外额外 PX4 Topic 会提示需要手工核验。布局文件不存在时仍可进入原来的手工订阅流程。

消息升级后如出现布局过期提示，可在工程根目录运行下面的一次性容器命令生成候选（不启动仿真、Agent 或 GUI）：

```bash
bash -c '
source Tools/plotjuggler_live/common.sh
initialize_state
select_docker
common_container_args
docker_cmd run --rm "${PJ_CONTAINER_ARGS[@]}" "$PJ_IMAGE" \
    python3 /opt/px4-monitor/python/generate_layout.py \
    --manifest /work/current/manifest.json \
    --output /work/layouts/local/updated-candidate.xml
'
```

生成器不会覆盖已有不同内容的文件；若该候选名已存在，请换一个输出文件名。在本地 `config/settings.env` 中设置 `PJ_LAYOUT=/work/layouts/local/updated-candidate.xml` 后重新启动监测，并完成原生订阅确认和曲线验证。如果 `/work/current/layouts/live.xml` 存在，它仍有优先级：先备份并移走该文件，或手动修正其中的 Topic。上述候选仅校验源码字段，不能自动视为已完成 GUI 验收。

## DDS 导出频率

| uORB 数据 | DDS 上限 |
|---|---:|
| vehicle_attitude | 50 Hz |
| vehicle_angular_velocity、vehicle_rates_setpoint | 各 100 Hz |
| vehicle_local_position | 50 Hz |
| vehicle_odometry、sensor_combined | 各 100 Hz |
| rate_ctrl_status | 50 Hz |
| vehicle_torque_setpoint | 100 Hz |
| actuator_motors | 50 Hz |
| estimator_status_flags | 5 Hz |

上限不保证消息始终达到该频率；受 uORB 实际发布、飞行状态和调度影响。诊断频率按主机经过时间统计，仿真运行慢于实时速度时，接收频率也会相应下降。它们不改变控制器或传感器内部更新频率。

## 诊断与配置

```bash
/home/amovlgf/PX4-Autopilot/Tools/plotjuggler_live/doctor.sh
```

检查实际数据接收，而非仅检查进程或 Topic 名称。诊断使用 Best Effort 订阅，默认采样 10 秒（`PJ_PROBE_SECONDS`），输出消息数、接收频率和状态，并将 JSON 报告保存在 `.state/logs/`。启动采样使用独立的 `PJ_STARTUP_SECONDS=30`；仿真速度较慢、时间同步尚未稳定时，可适当延长启动采样。

| 现象 | 检查方向 |
|---|---|
| Topic 不存在 | 对照当前 DDS 导出、版本后缀、命名空间和运行实例是否已重启 |
| 有 Topic 没消息 | uORB 当前是否发布、Agent/客户端日志、飞行状态及 QoS |
| 消息定义或类型不一致 | 先停止监测，重新运行 setup 同步本地源码；确保运行中的 PX4 与源码对应 |
| 诊断收到消息而图中无曲线 | 确认 ROS 插件已开始订阅、字段选择、布局及数据窗口 |
| Domain 或通信范围不一致 | Domain 与当前 PX4 对齐；`ROS_LOCALHOST_ONLY=auto` 自动跟随现有客户端，不修改飞控参数 |
| UDP 8888 已占用 | 检查是否为兼容 Agent；工具不会终止未知进程或自动换端口 |
| GUI 无法打开 | 从图形桌面终端运行，检查 DISPLAY、XAUTHORITY；CLI 验证可使用 --no-gui |
| Docker 权限不足 | 确认 Docker 服务运行且系统已为当前账号授予 docker 组权限；脚本会自动复用已授予权限，无需手工改变主组，也可使用已有非交互 sudo 授权 |

可复制 `config/settings.env.example` 为 `config/settings.env`，设置本地路径、端口、Domain、采样时长或缓存长度。`ROS_LOCALHOST_ONLY` 默认 `auto`，读取 PX4 DDS participant 的现有 Localhost only 状态并对齐监测端；如果手工设置为 `0` 或 `1`，必须与客户端一致。本次客户端为 `no`，对应 `0`，强制设为 `1` 会导致无法收数。该文件是可信本地 Shell 配置，使用前检查内容。不要把认证信息加入版本库。

`.state/` 保存编译产物、日志、独立消息工作区、运行记录和原始配置基线，默认不加入 Git。setup 和 stop 不清理这些数据。启动、停止通过同一把锁串行处理，容器按完整 ID 和归属标签检查。

## 验证与扩展

实际执行情况、限制、修改列表见 [VALIDATION.md](VALIDATION.md)。GUI 或飞行测试未执行时，不以 CLI 成功代替其验收。

新增监测 Topic 时，先确认本地消息定义、实际 uORB 发布和单位，再补充 DDS publications、同步消息并重新编译/启动 SITL，最后验证接收频率和布局。现有 `estimator_status_flags` 可扩展 EKF 状态曲线。额外 PID 内部变量需要先明确是否已有 uORB 字段；没有时另行设计最小导出。故障切换分析可后续增加相关状态与有效性曲线，本工具不会自行触发故障或改变传感器参数。

参考：[PX4 实时绘图](https://docs.px4.io/main/en/debug/plotting_realtime_uorb_data)、[ROS 2 用户指南](https://docs.px4.io/main/en/ros2/user_guide)、[uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds)。
