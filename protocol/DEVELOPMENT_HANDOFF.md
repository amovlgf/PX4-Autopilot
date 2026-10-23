# RID / UOM / 设备管理桥开发交接

更新日期：2026-10-23。范围为当前 PX4 仓库及 `codex/rid-device-management` 分支；
QGroundControl 和 ArduRemoteID 是独立仓库，其提交、构建和发布不包含在本次交接中。

## 开发基线

- 仓库：`git@github.com:amovlgf/PX4-Autopilot.git`。
- 分支：`codex/rid-device-management`。
- 基础提交：`020aacdf23`（CAN RID 解锁检查）、`79163d6909`（身份与 UOM 就绪检查）。
- 本次增量：通用设备管理桥、共享管理协议、相关测试及本交接文件。
- 编译目录和生成缓存已在提交前清理；后续开发需要重新生成构建工程。
- 遵循根目录 `AGENTS.md`，编辑前读取匹配路径的指导；提交使用 commit 技能。

## 已实现功能

| 功能 | 当前能力 | 主要代码入口 |
| --- | --- | --- |
| CAN RID 健康与解锁检查 | 读取 DroneCAN ArmStatus，3 秒超时；发现 CAN RID 后持续采用 CAN 状态，MAVLink 心跳不能掩盖其故障；解锁前刷新状态 | `src/modules/commander/Commander.cpp` |
| RID 身份快照 | GetNodeInfo/GetSet 读取 UID、产品码、登记号、UIN、类别、激活状态、revision、digest；前后复核 boot/revision，处理重启、替换、掉线和冲突 | `src/drivers/uavcan/RidIdentityClient.*`、`src/lib/remote_id/RidState.hpp` |
| UOM 会话与就绪 | 绑定飞控 boot、QGC 会话/来源、RID boot/revision/hash；单一活跃报送所有者，序号防旧包；心跳 3 秒、ACK 5 秒、身份 12 秒独立老化 | `src/modules/mavlink/RidProtocol.*`、`src/modules/commander/HealthAndArmingChecks/checks/openDroneIDCheck.*` |
| RID 动态数据与诊断 | Location/System 默认 5 Hz、静态身份 1 Hz；验证样本新鲜度、UTC 进展和椭球高；记录身份与 UOM 状态 | `src/drivers/uavcan/remoteid.*`、`src/modules/mavlink/streams/`、`src/modules/logger/` |
| 通用设备管理桥 | MAVLink → uORB → DroneCAN 服务 200；在线节点发现、最多 192 字节透明传输、回复返回原链路与发送者 | `src/modules/mavlink/MavlinkDeviceBridge.*`、`src/drivers/uavcan/UavcanDeviceBridge.*` |
| 桥接并发与超时 | 8 槽共享请求队列；每链路及 CAN 各一个在途请求；满队列 BUSY；排队/服务各 1 秒、响应 2.5 秒；拒绝迟到和错路回复，不自动重试 | `src/lib/device_bridge/`、`msg/DeviceBridgeRequest.msg`、`msg/DeviceBridgeResponse.msg` |
| 协议与生产/台架隔离 | MAVLink 2 自定义消息、共享管理 codec、生产构建拒绝 mock，专用台架构建允许 mock | `protocol/amov_rid_uom/`、`protocol/amov_device_bridge/`、`boards/amovlab/flycore/` |

`COM_RID_UOM=1` 要求有效激活身份、同一 RID 节点的新鲜健康状态及新鲜报送 ACK。
单独 UOM 故障在已解锁后只告警和记录，不新增自动返航、降落或停机动作。
原 Remote ID 策略和显式预检旁路仍按既有配置处理。

PX4 设备管理桥负责透明传输。共享协议定义 INFO、METADATA、GET、SET、STATUS、
ACTIVATE、RESET、REBOOT、PROVISION 九种操作；实际参数保存、激活、签名验证、
恢复和出厂配置由 QGC/目标设备实现。传输 OK 仅表示收到响应，业务结果需要继续解析；
TIMEOUT 表示执行结果未知，写操作不可直接当作未执行而盲目重试。
在线节点列表基于已接收的 DroneCAN NodeStatus，不是主动扫描。

## 协议约束

- `amov_rid_uom/amov.xml` 定义消息 61000–61004；构建时叠加生成，不修改上游 MAVLink 子模块。
- 新增 `AMOV_DEVICE_REQUEST=61003`、`AMOV_DEVICE_RESPONSE=61004`，长度 210/211、CRC 199/62。
- 管理载荷：请求 108 字节、回复 192 字节，传输上限 192 字节；定义见 `amov_device_bridge/management_protocol.h`。
- 修改协议时必须同步 PX4、QGroundControl、ArduRemoteID 三端的镜像文件，并运行镜像及互解码测试。
- `amov_rid_uom` 镜像包含 `amov.xml`、`generate.py`、`test_protocol.py`。
- `amov_device_bridge` 镜像包含 codec 头文件、C++/Python 测试、DSDL 服务文件和 `status_digest_vector.json`。
- 旧 `amov_rid_uom/README.md`、`INTEGRATION.md` 在本次工作区中已删除；可从 `79163d6909` 查阅历史身份 digest 布局、字段约定及联调验收表。旧步骤需要结合新管理桥重新核对。

## 构建、参数和诊断

以下所有命令从 `/home/amovlgf/PX4-Autopilot` 执行。两个 NuttX 目标应串行构建：

```sh
make amovlab_flycore_rid -j4
make amovlab_flycore_ridbench -j4
```

- `rid` 为生产配置，编译期不接受 mock UOM。
- `ridbench` 为台架配置，接受满足完整前提的 mock UOM，并显示测试提示。
- 两者启用 `CONFIG_UAVCAN_DEVICE_BRIDGE=y`，默认 `UAVCAN_ENABLE=2`、`COM_ARM_ODID=2`、`COM_RID_UOM=1`；已保存参数可能覆盖默认值。
- 当前 `firmware.prototype` board_id 与 `BOARD_TYPE` 为 **106**，来自基础提交；实际刷写前需与设备 bootloader/板型匹配，本次未进行验证。
- 诊断命令：`listener rid_identity`、`listener rid_uom_status`、`listener open_drone_id_arm_status`。

## 测试入口

```sh
make tests TESTFILTER="'CommanderRemoteID|RidState|RidBenchPolicy|DeviceBridge'"

python3 protocol/amov_device_bridge/test_protocol.py --sanitize \
  --mirror /home/amovlgf/ArduRemoteID/protocol/amov_device_bridge \
  --mirror /home/amovlgf/QGroundControl/protocol/amov_device_bridge

python3 protocol/amov_rid_uom/test_protocol.py \
  --px4-definitions src/modules/mavlink/mavlink/message_definitions/v1.0 \
  --pymavlink-root src/modules/mavlink/mavlink \
  --qgc-definitions /home/amovlgf/QGroundControl/.cache/rid-local/mavlink/message_definitions/v1.0 \
  --qgc-pymavlink-root /home/amovlgf/QGroundControl/.cache/rid-local/mavlink \
  --mirror /home/amovlgf/ArduRemoteID/protocol/amov_rid_uom \
  --mirror /home/amovlgf/QGroundControl/protocol/amov_rid_uom
```

`--pymavlink-root` 指向包含 `pymavlink` 包的父目录；传入包目录本身可能误用系统旧生成器。
QGC 缓存路径是当前机器的实际路径，换机器或清缓存后需重新准备。

本次实测：

- `amovlab_flycore_ridbench` 增量编译及链接通过，Flash 使用 1,879,336 字节（95.59%）。
- MAVLink 双向帧、损坏帧拒绝、wire 契约、三端镜像和碰撞检查：5 项通过。
- 管理 codec 三端镜像、边界/字段测试及 AddressSanitizer/UndefinedBehaviorSanitizer 检查通过；沙箱不支持 LeakSanitizer 时在沙箱外运行。
- `make format` 已执行，本次改动的 14 个源码 C/C++ 文件格式检查通过；全仓 `make check_format` 被 GPS 子模块已有 5 个文件的格式问题阻挡，这些与本次功能无关的文件保持原样。
- 原生 PX4 CMake/CTest：`unit-DeviceBridge` 8/8、`functional-DeviceBridgeQueue` 6/6 通过，使用真实 uORB 和 ThreadSanitizer，无竞态报告。
- C++ 测试复用了本机 GoogleTest 1.10.0；功能测试运行器未启动工作队列，初始化打印工作队列错误，但 6 项同步队列测试通过。本次未重跑基础 RID/Commander 的 31 项测试。

## 后续开发与验证

1. 验证生产 `rid` 目标、实际 bootloader 与板型配置；本次仅重新编译了 `ridbench`。
2. 完成 QGC → PX4 → RID 真机链路、节点发现、读取/写入、激活与冷启动持久化联调。
3. 验证多 GCS/多 RID、节点重启替换、CAN 高负载、掉线、迟到响应及写入结果未知时的恢复流程。
4. 完成无线接收互通、正式 UOM provider、账号/证书及报送验收。
5. 维护三端协议一致性，补全新的设备管理桥验收记录；软件测试通过不代表上述硬件或正式平台验收已完成。

下次 Codex 对话可直接要求：

> 在 `/home/amovlgf/PX4-Autopilot` 的 `codex/rid-device-management` 分支继续开发。
> 先阅读 `AGENTS.md` 和 `protocol/DEVELOPMENT_HANDOFF.md`，核对工作区与远程状态，
> 再执行本次新需求。保留生产/台架隔离、会话与身份时效规则以及设备桥透明传输边界。
