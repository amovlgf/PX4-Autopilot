# Flycore-Q 入库检测

## 固件和 USB 2.0 工位

在 PX4 开发环境中运行 `make amovlab_flycore_test`。烧录工位使用
`build/amovlab_flycore_test/amovlab_flycore_test.px4`，镜像和 Bootloader 板号均为
`1218`。该固件用于台架检测，PWM 只连接检测夹具的绿灯输入，工装不接电机、舵机等执行器。

进入 Bootloader 后，使用 PX4 烧录工具完成 USB 上传和镜像校验。

```sh
python3 Tools/px_uploader.py --port /dev/ttyACM0 build/amovlab_flycore_test/amovlab_flycore_test.px4
```

选择该板实际枚举的端口，保留烧录工具的板号匹配检查。重启后用 USB 串口终端连接飞控，
连按三次回车进入 NSH，运行 `ver all` 核对版本。USB 烧录及版本核对结果由独立工位保存。

## 检测夹具接线

| 接口 | 飞控设备或通道 | 夹具接法和判定 |
| --- | --- | --- |
| UART2 | `/dev/ttyS0` | 本端 TX 和 RX 短接 |
| UART8 | `/dev/ttyS6` | 本端 TX 和 RX 短接。产测固件使用 USB 调试控制台 |
| UART6 | `/dev/ttyS4` | 本端 TX 和 RX 短接 |
| UART7 | `/dev/ttyS5` | 本端 TX 和 RX 短接 |
| UART4 | `/dev/ttyS2` | RTK 的 GPS 数据口。夹具预配置 UM982，使飞控的 GPS 驱动可收到新数据 |
| UART5 | `/dev/ttyS3` | 6490 串口，TX/RX 交叉并共地，双向 MAVLink 2 |
| UART3 | `/dev/ttyS1` | 8550 串口，TX/RX 交叉并共地，双向 MAVLink 2 |
| SPI1、SPI2、SPI4 | 板载 BMI088、ICM42688P、FRAM | 板内检查，无需外部连线 |
| PWM | MAIN 1–8 | 接 8 路绿灯夹具。飞控报告闪灯动作，工位目视确认每路亮灭并单独记录 |
| CAN | CAN1 与 CAN2 | CAN_H 对 CAN_H，CAN_L 对 CAN_L，并共地。按夹具设计确认总线两端终端电阻 |
| I²C | I²C1 与 I²C4 | 夹具将同一块 BMI088 的 SDA、SCL、电源和 GND 接到两路接口，飞控逐路探测 |
| SD 卡 | SD 卡槽 | 插入工装用测试卡，飞控写入并读回校验测试文件 |

## 8550 发起检测

上电后入库检测服务自动启动。8550 发送开始消息，飞控空闲时从头执行一次全测。
6490 在 UART5 上响应 MAVLink `PING`。飞控在 UART3 发出各项结果后直接返回空闲。8550 收到有效报告后保存结果，并判定 UART3 双向通信通过。
消息定义及模拟对端见 [协议文档](../../../protocol/flycore_incoming_test/README.md)。

两路串口默认 115200、8N1、无流控。飞控参数 `FQ_8550_*` 和 `FQ_6490_*` 分别设置
波特率及对端的系统、组件 ID。USB 控制台上的参数修改可用 `param save` 保存至
`/fs/microsd/flycore_q_params.bson`，重启后生效。

使用 USB 控制台运行 `hw_test_all` 可本地全测，也可用 `hw_test <项目>` 运行单项。远程和本地检测
共用执行锁。入库记录包括 8550 保存的飞控检测结果、8 路绿灯目视结果和 USB 工位结果。

## 真机验收

先用模拟对端检查飞控及其夹具，再换接真实 8550 和 6490。故障测试使用默认串口配置，断电后插拔 SD 测试卡。
记录从发送 `START` 到收到 `RESULT` 的耗时，供 8550 设置结果等待时间。

| 场景 | 通过条件 |
| --- | --- |
| 烧录、本地检测和独立上电 | USB 烧录工具识别板号 `1218` 并校验通过。保存 `ver all` 输出，运行 `hw_test_all` 和 `hw_test sd`。断开 USB 后从夹具独立上电，8550 仍能启动一轮完整检测。 |
| 正常全测 | 8550 保存一条结果，`overall_pass=1`，`executed_mask=passed_mask=0x7FFF`，UART3 双向通过。工位分别确认 MAIN 1–8 每路绿灯有亮灭动作。 |
| 6490 无应答、缺 SD 卡 | 分别制造单项故障，再同时制造两项故障。三种情况下 `overall_pass=0`，`executed_mask=0x7FFF`，`passed_mask` 依次为 `0x7FDF`、`0x3FFF`、`0x3FDF`。8550 收到有效报告后仍记录 UART3 双向通过。 |
| UART3 断链或丢失报告 | 8550 记录结果接收超时及本轮飞控结果未知。恢复链路后确认旧轮次已结束，再发送相同的 `START`，取得新一轮的完整结果。无法确认旧轮次结束时，重启飞控并清空 8550 接收缓存后复测。 |
| 重复触发和本地互斥 | 远程全测期间再次发送 `START`，收到 `BUSY`，原轮仅产生一条 `RESULT`。USB 本地测试期间，远程请求收到 `BUSY`。远程测试期间运行 `hw_test_all`，本地执行位图为 `0x0000`。 |

台架记录以板卡序列号和检测时间为索引。

- 固件源码提交、`.px4` 和 `flycore_test.xml` 的 SHA-256、夹具版本、8550 和 6490 软件版本。
- USB 烧录校验日志和 `ver all` 输出。
- 每轮的场景、用时、`overall_pass`、`executed_mask`、`passed_mask`、UART3 状态。出现超时时保留工位日志。
- MAIN 1–8 各路绿灯的目视结果。
