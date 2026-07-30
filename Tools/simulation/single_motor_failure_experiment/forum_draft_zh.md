# 四旋翼三桨迫降保护：PX4 单电机故障降级着陆实验

> **重要说明**
>
> 这是一个 PX4 jMAVSim SITL 探索性实验，不是可直接用于实机的容错飞控方案。
> 本文中的“保护”是指单个电机停止后，系统尝试使用其余三个电机进入降级
> Land 流程。它不代表动力冗余，也不代表已经具备实机安全认证所需的故障
> 检测、结构强度、传感器可靠性和跨机型验证。

## 1. 为什么做这个实验

标准四旋翼只有四个独立升力源。任意一个电机停止后，剩余三个电机通常无法
同时维持完整的滚转、俯仰、偏航和总推力控制。

因此，这个实验没有尝试恢复正常悬停，也没有尝试继续执行任务。目标被限制为：

1. 正确识别被停止的电机；
2. 将该电机从控制分配中移除；
3. 立即切换到 Land；
4. 放弃绝对航向保持，允许机体绕偏航轴旋转；
5. 优先保留滚转、俯仰和下降控制，尝试降低撞击能量。

更准确地说，这是一种**三电机降级紧急着陆**，而不是“一台电机坏了仍能正常飞行”。

实验代码位于个人分支：

<https://github.com/amovlgf/PX4-Autopilot/tree/dev/power-redundancy-protection>

正式 24 组矩阵使用的固件版本为：

```text
f5bba53c2a53c0f7d7820341c19e67e6c6f92819
```

## 2. 修改后的故障处理链路

整体数据流如下：

![PX4 单电机故障处理链路](generated/plots/data_flow.png)

### 2.1 让注入的停止电机进入故障链路

PX4 的故障注入会产生 `motor_stop_mask`。实验修改后，只要
`fd_motor` 或 `motor_stop_mask` 非零，Commander 的 failsafe 标志就会报告电机
故障：

```cpp
fd_motor_failure = fd_status.fd_motor || fd_status.motor_stop_mask != 0;
```

这一步解决的是“仿真已经停止了某个电机，但 failsafe 和控制分配没有使用同一个
故障来源”的问题。

需要强调：正式矩阵使用的是软件故障注入。它验证了故障标志之后的处理链路，
没有验证实机 ESC 遥测、转速估计或电流异常能否可靠检测真实电机故障。

### 2.2 从控制分配中移除正确的电机

当 `CA_FAILURE_MODE=1` 时，Control Allocator 只接受第一个单电机故障。若
`motor_failure_mask` 不可用，则使用 `motor_stop_mask`。随后：

- 更新 `handled_motor_failure_mask`；
- 重新计算 effectiveness matrix；
- 将停止电机对应的 actuator setpoint 标记为不可分配；
- 在状态消息中发布实际处理的电机掩码。

矩阵分析会比较注入的 `motor_stop_mask` 与分配器的
`handled_motor_failure_mask`，防止出现“注入 Motor 1、实际移除 Motor 4”这类
看似进入保护、实际电机映射错误的问题。

### 2.3 四旋翼单电机故障立即进入 Land

实验条件为：

```text
CA_FAILURE_MODE=1
CA_ROTOR_COUNT=4
COM_ACT_FAIL_ACT=2
```

在这个组合下，电机故障动作被设为不可延迟，并允许通过模式切换接管。这样即使
`COM_FAIL_ACT_T` 设置为 5 s，四旋翼单电机故障仍会立即触发 Land，而不是继续
等待全局故障动作延迟。

这也是矩阵同时测试 `COM_FAIL_ACT_T=0 s` 和 `5 s` 的原因。

### 2.4 Land 任务释放航向保持

在四旋翼、Land、单电机故障三个条件同时成立时，自动飞行任务执行：

```cpp
_yaw_setpoint = NAN;
_yawspeed_setpoint = 0.f;
```

这里的含义不是让机体停止旋转，而是不再要求它跟踪一个不可实现的绝对航向。
三桨状态下机体仍可能高速自旋，但控制器可以把有限的控制能力更多地用于滚转、
俯仰和下降。

### 2.5 角速度控制进入降级模式

多旋翼角速度控制器在检测到单电机故障后：

- 使用当前偏航角速度作为偏航角速度目标；
- 清除偏航积分项；
- 保留原有偏航低通滤波；
- 将偏航力矩限制在 `±0.15`。

当前 `0.15` 是实验常量，不是经过多机型验证的参数。这是该实现不应直接进入
实机的一个重要原因。

## 3. 代码如何拆分

为了方便复现和审阅，个人分支将实验拆成了独立提交：

| 提交 | 内容 |
|---|---|
| `2b24c9f163` | 三电机降级迫降控制实现 |
| `4911da2819` | Control Allocation、Failsafe 和 MAVSDK 测试源码 |
| `64d471f428` | 矩阵执行、ULog 提取、校验和绘图工具 |
| `10b4c7be7e` | 使用标准 MAVLink 参数和命令协议 |
| `341fe61da9` | 同一 SITL 会话中的连续用例恢复支持 |
| `f5bba53c2a` | 每个矩阵用例使用独立可见 jMAVSim 会话 |

后续提交只补充文档和派生测试证据，不改变这次正式矩阵所使用的飞控代码。

## 4. 测试矩阵

矩阵覆盖以下组合：

- 故障电机：Motor 1、2、3、4；
- 起飞高度：2.5 m、10 m、20 m；
- `COM_FAIL_ACT_T`：0 s、5 s。

总计：

```text
4 motors × 3 heights × 2 delays = 24 cases
```

每个用例使用的关键参数：

```text
SYS_FAILURE_EN=1
FD_ACT_EN=0
MC_AIRMODE=1
CA_FAILURE_MODE=1
CA_ROTOR_COUNT=4
COM_ACT_FAIL_ACT=2
```

在注入故障前，自动执行器要求：

- 高度与目标值误差不超过 0.6 m；
- 三维速度不超过 1 m/s；
- 上述状态连续保持 2 s。

随后使用 `MAV_CMD_INJECT_FAILURE` 注入指定电机 `OFF`。

## 5. 为什么每个用例都重新启动 jMAVSim

早期测试曾在同一 jMAVSim 世界里连续运行多个用例。三桨迫降后，机体可能以较大
倾角停在地面。下一次测试会继承这个姿态，导致预检失败，或者让结果受到上一组
撞击状态影响。

最终矩阵使用 `run_isolated_matrix.py`：

1. 为每个用例打开新的可见 PX4 控制台和 jMAVSim GUI；
2. 等待 `Preflight check: OK`；
3. 只执行一组飞行；
4. 等待 ULog 完成写入；
5. 关闭该组 PX4/jMAVSim；
6. 保持 QGroundControl 运行并进入下一组。

这不是为了“让结果更好看”，而是为了让 24 个用例具有相同的仿真初始条件。

## 6. 判据

### 6.1 处理链路判据

以下条件用于确认代码链路是否正确：

- 注入电机与分配器处理电机的掩码完全一致；
- 故障电机输出确实停止；
- 分配器状态保持有限数值；
- 分配器处理响应不超过 0.5 s；
- Land 模式响应不超过 0.5 s；
- 24 个组合完整且 ULog 哈希唯一；
- ULog 中的固件 SHA 与预期版本一致。

### 6.2 探索性动态阈值

为了比较不同高度，不把“最后上锁”直接等同于“安全着陆”，实验另外定义：

- 最大绝对滚转角不超过 60°；
- 最大绝对俯仰角不超过 60°；
- 最大下沉速度不超过 3 m/s；
- 最大水平漂移不超过 5 m；
- ULog 中出现 landed 状态。

这些阈值只是本实验的数据分类线，不是 PX4 官方标准，也不是实机安全认证指标。

## 7. 结果

### 7.1 处理链路结果

24 份正式 ULog 的结果：

- 正确处理故障电机掩码：24/24；
- 分配器输出保持有限：24/24；
- 故障电机输出停止：24/24；
- 最大分配器处理响应：0.196 s；
- 最大 Land 响应：0.120 s；
- 严格矩阵校验：`errors: []`。

响应时间分布：

![故障处理和 Land 响应](generated/plots/response_delays.png)

从软件链路看，单电机停止能够快速到达 Control Allocator 和 Commander，并触发
预期的降级 Land 行为。`COM_FAIL_ACT_T=5 s` 没有把这一路径延迟 5 s。

### 7.2 动态阈值结果

| 起飞高度 | 用例数 | 阈值通过 | 自然上锁 | 强制上锁 |
|---|---:|---:|---:|---:|
| 2.5 m | 8 | 8 | 8 | 0 |
| 10 m | 8 | 0 | 8 | 0 |
| 20 m | 8 | 0 | 6 | 2 |
| **合计** | **24** | **8** | **22** | **2** |

矩阵热力图：

![三桨迫降矩阵](generated/plots/matrix_pass_heatmap.png)

这组结果说明：

- 2.5 m 的 8 个用例全部满足当前实验阈值；
- 10 m 的 8 个用例虽然都自然上锁，但至少违反一项动态阈值；
- 20 m 的 8 个用例全部违反动态阈值；
- Motor 4、20 m 的两个用例在 90 s 内都没有检测到触地，因此由测试程序强制上锁。

“自然上锁”只能说明 PX4 最终进入了上锁状态，不能证明撞击过程安全。

### 7.3 高度对动态响应的影响

![不同高度的动态响应](generated/plots/metrics_by_height.png)

全部矩阵观测到的极值：

- 最大下沉速度：47.980 m/s；
- 最大水平漂移：43.940 m；
- 最大绝对滚转角：179.935°；
- 最大绝对俯仰角：89.469°。

超过各动态阈值的用例数量：

- 下沉速度超过 3 m/s：16；
- 水平漂移超过 5 m：12；
- 绝对滚转角超过 60°：11；
- 绝对俯仰角超过 60°：11。

一个低空通过用例和一个中空失败用例的时序对比：

![代表性低空和中空响应](generated/plots/representative_comparison.png)

可以看到，三桨降级控制并没有创造新的控制自由度。高度增加后，机体有更多时间
积累偏航转速、姿态误差、水平速度和下沉速度，最终可能翻转或大范围漂移。

## 8. 如何复现

### 8.1 获取个人分支

```sh
git clone https://github.com/amovlgf/PX4-Autopilot.git
cd PX4-Autopilot
git fetch origin dev/power-redundancy-protection
git switch --track -c dev/power-redundancy-protection \
  origin/dev/power-redundancy-protection
git submodule update --init --recursive
```

若要与本文已提交的 CSV/JSON 完全对比，请检出正式测试固件：

```sh
git checkout f5bba53c2a53c0f7d7820341c19e67e6c6f92819
```

### 8.2 运行完整矩阵

需要 Linux 桌面环境、`gnome-terminal`、jMAVSim 依赖和 Python
`pymavlink`。QGroundControl 可以保持连接用于观察。

```sh
python3 Tools/simulation/single_motor_failure_experiment/run_isolated_matrix.py \
  --fail-fast
```

如果执行中断，可以从已完成的前缀继续：

```sh
python3 Tools/simulation/single_motor_failure_experiment/run_isolated_matrix.py \
  --resume --fail-fast
```

### 8.3 只运行一个代表性用例

先启动可见 jMAVSim：

```sh
make px4_sitl_default jmavsim
```

然后从另一个终端执行：

```sh
python3 Tools/simulation/single_motor_failure_experiment/run_matrix.py \
  --motors 1 --heights 2.5 --delays 0 --fail-fast
```

`--force-arm` 仅用于研究 SITL 连续用例恢复，不应出现在实机操作流程中。

### 8.4 生成分析结果

```sh
bash Tools/simulation/single_motor_failure_experiment/run_analysis.sh

python3 Tools/simulation/single_motor_failure_experiment/validate_matrix.py \
  Tools/simulation/single_motor_failure_experiment/generated/matrix/results.json \
  --expected-sha "$(git rev-parse HEAD)"
```

已提交的派生证据包括：

- `generated/matrix/results.csv`；
- `generated/matrix/results.json`；
- `generated/matrix/run_summary.json`；
- `generated/matrix/logs_manifest.csv`；
- `generated/plots/` 下的矩阵图和时序图。

原始 `.ulg` 没有提交到 Git。复现者可以生成自己的日志，并用 manifest 中的字段
检查文件大小、SHA-256、固件版本和矩阵身份。

## 9. 当前限制

1. **只验证了 jMAVSim SITL。**
   没有验证电调饱和、电机惯量、机臂弹性、结构冲击、电池压降和真实空气动力学。

2. **没有验证真实故障检测。**
   实验通过 MAVLink 注入停止电机，重点验证故障标志之后的处理链路。

3. **偏航力矩上限是硬编码实验值。**
   `0.15` 没有跨机型调参依据。

4. **只处理第一个单电机故障。**
   多电机故障不属于当前实现范围。

5. **高空结果不可接受。**
   10 m 和 20 m 均没有用例满足全部动态阈值。

6. **已有测试源码尚未完整执行。**
   本地缺少 `test/fuzztest` 子模块和 MAVSDK C++ 3.11.2，因此新增的
   `failsafe_test` 与 MAVSDK 目标仍存在验证缺口。

7. **适用范围不是 Flycore 专用。**
   代码位于通用 Commander、Flight Mode Manager、Rate Control 和 Control
   Allocator 路径。任何启用相同参数组合的四旋翼都可能进入该实验逻辑。

## 10. 我希望讨论的问题

我更希望把这份实验当成一个讨论起点，而不是一个完成的功能。特别想听取社区对
以下问题的意见：

1. 单电机失效时，是否应该完全释放航向目标，还是使用专门的自旋坐标系控制器？
2. 降级偏航力矩上限应该参数化，还是根据剩余控制裕度在线计算？
3. 对四旋翼来说，触发 Land 是否合理，还是应该引入独立的受控坠落模式？
4. 哪些 Gazebo 或真实电机模型更适合继续验证这种高速自旋状态？
5. 在考虑实机之前，最小的故障检测、HITL 和安全笼测试集合应该是什么？

欢迎复现实验、检查提交、使用自己的 ULog 重跑分析，或者提出更合适的控制方案。

---

代码分支：
<https://github.com/amovlgf/PX4-Autopilot/tree/dev/power-redundancy-protection>

测试环境、完整命令和已知阻塞项位于同目录的 `README.md`、`environment.txt` 和
`validation.md`。
