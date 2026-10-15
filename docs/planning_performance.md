# 装配轨迹规划与执行提速记录（2026-10-15）

## 结论与时间口径

本次最大收益是减少 IK 分支切换和关节绕行，缩短生成的轨迹及其执行时间。
接近阶段的轨迹仿真时长缩短约 **21.4 倍**，执行墙钟时间缩短约 **24.3 倍**。
规划求解原本只有几十毫秒，不能将上述收益表述为“规划算法快了 24 倍”。

| 日志字段 | 含义 |
|---|---|
| `PLAN planner_time` | MoveIt 返回的规划耗时，不含任务节点此前单独调用的 IK 服务 |
| `PLAN wall_time` | 围绕 `group.plan()` 测得的墙钟时间，包括调用开销，不含此前 IK |
| `PLAN trajectory_sim_time` | 轨迹末点的 `time_from_start`，表示预计执行的仿真时间 |
| `EXECUTE wall_time` | 执行及结束后反馈/有效性检查的墙钟耗时 |
| `EXECUTE sim_time` | 上述执行区间内 ROS 仿真时钟的增量 |
| `EXECUTE real_time_factor` | `sim_time / wall_time`；越低，仿真相对现实时间越慢 |
| `STATE=... OK wall_time` | 整个阶段耗时，包含该阶段的检查、IK、规划、执行与验证 |

## 核心改动

### 1. 先求接近当前姿态的 IK，再规划到关节目标

旧流程：`setPoseTarget()` → MoveIt 姿态约束采样及规划 → 执行。
七自由度手臂对同一末端位姿可能有多个关节解；短距离末端移动也可能选到绕行很大的解。
旧预抓取轨迹中，`arm_lj3` 从零位转到了约 −4.790 rad。

新流程在 `assembly_task.cpp` 的 `pose_target()` / `solve_pose()` 中实现：

1. 从 MoveIt 读取实际关节反馈，作为 IK 种子。
2. 调用现有 `/compute_ik`，指定原单臂组、TCP、目标位姿及 `avoid_collisions=true`。
3. 对每个机械臂旋转关节，检查 `q + 2πk`（`k=-2…2`），选择关节限位内最接近种子的等价角。
4. 检查目标状态有效性，使用 `setJointValueTarget()` → `plan()` → `execute()`。

例如 −4.790 rad 与约 1.493 rad 的 FK 等价；从零位出发，后者转动更少。
这是在有限候选中选最近的等价角，不保证 IK 解或整条路径为全局最优。
双臂搬运使用同样的有种子 IK，再由双臂组统一规划。

### 2. 双臂只规划 14 个机械臂关节

SRDF 新增 `jaka_kargo_dual_arms`，仅组合原左右臂子组。
原先用 21 轴全身组加非手臂轴 ±0.001 路径约束；躯干轻微物理偏移就可能使 HOME 起点失效，
触发修正段并导致整条路径被拒绝。新组将底盘、躯干排除在轨迹外，由唯一 JTC 保持。

这一改动修复了返回可靠性并减少规划变量；未单独测量它对求解耗时的贡献。

### 3. 分阶段提速，并降低渲染开销

- 常规速度/加速度比例由 0.1/0.1 提高到 0.3/0.3。
- APPROACH、PLACE 仍使用 0.1/0.1；它们的明显提速主要来自减少绕行。
- 物理步长保持 0.005 s，渲染步长从 0.005 s 改为 0.02 s，每帧推进四个物理步。
- 渲染次数减少不等于墙钟速度固定提高四倍；仍受 GPU、场景负载及反馈处理影响。

## 前后实测

下表比较的是同场景两轮运行中，显式有种子 IK 优化前后对应阶段。
两轮都已采用常规 0.3、精细 0.1 比例及 0.02 s 渲染步长；不能把表中收益再次归因于这些参数变化。

| 阶段与指标 | 优化前 | 优化后 |
|---|---:|---:|
| MOVE_TO_PREGRASP：MoveIt 规划耗时 | 0.029 s | 0.018 s |
| MOVE_TO_PREGRASP：轨迹仿真时长 | 13.939 s | 4.592 s |
| MOVE_TO_PREGRASP：执行墙钟耗时 | 41.307 s | 11.504 s |
| APPROACH：MoveIt 规划耗时 | 0.057 s | 0.038 s |
| APPROACH：轨迹仿真时长 | 38.478 s | 1.798 s |
| APPROACH：执行墙钟耗时 | 115.308 s | 4.754 s |
| APPROACH：轨迹点数 | 386 | 19 |

优化后 APPROACH 的 IK 解最大单关节变化为约 0.380 rad。
最终完整流程成功，总任务墙钟约 67.536 s，零件最终位置误差约 3.115 mm。
本轮长动作实时倍率约 0.317–0.402，仍未达到实时。

数据来自本机 `/tmp/kargo-physical-assembly/long-ik.log` 和 `assembly.log`；
这些临时文件可能被清理，关键结果已保存在本文与[物理验证记录](physical_validation_2026-10-15.md)。
两轮 IK 分支和中间关节姿态不同，未做多次统计或逐项消融，不能据此承诺固定提速倍率。

## 代码位置与复测

| 文件 | 对应实现 |
|---|---|
| [assembly_task.cpp](../src/jaka_kargo_planner/src/assembly_task.cpp) | `solve_pose()`、`pose_target()`、`dual_pose()`、`execute()` 及计时日志 |
| [jaka_kargo.srdf](../src/jaka_kargo_moveit_config/config/jaka_kargo.srdf) | 14 轴双臂组合组 |
| [assembly_scene.json](../src/jaka_kargo_isaacsim/config/assembly_scene.json) | 常规/精细比例、物理及渲染步长 |
| [isaacsim_moveit.py](../src/jaka_kargo_isaacsim/scripts/isaacsim_moveit.py) | SimulationContext 的物理/渲染时序 |
| [assembly_task.launch.py](../src/jaka_kargo_planner/launch/assembly_task.launch.py) | 常规速度与加速度比例参数 |

从重置后的 A 区场景启动 Isaac 和 ROS 控制栈，再执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch jaka_kargo_planner assembly_task.launch.py mode:=assembly \
  velocity_scaling:=0.3 acceleration_scaling:=0.3
```

按阶段比较 IK 最大关节变化、轨迹仿真时长、执行墙钟时间和实时倍率；将 IK 调用与
`group.plan()` 分开统计。重复测试须重置相同起点并记录多轮分布。
`setPlanningTime(15.0)` 和规划尝试次数 5 没有调小；`test_hold_seconds` 不影响装配速度。
自碰撞、环境碰撞、附着物体、关节限位及执行反馈检查均保留，执行完整轨迹 action。
