# 2026-10-15 起始碰撞修复与物理验证

## 原因与证据

- 原报错发生在起始碰撞检查，轨迹尚未下发：`arm_lj5_link/workbench`。
- 只读采集：`arm_lj1=0.9671 rad`、`body_j2=0.2874 rad`；控制器期望也保持在类似偏移姿态。
- 该实测状态的 URDF FK 与 Isaac TCP 相差不到 0.1 mm；第五连杆网格最低约 0.2989 m，确实低于 0.3000 m 桌面。
- 暂不激活轨迹控制器、保持物理驱动零目标时，机器人仍发生偏移。接触报告记录零位附近 `arm_rj2_link/base_link` 穿透约 0.0430 m。
- 底盘使用单一凸包，CAD 凹槽被填平，产生额外接触力；躯干随之偏转，左臂碰桌，过早激活的控制器又保持了偏移姿态。

## 修正

1. 在 USD session layer 将现有底盘碰撞体改为凸分解（64 个凸包上限、每包 64 顶点、体素分辨率 1000000），不保存源 USD。
2. 机器人碰撞体 contact offset 设置为 0.002 m、rest offset 为 0；保留右臂/底盘碰撞对及 MoveIt 环境碰撞检查。
3. 物理驱动增益在 Play 前写入 session layer，保留原力矩上限。
4. 仅在稳定零位反馈通过后激活轨迹控制器；超时终止启动。
5. 增加起始关节、碰撞深度和可选 PhysX 接触/驱动日志；修正 ROS SIGINT 后 Isaac 循环退出条件。

## 实际执行结果

经用户授权重启 Isaac、ROS 控制栈及 RViz，再运行：

```bash
ros2 launch jaka_kargo_planner assembly_task.launch.py mode:=single
```

观察到启动稳定性检查通过，之后控制器才激活；初始 `body_j2` 约 0.0013 rad。
接触报告中未再出现原右臂/底盘穿透。

| 阶段                 |  规划时间  | 执行墙钟时间 | 最大关节目标误差 |         结果           |
| 左腕第七轴 +0.08 rad  | 0.0065 s | 3.406 s     | 0.003956 rad  | 碰撞检查 valid，执行成功 |
| 返回 HOME            | 0.0186 s | 3.528 s     | 0.003289 rad  | 碰撞检查 valid，执行成功 |

任务最终输出 `STATE=SUCCESS mode=single`，进程正常退出。
运行日志保存在本机 `/tmp/kargo-physical-startup/single.log`，Isaac/控制器日志在同目录。
此轮只记录了终点误差，未记录运动中的位移；第七轴绕工具轴小角度自转，TCP 不平移，
不适合作为可见运动的验收。没有启动硬件 SDK。

## 可见单臂运动复验

用户反馈画面中没有看到运动后，检查到 Isaac 和 RViz 已退出，仅残留控制进程。
按此前授权重启整个仿真栈，改用左肘 `arm_lj4` 偏移 −0.35 rad，停留后返回。
规划及完整轨迹仍经 MoveIt 碰撞检查和原 JTC action 执行。

```bash
python3 src/jaka_kargo_isaacsim/test/run_physical_motion.py --execute --capture-desktop
```

独立订阅记录本次真实运行（非回环）：

| 数据 | 结果 |
|---|---|
| `/isaac_joint_commands` 左肘范围 | 0.350004 rad，469 条 |
| `/isaac_joint_states` 左肘范围 | 0.350000 rad，1313 条 |
| `/joint_states` 左肘范围 | 0.350000 rad，589 条 |
| Isaac 物理 TCP 最大平移 | 0.150663 m |
| 返回后左肘相对初始误差 | −0.000100 rad |
| 非左臂关节最大偏移 | 0.001400 rad/m |
| 去程/返回规划耗时 | 0.0168 / 约 0.017 s |
| 去程/返回执行墙钟耗时 | 11.161 / 11.353 s |
| 去程/返回最大关节目标误差 | 0.003613 / 0.003892 rad |

两段运动结束的碰撞检查均 valid，任务 `SUCCESS`，独立观察器
`PHYSICAL_MOTION_FEEDBACK_PASSED`。停留截图中可见肘部姿态变化。
日志、连续数据及前/停留/后截图位于 `/tmp/kargo-physical-motion/`
（`single.log`、`summary.json`、`trace.json`、`before.png`、`hold.png`、`after.png`）。
仿真与 RViz 保持运行供检查；本测试默认观察停留 5 秒，独立观察器使用 8 秒。
修改后 C++ 构建、Python 语法检查及 `git diff --check` 通过。
补充处理任务刚启动时发布者尚未被 DDS 发现的情况：只对首次检查中数量为零的端点等待
最多 5 秒，多发布者仍立即拒绝。之后隔离域软件回环的单臂、双臂及六个位姿 IK 均通过，
日志位于 `/tmp/kargo-visible-loopback-final/`。

## 双臂 HOME 规划失败修复及复验

用户的双臂去程成功，停留后返回规划报 `code=-2`。对应 MoveIt 日志显示
`body_j2` 起始约束中心为 0.002600 rad，返回前实测为 0.004000 rad，超出 ±0.001。
`FixStartStatePathConstraints` 尝试生成修正段，随后完整轨迹出现约束违规及桌面碰撞，
被规划流水线拒绝；该失败段未下发，现场两肘仍停在约 −0.35 rad。

SRDF 新增 `jaka_kargo_dual_arms`，仅引用原左右臂子组，共 14 轴。
双臂测试和装配的联合动作改用该组，去掉全身组的非手臂路径约束；
执行前检查轨迹中没有底盘/躯干轴。唯一 21 轴 JTC 继续接收部分关节目标，
全机器人有效性、环境及自碰撞检查保持启用。

编译并重启仿真栈加载 SRDF 后，独立运行：

```bash
python3 src/jaka_kargo_isaacsim/test/run_physical_motion.py --execute --mode dual --log-dir /tmp/kargo-physical-dual --capture-desktop
```

| 数据 | 结果 |
|---|---|
| 去程/返回规划组 | 均为 `jaka_kargo_dual_arms` |
| 去程/返回规划时间 | 约 0.022 / 0.031 s |
| 去程/返回执行墙钟时间 | 12.036 / 12.016 s |
| 去程/返回最大关节目标误差 | 0.003575 / 0.003934 rad |
| Isaac 左/右肘实际变化范围 | 0.3500 / 0.3501 rad |
| 最终左/右肘相对初始误差 | 0.0000 / −0.0001 rad |
| Isaac 左 TCP 最大平移 | 0.150644 m |
| 躯干及 AGV 最大实测偏移 | 0.0015 rad/m |

碰撞检查均 valid，任务 `STATE=SUCCESS mode=dual`，独立记录器
`PHYSICAL_MOTION_FEEDBACK_PASSED`。日志、采样及截图位于 `/tmp/kargo-physical-dual/`。
配置契约测试 5 项通过；隔离域软件回环注入去程后 `body_j2` 偏移 0.0014 rad，
单臂、双臂返回以及六个装配 IK 检查全部通过，日志在 `/tmp/kargo-dual-drift-loopback/`。

## GRASP 场景更新与速度修复，完整装配验证

原失败日志在 GRASP：Isaac 已报告 `Fixed joint created`，MoveIt 随后报告
`Tried to remove world object 'part_a', but it does not exist in this scene`。
附着 ADD 自动移除世界物体，而同一个 diff 又显式 REMOVE，导致返回失败。
修复去掉重复 REMOVE；附着和释放后通过 GetPlanningScene 检查唯一容器归属。
状态/IK 请求设置 `robot_state.is_diff=true`，保留场景已有的附着碰撞体。
Isaac 增加 `/assembly/grasped`，反馈固定约束 prim 的创建/删除。

常规速度与加速度比例从 0.1/0.1 提到 0.3/0.3，接近/放置保留 0.1/0.1；
任务 launch 支持覆盖常规比例。物理步长仍为 0.005 s，渲染步长改为 0.02 s。
首次物理复验已通过 GRASP，但接近动作出现大幅 IK 绕行（38.478 s 仿真轨迹）；
主动中止该轮，改用实测状态种子、碰撞感知 IK、限位内最近等价角，再交给 MoveIt 规划。

重置仿真后按默认参数运行 `mode:=assembly`，独立记录关节、零件位姿及 grasped 状态：

| 状态 | 实际墙钟耗时 | 结果 |
|---|---|---|
| INIT | 0.072 s | 有效起点 |
| MOVE_TO_PREGRASP | 11.532 s | 目标位姿误差通过 |
| APPROACH | 4.803 s | 轨迹约 1.798 s 仿真时间，精细速度 |
| GRASP | 0.146 s | 固定约束创建；attached=true、world=false |
| LIFT | 4.485 s | 实际零件离台 |
| TRANSFER | 13.302 s | 双臂 14 轴轨迹执行成功 |
| PLACE | 9.193 s | 精细速度放置 |
| RELEASE | 3.022 s | 约束解除；attached=false、world=true |
| RETREAT | 4.742 s | 撤离完成 |
| HOME | 16.238 s | 双臂返回 |

任务 `STATE=SUCCESS mode=assembly`，进程退出 0，总任务墙钟约 67.536 s。
独立物理观察器输出 `PHYSICAL_ASSEMBLY_VERIFIED`：

- 零件初始中心约 `[0.520000, 0.450000, 0.330000]` m，过程中最高 Z 为 0.546036 m。
- 最终中心 `[0.520126, 0.050018, 0.373113]` m，相对目标误差 0.003115 m。
- 最终姿态误差约 0.000977 rad；双臂回位后最大初始关节偏差 0.0004 rad。
- 固定约束反馈经历 false → true → false；记录 1279 条零件位姿样本。
- 所有运动结束的状态碰撞检查 valid；长动作实时倍率约 0.317–0.402，未达到实时。

日志、独立零件轨迹、汇总及截图位于 `/tmp/kargo-physical-assembly/`
（`assembly.log`、`summary.json`、`part_trace.json`、`carrying.png`）。
软件回环的单臂、双臂、躯干偏移返回、六个 IK 和实际 MoveIt 附着/释放场景切换均通过，
日志在 `/tmp/kargo-grasp-loopback-final/`。配置测试 6 项、USD 静态测试 5 项通过；
C++ 构建和 Python 语法检查通过。仿真保留任务完成状态，零件位于 B 区；重复演示需重置。

## 验证边界

本次证明单/双臂运动闭环及一轮完整固定约束装配流程可运行；这是单次成功结果，
尚未测量重复成功率。未实现真实手指抓持、双臂共同夹持、接触力装配或真实硬件验证。
