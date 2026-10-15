# 第一阶段：现有工程分析

## 模型与规划组

`src/jaka_kargo_description/urdf/jaka_kargo.urdf` 是模型源；MoveIt 的
`config/jaka_kargo.urdf.xacro` 包含它及 ros2_control 接口。左右臂分别为
`arm_lj1` 至 `arm_lj7`、`arm_rj1` 至 `arm_rj7`，均为旋转关节。
`body_j1` 为直线轴，`body_j2…4` 为旋转轴；`agv_x/y` 为直线轴、
`agv_yaw` 为旋转轴，共 21 个受控轴。轮子不属于装配规划组。
ROS 旋转量为 rad，直线量为 m；USD 旋转限位使用 degree。

原 SRDF 定义 `jaka_kargo_arm_l/r`（`body_j2_link` 到 `arm_l/r_tcp` 的链）、
`jaka_kargo_body`、`jaka_kargo_agv`、`jaka_kargo_full_robot`。
仅两条单臂链配置了 KDL IK；全身组没有单一 IK 求解器。
实现后补充 `jaka_kargo_dual_arms`，仅组合现有左右臂子组，共 14 轴。
双臂协调分别求单臂 IK，再用该组联合规划；底盘和躯干不进入轨迹。
初版全身组加 ±0.001 路径约束在 PhysX 躯干轻微偏移后导致 HOME 失败，已替换。
SRDF 有 `zero` 命名姿态和自碰撞排除表；这些排除项仍需实机模型审查。

URDF 两个 TCP 都是第七轴连杆沿局部 Z 偏移 0.2 m 的固定虚拟坐标系，
没有夹爪关节、夹爪控制器或工具碰撞几何。SRDF 的 `left_ee/right_ee`
引用整条手臂，不能据此认定存在夹爪。MVP 使用明确标注的固定约束抓取。

## 已有通信链路

`jaka_kargo_planner/src/kargo_moveit_server.cpp` 提供五个
`FollowJointTrajectory` action，直接登录 JAKA SDK，手臂通过 EDG servo
执行，AGV 通过 HTTP 和 odom 通信，并发布 `/kargo_joint_states`。
它不是提供 IK/规划的服务，仿真不应启动它。
`jaka_kargo_driver` 同样连接 SDK，提供 ROS 服务、关节反馈和 AGV 桥。

仿真已有可复用链路：MoveIt → `JointTrajectoryController` →
`topic_based_ros2_control/TopicBasedSystem` → `/isaac_joint_commands`
→ USD OmniGraph articulation controller → `/isaac_joint_states`
→ joint state broadcaster → `/joint_states` → MoveIt。
`isaacsim_moveit.py` 打开现有 USD 并连接状态和 `/clock` 的仿真时间。

## USD 静态检查

现有 `jaka_kargo_moveit.usd` 使用 m、Z-up；机器人位于
`/World/jaka_kargo`，固定根关节 `/World/jaka_kargo/root_joint`
具有 `PhysicsArticulationRootAPI`。21 个轴的名称、类型及换算后限位与
URDF 一致。USD 还包含轮子与历史仿真状态。
OmniGraph 发布器原目标是 `world` 刚体，控制器目标是机器人上级路径；
装配入口会显式指向实际 articulation root。
原地面依赖外部网络资产；装配入口创建本地地面，所有改动仅写 session layer。

## 缺失项与修改范围

当前 demo 调用系统 `moveit_configs_utils`；本机系统文件被定制过，仓库根的
`launches.py` 是副本，不能保证干净环境可复现。新增仓库内仿真 launch，
统一 `use_sim_time`、只启用一个全身轨迹控制器，避免分组控制器资源竞争。

- `moveit_config`：独立装配仿真入口和单一控制器配置；复用原 URDF，SRDF 补充双臂组合组。
- `isaacsim`：共享场景配置、静态模型审计、物理场景与固定约束抓取、零件反馈。
- `planner`：无 SDK 链接的 MoveGroup 单臂/双臂测试与装配状态机、碰撞场景同步。
- `README.md` / `docs/`：启动、验收、局限和排障。

不改动用户已有 USD 文件，不新增机器人模型或 ROS 自定义接口。
本环境存在 ROS 2 Humble、Isaac Sim 4.5。初次沙箱检查无法访问 NVIDIA 驱动；
随后在授权的桌面环境中已完成单臂、双臂运动，以及一轮固定约束抓取/搬运/释放装配和返回验证。
见 [物理验证记录](physical_validation_2026-10-15.md)。


后续验证更新：经用户授权在沙箱外实际启动 Isaac 4.5 和 RViz，已定位并修正底盘单凸包
引起的右臂初始穿透，真实单臂轨迹/返回测试通过。首次沙箱内的驱动访问限制不代表宿主机
无法运行仿真。详见 [物理验证记录](physical_validation_2026-10-15.md)。
