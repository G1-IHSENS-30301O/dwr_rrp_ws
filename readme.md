# DWR + RRP 复合机器人仿真项目

## 1. 项目简介

本项目在 **ROS Noetic + Gazebo** 中构建了一个由**差速轮式底盘 (DWR)** 和 **3 自由度 RRP 机械臂**组成的复合移动机器人。  
机器人能够在室内环境中自主导航，通过逆运动学控制机械臂抓取桌面上的物体，并将其运送到指定位置。

### 主要功能
- 差速驱动底盘控制（误差反馈律 + Dijkstra 全局路径规划）
- RRP 机械臂逆运动学（`θ1, θ2, d3`）与正运动学
- 基于 Gazebo 力矩服务的关节 PID 位置控制
- 夹爪开合控制及重力补偿
- 按优先级顺序执行多项抓取‑放置任务
- 带有桌椅、墙壁障碍物的仿真环境
- 底盘与机械臂协同工作（旅行姿态保持、顺序抓取）

---

## 2. 环境依赖

- **Ubuntu 20.04**
- **ROS Noetic** (完整版)
- **Gazebo 11**
- 必要的 ROS 功能包：
  - `rospy`
  - `geometry_msgs`, `nav_msgs`, `sensor_msgs`
  - `tf`
  - `gazebo_ros`, `gazebo_ros_control`, `gazebo_msgs`
  - `joint_state_controller`, `effort_controllers`, `position_controllers`
  - `controller_manager`

如果缺少某些包，可以用以下命令安装：
```bash
sudo apt update
sudo apt install ros-noetic-ros-control ros-noetic-ros-controllers ros-noetic-gazebo-ros-control ros-noetic-joint-state-controller ros-noetic-effort-controllers ros-noetic-position-controllers
```

---

## 3. 项目结构

```
dwr_rrp_ws/
├── src/
│   └── dwr_rrp_control/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   └── arm_control.yaml          # 机械臂控制器配置（已不使用）
│       ├── launch/
│       │   └── sim.launch                # 仿真启动文件
│       ├── models/
│       │   └── dwr_rrp_robot.sdf         # 机器人模型 (DWR + RRP)
│       ├── scripts/
│       │   ├── main.py                   # 控制脚本入口
│       │   └── dwr_rrp_control_lib/
│       │       ├── __init__.py
│       │       ├── controller.py         # 主控逻辑 (底盘、机械臂、任务调度)
│       │       ├── joint_controller.py   # 关节 PID 控制器
│       │       ├── dwr_kinematics.py     # 差速底盘运动学
│       │       ├── rrp_kinematics.py     # RRP 机械臂运动学 (逆解、正解)
│       │       ├── trajectory_planner.py # 全局路径规划 (Dijkstra)
│       │       └── params.py             # 参数定义 (尺寸、任务、障碍物)
│       └── worlds/
│           └── house.world               # Gazebo 世界文件（环境）
```

---

## 4. 编译与运行

### 4.1 创建工作空间并编译

```bash
cd ~
mkdir -p dwr_rrp_ws/src
cd dwr_rrp_ws/src
# 将本项目代码放入 src/dwr_rrp_control 中
```
```bash
cd ~/dwr_rrp_ws
catkin_make
source devel/setup.bash
```

### 4.2 启动 Gazebo 仿真环境

```bash
# 若虚拟机内存不足，建议使用无 GUI 模式
export LIBGL_ALWAYS_SOFTWARE=1
roslaunch dwr_rrp_control sim.launch gui:=false
```

等待终端出现 `[Msg] Connected to gazebo master` 且差速驱动插件加载成功。

### 4.3 运行控制脚本

打开新终端，输入：

```bash
source ~/dwr_rrp_ws/devel/setup.bash
rosrun dwr_rrp_control main.py
```

机器人将自动开始依次执行取放任务。

---

## 5. 仿真环境说明

`worlds/house.world` 定义了一个 **20 m × 15 m** 的室内场景，包含：
- 外围墙壁
- 内部隔墙（形成不同房间，门洞宽度 ≥ 1.5 m）
- 靠墙放置的三张桌子（分别放置药品、水杯、电话）
- 一张目标桌子（接收所有物体）
- 中央障碍桌（纯障碍物）
- 靠墙组合床和两个立柱障碍物
- 一个助行器（直接放在地面）

所有物体高度均被调整至 **1.45 m**（与机械臂第二关节大致等高），确保逆运动学有解。

---

## 6. 机器人模型概要

### 6.1 差速底盘
- 尺寸：0.7 m × 0.7 m × 0.1 m
- 左右驱动轮（半径 0.2 m，间距 0.8 m）
- 通过 `libgazebo_ros_diff_drive.so` 插件实现里程计和速度控制
- 控制接口：`/cmd_vel`（`geometry_msgs/Twist`）

### 6.2 RRP 机械臂
- **Joint 1** – 旋转关节（θ1），垂直轴，范围 [-π, π]
- **Joint 2** – 旋转关节（θ2），水平轴，范围 [-π/2, π/2]
- **Joint 3** – 移动关节（d3），沿竖直方向伸缩，范围 [-0.4, 0.1] m
- 末端执行器带有简易两指夹爪，开合范围 0 ~ 0.05 m

运动学参数（`params.py`）：
- `d1 = 1.15`  基座到 joint2 的高度
- `a2 = 0.8`   link2 水平长度
- `OFFSET_EE = 0.51`  末端执行器固定偏移
- 关节实际指令值 `d3_cmd ∈ [-0.4, 0.1]`

---

## 7. 控制策略

### 7.1 底盘运动
- **全局规划**：基于栅格地图 (0.2 m/格) 的 Dijkstra 算法，生成从当前位置到目标点的安全路径。
- **轨迹跟踪**：使用误差反馈控制律，计算线速度与角速度，并通过前视距离实现平滑追踪。
- 行驶过程中机械臂自动保持“旅行姿态”（`θ1=90°, θ2=0, d3=0`），避免碰撞。

### 7.2 机械臂操作
- **抓取流程**：
  1. 底盘移动到物体前方约 1 m 处。
  2. 进入“旅行姿态”，并利用 PID 闭环保持。
  3. 通过底盘微调使物体进入机械臂工作空间（距离 0.82～0.88 m，角度偏差 < 0.1 rad）。
  4. **顺序抓取**：
     - 先控制伸缩关节 d3 到位
     - 再调整关节 θ2 使末端与物体等高
     - 最后旋转关节 θ1 使夹爪对准物体方向
  5. 夹爪闭合并保持恒定力矩，举升物体。
  6. 运送至目标地点，重复上述放置流程。
- **重力补偿**：根据动力学逆解施加力矩，辅助举起重物。

---

## 8. 参数调整

所有可调参数集中在 `scripts/dwr_rrp_control_lib/params.py` 中，包括：
- 机械臂几何尺寸 (`d1`, `a2`, `OFFSET_EE`)
- 任务目标坐标 (`TASKS`)
- 地图栅格分辨率 (`MAP_RES`)、世界尺寸 (`MAP_SIZE_X`, `MAP_SIZE_Y`)
- 障碍物矩形列表 (`WALLS`)
- 最高速度、前视距离等控制参数

修改后无需重新编译，直接重新运行 `main.py` 即可生效。

---

## 9. 常见问题

**Q: 仿真启动后 Gazebo 窗口闪退？**  
A: 虚拟机内存不足导致。请使用 `gui:=false` 无界面模式，或为虚拟机分配更多内存（建议 ≥ 4 GB）。

**Q: 机械臂无法抓取物体？**  
A: 确保物体高度在 1.4 m 以上，且底盘已对准至 0.85 m 左右。检查 SDF 模型中 joint3 的限位是否为 `[-0.4, 0.1]`。

**Q: 全局路径规划失败？**  
A: 膨胀系数过大或门洞过窄。可在 `params.py` 中减小 `CELLS` 膨胀格数，或增大门洞宽度。

**Q: 关节运动超时或乱晃？**  
A: PID 增益过高或目标命令超出限位。调整对应关节的 `kp, ki, kd`，并确认 `d3_cmd` 被裁剪在 `[-0.4, 0.1]`。

---

## 10. 作者与许可

本项目为机器人学课程项目，仅供学习与研究使用。

**最后更新**：2026 年 7 月