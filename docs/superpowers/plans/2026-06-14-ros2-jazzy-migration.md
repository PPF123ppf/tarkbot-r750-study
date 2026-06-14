# ROS2 Jazzy 迁移实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: superpowers:subagent-driven-development 或 superpowers:executing-plans。步骤用 `- [ ]` 跟踪。

**Goal:** 修复厂商 ROS2 驱动包让 Jazzy 能 build/run，然后整体翻新 12 周学习计划为 ROS2 Jazzy + Nav2 版。

**Architecture:** 先在 `~/ros2_ws/src/tarkbot_robot` 标准 colcon workspace 修包（避免在中文路径下 build），用最小改动让 jazzy build 通过、launch 起得来；再基于这次修包验证后的真实命令，整体翻新学习计划。

**Tech Stack:** ROS2 Jazzy / colcon / Boost.Asio / Nav2 / slam_toolbox / Gazebo Harmonic / Markdown.

---

## Phase 1: 修包（Day 1–2）

### Task 1: 创建 colcon workspace 并复制驱动包

**Files:**
- Create: `~/ros2_ws/src/tarkbot_robot/`（从中文路径复制）

- [ ] **Step 1: 创建 workspace 目录**

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

- [ ] **Step 2: 复制（不要 ln -s，因 colcon 对软链不友好）驱动包**

```bash
cp -r "/home/ppf/workspace/全向移动自主作业机器人/塔克创新智能车资料/4 ROS功能包源码及教程资料/ROS2源码资料/ROS2驱动功能包源码/tarkbot_robot" \
      ~/ros2_ws/src/tarkbot_robot
```

- [ ] **Step 3: 验证目录结构**

Run: `ls ~/ros2_ws/src/tarkbot_robot/`
Expected: 看到 `CMakeLists.txt config include launch package.xml src`

- [ ] **Step 4: 第一次 build（预期失败，作为基线）**

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tarkbot_robot 2>&1 | tee /tmp/build1.log
```
Expected: 失败。常见错误：`fatal error: tf2/LinearMath/Quaternion.h: No such file` 或 `rclcpp not found`。把 build1.log 留着对比修复后的 build。

### Task 2: 修 package.xml 补缺失依赖

**Files:**
- Modify: `~/ros2_ws/src/tarkbot_robot/package.xml`

- [ ] **Step 1: 读取当前 package.xml**

Run: `cat ~/ros2_ws/src/tarkbot_robot/package.xml`
确认现有依赖只有 std_msgs / sensor_msgs / geometry_msgs / nav_msgs / tf2_msgs / tf2_ros，缺 rclcpp / tf2 / tf2_geometry_msgs。

- [ ] **Step 2: 在 `</package>` 前补三行依赖**

把文件中

```xml
  <depend>tf2_msgs</depend>
  <depend>tf2_ros</depend>
  <exec_depend>ros2launch</exec_depend>
```

改成

```xml
  <depend>rclcpp</depend>
  <depend>tf2</depend>
  <depend>tf2_msgs</depend>
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>
  <exec_depend>ros2launch</exec_depend>
```

- [ ] **Step 3: 验证 XML 格式**

Run: `xmllint --noout ~/ros2_ws/src/tarkbot_robot/package.xml && echo OK`
Expected: 输出 `OK`，无 XML 报错。

- [ ] **Step 4: 不 commit（项目不在 git 仓库内），记到变更日志**

```bash
echo "$(date +%F): package.xml 补 rclcpp / tf2 / tf2_geometry_msgs" >> ~/ros2_ws/src/tarkbot_robot/CHANGELOG-jazzy-fix.md
```

### Task 3: 修 CMakeLists.txt（补 rclcpp + 升 cmake）

**Files:**
- Modify: `~/ros2_ws/src/tarkbot_robot/CMakeLists.txt`

- [ ] **Step 1: 升 cmake_minimum_required**

把第一行

```cmake
cmake_minimum_required(VERSION 3.5)
```

改成

```cmake
cmake_minimum_required(VERSION 3.8)
```

- [ ] **Step 2: 在 find_package 区块补 rclcpp**

把这段

```cmake
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
```

改成

```cmake
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
```

- [ ] **Step 3: 把 `ament_target_dependencies` 行加上 tf2 / tf2_geometry_msgs**

把

```cmake
ament_target_dependencies(tarkbot_robot rclcpp std_msgs geometry_msgs nav_msgs  sensor_msgs tf2_ros tf2_msgs Boost)
```

改成

```cmake
ament_target_dependencies(tarkbot_robot rclcpp std_msgs geometry_msgs nav_msgs sensor_msgs tf2 tf2_ros tf2_msgs tf2_geometry_msgs Boost)
```

- [ ] **Step 4: 记到变更日志**

```bash
echo "$(date +%F): CMakeLists.txt cmake>=3.8, find_package(rclcpp/tf2/tf2_geometry_msgs)" >> ~/ros2_ws/src/tarkbot_robot/CHANGELOG-jazzy-fix.md
```

### Task 4: 修 tf2 头文件路径（.h → .hpp）

**Files:**
- Modify: `~/ros2_ws/src/tarkbot_robot/include/tarkbot_robot/tarkbot_robot.h`

- [ ] **Step 1: 检查 jazzy 上的实际头文件**

Run: `ls /opt/ros/jazzy/include/tf2/LinearMath/Quaternion*`
Expected: 看到 `Quaternion.hpp`（不是 .h）。Jazzy 把 tf2 头改成 .hpp 了。

- [ ] **Step 2: 改头文件 include**

把

```cpp
#include <tf2/LinearMath/Quaternion.h>
```

改成

```cpp
#include <tf2/LinearMath/Quaternion.hpp>
```

- [ ] **Step 3: 同步检查 cpp 里有没有同样问题**

Run: `grep -n "tf2/LinearMath" ~/ros2_ws/src/tarkbot_robot/src/tarkbot_robot.cpp`
如果有 `.h` 后缀，按 Step 2 同样改成 `.hpp`。

- [ ] **Step 4: 记到变更日志**

```bash
echo "$(date +%F): tarkbot_robot.h tf2 header .h -> .hpp (jazzy)" >> ~/ros2_ws/src/tarkbot_robot/CHANGELOG-jazzy-fix.md
```

### Task 5: 干净 build + launch 验证

**Files:** 无修改，只验证。

- [ ] **Step 1: 装运行时依赖（如未装）**

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-rclcpp ros-jazzy-tf2 ros-jazzy-tf2-ros ros-jazzy-tf2-msgs \
  ros-jazzy-tf2-geometry-msgs ros-jazzy-nav-msgs ros-jazzy-sensor-msgs \
  ros-jazzy-geometry-msgs ros-jazzy-std-msgs \
  libboost-thread-dev libboost-system-dev
```

- [ ] **Step 2: 干净 build**

```bash
cd ~/ros2_ws
rm -rf build install log
source /opt/ros/jazzy/setup.bash
colcon build --packages-select tarkbot_robot 2>&1 | tee /tmp/build2.log
echo "Exit: $?"
```
Expected: `Exit: 0`，summary 行 `Summary: 1 package finished`，可有 deprecation warning（接受）。

- [ ] **Step 3: source overlay**

```bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep tarkbot
```
Expected: 输出 `tarkbot_robot`。

- [ ] **Step 4: 无串口 dry launch（验证 launch 文件本身能解析）**

```bash
ros2 launch tarkbot_robot robot.launch.py --print
```
Expected: 输出 launch 描述，无 Python 报错。

- [ ] **Step 5: 实际 launch（无串口会卡在 openSerialPort）**

```bash
timeout 5 ros2 launch tarkbot_robot robot.launch.py 2>&1 | head -30
```
Expected: 看到 `Tarkbot Robot Set serial /dev/ttyACM0 at 230400 baud` 之后会因没插车报串口打开错。这一步只验证节点本身能起来，不要求看到数据。

- [ ] **Step 6: 记修复完成**

```bash
echo "$(date +%F): jazzy build PASS, launch dry-run PASS" >> ~/ros2_ws/src/tarkbot_robot/CHANGELOG-jazzy-fix.md
```

### Task 6: 真车回环验证（接 USB 后做，可延后到硬件到货）

**Files:** 无。

- [ ] **Step 1: 加 udev 规则（可选，先不加也行）**

```bash
ls /dev/ttyACM*
```
确认 STM32 USB 出现为 `/dev/ttyACM0`。如果是 `/dev/ttyACM1` 等其他号，临时改 launch 参数：`ros2 launch tarkbot_robot robot.launch.py robot_port:=/dev/ttyACM1`。

- [ ] **Step 2: 加 dialout 组**

```bash
sudo usermod -aG dialout $USER
# 注销重登录（不是重启 shell）
```

- [ ] **Step 3: 上电 + 启动节点**

```bash
ros2 launch tarkbot_robot robot.launch.py
```
Expected: 看到 "Tarkbot Robot Set serial /dev/ttyACM0 at 230400 baud" 后无错误。

- [ ] **Step 4: 另开终端，验证话题**

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic list
```
Expected: 看到 `/odom /imu /bat_vol /cmd_vel /tf`。

- [ ] **Step 5: 验证 odom 频率**

```bash
ros2 topic hz /odom
```
Expected: 接近 50Hz（48–52Hz 之间）。

- [ ] **Step 6: 验证 cmd_vel 闭环**

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.1}, angular: {z: 0.0}}'
```
Expected: 车前进一小段距离（先把车架空，免得撞）。`ros2 topic echo /odom --once` 看 `pose.pose.position.x` 有变化。

- [ ] **Step 7: 完整修包阶段标记结束**

```bash
echo "$(date +%F): real robot loopback PASS" >> ~/ros2_ws/src/tarkbot_robot/CHANGELOG-jazzy-fix.md
```

---

## Phase 2: 写新版学习计划（Day 3–4）

### Task 7: 建立新计划文件骨架 + 替换映射表

**Files:**
- Create: `/home/ppf/workspace/全向移动自主作业机器人/塔克创新智能车资料/学习计划-3个月-ROS2-Jazzy版.md`

- [ ] **Step 1: 用原计划做基底，复制成新文件**

```bash
SRC="/home/ppf/workspace/全向移动自主作业机器人/塔克创新智能车资料/学习计划-3个月.md"
DST="/home/ppf/workspace/全向移动自主作业机器人/塔克创新智能车资料/学习计划-3个月-ROS2-Jazzy版.md"
cp "$SRC" "$DST"
```

- [ ] **Step 2: 改顶部标题和说明，加 Jazzy 标识**

把第 1–8 行

```
# 3 个月学习计划 — 抢入机器人/嵌入式实习
> 身份：在校学生（本/硕）
> 目标：3 个月后能投递机器人或嵌入式实习...
> 硬件：1 个月内到货...
> 起点：已有 4 份技术笔记（FreeRTOS / IMU / ROS / 运动学）
> 节奏：周 15~25 h，工作日各 1.5~2 h + 周末半天集中
```

替换为

```
# 3 个月学习计划（ROS2 Jazzy 版）— 抢入机器人/嵌入式实习
> 身份：在校学生（本/硕）
> 环境：Ubuntu 24.04 + ROS2 Jazzy + Gazebo Harmonic
> 目标：3 个月后能投递机器人或嵌入式实习，简历有 1 个有亮点的开源项目（ROS2 + Nav2），面试能讲清 5~6 个技术点
> 硬件：1~2 周内到货，前期纯仿真 + 修包，硬件到了切真车
> 起点：已有 4 份技术笔记（FreeRTOS / IMU / ROS / 运动学）+ 厂商 ROS2 驱动包已修复
> 节奏：周 15~25 h，工作日各 1.5~2 h + 周末半天集中
```

- [ ] **Step 3: 在 "## 阶段总览" 之前插一节《ROS1→ROS2 命令替换映射表》**

在第 17 行 `## 阶段总览` 之前插入：

```markdown
## ROS1→ROS2 命令替换映射表（贯穿全计划）

| ROS1 | ROS2 Jazzy |
|---|---|
| `roscore` | （不需要，DDS 自发现） |
| `roslaunch pkg file.launch arg:=val` | `ros2 launch pkg file.launch.py arg:=val` |
| `rostopic echo /topic` | `ros2 topic echo /topic` |
| `rosrun pkg node` | `ros2 run pkg node` |
| `rosbag record/play` | `ros2 bag record/play`（mcap 格式） |
| `rqt_plot` | `ros2 run rqt_plot rqt_plot` 或 `plotjuggler` |
| `gmapping` | `slam_toolbox`（W7 主） |
| `cartographer` | `cartographer_ros`（W8 对比） |
| `amcl`（独立包） | `nav2_amcl` |
| `move_base` | **Nav2** (`nav2_bringup`) |
| `robot_pose_ekf` | `robot_localization`（`ekf_node`） |
| `dynamic_reconfigure` | ROS2 参数 + `ros2 param set` |
| `tf` (旧) | `tf2_ros`（`ros2 run tf2_tools view_frames`） |
| `catkin_make` | `colcon build` |

```

- [ ] **Step 4: 把"阶段总览"里的成果点改 ROS2 描述**

把第 26–28 行（每阶段交付物）改为：

```
- 第 4 周末：Python 仿真 + Gazebo Harmonic + 4 份技术笔记发 GitHub
- 第 8 周末：真车跑通 SLAM 建图（slam_toolbox + cartographer 对比），录视频
- 第 12 周末：自主导航 demo（Nav2）+ 加分项 YOLO + 简历投出 ≥ 20 家
```

### Task 8: 翻新 W1–W4（仿真起步阶段）

**Files:**
- Modify: `学习计划-3个月-ROS2-Jazzy版.md`（W1–W4 章节）

- [ ] **Step 1: W1.1 改装 Jazzy**

把 W1.1 行

```
| W1.1 | 装好 Ubuntu 20.04（双系统/虚拟机/WSL2）+ ROS Noetic | 2 h  | `roscore` 能跑 |
```

替换为

```
| W1.1 | Ubuntu 24.04 + ROS2 Jazzy 确认环境（已装则验证） | 1 h | `ros2 doctor --report` 无 ERROR；`echo $ROS_DISTRO` 输出 jazzy |
```

- [ ] **Step 2: W1.2 改 colcon + 修包（关联 Phase 1 完成）**

把 W1.2 行

```
| W1.2 | 编译 `tarkbot_robot` ROS 包成功 | 1 h | `catkin_make` 无报错 |
```

替换为

```
| W1.2 | 修复并编译 `tarkbot_robot` ROS2 包（见 Phase 1 修包计划） | 4 h | `colcon build` 退出码 0；`ros2 launch tarkbot_robot robot.launch.py --print` 不报错 |
```

- [ ] **Step 3: W2 增加 rclpy 练手任务**

在 W2.5 后插入新行（W2.5 之后，W2.6 之前）：

```
| W2.5b | 写 `min_vel_listener.py`：用 rclpy 订阅 /cmd_vel 打印速度 | 2 h | `ros2 run` 能跑，发 cmd_vel 能看到打印 |
```

W2.6 笔记主题改为《技术笔记-6种车型运动学.md》（保持原意）。

- [ ] **Step 4: W3 改 ROS2 launch + RViz2**

把 W3.1 / W3.2 / W3.3 改为：

```
| W3.1 | 手写简化 URDF：盒子车体 + 4 圆柱轮 + 雷达圆柱 | 4 h | RViz2 能看到完整车型 |
| W3.2 | 写 Python launch（`robot.launch.py`）加载 robot_state_publisher | 2 h | `ros2 run tf2_tools view_frames` TF 树完整 |
| W3.3 | 写 `fake_robot.py`（rclpy）：订阅 cmd_vel 发布 odom + TF | 4 h | 键盘遥控 RViz2 假车能动 |
```

W3.4 fusion2urdf 流程不变（URDF 在 ROS1/ROS2 通用）。

- [ ] **Step 5: W4 改 Gazebo Harmonic + ros2_control**

把 W4.1–W4.5 替换为：

```
| W4.1 | 装 Gazebo Harmonic + ros_gz_bridge + ros2_control | 2 h | `gz sim` 命令能跑；`ros2 pkg list grep ros_gz` 有内容 |
| W4.2 | fork `mecanum_drive_demo`（ROS2 jazzy 版），改尺寸为 R750 | 4 h | gz sim 里有 R750 形状的车 |
| W4.3 | 用 `mecanum_drive_controller`（ros2_control）+ `ros_gz_bridge` 桥接 cmd_vel | 3 h | 键盘遥控车在 Gazebo 里跑 |
| W4.4 | 加 ray sensor → ros_gz_bridge → /scan | 3 h | RViz2 能看到 /scan 点云 |
| W4.5 | 加 imu sensor → ros_gz_bridge → /imu | 2 h | /imu 话题有数据 |
```

- [ ] **Step 6: W4 阶段总结改 ROS2 工具链**

把 W4 周日交付的"录一段 2 分钟视频"行改为：

```
- 🎯 **阶段 1 总结**：录一段 2 分钟视频展示 Python 仿真 + RViz2 + Gazebo Harmonic，发 B 站
```

### Task 9: 翻新 W5–W9（真车 + SLAM + 导航阶段）

**Files:**
- Modify: `学习计划-3个月-ROS2-Jazzy版.md`（W5–W9 章节）

- [ ] **Step 1: W5 改 ROS2 命令 + sllidar_ros2**

把 W5.2 / W5.4 / W5.7 改为：

```
| W5.2 | 串口连上位机，跑 `ros2 launch tarkbot_robot robot.launch.py` | 2 h | `ros2 topic echo /odom` 有数据 |
| W5.4 | 装 `teleop_twist_keyboard`(ROS2)，键盘遥控真车跑起来 | 1 h | 车能动 |
| W5.7 | 雷达接 USB，跑 `sllidar_ros2`（思岚雷达），RViz2 看到点云 | 2 h | RViz2 里能转一圈看到障碍 |
```

把 W5.5 改为：

```
| W5.5 | 实测 odom 准不准：直走 1 m，看 `pose.pose.position.x` | 2 h | 误差 < 5%（不准就改 STM32 端 WHEEL_DIAMETER） |
```

- [ ] **Step 2: W6 改 PlotJuggler + ros2 bag**

把 W6.1 / W6.5 / W6.6 改为：

```
| W6.1 | 装 `plotjuggler-ros`，订阅 /odom 实时画 4 路 RT vs TG | 2 h | 能看见 4 路曲线 |
| W6.5 | 录一份 ros2 bag（mcap）：遥控车跑 1 分钟（含 odom/imu/scan） | 1 h | bag 文件 < 100 MB |
| W6.6 | 离线 `ros2 bag play` + RViz2 回放，验证一切正常 | 1 h | 回放时 TF/scan/odom 一致 |
```

- [ ] **Step 3: W7 整章改 slam_toolbox**

把 W7.1 / W7.2 / W7.5 改为：

```
| W7.1 | 装 `slam_toolbox` + 学官方 demo（turtlebot3 仿真） | 2 h | 能跑 turtlebot3 在线建图 demo |
| W7.2 | 写 `slam_toolbox.launch.py`，对接你的 odom + scan + TF | 3 h | 启动无报错，看到 /map 输出 |
| W7.5 | （挪到 W8）保留 slam_toolbox 在线建图为 W7 唯一主线 | - | - |
```

W7.6 笔记标题改为《技术笔记-SLAM建图实战(slam_toolbox).md》。

- [ ] **Step 4: W8 整章改 cartographer + Nav2 AMCL + robot_localization**

把 W8.1 / W8.4 / W8.5 改为：

```
| W8.0 | 源码装 `cartographer_ros`（jazzy 暂无 apt 包），跟 slam_toolbox 出图对比 | 4 h | 两份地图 + 一份对比笔记（gmapping/cartographer/slam_toolbox 三选二） |
| W8.1 | 跑 `nav2_amcl`：加载 W7 地图 + 实时定位（用 nav2_bringup 的 localization-only launch） | 3 h | RViz2 里粒子云收敛到正确位置 |
| W8.4 | 装 `robot_localization`，配 `ekf_node` 融合 odom + imu | 2 h | 输出的 /odometry/filtered 比纯轮式更稳 |
| W8.5 | 第 9 份技术笔记《技术笔记-AMCL与EKF融合(ROS2).md》 | 4 h | - |
```

- [ ] **Step 5: W9 整章改 Nav2**

把 W9.1 / W9.2 / W9.3 改为：

```
| W9.1 | 学 Nav2 架构（BT / Behavior Server / Planner / Controller / Costmap） | 6 h | 能讲清 BT 在 Nav2 里的角色，对比 ROS1 move_base |
| W9.2 | 配 `nav2_params.yaml`（global_costmap / local_costmap / planner_server / controller_server） | 6 h | `ros2 launch nav2_bringup navigation_launch.py` 启动无错 |
| W9.3 | 选 `dwb_local_planner`（DWA 的 ROS2 版） | 2 h | 能避障 |
```

W9.4–W9.7 工具升级：RViz2 用 `Nav2 Goal` 代替原 `2D Nav Goal`，文字改一下。W9 时间从 1 周扩到 1.5 周（吃掉 W10 前 3 天，下面 W10 调整）。

### Task 10: 翻新 W10–W12 + 面试题 + 新增附录 G/H

**Files:**
- Modify: `学习计划-3个月-ROS2-Jazzy版.md`（W10–W12 + 附录）

- [ ] **Step 1: W10 时间收紧（被 W9 吃了 3 天）**

把 W10 改为 4 天工作量（原是一周）。任务清单不变，但提示文字加：

```
> 注：本周时间被 W9 Nav2 调参吃掉前 3 天，剩 4 天集中冲简历 + 投递。
```

- [ ] **Step 2: W12 加分项改 YOLO + Nav2 联动**

把 W12.3 三选一替换为单一指定任务：

```
| W12.3 | 加分功能：USB 摄像头 + `usb_cam`(ROS2) + YOLOv8(nano, CPU) + Nav2 联动 | 8 h | RViz2 能看到检测框；检测到障碍时通过 `/cancel_goal` 取消导航；录 30 秒视频 |
```

下面对应的子选项 a/b/c 全删掉。

- [ ] **Step 3: 改面试题库的 ROS 那 10 题**

把附录 C 的"ROS（10 题）"整段替换为：

```markdown
## ROS2（10 题）

1. ROS1 Master vs ROS2 DDS：发现机制差异，DDS 的优势？
2. ROS2 QoS 五个核心 policy（reliability/durability/history/depth/liveliness），各自什么场景用？
3. Lifecycle Node 五个状态（unconfigured/inactive/active/finalized + transitioning），用在哪？
4. SingleThreadedExecutor vs MultiThreadedExecutor 区别？什么时候要用 callback group？
5. `nav_msgs/Odometry` 协方差矩阵为什么是 6×6？
6. 为什么 `Pose` 在 odom 系，`Twist` 在 base 系？
7. ROS2 launch（Python）和 ROS1 launch（XML）写法区别？OpaqueFunction 什么时候用？
8. Nav2 BT（Behavior Tree）相比 ROS1 move_base 状态机优势？
9. 如何在多机间共享 ROS2（ROS_DOMAIN_ID + Discovery Server）？
10. 怎么用 `ros2 bag` (mcap 格式) 调试，相比 ROS1 rosbag 优势？
```

- [ ] **Step 4: 在附录 D（避坑清单）后追加附录 G、H**

```markdown
# 附录 G：ROS2 关键概念速查（与 ROS1 对照）

| 概念 | ROS1 | ROS2 Jazzy | 备注 |
|---|---|---|---|
| 中间件 | TCPROS（自研） | DDS（FastDDS / CycloneDDS） | DDS 自带 QoS |
| 节点发现 | roscore（中心化） | DDS 自动发现 | 不用 roscore，但 ROS_DOMAIN_ID 隔离 |
| 接口编译 | catkin_make | colcon build | 工作区在 install/ 而非 devel/ |
| 单元 | Node | Node / LifecycleNode / Component | Component 可热插 |
| 调度 | spin / spinOnce | Executor + CallbackGroup | 多线程更显式 |
| 参数 | param + dynamic_reconfigure | rcl param + ros2 param set | 没有 .cfg |
| Action | actionlib | rclcpp_action | 接口几乎一致 |
| Bag | rosbag (.bag) | ros2 bag (.mcap 默认) | mcap 跨语言 |
| 导航栈 | move_base | Nav2 (BT-driven) | BT 让流程可视化 |
| SLAM | gmapping/cartographer | slam_toolbox/cartographer | toolbox 是 nav2 默认 |

# 附录 H：Jazzy 安装填坑清单

1. **官方源**：`https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html` 严格按 Noble 分支走。
2. **国内镜像**：清华源 `https://mirrors.tuna.tsinghua.edu.cn/ros2/ubuntu/`、中科大 `https://mirrors.ustc.edu.cn/ros2/ubuntu/`，但镜像偶尔不全，优先官方，慢就 fallback。
3. **第一次 colcon build 常见错**：
   - `find_package(rclcpp)` failed → 装 `ros-jazzy-rclcpp`
   - `tf2/LinearMath/Quaternion.h not found` → jazzy 改 .hpp，include 改后缀
   - `boost/bind.hpp` deprecation → 接受 warning，不修
4. **Gazebo Harmonic**：`sudo apt install ros-jazzy-ros-gz` 一行装，不要装 classic。
5. **Cartographer**：jazzy 暂无 apt 包，源码装：clone `cartographer_ros` jazzy 分支 → `colcon build --packages-up-to cartographer_ros`，可能需 abseil 依赖手装。
6. **WSL2 跑 Gazebo**：需 WSLg（Win11 默认）或 VcXsrv，`gz sim` 启动慢可设 `GZ_PARTITION` 加速发现。
```

---

## Phase 3: 终验（Day 4 末）

### Task 11: 整体一致性扫描

**Files:** `学习计划-3个月-ROS2-Jazzy版.md` 整体扫描。

- [ ] **Step 1: 扫残留 ROS1 命令**

```bash
DST="/home/ppf/workspace/全向移动自主作业机器人/塔克创新智能车资料/学习计划-3个月-ROS2-Jazzy版.md"
grep -nE "roscore|catkin_make|roslaunch[^.]|rostopic |rosrun |rosbag |rqt_plot|gmapping|move_base|robot_pose_ekf|amcl[^_]" "$DST"
```
Expected: 输出全部在"对照表"或"附录 G"上下文中（即"ROS1 → ROS2"对照说明里）。其余命令性出现都应是 `ros2 ...`。如果有"任务正文"残留，回相应 Task 改。

- [ ] **Step 2: 扫 turtlebot3 / Noetic 等 ROS1 专属术语**

```bash
grep -nE "Noetic|Indigo|Kinetic|Melodic" "$DST"
```
Expected: 无输出。

- [ ] **Step 3: 验证文件大小合理**

```bash
wc -l "$DST"
```
Expected: 行数在原计划 ±20% 内（原 467 行，新版 ~450–550 行合理）。

- [ ] **Step 4: 写终验日志**

```bash
echo "$(date +%F): 学习计划-ROS2-Jazzy版 终验通过" >> ~/ros2_ws/src/tarkbot_robot/CHANGELOG-jazzy-fix.md
```

---

## 自检（写完 plan 后我自己跑）

- ✅ Spec 第 5.1 修包清单 5 项 → Task 2 / 3 / 4 一一覆盖
- ✅ Spec 第 5.2 验证步骤 4 步 → Task 5 完整覆盖
- ✅ Spec 第 5.3.1 映射表 14 行 → Task 7 Step 3 完整放入新计划
- ✅ Spec 第 5.3.2 周级改动 12 行 → Task 8 / 9 / 10 全部覆盖（W1–W12）
- ✅ Spec 第 5.3.3 保留项 → 在 Task 7–10 中没动那些章节，符合"保留不动"
- ✅ Spec 第 5.3.4 新增附录 G/H → Task 10 Step 4 覆盖
- ✅ 无 TBD / TODO / 占位
- ✅ 所有命令均给出实际可执行形式（colcon / ros2 命令完整）
- ✅ 类型/路径/包名一致（tarkbot_robot 全程一致，路径 ~/ros2_ws/src/tarkbot_robot 全程一致）
- ⚠️ 项目不在 git 仓库内 → 计划里没用 `git commit`，改用 `CHANGELOG-jazzy-fix.md` 追加变更记录，已与 Spec §1 探索阶段确认一致

