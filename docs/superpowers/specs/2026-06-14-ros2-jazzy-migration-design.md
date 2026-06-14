# Spec：ROS2 Jazzy 学习路径迁移 + 驱动包最小修复

> 日期：2026-06-14
> 项目：塔克创新 TARKBOT R750 全向移动自主作业机器人
> 环境：Ubuntu 24.04 + ROS2 Jazzy

## 1. 背景

学习者环境为 Ubuntu 24.04 + ROS2 Jazzy，但已有的《学习计划-3个月.md》（467 行）按 Ubuntu 20.04 + ROS1 Noetic 编写，工具链命令、SLAM/定位/导航包名、launch 写法均不适配。

厂商提供的 ROS2 驱动包 `tarkbot_robot`（v1.0.0）协议与 STM32 固件 v3.61.0602 已验证完全匹配，可作为底层基础。但 ROS2 包本身在 jazzy 上 build 时存在依赖声明缺失等小问题，需要最小修复。

## 2. 目标

**两件交付物**：

1. **修复后的 ROS2 驱动包** — 在 Ubuntu 24.04 + Jazzy 环境下能 `colcon build` 通过，`ros2 launch` 拉得起，接 USB 后能看到 `/odom` `/imu` `/bat_vol` 数据。
2. **新版学习计划** `学习计划-3个月-ROS2-Jazzy版.md` — 整体翻新原计划，保留 12 周 / 3 阶段 / 周日交付物 / 验证标准骨架，工具链全替换为 ROS2 Jazzy + Nav2 体系。

## 3. 非目标

- 不重写驱动节点逻辑（不迁移到 LifecycleNode、不替换 Boost.Asio）。
- 不修复仅 warning 不影响 build 的问题（`ament_target_dependencies` deprecation、`<boost/bind.hpp>` deprecation）。
- 不删除原 ROS1 学习计划，保留作备查。
- 不写完整教程 / 课件，仅写学习路径与验证标准。

## 4. 执行顺序

**先修包后写计划**：

- Day 1–2：修包（W1.2 任务的实际落地）
- Day 3–4：写新版学习计划
- Day 5+：按新计划 W1 起步

理由：修包验证一遍工具链后写计划更扎实，命令不会"理论上对、实际漏依赖"。

## 5. 设计

### 5.1 包修复清单（最小集）

| # | 文件 | 问题 | 修法 |
|---|---|---|---|
| 1 | `package.xml` | 缺 `<depend>rclcpp</depend>` | 补 |
| 2 | `package.xml` | 缺 `<depend>tf2</depend>` `<depend>tf2_geometry_msgs</depend>` | 补 |
| 3 | `CMakeLists.txt` | 缺 `find_package(rclcpp REQUIRED)` | 补 |
| 4 | `CMakeLists.txt` | `cmake_minimum_required` 太低 | 改 3.8 |
| 5 | `include/tarkbot_robot/tarkbot_robot.h` | jazzy 中 `tf2/LinearMath/Quaternion.h` → `.hpp` | 改头文件名 |

**不修但记录**：

- `ament_target_dependencies` 在 jazzy 上是 deprecated 但能用（warning 接受）
- `<boost/bind.hpp>` 同上

### 5.2 验证步骤（W1.2 验证标准）

1. `colcon build --packages-select tarkbot_robot` 退出码 0
2. `source install/setup.bash && ros2 launch tarkbot_robot robot.launch.py` 不报错（无串口能起，等串口）
3. 接 USB 后 `ros2 topic list` 看到 `/odom /imu /bat_vol /cmd_vel`
4. `ros2 topic hz /odom` 接近 50Hz

### 5.3 学习计划翻新规则

#### 5.3.1 命令替换映射表（贯穿全文）

| ROS1 | ROS2 Jazzy |
|---|---|
| `roscore` | （不需要，DDS 自发现） |
| `roslaunch pkg file.launch arg:=val` | `ros2 launch pkg file.launch.py arg:=val` |
| `rostopic echo /topic` | `ros2 topic echo /topic` |
| `rosrun pkg node` | `ros2 run pkg node` |
| `rosbag record/play` | `ros2 bag record/play`（mcap 格式） |
| `rqt_plot` | `ros2 run rqt_plot rqt_plot` 或 `plotjuggler` |
| `gmapping` | `slam_toolbox`（W7 主） |
| `cartographer` | `cartographer_ros`（W8 对比，源码装） |
| `amcl`（独立包） | `nav2_amcl`（Nav2 子组件） |
| `move_base` | **Nav2** (`nav2_bringup`) |
| `robot_pose_ekf` | `robot_localization`（`ekf_node`） |
| `dynamic_reconfigure` | ROS2 参数 + `ros2 param set` |
| `tf` (旧) | `tf2_ros`（`ros2 run tf2_tools view_frames`） |
| `catkin_make` | `colcon build` |

#### 5.3.2 章节级改动

| 周 | 原内容 | 翻新后 |
|---|---|---|
| W1 | Ubuntu 20.04 + Noetic + catkin_make | Ubuntu 24.04 + Jazzy + colcon；W1.2 = 修包后 build 通过 |
| W2 | Python 数学仿真 | 保留（与 ROS 无关），新增"用 `rclpy` 写最小订阅节点"练手 |
| W3 | URDF + RViz + robot_state_publisher | URDF 几乎一致；launch 用 Python `LaunchDescription`；RViz2 |
| W4 | Gazebo + ros-control + 麦轮 plugin | **Gazebo Harmonic** + `ros_gz_bridge` + `ros2_control` + `mecanum_drive_controller` |
| W5 | 真车 + udev + rplidar_ros + teleop_twist_keyboard | rplidar 用 `sllidar_ros2`；teleop_twist_keyboard 的 ROS2 版 |
| W6 | rqt_plot 调 PID + rosbag | PlotJuggler + `ros2 bag` (mcap) |
| W7 | gmapping | **slam_toolbox 在线建图** |
| W8 | cartographer + AMCL + robot_pose_ekf | **cartographer_ros 对比** + Nav2 AMCL + `robot_localization` |
| W9 | move_base + costmap + DWA | **Nav2 完整栈**（NavFn / DWB） |
| W10–11 | 简历/投递/面试题 | 面试题加 ROS2 章节（DDS / Lifecycle / Executor / QoS） |
| W12 | (空) 加分项 | **USB 摄像头 + `usb_cam` + YOLOv8 + Nav2 联动**（看到障碍发 cancel goal 作为最小验证） |

#### 5.3.3 保留不动

- 整体节奏（周 15–25h）、3 阶段划分、周日交付物、附录 A 自查清单、附录 B 项目讲法、附录 D 避坑清单
- 附录 C 的嵌入式/C++/Linux/控制 4 类面试题（仅 ROS 那 10 题改 ROS2）

#### 5.3.4 新增

- 附录 G：ROS2 关键概念速查（DDS / QoS / Lifecycle / Executor / Component / Action 与 ROS1 对照）
- 附录 H：Jazzy 安装填坑清单（清华/中科大源 + 官方 fallback、第一次 colcon build 常见错）

## 6. 验证

**修包**：5.2 节四步通过即视为完成。

**学习计划**：满足以下检查表

- [ ] 12 周结构完整保留
- [ ] 每周日交付物可量化验证
- [ ] 5.3.1 映射表全文一致执行（无残留 ROS1 命令）
- [ ] W7 / W8 / W9 / W12 关键章节命令在 Jazzy 上可执行
- [ ] 附录 G、H 已新增
- [ ] 简历模板（附录 B）量化指标继续适用

## 7. 风险

| 风险 | 缓解 |
|---|---|
| Cartographer 在 jazzy 上需源码装，可能耗时超 W8 预算 | W8 把 cartographer 标记为"可选挑战"，slam_toolbox 已是 W7 主线 |
| Gazebo Harmonic 中文教程少 | 计划里给 ROS2 官方文档 + Construct 教程链接 |
| ROS2 Nav2 调参比 move_base 复杂 | W9 时间从 1 周扩到 1.5 周，必要时挪 W10 简历前半时间 |
| YOLOv8 + Nav2 联动可能需 GPU | W12 标"可选"，CPU 跑 nano 版本 ~5 FPS 也够演示 |

## 8. 工作量估算

- 修包：4–8 小时（含验证）
- 写计划：8–12 小时
- 总计：1.5–2.5 工作日

---

## 自检记录

- ✅ 无 TBD / TODO / 占位
- ✅ 修包清单与验证步骤包名/路径一致
- ✅ 学习计划翻新规则与决策一致（两 SLAM、Gazebo Harmonic、YOLO）
- ✅ 范围聚焦两件事，无外溢
- ✅ 5.3.4 附录 H 已加入"官方 fallback"以防国内镜像不全
