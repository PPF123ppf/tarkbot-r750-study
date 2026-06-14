# 技术笔记 — ROS 架构与节点（TARKBOT R750）

> ROS 包：`4 ROS功能包源码及教程资料/ROS1源码资料/tarkbot_robot_r750_20260530.zip` → `tarkbot_robot/`
> 节点二进制：`tarkbot_robot`（C++ ROS1 节点，唯一一个节点）
> 上位机典型平台：Jetson / 树莓派 / 工控机，跑 Ubuntu 20.04 + ROS1 Noetic
> 与下位机连接：USB CDC（`/dev/tarkbot_base` → STM32 UART2）或 TTL（→ STM32 UART4），波特率 230400

---

## 0. 一句话理解整个系统

**STM32（下位机）= 实时控制 + 传感器采集**；**ROS（上位机）= 算法、决策、可视化、SLAM/导航**。
两边通过 **串口 + 塔克 X-Protocol 二进制帧** 以 50 Hz 双向通信。`tarkbot_robot` 这个节点只干一件事：**把串口数据翻译成 ROS 话题/服务，再把 ROS 话题翻译成串口指令**。它是上位机生态和下位机硬件之间的"翻译官"。

```
┌─────────────────  上位机 (ROS Master)  ─────────────────┐
│                                                         │
│  键盘遥控   /  Web 控制台  /  导航栈 (move_base / Nav2)  │
│  rqt_graph  /  rviz       /  AMCL  /  gmapping          │
│           │                                             │
│           ▼  发布 /cmd_vel                              │
│  ┌───────────────────────────────────────────────────┐  │
│  │            tarkbot_robot 节点                     │  │
│  │  订阅: /cmd_vel, /beep                            │  │
│  │  服务端: /Light_Server                            │  │
│  │  发布:   /odom, /imu, /bat_vol                    │  │
│  │  TF:     odom -> base_footprint                   │  │
│  │  动态参数: imu_calibrate, RGB_*, light_calibrate │  │
│  │            ▲                                      │  │
│  │ 接收线程：boost::asio                              │  │
│  └────────────│─────────────────────────────────────┘  │
└────────────── │ 串口 230400 (X-Protocol)──────────────┘
               ▼
       ┌────────────────────────────────────┐
       │  STM32F407 OpenCTR H60 (FreeRTOS)  │
       │  Robot_Task @50Hz  →  20B 数据帧   │
       │  UART RX ISR   →  cmd_vel 解包     │
       └────────────────────────────────────┘
```

## 1. ROS 1 基础概念速记（套到本项目）

| 概念                  | 通俗解释                                              | 本项目示例                                                              |
| --------------------- | ----------------------------------------------------- | ----------------------------------------------------------------------- |
| **Master**            | "电话总机"，所有节点要先到 Master 注册                | `roscore` 启动后才能跑 `tarkbot_robot`                                  |
| **Node 节点**         | 一个独立进程，承担一份职责                            | `tarkbot_robot_node`（本项目唯一节点）                                  |
| **Topic 话题**        | 单向广播管道，发布/订阅模式，多对多                   | `/cmd_vel`, `/odom`, `/imu`, `/bat_vol`, `/beep`                        |
| **Message 消息**      | 话题里跑的数据结构                                    | `geometry_msgs/Twist`, `nav_msgs/Odometry`                              |
| **Service 服务**      | 双向请求-响应（同步），一对一                         | `/Light_Server`（请求 RGB 设置，响应"Success change light"）            |
| **Param Server 参数** | Master 上的全局键值表                                 | `~odom_frame`, `~robot_port_baud`                                       |
| **Dynamic Reconfig**  | 运行时可改的参数，带回调                              | IMU 校准触发、RGB 实时调色（`cfg/robot.cfg`）                           |
| **TF 坐标变换**       | 机器人各部件相对位置的"动态家谱"                      | `odom → base_footprint`                                                 |
| **Frame 坐标系**      | 三维空间里的某个固定参考系                            | `odom_frame`, `base_frame=base_footprint`, `imu_frame=imu_link`         |
| **Launch 启动文件**   | XML 描述要起哪些节点、传哪些参数                      | `launch/robot.launch`                                                   |
| **catkin / package**  | ROS1 包管理 + 构建系统                                | `package.xml`（依赖声明）+ `CMakeLists.txt`（构建规则）                 |

> **ROS1 vs ROS2** —— 本项目源码是 ROS1（Noetic）。`ROS2源码资料` 文件夹空着，说明厂家还没放 ROS2 版本。两者核心概念基本相通，主要区别：ROS2 用 DDS 通信、不需要 Master、节点带 QoS、launch 文件用 Python。

## 2. 包结构（`tarkbot_robot/`）

```
tarkbot_robot/
├── package.xml          ← ROS 包元数据：名字、版本、依赖
├── CMakeLists.txt       ← 构建脚本（编译 cpp、生成 srv 头、注册动态参数）
├── include/
│   └── tarkbot_robot.h  ← 类声明 + 协议 ID 宏 + IMU 比例尺
├── src/
│   └── tarkbot_robot.cpp← 唯一源文件，767 行，包含全部逻辑
├── srv/
│   └── Light_Set.srv    ← 自定义服务消息
├── cfg/
│   └── robot.cfg        ← 动态参数描述（Python，会被 catkin 自动转成头文件）
└── launch/
    └── robot.launch     ← 启动入口
```

**CMakeLists 关键依赖**：`roscpp / nav_msgs / sensor_msgs / geometry_msgs / std_msgs / tf / dynamic_reconfigure / message_generation`。串口走 Boost.Asio（`#include <boost/asio.hpp>`），无需第三方 ROS 串口包。

## 3. 启动流程（`launch/robot.launch`）

```bash
roslaunch tarkbot_robot robot.launch robot_type:=robot_mec
```

可选 `robot_type` —— `robot_mec`（默认麦轮）/ `robot_fwd`（四轮差速）/ `robot_twd`（两轮差速）/ `robot_akm`（阿克曼）/ `robot_tak`（履带，写错应为 tnk）/ `robot_omni`（全向 omni）。launch 做了三件事：

1. 定义可覆盖参数（odom_frame、串口设备名、波特率、是否发 TF 等）
2. 起一个节点 `tarkbot_robot`（`pkg=tarkbot_robot type=tarkbot_robot`）
3. 把 `robot_type` 透传给节点的私有参数 `robot_type_send`

> **`/dev/tarkbot_base`**：launch 默认串口名是 `/dev/tarkbot_base`，这是个 udev 软链接，需要你在上位机配 udev 规则（按 USB 设备 VID/PID 自动绑定，避免每次重新插拔变成 ttyUSB0/ttyUSB1）。如果没配，就改成 `/dev/ttyACM0` 或 `/dev/ttyUSB0`。


## 4. 节点接口全表

### 4.1 发布的话题（节点 → 外部）

| 话题       | 消息类型                  | 频率   | 内容                                                                                      |
| ---------- | ------------------------- | ------ | ----------------------------------------------------------------------------------------- |
| `/odom`    | `nav_msgs/Odometry`       | 50 Hz  | 位姿（position + orientation）+ 实时速度（twist）+ 协方差矩阵（静止/运动两套）            |
| `/imu`     | `sensor_msgs/Imu`         | 50 Hz  | 线加速度 (m/s²) + 角速度 (rad/s) + 四元数（仅 z/w 有效，roll/pitch 故意置 0）             |
| `/bat_vol` | `std_msgs/Float32`        | 50 Hz  | 电池电压 V                                                                                |

### 4.2 订阅的话题（外部 → 节点）

| 话题       | 消息类型                  | 用途                                                              |
| ---------- | ------------------------- | ----------------------------------------------------------------- |
| `/cmd_vel` | `geometry_msgs/Twist`     | 速度指令：`linear.x` 前后 m/s，`linear.y` 左右 m/s，`angular.z` 转向 rad/s |
| `/beep`    | `std_msgs/Int8`           | 蜂鸣器：1=短鸣 200ms，2=长鸣 1s                                   |

### 4.3 服务

| 服务            | 类型                          | 请求字段                                       | 响应      |
| --------------- | ----------------------------- | ---------------------------------------------- | --------- |
| `/Light_Server` | `tarkbot_robot/Light_Set`     | `RGB_M`(模式 1~6) `RGB_S`(子模式) `RGB_T`(时间) `RGB_R/G/B`(颜色) | `string result` |

调用示例：
```bash
rosservice call /Light_Server "{RGB_M_: 1, RGB_S_: 0, RGB_T_: 0, RGB_R_: 0, RGB_G_: 255, RGB_B_: 0}"
```

### 4.4 TF 坐标变换

- 节点持续广播 **`odom → base_footprint`** 的动态变换（频率 50 Hz）
- 由 `pub_odom_tf` 参数控制是否发布（默认 true）
- **不发布 `imu_link → base_footprint`**：需要你自己用 `static_transform_publisher` 或 URDF 提供，否则 robot_localization / EKF 用不上 IMU

### 4.5 私有参数（启动时设置）

| 参数                | 默认值              | 含义                                       |
| ------------------- | ------------------- | ------------------------------------------ |
| `~odom_frame`       | `odom`              | 里程计坐标系名                             |
| `~base_frame`       | `base_footprint`    | 机器人基坐标系名                           |
| `~imu_frame`        | `imu_link`          | IMU 坐标系名（只用于 imu 消息 header）     |
| `~odom_topic`       | `odom`              | 发布的里程计话题名                         |
| `~imu_topic`        | `imu`               |                                            |
| `~battery_topic`    | `bat_vol`           |                                            |
| `~cmd_vel_topic`    | `cmd_vel`           |                                            |
| `~robot_port`       | `/dev/ttyTHS1`      | 串口设备（launch 里覆盖成 `/dev/tarkbot_base`） |
| `~robot_port_baud`  | `230400`            |                                            |
| `~pub_odom_tf`      | `true`              | 是否发 odom→base TF                        |
| `~robot_type_send`  | `robot_mec`         | 启动时下发到 STM32 的车型号（1=MEC..6=OMT）|

### 4.6 动态参数（运行时改，`cfg/robot.cfg`）

| 参数              | 类型 | 范围   | 作用                               |
| ----------------- | ---- | ------ | ---------------------------------- |
| `imu_calibrate`   | bool | -      | 勾选→下发 IMU 零偏校准指令，自动复位 |
| `light_calibrate` | bool | -      | 勾选→下发 RGB 设置存 EEPROM        |
| `RGB_M`           | int  | 1~10   | 灯效模式                           |
| `RGB_S`           | int  | 1~10   | 子模式                             |
| `RGB_T`           | int  | 0~255  | 时间参数                           |
| `RGB_R/G/B`       | int  | 0~255  | RGB 颜色                           |

调用方式：`rosrun rqt_reconfigure rqt_reconfigure`，弹出 GUI 实时拖滑块。


## 5. 节点内部数据流（`tarkbot_robot.cpp` 详解）

```
                     ┌─────  TarkbotRobot 类（构造函数即初始化全部）─────┐
                     │                                                   │
   ros::spin()       │                                                   │
   主线程 ───────────┤  cmd_vel / beep 回调（订阅）                      │
                     │      ↓ 打包 6B / 1B + 帧头帧尾                    │
                     │  sendSerialPacket()  ←  /Light_Server 回调        │
                     │      ↓                                            │
                     │  boost::asio::write() ──→ 串口                    │
                     │                                                   │
                     │  动态参数回调  →  IMU校准 / 灯效保存指令          │
                     │                                                   │
   recvSerial_thread │  recvCallback() 死循环：                          │
   独立 boost 线程───┤    boost::asio::read() 1 字节                     │
                     │    状态机解析 X-Protocol（帧头 AA 55 + 长度 + ID  │
                     │      + payload + 校验和）                         │
                     │    收齐一帧 → recvDataHandle()                    │
                     │      ↓                                            │
                     │    分类按 ID 处理：                                │
                     │      ID_CPR2ROS_DATA(0x10): 24B 综合数据          │
                     │        → 解析 imu_data_, vel_data_, bat_vol_data_ │
                     │        → 积分计算 pos_data_                       │
                     │        → calculateImuQuaternion() 互补滤波        │
                     │        → publishOdom() / publishImu() / publishBatVol() │
                     │        → publishOdomTF()                          │
                     └───────────────────────────────────────────────────┘
```

**关键设计点**：

1. **双线程模型** —— ROS 主线程跑 `ros::spin()` 处理订阅回调；一个独立 `boost::thread` 跑 `recvCallback()` 阻塞读串口。两者通过类成员变量共享 IMU/odom 数据，**没有显式锁** —— 因为 ROS 发布器内部线程安全，且只有 `recvCallback` 一个生产者写。
2. **构造函数永不返回** —— `TarkbotRobot()` 末尾 `ros::spin()` 阻塞，直到节点关闭才返回，析构发"停车 + 静音"指令到 STM32（防止 ROS 意外退出后小车继续跑）。
3. **里程计积分在 ROS 端做**（`tarkbot_robot.cpp:324`）：
   ```cpp
   pos_data_.pos_x += (vx*cos(θ) - vy*sin(θ)) * 0.02;   // DATA_PERIOD = 20ms
   pos_data_.pos_y += (vx*sin(θ) + vy*cos(θ)) * 0.02;
   pos_data_.angular_z += vz * 0.02;
   ```
   为什么不在 STM32 算？因为 ROS 端用 double 精度，且需要的是**累积位姿**，下位机只需关心当前速度。

4. **协方差矩阵两套**：静止 vs 运动（`tarkbot_robot.cpp:404`）。
   - 静止时：编码器准（线速度协方差 `1e-9`），陀螺仪有零飘（角度协方差 `1e-9` 极信任，意思是角度不变）
   - 运动时：编码器有打滑（线速度协方差 `1e-3`），IMU 角速度更准（角度协方差 `1e3` 的较大值表示要让 EKF 多依赖 IMU）
   这样喂给 `robot_pose_ekf` 或 Nav2 的 EKF 时，融合权重会自适应。

5. **IMU 四元数 = 互补滤波（Mahony 算法简化版）**（`tarkbot_robot.cpp:668`）：
   - 用加速度计估算"重力方向" → 与"由四元数推算的重力方向"做叉积 → 得到**误差**
   - 误差 × Kp = 修正量，加到陀螺仪积分上 → 得到稳定四元数
   - **但代码故意把 `orientation.x = 0; y = 0` 强制清零**（line 361），只保留 z/w —— 因为本质是平面机器人，roll/pitch 没意义，留着只会让 RViz 显示得很怪。
   - 用 Quake 著名的 `0x5f375a86` 魔术常数算 `1/sqrt(x)`（line 763），省掉浮点除法和 sqrtf。


## 6. 通信协议 X-Protocol

帧格式：

```
+------+------+------+------+--------+--------+
| 0xAA | 0x55 | LEN  |  ID  |  DATA  | CHKSUM |
+------+------+------+------+--------+--------+
  帧头1  帧头2  总长   帧编号   N字节    累加和
       (LEN = 数据长度 + 5)             (前面所有字节相加 8位截断)
```

**ROS → STM32（指令）**：

| ID    | 名称              | DATA 长度 | 含义                                            |
| ----- | ----------------- | --------- | ----------------------------------------------- |
| 0x50  | `ID_ROS2CTR_VEL`  | 6         | cmd_vel，3×int16，单位 mm/s 或 mrad/s           |
| 0x51  | `ID_ROS2CTR_IMU`  | 1         | 0x55 触发 IMU 重新零偏校准                      |
| 0x52  | `ID_ROS2CTR_LGT`  | 6         | 灯光设置 M/S/T/R/G/B                            |
| 0x53  | `ID_ROS2CTR_LST`  | 1         | 灯光参数存 EEPROM                                |
| 0x54  | `ID_ROS2CTR_BEEP` | 1         | 蜂鸣器（0=关，1=短鸣，2=长鸣）                  |
| 0x5A  | `ID_ROS2CTR_RTY`  | 1         | 节点启动时下发车型号                            |

**STM32 → ROS（数据）**：

| ID   | 名称              | DATA 长度 | 内容                                                                              |
| ---- | ----------------- | --------- | --------------------------------------------------------------------------------- |
| 0x10 | `ID_CPR2ROS_DATA` | 20        | acc(6) + gyro(6) + vel(6) + vol(2)，全部大端 int16                                |

**单位换算**（`include/tarkbot_robot.h`）：
```c
ACC_RATIO  = 4 * 9.8 / 32768       // raw → m/s²，量程 ±4g
GYRO_RATIO = (512 * π/180) / 32768 // raw → rad/s，量程 ±512°/s
vel = raw / 1000.0                 // raw mm/s → m/s
vol = raw / 100.0                  // raw → 伏特
```

## 7. 构建与启动

### 7.1 创建 catkin workspace 并编译

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
unzip ~/Downloads/tarkbot_robot_r750_20260530.zip -d .
cd ..
catkin_make
source devel/setup.bash    # 把 ~/.bashrc 末尾追加这行可省事
```

`catkin_make` 会做的事：
1. 解析 `package.xml` 的依赖；
2. 用 `cfg/robot.cfg` 生成 `robotConfig.h`（动态参数头文件）；
3. 编译 `srv/Light_Set.srv` 成 `Light_Set.h` + `Light_Set.cpp`；
4. 编译 `tarkbot_robot.cpp` → `devel/lib/tarkbot_robot/tarkbot_robot`。

### 7.2 启动

```bash
roscore                                                          # 终端 1
roslaunch tarkbot_robot robot.launch robot_type:=robot_mec       # 终端 2
```

### 7.3 验证联通

```bash
rostopic list                       # 应看到 /odom /imu /bat_vol /cmd_vel
rostopic hz /odom                   # 应稳定 ~50 Hz
rostopic echo /bat_vol              # 应能看到电压数字
rostopic pub -r 10 /cmd_vel geometry_msgs/Twist '[0.1,0,0]' '[0,0,0]'
                                    # 让小车以 0.1 m/s 前进
rosrun tf view_frames                # 生成 frames.pdf 看 TF 树
rqt_graph                           # 看节点-话题图
rosrun rqt_reconfigure rqt_reconfigure  # 调 PID/灯效（注意 cfg 里没 PID，PID 还在固件里）
```


## 8. 与 STM32 端的交叉对照

| 功能              | STM32 端（FreeRTOS）                              | ROS 端（`tarkbot_robot.cpp`）                     |
| ----------------- | ------------------------------------------------- | ------------------------------------------------- |
| 50 Hz 主循环      | `Robot_Task` + `vTaskDelayUntil(20ms)`            | `recvCallback` 死循环按帧到来频率（被 STM32 节奏带）|
| cmd_vel 接收      | `ax_uart2.c:172` 写入 `R_Vel.TG_IX/IY/IW`         | `cmdVelCallback()` 打包发出 0x50 帧                |
| 速度输出          | `R_Vel.RT_IX/IY/IW`（运动学正解）                 | `vel_data_.linear_x / linear_y / angular_z`       |
| IMU 数据上报      | `ROBOT_IMUHandle()` IMU→ROS 坐标系转换            | 按 `ACC_RATIO/GYRO_RATIO` 换成 SI 单位             |
| IMU 校准命令      | `ax_imu_calibrate_flag = 1` → `Trivia_Task` 处理  | 动态参数 `imu_calibrate=true` → 0x51 帧            |
| 灯效设置          | `R_Light.M/S/T/R/G/B`                             | `/Light_Server` 服务 + 动态参数（0x52 帧）        |
| 灯效保存到 EEPROM | `ax_light_save_flag = 1` → `Trivia_Task` 处理     | 动态参数 `light_calibrate=true` → 0x53 帧         |
| 蜂鸣器            | `ax_beep_ring = 1/2`                              | `/beep` 话题 → 0x54 帧                            |
| 车型              | `#define ROBOT_TYPE ROBOT_MEC`（编译期固定）      | `~robot_type_send` 启动时下发 0x5A 帧（**当前固件不支持运行时切换**） |
| 里程计积分        | 不做                                              | `recvDataHandle()` 里 `pos += vel * dt`           |
| 四元数            | 不做                                              | `calculateImuQuaternion()` Mahony 互补滤波         |
| 协方差            | 不做                                              | 静止/运动两套，喂给 EKF                           |

## 9. 上层生态对接

`tarkbot_robot` 节点跑起来后，下面这些标准 ROS 包能直接用：

| 上层栈              | 需要本节点提供的接口                                          |
| ------------------- | ------------------------------------------------------------- |
| `teleop_twist_keyboard` / `teleop_twist_joy` | `/cmd_vel` 订阅                                |
| `gmapping` / `cartographer` 建图  | `/odom`（含位姿+协方差）+ `tf: odom→base_footprint` + 雷达 `/scan` |
| `amcl` 定位                       | 静态地图 + `/scan` + `/odom`                                  |
| `move_base` (Nav1) / `nav2` (Nav2) | `/cmd_vel` 发布 + `/odom` + 代价地图 + 全局/局部规划器       |
| `robot_pose_ekf`                  | `/odom`（带协方差）+ `/imu`（带协方差）→ 输出 `/robot_pose_ekf/odom_combined` |
| `rviz` 可视化                     | TF 树完整 + URDF（雷达坐标 / 摄像头坐标需自行加 static TF）   |

> **没现成 URDF**：本包不含 URDF 文件，3D 模型在 `7 3D模型文件/`（STEP 格式）。要在 RViz 里看完整机器人轮廓，需要自己用 SolidWorks-to-URDF 或 fusion2urdf 插件转换。

## 10. 常见坑

| 现象                                | 原因 / 排查                                                                        |
| ----------------------------------- | ---------------------------------------------------------------------------------- |
| 启动报 `Open Port Failed`           | 串口设备名错；用户没在 `dialout` 组（`sudo usermod -aG dialout $USER` 后重新登录） |
| `rostopic hz /odom` 频率不稳        | 串口接触不良；或上位机 USB 总线被其他设备占满（雷达共用 USB）                      |
| 小车走直线偏一边                    | IMU 零偏未校准 / 编码器反向 / 左右轮直径不一致 → 在 STM32 端调 `ax_kinematics.c`   |
| TF: `odom -> base_footprint` 缺失   | launch 里 `pub_odom_tf=false`；改回 true                                           |
| 找不到 IMU 角度                     | 节点故意把 `orientation.x/y` 置 0，且无 `imu_link → base_footprint` 的 TF；自己加 static_transform_publisher |
| 灯光服务报 "提交的数据异常"         | `RGB_M_` 必须 1~6（对应 STM32 的 LEFFECT1~6），别填 0 或 >6                        |
| 切换车型不生效                      | 当前 STM32 固件 `ROBOT_TYPE` 是编译期宏，launch 传 `robot_type` 只发了消息但**STM32 收到后没用** —— 要换车型必须重新编译固件（改 `ax_robot.h:158`） |
| ROS2 启动                           | 包里**只有 ROS1 源码**，ROS2 源码资料目录是空的                                    |

## 11. 快速参考：源码定位

| 想看 / 想改                         | 文件:行号                                                       |
| ----------------------------------- | --------------------------------------------------------------- |
| 节点入口 + 构造                     | `src/tarkbot_robot.cpp:22~137`                                  |
| 串口接收线程                        | `src/tarkbot_robot.cpp:230~295`                                 |
| X-Protocol 收帧解析                 | `src/tarkbot_robot.cpp:300~339`                                 |
| 发布 odom + 协方差矩阵              | `src/tarkbot_robot.cpp:382~441`                                 |
| 发布 IMU + 协方差                   | `src/tarkbot_robot.cpp:344~377`                                 |
| `cmd_vel` 回调                      | `src/tarkbot_robot.cpp:489~503`                                 |
| 动态参数回调                        | `src/tarkbot_robot.cpp:555~607`                                 |
| Mahony 四元数算法                   | `src/tarkbot_robot.cpp:668~748`                                 |
| invSqrt（Quake 魔法）               | `src/tarkbot_robot.cpp:753~767`                                 |
| 协议 ID + 单位常量                  | `include/tarkbot_robot.h:59~79`                                 |
| 服务定义                            | `srv/Light_Set.srv`                                             |
| 动态参数定义                        | `cfg/robot.cfg`                                                 |
| 启动入口                            | `launch/robot.launch`                                           |
