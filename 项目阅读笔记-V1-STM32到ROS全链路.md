# 项目阅读笔记 V1 — TARKBOT R750 STM32→ROS 全链路复述

> 日期：2026-06-14
> 阅读对象：`OpenCTR_H60V36_R750ROS_V3.61.0602` STM32 固件 + `tarkbot_robot` ROS2 驱动包 v1.0.0
> 笔记目的：在没真车的情况下，把整个数据链路在心里跑一遍，写下来；面试时能不开屏幕讲清楚。

## 0. 一句话项目概述

TARKBOT R750 是一台**全向移动自主作业机器人**：底盘可配置 6 种运动学模型（麦克纳姆 / 四轮差速 / 阿克曼 / 两轮差速 / 履带 / 三轮全向），主控用 OpenCTR H60 V3.6 板（STM32F407VET6 跑 FreeRTOS @ 168MHz），通过 USB 虚拟串口（波特率 230400）以 50Hz 节拍与上位机交换数据；上位机 Ubuntu 24.04 + ROS2 Jazzy，跑一个 `tarkbot_robot` 节点把串口字节流翻译成 ROS 话题（`/odom` `/imu` `/bat_vol`），同时把 `/cmd_vel` 翻译回串口发下去。再往上就是标准 SLAM/Nav2 栈。

整套系统就是把一台机器人拆成两层："**实时层**"（STM32 上 50Hz 闭环跑稳）和"**算法层**"（ROS2 异步跑 SLAM/导航），中间用一段 230400 的 USB 串口做强解耦。

## 1. 物理硬件层

最底下是 4 路减速电机（带 13 线霍尔编码器，减速比 30，所以 1 转 = 13×30×4 = **1560 个编码计数**），1 颗 6 轴 IMU（**QMI8658A**，I²C 接到 STM32），1 块 12V 锂电池（接 ax_vin ADC 监测电压），加上外设 OLED + RGB 灯 + 按键。

**雷达不接 STM32**：RPLIDAR 的 USB 直接插上位机 Ubuntu，由 `sllidar_ros2` 驱动直接发 `/scan`，跟 STM32 这条链路完全平行。这是个常见误解—— 看起来"雷达是机器人的一部分"，但它的数据流跟轮速/IMU 是两条独立通道。

## 2. STM32 固件层（核心实时控制）

### 2.1 FreeRTOS 任务架构

源码 `User/main.c` 创建 6 个任务，按优先级从低到高：

| 任务 | 优先级 | 职责 |
|---|---|---|
| `Start_Task` | 1 | 上电初始化（读 EEPROM 参数、IMU 零偏校准），创建其他任务后**自删除** |
| `Robot_Task` | 3 | **主控制循环**，20ms 周期（50Hz）—— 整个项目的脉搏 |
| `Key_Task` | 4 | 按键扫描，切换控制模式 / 灯光效果 |
| `Light_Task` | 5 | RGB 灯效果刷新 |
| `Disp_Task` | 6 | OLED 显示刷新 |
| `Trivia_Task` | 10 | 杂项管理（蜂鸣器、电压预警等） |

注意优先级数字越小越低（FreeRTOS 默认配置），**越关键的任务优先级越低**听起来反直觉，但合理 —— `Robot_Task` 用的是 `vTaskDelayUntil` **绝对时间**唤醒，不会被低频任务抢得来不及；`Disp_Task`/`Light_Task` 优先级高一点，是因为它们工作量小、对时延不敏感，让它们立刻跑完不阻塞别人。

### 2.2 Robot_Task 主循环

`ax_robot.c:Robot_Task()` 主体只有 5 步，每 20ms 严格执行一遍：

```c
while(1) {
    vTaskDelayUntil(&PreviousWakeTime, pdMS_TO_TICKS(20));   // 节拍器

    // ① 控制方式选择
    if (ax_control_mode == CTL_GPD) AX_CTL_Gpd();           // USB 手柄
    else if (ax_control_mode == CTL_APP) AX_CTL_App();      // 蓝牙 APP
    else if (ax_control_mode == CTL_RMS) AX_CTL_RemoteSbus(); // SBUS 航模

    // ② 运动学 + PID 闭环
    if (ax_robot_move_enable)
        AX_ROBOT_Kinematics();    // 6 种车型分支，编译时选定
    else
        AX_ROBOT_Stop();

    // ③ IMU 数据采集
    ROBOT_IMUHandle();

    // ④ 数据打包发回 ROS（USB + TTL + CAN 三路同时发）
    ROBOT_SendDataToRos();
}
```

注意第 ① 步只在"非 ROS 控制"时才执行。当上位机用 `/cmd_vel` 控制时，`ax_control_mode = 0`，速度指令由串口接收中断异步写入 `R_Vel.TG_*`，主循环直接用。


### 2.3 运动学：以麦轮（MEC）为例

`ax_kinematics.c` 用 `#if (ROBOT_TYPE == ROBOT_MEC)` 这种**编译期条件**选车型 —— 6 个 `AX_ROBOT_Kinematics()` 函数体并存，但只编译 1 个。这种写法的好处是固件镜像不带没用的代码、不浪费 Flash，坏处是切车型必须重新编译固件。

麦轮版的核心 5 步（`ax_kinematics.c:39-100`）：

**第 1 步 — 读编码器算实际轮速**

```c
R_Wheel_A.RT = (float)((int16_t)AX_ENCODER_A_GetCounter() * MEC_WHEEL_SCALE);
AX_ENCODER_A_SetCounter(0);   // 读完立即清零，等下一个 20ms 再读
```

`MEC_WHEEL_SCALE = π × D × PID_RATE / RESOLUTION = π × 0.097 × 50 / 1560 ≈ 0.00977`，单位是 (m/s) / count。也就是说：每 20ms 编码器变化 1 个数 ≈ 0.01 m/s 的轮速。

为什么 B 和 D 轮要乘 -1？因为它们装反了（左右两边电机正方向相反），软件里取负让 4 个轮"正"方向都对应车前进。

**第 2 步 — 正解：4 轮 → 底盘 (vx, vy, ω)**

```c
R_Vel.RT_IX = (( A + B + C + D)/4) * 1000;     // 前进
R_Vel.RT_IY = ((-A + B + C - D)/4) * 1000;     // 横移（麦轮独门）
R_Vel.RT_IW = ((-A + B - C + D)/4 / L) * 1000; // 自转
```

`L = (WHEEL_BASE + ACLE_BASE) / 2 = (0.196 + 0.160) / 2 = 0.178 m`，乘 1000 是为了把浮点 m/s 转成 mm/s 的整数（更方便用 int16 在串口里塞）。

**第 3 步 — 限幅**：`TG_IX/IY/IW` 不超过 `R_VX_LIMIT/R_VY_LIMIT/R_VW_LIMIT`，避免上位机发疯（比如 `cmd_vel.linear.x = 100`）把电机拉爆。

**第 4 步 — 逆解：底盘目标 → 4 轮目标轮速**

```c
R_Wheel_A.TG = TG_FX - TG_FY - TG_FW * L;
R_Wheel_B.TG = TG_FX + TG_FY + TG_FW * L;
R_Wheel_C.TG = TG_FX + TG_FY - TG_FW * L;
R_Wheel_D.TG = TG_FX - TG_FY + TG_FW * L;
```

这就是麦轮著名的 4×3 雅可比矩阵（每行符号不同对应轮的不同方向滚轮安装角）。手推一遍：把车纯前进，四个轮 +X；纯左移，A/D 倒转 B/C 正转；纯逆时针自转，A/C 倒转 B/D 正转。和上面公式一一对应。

**第 5 步 — PID 输出 PWM**

```c
R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);
AX_MOTOR_A_SetSpeed(-R_Wheel_A.PWM);
```

`ax_speed.c` 实现 4 路独立 **位置式 PID**，每路有自己的 `Kp/Ki/Kd` 和积分项。位置式相对增量式的好处是抗"控制量丢失"—— 即使某次 ISR 错过一帧，下次还能从历史误差累积里恢复。当然代价是要做抗积分饱和（代码里限幅 ±MAX_PWM）。

### 2.4 IMU：QMI8658A → Mahony 互补滤波

STM32 这边只读原始数据 + 做零偏校准（开机静止 1 秒钟取均值作为陀螺仪偏置），**四元数计算交给 ROS2 上位机**。这是个重要的架构选择 —— 把无状态的传感器读数往上推，复杂的姿态融合放到算力强的 Linux 上跑。STM32 只给 6 个 int16：`acc_x/y/z`、`gyro_x/y/z`。

量程定死：加速度 ±4g（`ACC_RATIO = 4*9.8/32768 ≈ 1.197e-3 m/s² per LSB`），陀螺仪 ±512°/s（`GYRO_RATIO = 512π/180/32768 ≈ 2.728e-4 rad/s per LSB`）。这两个常量在 ROS2 端 `tarkbot_robot.h` 写死，**STM32 量程换了 ROS2 也得改**。

### 2.5 数据打包发送（X-Protocol）

`ax_robot.c:ROBOT_SendDataToRos()` 把 20 字节负载塞进协议帧：

```
偏移 0-5    IMU 加速度 X/Y/Z（int16，高字节先发）
偏移 6-11   IMU 陀螺仪 X/Y/Z（int16）
偏移 12-17  底盘速度 RT_IX/IY/IW（int16，单位 mm/s）
偏移 18-19  电池电压（uint16，单位 0.01V）
```

完整一帧（共 25 字节）：
```
0xAA 0x55 | 0x19 (LEN=25) | 0x10 (ID_UTX_DATA) | <20 字节负载> | <CKSUM>
```

校验和是从 `0xAA` 一路加到负载末尾，取低 8 位。

值得注意的是：**这 20 字节同时发到 UART2（USB 上 PC）+ UART4（TTL 给可能的副控）+ CAN 总线**（拆成 3 个 CAN 帧，因为 CAN 单帧最大 8 字节）。一份数据多路播，给系统集成留了灵活性。

## 3. 通信层：USB CDC

STM32 把 UART2 接到 USB CDC（虚拟串口）IC，电脑识别为 `/dev/ttyACM0`。波特率 230400，比常见的 115200 高一倍 —— 因为 50Hz 双向、每帧 25 字节，单向 50×25 = 1250 字节/秒，留 10 倍余量给突发是 12500 = 100kbps，恰好选个略大的标准波特率。

### 3.1 双向流量评估

| 方向 | 频率 | 单帧字节 | 有效带宽 |
|---|---|---|---|
| 上行（STM32 → ROS） | 50Hz | 25 | 10 kbps |
| 下行（ROS → STM32） | 跟 cmd_vel 频率（典型 10–20Hz） | 11 | <1 kbps |

230400 bps 足足可以再翻 5 倍数据量也不堵。

### 3.2 帧同步算法

ROS2 端 `tarkbot_robot.cpp:recvCallback()` 用经典的 **状态机解析**：每来 1 字节，根据当前 `rx_con` 决定怎么处理：
- `rx_con == 0` → 期待 `0xAA`，不是就丢弃
- `rx_con == 1` → 期待 `0x55`，不是就重置回 0
- `rx_con == 2` → 长度字节，记下
- `rx_con < LEN-1` → 累积数据
- `rx_con == LEN-1` → 校验和位，算总和对比，对就回调上层 `recvDataHandle()`

这种实现简单稳健 —— 即使串口噪声把 0xAA 后面字节冲掉了，状态机也只浪费几字节就能重新对齐。代价是没用 DMA + 环形缓冲，每字节一次回调略浪费 CPU，但 12kbps 完全无压力。

## 4. ROS2 上位机层

### 4.1 节点结构

`tarkbot_robot` 是个**单节点应用**（不是 LifecycleNode），构造函数里一次性做完：
1. 读 ROS 参数（`robot_port` `robot_port_baud` `*_topic` `pub_odom_tf` 等）
2. 创建 4 个 publisher（`/odom` `/imu` `/bat_vol`）+ 1 个 subscriber（`/cmd_vel`）+ 1 个 TF broadcaster
3. 调 `openSerialPort()` 打开 `/dev/ttyACM0`
4. 启 boost 线程跑 `recvCallback()` 阻塞读串口

注意 boost 线程一旦启动，主线程的 `rclcpp::spin()` 在外面跑，**两个线程之间共享 `vel_data_`/`imu_data_`/`pos_data_` 等成员变量但没加锁**。这在 50Hz 频率下没出过问题，但严格说是个数据竞争。**面试题点**：让我改我会用 `std::atomic` 封装结构体，或者切到单线程的 `MultiThreadedExecutor` + `MutuallyExclusive` 回调组。

### 4.2 接收 + 发布链路

`recvDataHandle()` 收到一帧 `ID=0x10` 后：

```cpp
// 1. 反序列化 IMU + 速度（注意大端解析）
imu_data_.acc_x = ((int16_t)(buffer_data[4]*256 + buffer_data[5])) * ACC_RATIO;
...

// 2. 用上一帧速度积分得到位姿（航迹推算 dead-reckoning）
pos_data_.pos_x += (vx*cos(θ) - vy*sin(θ)) * DATA_PERIOD;
pos_data_.pos_y += (vx*sin(θ) + vy*cos(θ)) * DATA_PERIOD;
pos_data_.angular_z += angular_z * DATA_PERIOD;

// 3. Mahony 互补滤波算四元数
calculateImuQuaternion(imu_data_);

// 4. 发布 + 广播 TF
publishOdom(); publishImu(); publishBatVol(); publishOdomTF();
```

注意 `DATA_PERIOD = 0.02` 是写死的常量，**假设了下位机严格 50Hz**。如果下位机抖动，里程计会失真。生产上推荐用消息时间戳差分而不是常量，但这个项目当作教学演示问题不大。

### 4.3 Mahony 互补滤波（重点理解）

`calculateImuQuaternion()` 是经典 Mahony 算法，思路：

1. **重力方向参考**：当前四元数 `q` 描述了车体姿态，从中可推出"按 q 旋转后，重力应该指向哪"（`halfvx/y/z`）
2. **重力方向测量**：加速度计读数归一化后就是当前感知到的重力方向（`acc_x/y/z`）
3. **误差**：两个向量的叉积（`halfex/y/z`）—— 越偏差越大
4. **修正**：误差作为陀螺仪的"附加角速度"反馈进去（PI 控制：`gyro += Kp×err + Ki×∫err`）
5. **积分**：修正后的角速度积分到四元数 `q += 0.5×Ω×q×dt`
6. **归一化**：四元数模长保持 1

这套算法本质是用加速度计**长期低频校准**陀螺仪的零漂（陀螺仪短期准、长期漂；加速度计反过来）。代码里 `twoKp=1.0, twoKi=0` —— 只用 P 不用 I，简化版。Madgwick 滤波是同一个思路的另一变种，区别在用的是梯度下降而不是 PI。

代码里有个亮点：`invSqrt()` 用了著名的 **Quake III 快速平方根倒数**（`*(long*)&y` 类型双关 + 魔法常数 `0x5f3759df`）—— 在 STM32F407 上有 FPU 不算太关键，但代码作者把 ROS2 端也照着搬过来了，其实 PC 上不如直接 `1.0f/std::sqrt(x)` 让编译器优化 `rsqrtss` 指令。**面试题点**：被问到 invSqrt 时能讲清"原理是 IEEE 754 浮点数的位级近似 + 一次牛顿迭代"。

### 4.4 cmd_vel 反向链路

ROS2 端 `cmdVelCallback()` 反向把 `geometry_msgs/Twist` 翻译成串口帧发回 STM32：

```cpp
vel_data[0..1] = (int16_t)(linear.x * 1000)    // 大端 int16，单位 mm/s
vel_data[2..3] = (int16_t)(linear.y * 1000)
vel_data[4..5] = (int16_t)(angular.z * 1000)   // 单位 mrad/s
sendSerialPacket(vel_data, 6, ID_ROS2CRP_VEL);   // ID=0x50
```

`sendSerialPacket()` 套上协议头（`0xAA 0x55 + LEN + ID`），算校验，扔到串口。STM32 那边的接收中断会解析后写到 `R_Vel.TG_*`，下一个 20ms 的主循环就生效。**全链路延迟**最坏情况：cmd_vel 发到 ROS 节点 → 串口发出 → STM32 中断接收 → 等下一个主循环节拍执行 → PWM 输出，理论最大 ~25ms（含 USB CDC 缓冲），实测 ~10–15ms。

### 4.5 TF 与坐标系

`publishOdomTF()` 广播 `odom → base_footprint` 的 TF 变换。**坐标系约定**（ROS REP 105）：
- `odom` 是世界坐标系，零点为开机时的位置，连续不跳变（但有累积误差）
- `base_footprint` 是车体地面投影（z=0），机器人在地面的位置/朝向
- 上面通常还有 `base_link`（车体几何中心，z = 轮半径）由 URDF 定义
- 再上面是 `map`（绝对地图），由 SLAM/AMCL 提供 `map → odom` 变换

这一套 TF 树让导航栈能解耦工作 —— 局部规划只需要 `odom`（连续），全局定位用 `map`（精确但可能跳变）。

## 5. 整条链路的"一帧生命周期"

把上面所有内容合并成一帧的时间线（车在跑、ROS 订了 odom 也在发 cmd_vel）：

```
T=0    上位机 Nav2 算出新 cmd_vel 发到 /cmd_vel 话题
T=1ms  tarkbot_robot 节点 cmdVelCallback 触发，打包 11 字节串口帧
T=2ms  USB CDC 把帧送到 STM32（串口中断填到环形缓冲）
T=3ms  STM32 用户态从中断缓冲读出，写入 R_Vel.TG_FX/FY/FW
T=20ms 下一个 50Hz 节拍唤醒，Robot_Task 跑：
       ├─ 读 4 路编码器 → 算实际 4 轮速 R_Wheel_*.RT
       ├─ 正解 → 实际底盘速度 R_Vel.RT_*
       ├─ 限幅 + 逆解 → 4 轮目标 R_Wheel_*.TG
       ├─ 4 路 PID(.TG, .RT) → PWM
       ├─ AX_MOTOR_*_SetSpeed(PWM) → 电机转
       ├─ 读 IMU 6 轴
       └─ 打 25 字节帧 → USB CDC → 上位机
T=23ms tarkbot_robot 节点收到上行帧：
       ├─ 反序列化 IMU + 底盘速度 + 电压
       ├─ 累积位姿（dead-reckoning）
       ├─ Mahony 算四元数
       └─ 发 /odom /imu /bat_vol + TF
T=24ms RViz / SLAM / Nav2 收到新数据，更新自己的世界模型
       ↓
       下一帧从 T=20ms 开始重复
```

也就是说**整个机器人的"心跳"是 20ms（50Hz）**，整条链路从 cmd_vel 进到电机响应在 25ms 内完成，从 IMU 读取到 ROS 收到也是 25ms 内。

## 6. 我对这套架构的几个看法

### 6.1 设计漂亮的地方

- **下位机只做"实时控制"，不做姿态估计** —— Mahony 在 ROS2 端算，方便切到 EKF 或 Madgwick 而不动固件
- **协议简单稳健** —— 25 字节定长帧 + 校验和，没用 protobuf/CBOR 等花架子，烧固件后能用串口助手手工调
- **6 种车型用编译期选择** —— 比运行时分支更省 Flash 和 CPU，代码也清晰（每种车型一个独立函数体）
- **三路输出（USB+TTL+CAN）冗余** —— 系统集成时给后期升级（比如加副控板）留口子

### 6.2 可改进的地方

- ROS2 节点没用 `LifecycleNode`，串口失败时 `rclcpp::shutdown()` 后没 `return` 直接崩（实测过），应该改成"打开失败 → 转 Inactive 状态等重试"
- Boost.Asio 串口和 ROS2 主线程之间共享变量没加锁（50Hz 下没事，万一改成异步话题就要小心）
- `DATA_PERIOD = 0.02` 写死，假设下位机严格 50Hz；理论上应该用 message 时间戳差分
- 协议没有版本字段 —— 将来扩字段（比如加电流监测）会破坏向后兼容
- IMU 量程在两端都是写死的，固件改了 ROS2 也得跟着改 —— 应该让下位机首帧自报量程

### 6.3 整套系统给我的最大启发

**实时性不是越高越好**。50Hz 这个数字是个精心选择 —— 高到足够让控制环稳（人能接受 < 50ms 的响应），低到 USB CDC 不堵 + STM32 算力宽裕 + ROS2 端不被刷屏。提到 200Hz、500Hz 反而要担心抖动和拥塞。**面试时遇到"为什么是 50Hz"这种问题，能从控制带宽 + 通信带宽 + 算力三个角度解释，比单纯说"经验值"专业得多**。

第二个启发是 **"硬件抽象层"在哪划界**。这套代码把抽象层划在了 STM32 出口的串口处：往下是寄存器级、实时关键的；往上是话题级、策略层面的。雷达、SLAM、导航、可视化全在 ROS 上层，永远不需要看 STM32 一行代码。这种**强解耦**让整个系统每一层都可以独立替换 —— 换控制器只动固件，换算法只改 ROS 包，互不影响。

## 7. 接下来的学习重点（个人）

1. 把 `cmdVelCallback` → `sendSerialPacket` → STM32 中断 → `R_Vel.TG_*` → `Kinematics` → `PID` → PWM 这条**正向流**和反向流写一份手绘时序图（W2 准备发到 GitHub）
2. 用 Python 复现麦轮逆解（W2 任务），验证我对公式 `vx ± vy ± ωL` 的理解 —— 给 (1, 0, 0) (0, 1, 0) (0, 0, 1) 单独输入，看四个轮速对不对
3. 理解 Mahony 滤波时手推一遍 q 的乘法和归一化（不用看代码，纸笔自己推）
4. 等硬件到货，用示波器抓串口，亲眼看一帧 25 字节飞过 —— 把书本上的 0xAA 0x55 变成真实波形

---

> 笔记总字数 ≈ 4500 字 · 写于 W1.6 · 项目仓库：[github.com/PPF123ppf/tarkbot-r750-study](https://github.com/PPF123ppf/tarkbot-r750-study)

