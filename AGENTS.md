# AGENTS.md

This file provides guidance to Codex (Codex.ai/code) when working with code in this repository.

## Project Overview

TARKBOT R750系列全向移动自主作业机器人底盘开发资源包。基于OpenCTR H60控制器(STM32F407VET6, 168MHz)和FreeRTOS，支持6种机器人运动学模型，通过ROS驱动包与上位机通信。

## Repository Structure

```
├── 1 机器人底盘开发手册和教程/     — 开发手册PDF + 视频教程
├── 2 STM32机器人底盘源码和固件/    — STM32固件源码(Keil MDK5) + 预编译.hex
├── 3 STM32机器人开发教程/          — 代码讲解视频 + 运动学模型教程
├── 4 ROS功能包源码及教程资料/      — ROS1/ROS2驱动包 + 使用教程视频
├── 5 OpenCTR H60控制器资料/        — 控制器原理图 + 开发手册
├── 6 软件工具与驱动程序/           — Keil MDK5, 串口工具, 蓝牙APP等
├── 7 3D模型文件/                   — STEP格式3D模型(R750-AKM/FWD/MEC/TWD)
├── 8 通用学习视频教程和资料/       — Linux基础 + ROS基础教程
└── improve_part/                   — 电机控制学习示例(PID/步进/插补等)
```

## STM32 Firmware Architecture

**源码路径:** `2 STM32机器人底盘源码和固件/源码文件/OpenCTR_H60V36_R750ROS_V3.61.0602/OpenCTR_H60V36_R750ROS_V3.61.0602/`

**构建:** 用Keil MDK5打开 `Project/xproject.uvprojx`，编译后通过USB串口或SWD下载。

**FreeRTOS任务架构 (main.c):**
| 任务 | 优先级 | 功能 |
|------|--------|------|
| `Start_Task` | 1 | 初始化：加载EEPROM参数→IMU陀螺仪零偏校准→创建其他任务后自删除 |
| `Robot_Task` | 3 | 主控制循环：20ms周期(50Hz)，控制模式选择→运动学解算→IMU采集→数据回传ROS |
| `Key_Task` | 4 | 按键处理，切换控制模式/灯光效果 |
| `Light_Task` | 5 | RGB彩灯效果控制 |
| `Disp_Task` | 6 | OLED显示刷新 |
| `Trivia_Task` | 10 | 杂项管理 |

**核心模块 (Robot/):**
- `ax_robot.h` — 全局数据结构定义(ROBOT_Velocity, ROBOT_Wheel, ROBOT_Imu, ROBOT_Light)、机器人类型宏、运动学参数、通信协议帧头、控制模式枚举
- `ax_kinematics.c` — 6种运动学模型实现，编译时通过 `ROBOT_TYPE` 宏选择(MEC/FWD/AKM/TWD/TNK/OMT)。每种模型包含：编码器读数→轮速→底盘速度(正向)；目标底盘速度→目标轮速→PID→电机PWM(逆向)
- `ax_speed.c/h` — 4路独立PID速度闭环控制器
- `ax_control.c/h` — 手柄/APP/SBUS遥控三种控制方式的速度指令解析
- `ax_robot.c` — `Robot_Task`主循环、IMU坐标系转换(IMU坐标系→ROS坐标系)、X-Protocol数据打包发送

**驱动层 (Driver/):**
- 4路串口: UART1(调试), UART2(ROS USB通信), UART3(蓝牙), UART4(ROS TTL通信)
- 外设驱动: `ax_motor`, `ax_encoder`, `ax_imu`(QMI8658A), `ax_servo`, `ax_can`, `ax_eeprom`, `ax_rgb`, `ax_oled`, `ax_sbus`, `ax_gamepad`, `ax_beep`, `ax_vin`(电压检测)

**通信协议:** 塔克通用X-Protocol，帧头+长度+数据+校验。帧头定义在 `ax_robot.h` 的 `ID_URX_*` / `ID_UTX_*` 宏中。UART2/UART4波特率230400。

**机器人类型选择:** 修改 `ax_robot.h` 中 `ROBOT_TYPE` 宏，重新编译即可切换车型。各车型的运动学参数(WHEEL_BASE, WHEEL_DIAMETER等)在同一文件中定义。

## ROS Package

**路径:** `4 ROS功能包源码及教程资料/ROS1源码资料/tarkbot_robot_r750_20260530.zip` → `tarkbot_robot/`

**构建 (ROS1):**
```bash
# 将tarkbot_robot解压到catkin workspace的src目录
catkin_make
```

**启动:**
```bash
roslaunch tarkbot_robot robot.launch robot_type:=robot_mec
```
`robot_type` 可选: `robot_mec` / `robot_fwd` / `robot_twd` / `robot_akm`

**节点架构 (`tarkbot_robot.cpp`):**
- 串口通信: Boost.Asio异步读取，TarkbotRobot类封装
- **发布话题**: `odom`(nav_msgs/Odometry), `imu`(sensor_msgs/Imu), `bat_vol`(std_msgs/Float32)
- **订阅话题**: `cmd_vel`(geometry_msgs/Twist)
- **服务**: `light_set`(tarkbot_robot/Light_Set)
- **动态参数**: `robot.cfg` — PID参数、IMU协方差等可在运行时通过rqt_reconfigure调整
- **TF变换**: odom→base_footprint
- 数据周期: 50Hz (20ms)，与下位机同步
- IMU四元数通过加速度计+陀螺仪互补滤波计算

## improve_part 目录

39个独立的STM32电机控制学习项目，每个项目包含完整的Keil工程(User/目录下有源码，Doc/目录下有说明)。涵盖:
- PID控制: 位置式/增量式，速度环/位置环/电流环，有刷/无刷电机
- 步进电机: 梯形/S形加减速、逐点比较法直线/圆弧插补(各象限)
- 编码器测速: 减速电机/无刷电机(霍尔)/步进电机

每个项目的User/目录下有独立的 `main.c` 和驱动代码，可直接打开编译学习。
