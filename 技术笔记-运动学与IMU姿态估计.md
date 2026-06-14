# 麦轮全向运动学与 IMU 姿态估计技术笔记

> 基于 TARKBOT R750 / OpenCTR H60 (STM32F407VET6) / FreeRTOS V3.61.0602
> 关键源码均给出文件路径与行号；公式与实现一一对应。

---

## 0. 项目概览

- 主控：STM32F407VET6，168 MHz，FreeRTOS 多任务
- 底盘：四轮麦克纳姆（`ROBOT_TYPE == ROBOT_MEC`，`ax_robot.h:158`）
- 控制周期：50 Hz（`Robot_Task` `vTaskDelayUntil 20 ms`，`ax_robot.c:99-108`）
- 传感：4 路正交编码器 + QMI8658A 6 轴 IMU
- 执行：4 路 H 桥直流电机，PWM 20 kHz
- 上行：CAN（0x0121/0x0122/0x0123）+ UART2/UART4 X-Protocol，姿态融合在 ROS 上位机完成

主控制循环骨架（`ax_robot.c:104-135`）：

```c
while (1) {
    vTaskDelayUntil(&PreviousWakeTime, pdMS_TO_TICKS(20));   // 50 Hz
    if (ax_control_mode != 0) { ... }                        // 手柄/APP/SBUS 注入速度
    if (ax_robot_move_enable) AX_ROBOT_Kinematics();         // 闭环运动学
    else                      AX_ROBOT_Stop();
    ROBOT_IMUHandle();                                       // 读 IMU + 坐标系变换
    ROBOT_SendDataToRos();                                   // CAN/UART 上传
}
```

> 整个机器人逻辑都在 `Robot_Task` 一个 50 Hz 循环里，没有中断式控制，便于调试。

---

## 一、麦克纳姆轮全向运动学

### 1.1 坐标系与轮序约定

```


辊子方向（俯视）：
    Front
  A ╲      ╱ B
        ┼
  C ╱      ╲ D
    Rear

- A 左前 \ 辊子   B 右前 /
- C 左后 /         D 右后 \
```

辊子方向决定了 X、Y、W 三个自由度对每个轮速的符号组合，下一节直接给出。

### 1.2 运动学参数（`ax_robot.h:163-167`）

```c
#define MEC_WHEEL_BASE        0.196f   // 轮距（左右轮间距），m
#define MEC_ACLE_BASE         0.160f   // 轴距（前后轮间距），m
#define MEC_WHEEL_DIAMETER    0.097f   // 轮直径，m
#define MEC_WHEEL_RESOLUTION  1560.0f  // 编码器分辨率：13 线 × 30 减速比 × 4 倍频 = 1560 脉冲/轮转
#define MEC_WHEEL_SCALE       (PI*MEC_WHEEL_DIAMETER*PID_RATE/MEC_WHEEL_RESOLUTION)
                                       // 一个采样周期里 1 个脉冲对应的轮线速度 (m/s)
```

`MEC_WHEEL_SCALE` 推导：

```
每脉冲位移   = π·D / RESOLUTION = π × 0.097 / 1560 ≈ 1.953e-4 m
每周期一个脉冲 → 速度 = 每脉冲位移 × PID_RATE = 1.953e-4 × 50 ≈ 9.77e-3 m/s

代码里直接：轮速 = 脉冲计数 × MEC_WHEEL_SCALE
```

等效力臂：

```
L = WHEEL_BASE/2 + ACLE_BASE/2 = 0.098 + 0.080 = 0.178 m
```

后面所有“±L·ω”里的 L 都是这个值。

### 1.3 逆解：底盘速度 → 四轮目标线速度（`ax_kinematics.c:74-77`）

```c
R_Wheel_A.TG = TG_FX - TG_FY - TG_FW * (MEC_WHEEL_BASE/2 + MEC_ACLE_BASE/2);
R_Wheel_B.TG = TG_FX + TG_FY + TG_FW * (MEC_WHEEL_BASE/2 + MEC_ACLE_BASE/2);
R_Wheel_C.TG = TG_FX + TG_FY - TG_FW * (MEC_WHEEL_BASE/2 + MEC_ACLE_BASE/2);
R_Wheel_D.TG = TG_FX - TG_FY + TG_FW * (MEC_WHEEL_BASE/2 + MEC_ACLE_BASE/2);
```

矩阵形式：

```
| W_A |   |  1  -1  -L | | Vx |
| W_B | = |  1   1   L | | Vy |
| W_C |   |  1   1  -L | | Vw |
| W_D |   |  1  -1   L |
```

直观理解：每个轮的线速度 = X 平移分量 ± Y 平移分量 ± 旋转分量，符号由 45° 辊子方向决定。

### 1.4 正解：编码器 → 实测底盘速度（`ax_kinematics.c:43-58`）

读编码器并清零（B/D 编码器极性与 A/C 相反）：

```c
R_Wheel_A.RT =  ENC_A * MEC_WHEEL_SCALE;   AX_ENCODER_A_SetCounter(0);
R_Wheel_B.RT = -ENC_B * MEC_WHEEL_SCALE;   AX_ENCODER_B_SetCounter(0);
R_Wheel_C.RT =  ENC_C * MEC_WHEEL_SCALE;   AX_ENCODER_C_SetCounter(0);
R_Wheel_D.RT = -ENC_D * MEC_WHEEL_SCALE;   AX_ENCODER_D_SetCounter(0);

R_Vel.RT_IX = (( WA + WB + WC + WD)/4) * 1000;                  // X (mm/s)
R_Vel.RT_IY = ((-WA + WB + WC - WD)/4) * 1000;                  // Y (mm/s)
R_Vel.RT_IW = ((-WA + WB - WC + WD)/4 / L) * 1000;              // ωz (mrad/s)
```

正解是逆解的**伪逆/最小二乘解**——4 个轮速观测过定义 3 个底盘自由度，取平均能抑制单轮打滑造成的偏差。`*1000` 是为了用 `int16_t` 上传时保留毫米/毫弧度精度。

### 1.5 速度限幅与单位转换（`ax_kinematics.c:61-71`，`ax_robot.h:231-233`）

```c
#define R_VX_LIMIT  1500   // |Vx| ≤ 1.5 m/s
#define R_VY_LIMIT  1200   // |Vy| ≤ 1.2 m/s
#define R_VW_LIMIT  6280   // |ωz| ≤ 6.28 rad/s ≈ 360°/s

// 限幅在 mm/s 整数域做，避免浮点比较；之后再 /1000 转 m/s
TG_FX = TG_IX / 1000.0f;
TG_FY = TG_IY / 1000.0f;
TG_FW = TG_IW / 1000.0f;
```

> 上层 ROS 用 `geometry_msgs/Twist` 传 m/s；STM32 内部统一用 mm/s 做整数处理，最后再回浮点做逆解。

### 1.6 运动模式由速度组合自然实现

代码里没有"模式切换"，三自由度任意线性组合：

| 模式 | TG_IX | TG_IY | TG_IW | 现象 |
|------|-------|-------|-------|------|
| 直行 | ≠0 | 0 | 0 | 四轮同速同向 |
| 横移 | 0 | ≠0 | 0 | A、D 反向；B、C 同向 |
| 原地自转 | 0 | 0 | ≠0 | 左右两侧反向 |
| 斜向平移 | ≠0 | ≠0 | 0 | X+Y 合成任意方向 |
| 弧线 | ≠0 | 0 | ≠0 | 边走边转 |
| 全向复合 | ≠0 | ≠0 | ≠0 | 平移 + 自转同时进行 |

---

## 二、速度闭环控制

> 闭环结构：50 Hz 增量式控制律 → 共享累加器 `motor_pwm_out[4]` → H 桥互补 PWM。
> 文件：`Robot/ax_speed.c`、`Driver/ax_motor.c`、`Robot/ax_robot.c`。

### 2.1 增量式速度控制器（`ax_speed.c:37-125`）

四个轮子的 `AX_SPEED_PidCtl{A,B,C,D}` 结构完全一致，仅访问 `motor_pwm_out[]` 不同下标：

```c
int16_t motor_pwm_out[4] = {0};            // 文件级共享，4 路 PWM 累加器
extern int16_t ax_motor_kp;                // ax_robot.c:55  = 800
extern int16_t ax_motor_kd;                // ax_robot.c:56  = 1000

int16_t AX_SPEED_PidCtlA(float spd_target, float spd_current)
{
    static float bias, bias_last;          // 静态变量，跨周期保留
    bias = spd_target - spd_current;       // ① 当前误差 e[k]
    motor_pwm_out[0] += ax_motor_kp*bias + ax_motor_kd*(bias - bias_last); // ② Δu 累加
    bias_last = bias;                      // ③ 记录 e[k-1]
    return motor_pwm_out[0];
}
```

控制律：

```
e[k]   = target − current
Δu[k]  = Kp · e[k] + Kd · (e[k] − e[k−1])
u[k]   = u[k−1] + Δu[k]
```

> 注意：虽然源码变量名叫 `kp/kd`，但因为输出用了 `u[k]=u[k-1]+...` 累加，
> `Kp·e[k]` 这一项在输出层表现为积分作用，`Kd·Δe[k]` 表现为比例/阻尼作用。
> 因此它不是标准位置式 PID，而是一个把积分与误差差分项耦合在一起的简化增量控制器。

### 2.2 增益与单位

| 量 | 值 / 单位 | 含义 |
|----|-----------|------|
| `ax_motor_kp` | 800 | 当前速度偏差 1 m/s → 累加器每周期 +800 计数 |
| `ax_motor_kd` | 1000 | 速度偏差变化量 1 m/s/周期 → 累加器 +1000 计数 |
| 控制周期 | 20 ms（50 Hz） | `Robot_Task` `vTaskDelayUntil` |
| 输出值域 | int16，最终被 `±4200` 截断 | PWM 计数（4200 = 100% 占空比） |

`ax_motor_kp = 800` 不是无量纲增益——它把"m/s 偏差"映射到"4200 计数 PWM 域"，与轮径、编码器、PWM ARR 都耦合。换硬件就要重新整定。

### 2.3 控制流（`ax_kinematics.c:80-89`）

```c
R_Wheel_A.PWM = AX_SPEED_PidCtlA(R_Wheel_A.TG, R_Wheel_A.RT);
R_Wheel_B.PWM = AX_SPEED_PidCtlB(R_Wheel_B.TG, R_Wheel_B.RT);
R_Wheel_C.PWM = AX_SPEED_PidCtlC(R_Wheel_C.TG, R_Wheel_C.RT);
R_Wheel_D.PWM = AX_SPEED_PidCtlD(R_Wheel_D.TG, R_Wheel_D.RT);

AX_MOTOR_A_SetSpeed(-R_Wheel_A.PWM);  // 整车正方向 ↔ 电机正转方向的极性补偿
AX_MOTOR_B_SetSpeed(-R_Wheel_B.PWM);
AX_MOTOR_C_SetSpeed( R_Wheel_C.PWM);
AX_MOTOR_D_SetSpeed( R_Wheel_D.PWM);
```

> 整车坐标系 → 电机安装方向通过这一组 ± 一次性对齐；编码器极性已经在 `R_Wheel_*.RT` 那一步处理过。

### 2.4 H 桥互补 PWM（`ax_motor.c:104-312`）

四路电机都是同一个范式（以 A 为例）：

```c
void AX_MOTOR_A_SetSpeed(int16_t speed) {
    int16_t temp = (speed > 4200) ? 4200 : (speed < -4200 ? -4200 : speed);
    if (temp > 0) {                          // 正转
        TIM_SetCompare1(TIM1, 4200);         // CH1 常高
        TIM_SetCompare2(TIM1, 4200 - temp);  // CH2 占空比反向
    } else {                                 // 反转
        TIM_SetCompare2(TIM1, 4200);         // CH2 常高
        TIM_SetCompare1(TIM1, 4200 + temp);
    }
}
```

时序（正转）：

```
CH1  ████████████████████████████████ 4200 (常高)
CH2  ████████████____________________ 4200 - speed

speed = 0     → CH1 = CH2，电机两端等电位，停转
speed > 0     → CH1 比 CH2 高，正转
speed < 0     → CH2 比 CH1 高，反转
```

PWM 频率均为 **20 kHz**（高于人耳，远高于电机电气时间常数）：

| 定时器 | 总线 | 时钟 | PSC | 计数频率 | ARR | PWM 频率 |
|--------|------|------|-----|----------|-----|----------|
| TIM1（A、B） | APB2×2 | 168 MHz | 1 | 84 MHz | 4200 | 20 kHz |
| TIM9（C） | APB2×2 | 168 MHz | 1 | 84 MHz | 4200 | 20 kHz |
| TIM12（D） | APB1×2 | 84 MHz | 0 | 84 MHz | 4200 | 20 kHz |

> 关键规则：当 APBx 预分频 > 1 时，定时器时钟自动 ×2 以补偿分频损失，所以 TIM1/9/12 最终都到 84 MHz。

### 2.5 两个工程隐患

#### 隐患 ①：`int16_t` 累加器溢出 → 电机突然反转

`motor_pwm_out[]` 是 `int16_t`，**累加器内部无限幅**，限幅只发生在 `AX_MOTOR_*_SetSpeed` 入口：

```
PidCtl 累加 (int16_t, 不限幅) → SetSpeed (限 ±4200) → TIM
```

堵转时（误差 ≈ 2 m/s 持续，Δu ≈ 1600/周期）：

```
周期    motor_pwm_out[0]      SetSpeed 限幅后    实际 PWM
 1            1600                 1600           正转 38%
 5            8000                 4200           正转 100%
20           32000                 4200           正转 100%
21    32000+1600 = 33600 → −31936  −4200          反转 100%  ← 翻转
```

`int16_t` max 32767，33600 溢出后补码回绕为 −31936，下一周期电机突然全速反转，但实际偏差仍为正——非常危险。

#### 隐患 ②：积分饱和（windup）

即使把 `motor_pwm_out` 改成 `int32_t` 不再溢出，长时间饱和时累加器还会跑到几万：

```c
// 内部已是 30000，但输出端被截断为 4200
SetSpeed(motor_pwm_out[0]);          // 输出 4200
// 误差刚反转，Δu = -2000
motor_pwm_out[0] = 30000 - 2000 = 28000;
SetSpeed(motor_pwm_out[0]);          // 仍输出 4200
```

需要十几个周期（数百 ms）才能把累加器拉回 ±4200 区间，期间控制器对反向偏差完全无响应。

#### 推荐修复：累加器一并限幅（anti-windup clamp）

```c
motor_pwm_out[i] += ax_motor_kp * bias + ax_motor_kd * (bias - bias_last);
if (motor_pwm_out[i] >  4200) motor_pwm_out[i] =  4200;
if (motor_pwm_out[i] < -4200) motor_pwm_out[i] = -4200;
return motor_pwm_out[i];
```

累加器与输出同步钳位，既消除溢出，也避免 windup。

### 2.6 完整闭环示意

```
   /cmd_vel ─→ 限幅(mm/s) ─→ 逆解 ─→ 目标轮速 (m/s)
                                          │
        ┌─────────────────────────────────┘
        ▼
   ┌─────────┐    ┌──────────┐   ┌──────────┐
   │ e=Tg-Rt │ → │ 增量式速度控制器 │ → │ 限幅 ±4200 │ → PWM → H 桥 → 电机
   └────┬────┘    └──────────┘   └──────────┘                │
        │ Rt                                                 │
        └────────────── 编码器 × WHEEL_SCALE ←───── 减速器 ←─┘
```

---

## 三、IMU 姿态估计

> 当前固件**只在 STM32 上做：读 raw IMU → 零偏校准 → 坐标系变换 → 上传**。
> 姿态融合（互补/Mahony）在 ROS 上位机完成。下面 3.4 是把 Mahony 下放到 MCU 的可选方案。

### 3.1 硬件与配置（`Driver/ax_imu.c`，`ax_imu.c:48-80`）

| 项目 | 参数 |
|------|------|
| IMU | QMI8658A（6 轴：加速度计 + 陀螺仪） |
| 接口 | 软件模拟 I²C（`PB6=SCL`，`PB7=SDA`） |
| 设备地址 | 0x6A |
| 加速计 | 量程 ±4 G，ODR 500 Hz，内建 LPF `LSPA_MODE_3` |
| 陀螺仪 | 量程 ±512 °/s，ODR 500 Hz，内建 LPF `LSPG_MODE_3` |
| 自检 | 启动时执行 `CTRL9_CMD_ONDEMANDCALIVRATION`，自带零偏自检 |

> Raw 单位换算：`acc_g = raw / 32768 × 4`，`gyro_dps = raw / 32768 × 512`。

### 3.2 STM32 端流程（`ax_robot.c:143-169`）

```c
void ROBOT_IMUHandle(void)
{
    AX_IMU_GetAccData(ax_imu_acc_data);

    // 加速度计：IMU 坐标系 → ROS REP-103 坐标系
    R_Imu.ACC_X =  ax_imu_acc_data[1];   // ROS_X ←  IMU_Y
    R_Imu.ACC_Y = -ax_imu_acc_data[0];   // ROS_Y ← -IMU_X
    R_Imu.ACC_Z =  ax_imu_acc_data[2];   // ROS_Z ←  IMU_Z

    AX_IMU_GetGyroData(ax_imu_gyro_data);

    // 静态零偏补偿（offset 在 Trivia_Task 校准时存为负值）
    ax_imu_gyro_data[0] += ax_imu_gyro_offset[0];
    ax_imu_gyro_data[1] += ax_imu_gyro_offset[1];
    ax_imu_gyro_data[2] += ax_imu_gyro_offset[2];

    R_Imu.GYRO_X =  ax_imu_gyro_data[1];
    R_Imu.GYRO_Y = -ax_imu_gyro_data[0];
    R_Imu.GYRO_Z =  ax_imu_gyro_data[2];
}
```

### 3.3 陀螺仪零偏校准（`ax_robot.c:317-366`，Trivia_Task）

校准触发后（按键长按或上位机指令置 `ax_imu_calibrate_flag=1`）：

```c
ax_imu_gyro_offset[0..2] = 0;      // 清零旧偏置
vTaskSuspend(Robot_Task_Handle);    // 暂停主控制以保证静止采样

for (i = 0; i < 10; i++) {          // 10 次平均
    vTaskDelay(20);
    AX_IMU_GetGyroData(gyro_data);
    ax_imu_gyro_offset[0] += gyro_data[0];
    ax_imu_gyro_offset[1] += gyro_data[1];
    ax_imu_gyro_offset[2] += gyro_data[2];
}
ax_imu_gyro_offset[0] = -ax_imu_gyro_offset[0]/10;  // 取负 → 后面 +offset 即抵消零偏
ax_imu_gyro_offset[1] = -ax_imu_gyro_offset[1]/10;
ax_imu_gyro_offset[2] = -ax_imu_gyro_offset[2]/10;

vTaskResume(Robot_Task_Handle);
```

> 局限：只校准静态零偏；温漂、scale-factor、轴间耦合都没补偿。要更精确的姿态请上位机做。

### 3.4 数据上行（`ax_robot.c:177-220`）

每个 50 Hz 周期 `ROBOT_SendDataToRos()` 将 20 字节打包同时发到 CAN 与两路 UART：

| 通道 | ID | 长度 | 内容 |
|------|----|------|------|
| CAN | `0x0121` | 6 | ACC_X/Y/Z（高低字节，big-endian） |
| CAN | `0x0122` | 6 | GYRO_X/Y/Z |
| CAN | `0x0123` | 8 | RT_IX、RT_IY、RT_IW（mm/s, mrad/s）+ 电池电压 |
| UART2/UART4 | `ID_UTX_DATA = 0x10` | 20 | X-Protocol 帧，相同 20 字节 |

> 注意上传的是 **raw IMU + 整数化里程计**，不带四元数；姿态融合在上位机。

### 3.5 上位机姿态融合（说明）

ROS 端读取 raw IMU 后通常用互补滤波或 Mahony 计算 `sensor_msgs/Imu` 的四元数；
也可以由 `robot_pose_ekf` / `robot_localization` 把里程计 + IMU 融合成 `/odom`。

> 当前 STM32 工程**没有**与该 ROS 节点同仓的源码；ROS 端实现以你实际跑的节点为准。

---

## 四、Mahony 滤波（如需下放到 MCU）

### 4.1 算法流程

```
    加速度计 (ax,ay,az)          陀螺仪 (gx,gy,gz)
           │                          │
           ▼                          ▼
    归一化到单位向量            减去零偏
           │                          │
           ▼                          │
    ┌──────────────────┐              │
    │ 用四元数预测重力   │              │
    │ v = q ⊗ (0,0,1)  │              │
    └────────┬─────────┘              │
             │                        │
             ▼                        │
    ┌──────────────────┐              │
    │ 叉积误差:         │              │
    │ e = a × v        │              │
    └────────┬─────────┘              │
             │                        │
             ▼                        │
    ┌──────────────────┐              │
    │ PI补偿器:         │              │
    │ ω' = ω + Kp·e +  │◄─────────────┘
    │       Ki·∫e·dt   │
    └────────┬─────────┘
             │ 修正后的角速度 ω'
             ▼
    ┌──────────────────┐
    │ 四元数微分方程:    │
    │ q̇ = 0.5·q ⊗ ω'   │
    │ (一阶龙格库塔积分)  │
    └────────┬─────────┘
             │
             ▼
     ┌───────────────┐
     │ 四元数归一化    │
     │ q = q / ||q||  │
     └───────┬───────┘
             │
             ▼
      q0,q1,q2,q3 (最终姿态)
```

### 4.2 核心理念

Mahony 是**显式互补滤波器**——不是卡尔曼滤波，没有协方差，也不需要统计模型，计算量极轻。

- 陀螺仪：高频准、短时积分准，但低频会零偏漂移
- 加速度计：静态时直接提供重力方向（低频准），但运动加速度和振动会污染高频
- 用 `e = a × v`（实测重力 × 当前姿态预测重力）作为姿态误差
- PI 反馈这个 e 去修正陀螺仪角速度，使四元数被牵引向加速度计观测的"真下方"

三个公式合在一起：

```
e   = a × v
ω'  = ω + Kp · e + Ki · ∫e dt
q̇  = 0.5 · q ⊗ ω'
q  := q + q̇ · Δt;  q := q / ||q||
```

### 4.3 PI 增益调参经验

| 参数 | 调大 | 调小 |
|------|------|------|
| Kp | 收敛快，但振动噪声更易注入姿态 | 平滑但漂移大 |
| Ki | 抵消零偏快，但易过冲、可能低频震荡 | 静差消除慢 |

工程起点：`Kp = 1.0`，`Ki = 0.0`。先确认收敛后再加 `Ki = 0.005~0.02` 抵消残余零偏。

> Yaw 仅靠 acc + gyro 不可观测——必须加磁力计或外部航向参考（视觉/编码器/SLAM）才能稳住，否则只能短期估计。

### 4.4 四元数 → 欧拉角

```
Roll  φ = atan2(2(q0 q1 + q2 q3), 1 − 2(q1² + q2²))
Pitch θ = asin (2(q0 q2 − q1 q3))
Yaw   ψ = atan2(2(q0 q3 + q1 q2), 1 − 2(q2² + q3²))
```

注意 Pitch 的 `asin` 在 ±90° 附近退化为万向锁；如果云台或机械臂会越过这个角度，直接用四元数运算更稳。

---

## 五、完整系统数据流

```
ROS 上位机
  /cmd_vel (Twist) ──CAN 0x0181 / X-Protocol──▶ 目标 Vx, Vy, ωz
                                        │
                                        ▼
STM32F407 OpenCTR H60   Robot_Task @ 50 Hz   (Robot/ax_robot.c)
┌────────────────────────────────────────────────────────────┐
│ ① 4×Encoder ──→ 正解（ax_kinematics.c:43-58）→ RT_IX/IY/IW │
│ ② 上行 Tg 速度限幅（±VX/VY/VW，ax_kinematics.c:61-66）       │
│ ③ 逆解矩阵 ──→ 4 轮目标 m/s（ax_kinematics.c:74-77）         │
│ ④ 增量式速度控制器（ax_speed.c:37-125）→ motor_pwm_out[]            │
│ ⑤ AX_MOTOR_x_SetSpeed → TIM1/TIM9/TIM12 → H 桥 → 4 路直流电机│
│                                                              │
│ ⑥ QMI8658A I²C 读 ACC/GYRO（ax_imu.c）                      │
│ ⑦ 零偏抵消 + 坐标系映射 → R_Imu（ax_robot.c:143-169）        │
│ ⑧ CAN 0x0121/0x0122/0x0123 + UART2/UART4 → 上传 raw IMU + 里程计│
└────────────────────────────────────────────────────────────┘
                                        │
                                        ▼
ROS 上位机
  ⑨ 解析 raw IMU → 互补 / Mahony → sensor_msgs/Imu
  ⑩ 里程计 + IMU 融合 → /odom（robot_localization 等可选）
```

---

## 六、关键源码索引

| 功能 | 文件 | 关键符号 |
|------|------|----------|
| 主任务循环 | `Robot/ax_robot.c` | `Robot_Task` / `ROBOT_IMUHandle` / `ROBOT_SendDataToRos` |
| 运动学解算 | `Robot/ax_kinematics.c` | `AX_ROBOT_Kinematics`（含 6 种车型） |
| 速度 PD | `Robot/ax_speed.c` | `AX_SPEED_PidCtlA~D` / `motor_pwm_out[]` |
| 电机 PWM / H 桥 | `Driver/ax_motor.c` | `AX_MOTOR_AB_Init` / `AX_MOTOR_CD_Init` / `AX_MOTOR_*_SetSpeed` |
| 编码器 | `Driver/ax_encoder.c` | `AX_ENCODER_*_GetCounter` / `SetCounter` |
| IMU 驱动 | `Driver/ax_imu.c` | `AX_IMU_Init/ConfigAcc/ConfigGyro/GetAccData/GetGyroData` |
| CAN | `Driver/ax_can.c` | `AX_CAN_Init/SendPacket`（ID `0x0121-0x0123`、`0x0181`） |
| UART | `Driver/ax_uart{2,4}.c` | `AX_UART{2,4}_SendPacket` |
| 全局参数 / 车型宏 | `Robot/ax_robot.h` | `ROBOT_TYPE` / `MEC_*` / `R_V*_LIMIT` / `PID_RATE` |
| 系统启动 | `Robot/main.c` | `main` / `Start_Task` |

---

*TARKBOT R750 / OpenCTR H60 V3.61.0602*
