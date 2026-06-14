# 技术笔记 — FreeRTOS 架构（TARKBOT R750 STM32 底盘固件）

> 源码：`2 STM32机器人底盘源码和固件/源码文件/OpenCTR_H60V36_R750ROS_V3.61.0602/`
> MCU：STM32F407VET6（168 MHz, Cortex-M4F），固件版本 `V3.61`
> RTOS：FreeRTOS（抢占式调度，1 ms tick）

## 0. 一句话理解

**裸机 `main()` 只做硬件初始化 → 创建 `Start_Task` → 启动调度器**；之后所有业务逻辑都在 6 个 FreeRTOS 任务里跑，靠**全局结构体（`R_Vel`/`R_Wheel_*`/`R_Imu`/`R_Light`）**共享数据，靠**优先级 + `vTaskDelay/vTaskDelayUntil`**实现协作多任务。`Robot_Task` 是机器人的"心跳"——50 Hz 周期把 `cmd_vel → 轮速PID → PWM` 跑一遍，再把 `IMU + 里程计 + 电压` 打包回传给 ROS。

## 1. FreeRTOS 配置（`Robot/FreeRTOSConfig.h`）

| 配置项                      | 值                  | 含义 / 影响                                           |
| --------------------------- | ------------------- | ----------------------------------------------------- |
| `configUSE_PREEMPTION`      | `1`                 | 抢占式调度。高优先级就绪即抢占低优先级                |
| `configCPU_CLOCK_HZ`        | `SystemCoreClock`   | 168 MHz                                               |
| `configTICK_RATE_HZ`        | `1000`              | 系统节拍 1 kHz，`vTaskDelay(1)` ≈ 1 ms                |
| `configMAX_PRIORITIES`      | `5`                 | **合法优先级 0~4**。⚠️ 见下文"优先级陷阱"             |
| `configMINIMAL_STACK_SIZE`  | `130` words (=520B) | 单位是 word（4 字节），不是字节                       |
| `configTOTAL_HEAP_SIZE`     | `75 KB`             | FreeRTOS 内部堆，所有 `xTaskCreate` 的 TCB+栈都从这分 |
| `configUSE_MUTEXES`         | `1`                 | 启用互斥量（本工程未实际使用）                        |
| `configUSE_TIMERS`          | `1`                 | 启用软件定时器                                        |
| `configTIMER_TASK_STACK_DEPTH` | `260 words`      | 定时器服务任务栈                                      |

**SysTick** 由 FreeRTOS 接管做 tick；`stm32f4xx_it.c` 里 `SysTick_Handler / PendSV_Handler / SVC_Handler` 都重定向到 FreeRTOS port 层。

## 2. 启动流程（`Robot/main.c`）

```
上电
 │
 ▼
main()  ← 仍处于裸机环境（无任务、无调度）
 │
 ├─ NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4)   // 全部 4 位抢占优先级，0 位子优先级（FreeRTOS 推荐）
 │
 ├─ 外设初始化（按依赖顺序，无 RTOS API 调用）
 │   舵机/电机/延时/LED/按键/电压/蜂鸣器/EEPROM
 │   UART1(230400 调试) / UART2(230400 ROS-USB) / UART3(115200 蓝牙) / UART4(230400 ROS-TTL)
 │   SBUS / CAN / 4路编码器 / RGB / OLED / IMU(QMI8658A) / USB Host(手柄)
 │
 ├─ xTaskCreate(Start_Task, prio=1, stack=128)       // 唯一一个用 main 创建的任务
 │
 └─ vTaskStartScheduler()    ← 进入 RTOS，main() 不再返回

──────────── RTOS 调度开始 ────────────

Start_Task（一次性初始化任务）:
 │
 ├─ 读 EEPROM[0x00] 是否为 0x88（初始化标志）
 │     ├─ 是：从 EEPROM 加载 RGB 灯效参数 + AKM 舵机零偏（地址 0x20~0x25, 0x50）
 │     └─ 否：写入默认参数 + 标志位（首次烧录恢复出厂设置）
 │
 ├─ 舵机回中位（AKM 车型用，其他车型这步无副作用）
 │
 ├─ 陀螺仪零偏校准：采 10 次 → 取平均 → 取负 → 存到 ax_imu_gyro_offset[3]
 │   ⚠️ 校准期间机器人必须静止，否则 yaw 会持续漂移
 │
 ├─ OLED 显示首页（按 ROBOT_TYPE 宏切换 MEC/FWD/AKM/TWD/TNK/OMT 三字标识）
 │
 ├─ taskENTER_CRITICAL()                            // 进入临界区，避免创建过程中被新任务抢占
 │   xTaskCreate(Robot_Task,  prio=3,  stack=256)
 │   xTaskCreate(Key_Task,    prio=4,  stack=128)
 │   xTaskCreate(Light_Task,  prio=5,  stack=128)
 │   xTaskCreate(Disp_Task,   prio=6,  stack=128)   // ⚠️ >MAX_PRIORITIES-1，被截断到 4
 │   xTaskCreate(Trivia_Task, prio=10, stack=128)   // ⚠️ 同上，被截断到 4
 │
 ├─ vTaskDelete(StartTask_Handler)                  // 自删除：栈与 TCB 在 idle task 中回收
 └─ taskEXIT_CRITICAL()
```

> **优先级陷阱**：`configMAX_PRIORITIES = 5`，合法范围 `0~4`。源码里 Disp_Task=6、Trivia_Task=10 都越界，FreeRTOS 内部 `xTaskCreate` 会用 `configASSERT` 检查（debug 时触发断言），release 下会被静默截断到 `configMAX_PRIORITIES-1 = 4`。**实际效果**：Key_Task / Light_Task / Disp_Task / Trivia_Task **都跑在优先级 4**，谁先就绪谁先跑，时间片轮转（如果 `configUSE_TIME_SLICING` 默认开）。要修正：把 `configMAX_PRIORITIES` 改成 11，或把任务优先级降到 0~4 范围。


## 3. 任务全景表

| 任务         | 优先级（声明/实际） | 栈   | 周期           | 职责                                                                                       | 关键 API                                                            |
| ------------ | ------------------- | ---- | -------------- | ------------------------------------------------------------------------------------------ | ------------------------------------------------------------------- |
| `Start_Task` | 1 / 1               | 128  | 一次性         | 加载 EEPROM 参数；陀螺仪零偏校准；创建其他任务；自删除                                     | `xTaskCreate`, `taskENTER_CRITICAL`, `vTaskDelete`                  |
| `Robot_Task` | 3 / 3               | 256  | **20 ms 严格** | 解析控制源 → 运动学解算 → IMU 采集 → ROS/USB/TTL/CAN 数据回传                              | `vTaskDelayUntil`（关键！周期不漂）                                 |
| `Key_Task`   | 4 / 4               | 128  | 50 ms 轮询     | 短按切换灯效；中按（3~10s）AKM 舵机零偏校准；长按（>10s）恢复出厂                          | `vTaskDelay`                                                        |
| `Light_Task` | 5 / **4**           | 128  | 30 ms (≈33Hz)  | 调用 `AX_LIGHT_Show()` 刷新 RGB 灯带                                                       | `vTaskDelay`                                                        |
| `Disp_Task`  | 6 / **4**           | 128  | 200 ms         | OLED 刷新：控制源、电压、yaw 角速度、4 轮实时速度                                          | `vTaskDelay`                                                        |
| `Trivia_Task`| 10 / **4**          | 128  | 500 ms         | 电池监控（40%/20%/10% 三级告警，10% 持续 10 次→挂起 Robot/Disp 进入低功耗保护）；IMU 重新校准；EEPROM 灯效保存；蜂鸣器异步响 | `vTaskSuspend`, `vTaskResume`, `vTaskDelay`                         |

**任务句柄全是全局变量**（`Robot_Task_Handle` 等），方便 `Trivia_Task` 在低电压/校准时挂起 / 恢复 `Robot_Task`。


## 4. `Robot_Task` —— 心跳循环详解

```c
void Robot_Task(void* parameter)
{
    static portTickType PreviousWakeTime;
    const  portTickType TimeIncrement = pdMS_TO_TICKS(20);  // 20 ms
    PreviousWakeTime = xTaskGetTickCount();

    while(1)
    {
        vTaskDelayUntil(&PreviousWakeTime, TimeIncrement);   // ① 严格 50 Hz

        // ② 控制源选择（ax_control_mode 是 UART RX 中断里写的）
        if (ax_control_mode == CTL_GPD)  AX_CTL_Gpd();        // 手柄
        else if (ax_control_mode == CTL_APP) AX_CTL_App();    // 蓝牙 APP
        else if (ax_control_mode == CTL_RMS) AX_CTL_RemoteSbus(); // SBUS 遥控
        // CTL_ROS（默认 0）由 UART2/UART4 RX ISR 直接写入 R_Vel.TG_IX/IY/IW，无需轮询

        // ③ 运动学 + PID
        if (ax_robot_move_enable)
            AX_ROBOT_Kinematics();   // 编码器→轮速→底盘速度（正向） + 目标→PID→PWM（逆向）
        else
            AX_ROBOT_Stop();

        // ④ IMU
        ROBOT_IMUHandle();
            // 读 acc/gyro，应用零偏，IMU坐标系 → ROS坐标系
            // ROS X = IMU Y;  ROS Y = -IMU X;  ROS Z = IMU Z

        // ⑤ 上报
        ROBOT_SendDataToRos();
            // 20 字节包：6B acc + 6B gyro + 6B 底盘速度 + 2B 电压
            // 同时发 UART2 / UART4 / CAN（3 帧）
    }
}
```

### 4.1 为什么用 `vTaskDelayUntil` 而不是 `vTaskDelay`

- `vTaskDelay(20)` = "现在起睡 20 ms"，循环周期 = `20ms + 任务体执行时间`，**会漂**。
- `vTaskDelayUntil(&prev, 20)` = "下次唤醒在 `prev+20ms`"，**周期严格 20 ms**，里程计积分才不会带累积误差。
- ROS 上层 `tarkbot_robot.cpp` 默认按 50 Hz 同步处理，下位机必须严格周期，否则 IMU/odom 时间戳和实际不匹配，AMCL/EKF 会发散。

### 4.2 共享数据流（不用队列/信号量，全靠全局结构体）

```
                    ┌─────────────── UART2 / UART4 RX ISR ────────────────┐
                    │  cmd_vel 数据包 → R_Vel.TG_IX / TG_IY / TG_IW       │
ROS cmd_vel ────────┤  灯效指令     → R_Light.M/S/T/R/G/B               │
                    │  IMU 校准请求 → ax_imu_calibrate_flag = 1           │
                    │  车型查询     → 回 ROBOT_TYPE                        │
                    └─────────────────────────────────────────────────────┘
                                          │ 写
                                          ▼
                            ┌───── 全局变量 ─────┐
                            │  R_Vel             │ ← Robot_Task  读 TG，算 RT，写 RT 回结构体
                            │  R_Wheel_A/B/C/D   │ ← Robot_Task  读编码器，写 RT/TG/PWM
                            │  R_Imu             │ ← Robot_Task  读 IMU，写 ACC/GYRO
                            │  R_Light           │ ← Light_Task  读，刷 RGB
                            │  R_Bat_Vol         │ ← Trivia_Task 写
                            │  ax_control_mode   │ ← UART RX 写，Robot_Task 读
                            │  ax_*_flag         │ ← Trivia_Task 处理跨任务请求
                            └────────────────────┘
                                          │ 读
                                          ▼
                            打包 20B → UART2 / UART4 / CAN ──→ ROS
```

> **没有锁**。安全性靠两点：
> 1. `R_Vel.TG_*` 是 16 位整数，STM32F4 上对齐的 16/32 位读写是原子的；
> 2. 写入方（ISR）和读取方（Robot_Task @ 50Hz）频率差很大，ABA / 撕裂风险极低。
>
> 但 `R_Imu` 是被 Robot_Task 自己读写、且只在该任务内消费 → 单线程访问，天然安全。


## 5. 全局结构体（`Robot/ax_robot.h`）

```c
typedef struct {
    double  RT;     // 实时轮速 m/s（编码器反算）
    float   TG;     // 目标轮速 m/s（运动学正解给的）
    short   PWM;    // PID 输出 PWM
} ROBOT_Wheel;       // R_Wheel_A / _B / _C / _D

typedef struct {
    short   RT_IX, RT_IY, RT_IW;   // 实时底盘速度（int16，单位 mm/s 或 mrad/s）
    short   TG_IX, TG_IY, TG_IW;   // ROS 下发的目标底盘速度（int16，×1000 后传输）
    float   RT_FX, RT_FY, RT_FW;   // 同上，浮点版
    float   TG_FX, TG_FY, TG_FW;
} ROBOT_Velocity;    // R_Vel

typedef struct {
    short   ACC_X, ACC_Y, ACC_Z;   // 已转到 ROS 坐标系
    short   GYRO_X, GYRO_Y, GYRO_Z;
} ROBOT_Imu;         // R_Imu

typedef struct {
    uint8_t M, S, T;               // 灯效模式 / 子模式 / 时间参数
    uint8_t R, G, B;               // 颜色
} ROBOT_Light;       // R_Light
```

整型 / 浮点双份是为了**通信用整型（带宽小、定点）**、**运算用浮点（精度高）**。

## 6. 通信协议（X-Protocol）

帧头由 `ax_robot.h` 中的 `ID_*` 宏定义：

| 方向 | ID                | 含义                             |
| ---- | ----------------- | -------------------------------- |
| TX   | `0x10 ID_UTX_DATA`| 综合数据（acc+gyro+vel+vol = 20B）|
| RX   | `0x50 ID_URX_VEL` | cmd_vel（TG_IX/IY/IW）           |
| RX   | `0x51 ID_URX_IMU` | IMU 零偏校准命令                 |
| RX   | `0x52 ID_URX_LG`  | 灯效全局开关                     |
| RX   | `0x53 ID_URX_LS`  | 灯效设置                         |
| RX   | `0x54 ID_URX_BP`  | 蜂鸣器                           |
| RX   | `0x5A ID_URX_RTY` | 查询机器人型号（回 `ROBOT_TYPE`）|
| CAN  | `0x0121/22/23`    | 同样 20B 数据，拆 6+6+8 三帧     |

UART2 (USB CDC) 与 UART4 (TTL) **完全镜像**，做冗余 —— ROS 主机可任选其一。

## 7. 跨任务协作的 4 种模式

1. **共享全局变量 + 周期轮询**（最常见）
   - 例：`Robot_Task` 周期读 `R_Vel.TG_*`，UART RX 异步写。
2. **任务挂起 / 恢复**
   - `Trivia_Task` 在 IMU 重新校准、EEPROM 写入、低电压保护时调 `vTaskSuspend(Robot_Task_Handle)` → 操作完 `vTaskResume()`。这是最重的同步手段，但避免了 PID 在校准期间用错误零偏。
3. **临界区**
   - `Start_Task` 创建子任务时 `taskENTER_CRITICAL()`，防止刚创建的任务立即抢占自己。
4. **请求标志位**
   - `ax_imu_calibrate_flag` / `ax_light_save_flag` / `ax_beep_ring`：发起方设 1，`Trivia_Task` 轮询并清零。简单但有最多 500 ms 延迟（Trivia 的循环周期）。


## 8. 中断与任务的交互

- **NVIC 优先级分组**：`NVIC_PriorityGroup_4` → 4 位抢占，0 位响应。FreeRTOS port 要求**不嵌入 RTOS API 的 ISR 优先级 < `configMAX_SYSCALL_INTERRUPT_PRIORITY`**（数值越小优先级越高）。
- **UART RX**：在 ISR 中直接解包写 `R_Vel.TG_*` 等全局变量。**没有用** `xQueueSendFromISR` —— 因为只是 16 位整数赋值，原子。
- **编码器**：定时器硬件计数，ISR 不参与；`AX_ROBOT_Kinematics()` 在任务里读 CNT 寄存器再清零。

## 9. 资源占用估算

```
任务         栈 (words → bytes)   TCB ~  消耗
─────────────────────────────────────────────────
Start_Task   128 → 512 B          92 B   604 B  （创建完即销毁）
Robot_Task   256 → 1024 B         92 B   1116 B
Key_Task     128 → 512 B          92 B   604 B
Light_Task   128 → 512 B          92 B   604 B
Disp_Task    128 → 512 B          92 B   604 B
Trivia_Task  128 → 512 B          92 B   604 B
Idle Task    130 → 520 B          92 B   612 B
Timer Task   260 → 1040 B         92 B   1132 B
─────────────────────────────────────────────────
                                          ≈ 6 KB
heap 总量：75 KB → 余量充裕，可加任务/缓冲区
```

## 10. 维护提示 / 常见坑

| 想做的事                           | 怎么改                                                                                                                |
| ---------------------------------- | --------------------------------------------------------------------------------------------------------------------- |
| 切换车型                           | 改 `ax_robot.h` 第 158 行 `ROBOT_TYPE` 宏，重新编译                                                                   |
| 改控制周期                         | `Robot_Task` 内的 `pdMS_TO_TICKS(20)`，**同时改 ROS `tarkbot_robot.cpp` 的发布频率**，否则 odom 时间戳不对           |
| 修复优先级越界                     | `FreeRTOSConfig.h` `configMAX_PRIORITIES` 改成 11；或把 Disp/Trivia 优先级改成 0~4 范围                              |
| 加新任务                           | 在 `Start_Task` 临界区里 `xTaskCreate`；优先级 ≤ 4；栈先给 128 words，跑起来用 `uxTaskGetStackHighWaterMark` 检查    |
| 加共享数据                         | 优先用全局结构体；如果是 32 位以上、跨任务读改写，加互斥量（`configUSE_MUTEXES` 已开）                                |
| 调试任务卡死                       | 打开 `configCHECK_FOR_STACK_OVERFLOW = 2`、`configUSE_MALLOC_FAILED_HOOK = 1`，串口打印 hook 信息                     |
| ROS 收不到数据                     | 先 `printf` 到 UART1（230400）确认下位机在跑；再用串口工具看 UART2/UART4 是否有 0x10 帧                              |

## 11. 与 ROS 端的对应关系

| STM32 端                      | ROS 端 (`tarkbot_robot.cpp`)              |
| ----------------------------- | ----------------------------------------- |
| `Robot_Task` 50 Hz            | `ros::Rate(50)` 主循环                    |
| `R_Vel.TG_*` ←= UART RX       | 订阅 `/cmd_vel` (geometry_msgs/Twist)     |
| `R_Vel.RT_*` →= UART TX       | 发布 `/odom` (nav_msgs/Odometry) + TF     |
| `R_Imu.*` →= UART TX          | 发布 `/imu` (sensor_msgs/Imu)，互补滤波算四元数 |
| `R_Bat_Vol` →= UART TX        | 发布 `/bat_vol` (std_msgs/Float32)        |
| `ID_URX_LS / LG`              | service `/light_set`                      |
| PID Kp/Kd（`ax_motor_kp/kd`） | rqt_reconfigure → `robot.cfg`             |

---

**参考源码定位**

- 任务定义 / 创建：`Robot/main.c:30~67`、`Robot/main.c:283~334`
- `Robot_Task` 主循环：`Robot/ax_robot.c:92~136`
- 6 种运动学：`Robot/ax_kinematics.c`（`#if ROBOT_TYPE == ...` 切换）
- PID：`Robot/ax_speed.c`
- UART RX 解包：`Driver/ax_uart2.c:160~230`、`Driver/ax_uart4.c:160~230`
- IMU 驱动：`Driver/ax_imu.c`（QMI8658A）
- FreeRTOS 配置：`Robot/FreeRTOSConfig.h`
