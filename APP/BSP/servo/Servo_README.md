# Servo 舵机驱动

## 硬件参数

| 项目 | 配置 |
|---|---|
| 舵机型号 | DS3115 |
| PWM 定时器 | TIM1 |
| Yaw 舵机 | TIM1_CH3，PE13 |
| Pitch 舵机 | TIM1_CH4，PE14 |
| PWM 频率 | 50 Hz |
| TIM1 输入时钟 | 162 MHz |
| PSC | 179 |
| 计数频率 | 900 kHz |
| ARR | 17999 |
| 脉宽 | 500~2500 us |
| 机械角度 | 0~180° |
| 软件分辨率 | 0.1° |

由于计数器周期为 `1 / 900 kHz = 1.111... us`，500 us 和 2500 us 分别对应比较值 450 和 2250。整个角度范围共有 1800 个比较值，正好对应 0.1° 分辨率。

> 如果以后修改系统时钟、PSC 或 ARR，必须同步修改 `Servo.h` 中的 `SERVO_TIMER_COUNTER_HZ`，否则脉宽换算会错误。

## 滤波流程

每次调用 `Servo_Update()` 时，角度指令依次经过：

1. 一阶低通滤波；
2. 一维卡尔曼滤波；
3. 0.1° 量化；
4. 写入 TIM1 CCR3/CCR4。

默认一阶滤波系数和卡尔曼参数位于 `Servo.h`：

```c
SERVO_FIRST_ORDER_ALPHA
SERVO_KALMAN_PROCESS_NOISE
SERVO_KALMAN_MEASUREMENT_NOISE
```

## 初始化

必须先完成 `MX_TIM1_Init()`，再调用：

```c
if (Servo_Init() != HAL_OK) {
    Error_Handler();
}
```

初始化后两个舵机均输出 90°。`Servo_Init()` 是 PWM 启动兼容的：即使 `main.c` 已经调用过 `HAL_TIM_PWM_Start()`，也不会重复启动该通道。

## 安装方向反转

当前机械安装要求 Yaw、Pitch 两个舵机都反向，配置位于 `Servo.h`：

```c
#define SERVO_YAW_REVERSED   1U
#define SERVO_PITCH_REVERSED 1U
```

反向仅在最终 PWM 比较值换算时执行，即 `0° ↔ 180°`、`90°` 保持不变。上层仍使用正常的软件角和数学方向，因此不要在 `Gimbal.c` 中再次对角度取负。若以后改变安装方向，只需把相应宏改为 `0U`。

## 使用示例

```c
Servo_Init();
Servo_SetAngle(SERVO_YAW, 120.3f);
Servo_SetAngle(SERVO_PITCH, 65.0f);

/* 建议由 10 ms 周期任务持续调用 */
for (;;) {
    Servo_Update();
    osDelay(10);
}
```

`Servo_SetAngle()` 只更新目标值，不阻塞等待舵机完成运动。只有持续调用 `Servo_Update()`，滤波后的 PWM 才会逐步到达目标。

## 立即停止

```c
Servo_Stop(SERVO_YAW);
Servo_StopAll();
```

停止函数会立即把目标、低通状态和卡尔曼状态锁定到当前 PWM 指令角度，不关闭 PWM，因此舵机会保持当前位置。

## 注意事项

- DS3115 必须使用独立、足够电流的电源，舵机电源地与 MCU 地必须共地。
- 本驱动没有位置反馈，`current_angle_deg` 表示当前输出给舵机的 PWM 软件角度，并非编码器实测角度。
- 公共接口默认在任务上下文调用；若多个任务同时控制舵机，需要在上层增加互斥保护。
