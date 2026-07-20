# 双轴舵机云台驱动

该驱动基于 STM32 HAL 和 TIM1 PWM，支持云台水平轴、俯仰轴的直接控制和平滑控制。

## 硬件配置

| 云台轴 | 引脚 | 定时器通道 | 驱动编号 |
| --- | --- | --- | --- |
| 水平轴 Pan | PE13 | TIM1_CH3 | `SERVO_PAN` |
| 俯仰轴 Tilt | PE14 | TIM1_CH4 | `SERVO_TILT` |

TIM1 使用 1 MHz 计数频率和 20000 计数周期，对应 50 Hz PWM。默认角度范围为
0~180 度，脉宽范围为 500~2500 us，上电初始化到 90 度中位。

## 初始化

`main.c` 中先执行 `MX_TIM1_Init()`，然后在任务中初始化舵机：

```c
if (Servo_Init() != HAL_OK) {
    Error_Handler();
}
```

## 直接控制

```c
Servo_SetAngle_Direct(SERVO_PAN, 60.0f);
Servo_SetAngle_Direct(SERVO_TILT, 120.0f);

/* 同时设置两个轴 */
Servo_SetGimbalAngle_Direct(90.0f, 90.0f);
```

所有角度在写入 PWM 前都会自动限制到配置范围内。不同型号舵机的安全角度和脉宽可以在
`Servo_Config` 中分别调整。

## 平滑控制

```c
Servo_SetGimbalAngle_Smooth(45.0f, 120.0f, 60.0f, 120.0f);

for (;;) {
    Servo_Loop_Process();
    osDelay(10);
}
```

`max_speed` 单位为度/秒，`accel` 单位为度/秒平方。平滑运动期间必须以较稳定的周期调用
`Servo_Loop_Process()`。

## 测试任务

`Task/servo_test.c` 会先等待 2 秒，然后依次测试水平轴、俯仰轴和双轴联动，运动范围限制在
45~135 度，结束后回到 90 度。当前 `StartDefaultTask()` 已调用 `servo_test_run()`。

恢复底盘测试时，将 `../../../Core/Src/freertos.c` 中的：

```c
servo_test_run();
```

改回：

```c
chassis_test_run();
```

并同步将头文件 `servo_test.h` 改回 `chassis_test.h`。

## 接线注意

- 舵机建议使用独立 5 V 电源，STM32 与舵机电源必须共地。
- 首次上电不要安装负载，确认两个轴的实际方向和机械行程后再调整角度范围。
- 某些舵机只允许 1000~2000 us，出现撞限位或抖动时应先缩小脉宽范围。
