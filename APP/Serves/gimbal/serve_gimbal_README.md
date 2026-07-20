# 二维舵机云台服务

`serve_gimbal` 位于应用服务层，用于管理二维云台的水平轴和俯仰轴。模块负责安全角度范围、
运动参数、目标位置、周期调度和运行状态，不直接操作 STM32 定时器或 GPIO。

底层 PWM 输出和平滑运动由 `APP/BSP/servo/Servo.c` 提供：

```text
Task
  |
  v
serve_gimbal
  |
  v
Servo BSP -> TIM1 PWM
```

## 文件说明

| 文件 | 用途 |
| --- | --- |
| `serve_gimbal.h` | 公共类型、配置结构和控制接口 |
| `serve_gimbal.c` | 云台状态、参数检查和底层舵机调用 |

## 默认参数

调用 `Serve_Gimbal_Init(NULL)` 时使用以下默认参数：

| 参数 | 默认值 | 单位 |
| --- | ---: | --- |
| Pan 安全范围 | 0~180 | 度 |
| Tilt 安全范围 | 0~180 | 度 |
| Pan 中位 | 90 | 度 |
| Tilt 中位 | 90 | 度 |
| 最大速度 | 60 | 度/秒 |
| 加速度 | 120 | 度/秒平方 |
| 处理周期 | 10 | ms |

目标角超过配置的安全范围时，接口返回 `SERVE_GIMBAL_INVALID_ARGUMENT`，不会自动限幅。

## 初始化

使用云台服务前，必须先完成 TIM1 和相关 GPIO 的 CubeMX 初始化：

```c
#include "serve_gimbal.h"

MX_TIM1_Init();

if (Serve_Gimbal_Init(NULL) != SERVE_GIMBAL_OK) {
    Error_Handler();
}
```

初始化会调用底层 `Servo_Init()` 启动两路 PWM，并将云台移动到配置的中位角。

## 自定义配置

机械结构不允许舵机转动到完整的 0~180 度时，应设置更小的安全范围：

```c
static const Serve_Gimbal_Config_t gimbal_config = {
    .pan_min_angle = 30.0f,
    .pan_max_angle = 150.0f,
    .tilt_min_angle = 45.0f,
    .tilt_max_angle = 135.0f,
    .pan_center_angle = 90.0f,
    .tilt_center_angle = 90.0f,
    .max_speed = 60.0f,
    .acceleration = 120.0f,
    .process_period_ms = 10U,
};

if (Serve_Gimbal_Init(&gimbal_config) != SERVE_GIMBAL_OK) {
    Error_Handler();
}
```

最小角必须小于最大角，中位角必须位于安全范围内，速度、加速度和处理周期必须大于 0。

## 周期处理

任务需要持续调用 `Serve_Gimbal_Process()`。函数内部按照 `process_period_ms` 控制实际处理频率，
不会在调用间隔不足时重复更新底层舵机。

```c
uint32_t wake_tick = osKernelGetTickCount();

for (;;) {
    Serve_Gimbal_Process(HAL_GetTick());
    wake_tick += 10U;
    osDelayUntil(wake_tick);
}
```

`now_ms` 使用无符号时间差计算，允许 `HAL_GetTick()` 正常溢出回绕。该接口不阻塞，默认只在任务
上下文调用，不要从中断服务函数调用。

## 设置目标角

平滑移动到指定角度：

```c
Serve_Gimbal_Status_t status;

status = Serve_Gimbal_SetTarget(45.0f, 120.0f);
if (status != SERVE_GIMBAL_OK) {
    /* 处理未初始化或角度越界 */
}
```

直接设置角度，不经过平滑运动过程：

```c
Serve_Gimbal_SetTargetDirect(90.0f, 90.0f);
```

直接控制适合初始化、校准和调试，不建议在带负载高速运动时频繁调用。

## 运动参数

```c
Serve_Gimbal_SetMotionProfile(80.0f, 160.0f);
Serve_Gimbal_SetTarget(60.0f, 110.0f);
```

`max_speed` 单位为度/秒，`acceleration` 单位为度/秒平方。新参数用于之后下发的平滑目标，
不会重新计算已经开始执行的运动。

## 回中与停止

```c
Serve_Gimbal_Center();
Serve_Gimbal_Stop();
```

- `Serve_Gimbal_Center()`：使用当前运动参数平滑回到配置的中位角。
- `Serve_Gimbal_Stop()`：停止两轴当前运动，并保持当前 PWM 输出位置。

## 获取状态

```c
Serve_Gimbal_State_t state;

if (Serve_Gimbal_GetState(&state) == SERVE_GIMBAL_OK) {
    if (!state.is_moving) {
        /* 两个轴都已停止运动 */
    }
}
```

状态包括当前 Pan/Tilt 角度、目标角度和运动标志。当前角度来自底层驱动的开环指令状态，
不代表编码器或其他传感器测得的真实机械角度。

## 返回状态

| 返回值 | 含义 |
| --- | --- |
| `SERVE_GIMBAL_OK` | 操作成功 |
| `SERVE_GIMBAL_ERROR` | 底层舵机初始化失败 |
| `SERVE_GIMBAL_INVALID_ARGUMENT` | 配置、目标角度或输出指针无效 |
| `SERVE_GIMBAL_NOT_INITIALIZED` | 尚未调用 `Serve_Gimbal_Init()` |

## 使用注意

- 本模块没有内部锁，所有接口应由同一个任务调用；多任务访问时由上层增加互斥保护。
- 首次调试应缩小安全角度范围，确认方向和机械限位后再扩大范围。
- 舵机建议使用独立 5 V 电源，并确保舵机电源与 STM32 共地。
- 若云台出现抖动，应先检查供电、机械间隙、PWM 周期和底层舵机脉宽范围。
