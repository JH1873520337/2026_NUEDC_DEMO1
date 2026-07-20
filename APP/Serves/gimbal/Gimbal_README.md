# Gimbal 二维云台驱动

## 硬件映射

二维云台由两个 DS3115 舵机组成：

| 轴 | 舵机 | PWM 通道 | 舵机机械角 | 云台软件角 |
|---|---|---|---|---|
| Yaw | 舵机 1 | TIM1_CH3（PE13） | 0~180° | -90~90° |
| Pitch | 舵机 2 | TIM1_CH4（PE14） | 0~180° | -90~90° |

软件角 `0°` 对应舵机机械角 `90°`。云台层不直接操作 TIM，而是统一调用 `Servo` BSP。

## 初始化

```c
#include "Gimbal.h"

if (Gimbal_Init() != GIMBAL_STATUS_OK) {
    Error_Handler();
}
```

`Gimbal_Init()` 会调用 `Servo_Init()`，使两个舵机位于机械角 90°，随后将 Yaw、Pitch 的软件当前位置和目标位置都记录为 0°。

## 周期更新

控制接口只设置目标，实际运动由非阻塞函数 `Gimbal_Update()` 推进：

```c
for (;;) {
    Gimbal_Update();
    osDelay(10);
}
```

当前工程由 `freertos.c` 中的 `gimbal_update` 任务每 10 ms 调用一次。每次更新使用同一个缩放系数计算两轴步进，因此保持二维运动方向，同时保证任意一个舵机单次指令变化均不超过 2°。`Gimbal_Update()` 已在内部调用 `Servo_Update()`，使用云台层时不要在同一周期再次调用 `Servo_Update()`。

## 绝对位置控制

```c
Gimbal_SetAbsolute(30.0f, -15.0f);
```

目标表示相对于初始化正对方向的软件角度。超出 -90~90° 的输入会自动限制到安全范围。

## 相对位置控制

```c
Gimbal_SetRelative(5.0f, -2.0f);
```

相对量以调用瞬间结构体记录的当前位置为基准，而不是以上一次目标位置为基准。这样在运动过程中下发新命令时，不会把尚未完成的旧目标重复累计。

## 向量控制

```c
Gimbal_SetVector(2.0f, 1.0f);
```

- `X` 映射到 Yaw，`Y` 映射到 Pitch。
- `(X, Y)` 表示云台初始正对方向垂直平面上的运动方向；向量长度不表示终点距离。
- 驱动保持 `X:Y` 的方向比例向前运动，并预先计算沿该方向首先到达的机械边界。
- 当任意一轴到达 `-90°` 或 `90°` 时，两轴同时停止，状态中的 `limit_reached` 置 1。
- `(0, 0)` 没有方向，会立即停止并返回 `GIMBAL_STATUS_INVALID_ARGUMENT`。

## 立即停止

```c
Gimbal_Stop();
```

停止函数会立即把两轴目标锁定为当前 PWM 软件角度，并继续输出 PWM 以保持当前位置。由于系统没有舵机位置反馈，结构体中的当前位置是当前发送给舵机的角度，不是编码器实测角度。

## 暂停、继续与复位续行

```c
Gimbal_Pause();
Gimbal_Start();
Gimbal_ResetToOrigin();
```

- `Gimbal_Pause()` 保存当前最终目标并保持当前位置，等待启动。
- `Gimbal_Start()` 从暂停位置继续执行保存的最终目标。
- `Gimbal_ResetToOrigin()` 保存当前最终目标，运动到云台原点 `(0,0)` 后保持等待。
- 原点等待时调用 `Gimbal_Start()` 才会继续复位前的原目标。
- 复位途中也可以暂停；第一次启动先继续回原点，到达后再次等待启动。
- `Gimbal_Resume()` 保留为 `Gimbal_Start()` 的兼容别名。
- 新的绝对、相对、向量命令以及 `Gimbal_Stop()` 会取消尚未完成的暂停或复位恢复上下文。

## 状态读取

```c
const Gimbal_State_t *state = Gimbal_GetState();

float yaw = state->current_yaw_deg;
float pitch = state->current_pitch_deg;
uint8_t moving = state->moving;
```

`Gimbal_State_t` 同时保存当前位置、最终目标、当前分段指令、控制模式、原始向量、限位状态和底层驱动故障状态。

## 线程安全

所有接口默认从同一个 RTOS 任务调用。如果多个任务都需要控制云台，应在上层使用互斥锁或消息队列串行化命令。
