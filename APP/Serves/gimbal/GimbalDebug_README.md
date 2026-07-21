# GimbalDebug（USART1 裸机调试接口）

## 1. 当前实现

本模块使用 **裸机主循环 + STM32 HAL 串口中断接收**，不创建 FreeRTOS 任务，也不启动 RTOS 内核。

- 串口：USART1
- TX：PA9
- RX：PA10
- 波特率：115200
- 数据格式：8 数据位、无校验、1 停止位、无流控
- 接收：`HAL_UART_Receive_IT()`，每次接收 1 字节
- IRQ：只调用 CubeMX/HAL 标准入口 `HAL_UART_IRQHandler(&huart1)`
- 发送：主循环中使用 `HAL_UART_Transmit()`
- 主循环周期：建议 5 ms

USART1 中断函数不再直接读取串口数据寄存器，也不再调用 GimbalDebug 自定义 IRQ 函数。HAL 收到一个字节后进入 `HAL_UART_RxCpltCallback()`；应用层将回调转交给 `GimbalDebug_OnUartRxComplete()`，由模块完成协议同步、命令入队并重新开启下一字节中断接收。中断回调中不会打印或驱动舵机。

## 2. 三字节命令格式

固定发送 3 个二进制字节：

`[轴][方向][角度绝对值 x10]`

| 字节 | 含义 | 合法值 |
|---|---|---|
| byte0 | 轴 | `0x59` = Yaw（ASCII Y），`0x50` = Pitch（ASCII P） |
| byte1 | 方向 | `0x00` = 负方向，`0x01` = 正方向 |
| byte2 | 角度绝对值的 10 倍 | `0x00~0xFF`，即 0.0°~25.5° |

示例：

- `50 00 05`：Pitch -0.5°
- `50 00 32`：Pitch -5.0°（0x32 = 十进制 50）
- `50 01 14`：Pitch +2.0°（0x14 = 十进制 20）
- `59 01 1E`：Yaw +3.0°（0x1E = 十进制 30）

**必须打开串口助手的 HEX/十六进制发送。** 如果按普通文本发送 `50 00 05`，实际发送的是 ASCII 字符，不是协议要求的三个字节。

## 3. 裸机接入方式

如果要让 `GimbalDebug` 模块独占 USART1 接收，在用户代码区按下面方式接入：

```c
MX_TIM1_Init();
MX_USART1_UART_Init();

/* Gimbal_Init 内部写入校准零位后再启动 TIM1_CH3/CH4。 */
if (Gimbal_Init() != GIMBAL_STATUS_OK)
{
    Error_Handler();
}

/* 启动 GimbalDebug 的 USART1 单字节中断接收。 */
if (GimbalDebug_Init() != HAL_OK)
{
    Error_Handler();
}

while (1)
{
    /* 内部已经调用 Gimbal_Update()，不要再次重复调用。 */
    GimbalDebug_Update();
    HAL_Delay(5);
}
```

还需要在项目已有的 HAL 回调中转交接收完成和错误事件：

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    GimbalDebug_OnUartRxComplete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    GimbalDebug_OnUartError(huart);
}
```

**USART1 同一时刻只能有一个中断接收拥有者。** 如果 `main.c` 已经使用自己的 `mess[]` 缓冲区调用：

```c
HAL_UART_Receive_IT(&huart1, mess, 3);
```

就不要再调用 `GimbalDebug_Init()`；否则第二次启动接收会返回 `HAL_BUSY`。应当在“自定义接收逻辑”和“GimbalDebug 协议模块”中二选一。

裸机方案不要调用 `osKernelInitialize()`、`MX_FREERTOS_Init()`、`osKernelStart()` 或 `osDelay()`。

## 4. USART1 标准 HAL 中断链

`Core/Src/stm32f4xx_it.c` 中保持 CubeMX 生成的标准入口：

```c
void USART1_IRQHandler(void)
{
    HAL_UART_IRQHandler(&huart1);
}
```

不要在这个函数里读取 USART1 数据寄存器，也不要在这里解析协议。完整接收链如下：

```text
HAL_UART_Receive_IT()
    -> USART1_IRQHandler()
    -> HAL_UART_IRQHandler(&huart1)
    -> HAL_UART_RxCpltCallback()
    -> GimbalDebug_OnUartRxComplete()
    -> HAL_UART_Receive_IT() 重新挂接下一字节
```

这样 USART1 的接收状态、错误标志和数据寄存器都只由 HAL 管理，不会提前取走本应交给 HAL 回调的数据。

## 5. 上电后应看到的返回

固件启动后立即输出：

```text
[Gimbal] USART1 ready (bare metal): HEX [Y/P][dir 0/1][angle x10]
[Gimbal] Yaw=0.0 deg, Pitch=0.0 deg
```

如果串口助手打开得较晚，在收到第一条完整有效命令之前，固件每 1 秒重复输出：

```text
[Gimbal] WAIT USART1 HEX: P-0.5 = 50 00 05
```

收到 `50 00 05` 后，应先返回 ACK：

```text
[Gimbal] RX: Pitch -0.5 deg
```

运动完成后再返回软件角：

```text
[Gimbal] Yaw=0.0 deg, Pitch=-0.5 deg
```

## 6. 无返回时如何定位

1. **每秒 WAIT 都没有**：这是 TX 链路或固件运行问题。检查是否烧录了最新 ELF、MCU 是否进入 `Error_Handler()`、串口助手是否接在 PA9、USB-TTL RX 是否与 PA9 交叉连接。
2. **有 WAIT，但发命令后仍持续 WAIT**：确认 USB-TTL TX 接 PA10，打开 HEX 发送，关闭自动追加多余数据。
3. **主程序自己的回调要求接收 3 字节**：必须发送完整 3 字节后才会调用 `HAL_UART_RxCpltCallback()`；少一个字节时看不到回调。
4. **有 RX ACK，但舵机不动**：USART1 接收正常，检查舵机供电、共地、TIM1_CH3/CH4 和运动角是否过小。
5. **出现 invalid data 警告**：发送的是 ASCII 文本、方向字节不是 00/01，或帧发生错位。重新以 HEX 发送完整三字节。
6. **出现 movement timeout**：命令已经收到，但云台状态机在 5 秒内没有完成。检查舵机滤波、限位和主循环调用周期。

建议第一次发送 `50 00 32`（Pitch -5.0°）确认运动明显，再测试较小角度。

## 7. 机械零位校准

2026-07-21 实机复测：旧补偿下必须将软件角运动到 Yaw=-3°、Pitch=-7° 才能正确对正。现已把这两个偏移并入 PWM 零位补偿，使初始化后的正确机械位置直接对应软件角 0°。

配置位于 `APP/BSP/servo/Servo.h`：

```c
#define SERVO_YAW_ZERO_TRIM_DEG   (-6.0f)
#define SERVO_PITCH_ZERO_TRIM_DEG (-2.0f)
```

补偿只作用于最终 CCR 映射，因此初始化后软件角仍返回 Yaw=0.0°、Pitch=0.0°。

重新安装后，可先记录“装置正确对正时所需的软件角”，再把该软件角直接累加到当前 trim：`new_trim = old_trim + correct_software_angle`。两轴均反向安装时，这种方法可避免根据物理正方向猜测符号。