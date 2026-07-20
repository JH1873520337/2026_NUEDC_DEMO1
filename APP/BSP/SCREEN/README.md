# 串口屏通信驱动

本模块实现串口屏按键协议 `55 55 CMD` 的接收与解析，并支持主控板向串口屏
发送水平、垂直角度。按键命令范围为 `0xA0` 至 `0xA5`。

## 硬件与 CubeMX 配置

| 项目 | 配置 |
| --- | --- |
| MCU | STM32F407VETx |
| 串口 | UART5 |
| 主控 TX | PC12，连接串口屏 RX（仅需接收按键时可不接） |
| 主控 RX | PD2，连接串口屏 TX |
| 串口参数 | 115200 bit/s、8 数据位、1 停止位、无校验、无流控 |
| 中断 | UART5 global interrupt，抢占优先级 5 |

串口屏与主控板必须共地，并确认双方使用兼容的 TTL IO 电平。

## 协议命令

| 命令 | 含义 |
| --- | --- |
| `55 55 A0` | 回中 |
| `55 55 A1` | 暂停 |
| `55 55 A2` | 启动 |
| `55 55 A3` | 屏幕边线 |
| `55 55 A4` | A4 靶纸 |
| `55 55 A5` | 开始追踪 |

状态机按字节解析，不依赖一次中断收到完整帧。错误命令会被丢弃；当命令
位置再次收到 `0x55` 时保持同步，从而正确处理 `55 55 55 A0` 这类重叠帧头。

## 使用方式

`main.c` 在 UART5 初始化完成后调用 `SCREEN_Init()`。任务层只需非阻塞读取：

```c
SCREEN_Command_t command;

while (SCREEN_GetCommand(&command) == SCREEN_OK) {
    switch (command) {
    case SCREEN_CMD_HOME:
        /* 请求系统回中。 */
        break;
    case SCREEN_CMD_PAUSE:
        /* 请求暂停。 */
        break;
    default:
        break;
    }
}
```

发送角度时，驱动自动保留一位小数并添加 Nextion 的三个 `0xFF` 结束字节：

```c
if (SCREEN_SendAngles(35.6f, -12.3f, 10U) != SCREEN_OK) {
    /* 根据需要记录通信错误。 */
}
```

实际发送内容为：

```text
tHorizontal.txt="35.6" FF FF FF
tVertical.txt="-12.3" FF FF FF
```

当前默认运行的 `Task_GimbalCalibration` 每 100 ms 发送一次标定角度；
`Task_Basic2` 也会以相同周期发送云台指令角度。单条发送采用阻塞式 HAL 接口，
超时为 10 ms；正常 115200 bit/s 下两条指令约几毫秒完成。

接收队列可保存 7 个待处理命令。队列满时丢弃最新命令，不阻塞中断；可通过
`SCREEN_GetStats()` 查看有效帧、无效命令、队列溢出和 UART 错误计数。

## 上下文与限制

- `SCREEN_UART_RxCpltCallback()` 和 `SCREEN_UART_ErrorCallback()` 仅在 HAL UART
  中断回调中调用，不执行业务动作或阻塞操作。
- `SCREEN_GetCommand()` 设计为单任务消费者接口，不应由多个任务同时调用。
- `SCREEN_SendAngles()` 只能在任务上下文调用，不应在中断中调用；最坏阻塞时间
  为两倍传入的单条指令超时时间。
- 当前协议没有校验字段，线路受干扰时只能依靠帧头和命令范围重新同步。
- 若重新生成 CubeMX 代码，应确认 UART5 NVIC、`UART5_IRQHandler()` 和用户代码区
  中的 SCREEN 初始化及回调转发仍然保留。
