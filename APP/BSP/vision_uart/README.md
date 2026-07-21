# USART1 视觉协议接收器

`vision_uart` 通过 STM32 USART1 RX DMA 接收 MaixCAM 字节流，并分别解析 MODE1 和 MODE2 数据帧。

## 硬件配置

- USART1：`115200 8N1`
- STM32 接收引脚：`PB7`（`USART1_RX`）
- STM32 发送引脚：`PB6`（`USART1_TX`，当前未使用）
- DMA 接收：`DMA2_Stream2`，通道 4
- MaixCAM TX 与 STM32 必须共地，并使用 3.3 V 逻辑电平。

## 流式解析器

接收器逐字节处理数据并搜索帧头 `AA 55`。读取消息类型后，解析器会选择对应的预期帧长，校验低八位累加和，并在校验通过后发布一份完整的数据快照。

该解析方式能够处理拆包、多个连续数据包、帧前噪声，以及无效数据包后的自动重新同步。

### MODE1：消息类型 `01`，共 14 字节

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | 消息类型 `01` |
| 3 | 1 | bit0：目标点有效；bit1：激光点有效 |
| 4 | 1 | 递增序号 |
| 5 | 2 | 目标点 X，有符号小端 `int16` |
| 7 | 2 | 目标点 Y，有符号小端 `int16` |
| 9 | 2 | 激光点 X，有符号小端 `int16` |
| 11 | 2 | 激光点 Y，有符号小端 `int16` |
| 13 | 1 | 字节 0-12 累加和的低八位 |

MODE1 使用图像绝对像素坐标：原点位于图像左上角，X 轴向右为正，Y 轴向下为正。

### MODE2：消息类型 `02`，共 26 字节

| 偏移 | 长度 | 字段 |
| --- | --- | --- |
| 0 | 2 | 帧头 `AA 55` |
| 2 | 1 | 消息类型 `02` |
| 3 | 1 | bit0：四角坐标有效；bit1：激光点有效 |
| 4 | 1 | 递增序号 |
| 5 | 16 | Q1、Q2、Q3、Q4 的 `(X,Y)`，有符号小端 `int16` |
| 21 | 4 | 激光点 `(X,Y)`，有符号小端 `int16` |
| 25 | 1 | 字节 0-24 累加和的低八位 |

MODE2 使用相对矩形中心的像素坐标：X 轴向右为正，Y 轴向上为正。

## 任务层调用方法

USART1 初始化完成后，`main.c` 已调用 `VisionUart_Start()`。任务可按以下方式读取解析完成的数据快照：

```c
VisionUartMode_t mode = VisionUart_GetActiveMode(200U);

if (mode == VISION_UART_MODE1) {
    VisionUartMode1Frame_t frame;
    if (VisionUart_GetMode1Latest(&frame, 200U)) {
        if (frame.target_valid && frame.laser_valid) {
            /* 在此使用 frame.target 和 frame.laser。 */
        }
    }
} else if (mode == VISION_UART_MODE2) {
    VisionUartMode2Frame_t frame;
    if (VisionUart_GetMode2Latest(&frame, 200U)) {
        if (frame.corners_valid && frame.laser_valid) {
            /* frame.quadrant[0..3] 依次对应 Q1..Q4。 */
        }
    }
}
```

串口链路诊断可使用 `VisionUart_GetValidFrameCount()` 和 `VisionUart_GetInvalidFrameCount()` 分别读取有效帧数和无效帧数。`VisionUart_GetReceivedByteCount()` 返回 USART1 RX DMA 已交付的全部字节数。
