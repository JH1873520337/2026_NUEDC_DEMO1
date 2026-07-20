# USART1 vision protocol receiver

`vision_uart` receives the MaixCAM byte stream through STM32 USART1 RX DMA and
decodes MODE1 and MODE2 packets independently.

## Hardware

- USART1: `115200 8N1`
- STM32 RX: `PB7` (`USART1_RX`)
- STM32 TX: `PB6` (`USART1_TX`, currently not required)
- DMA RX: `DMA2_Stream2`, channel 4
- MaixCAM TX and STM32 must share a common ground and use 3.3 V logic.

## Stream parser

The receiver consumes one byte at a time and searches for the `AA 55` header.
After reading the message type it selects the expected frame length, verifies
the low-eight-bit additive checksum, and then publishes a complete snapshot.
This handles split packets, multiple back-to-back packets, leading noise, and
automatic resynchronization after an invalid packet.

### MODE1, message type `01`, 14 bytes

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 2 | Header `AA 55` |
| 2 | 1 | Message type `01` |
| 3 | 1 | bit0 target valid, bit1 laser valid |
| 4 | 1 | Sequence |
| 5 | 2 | Target X, signed little-endian int16 |
| 7 | 2 | Target Y, signed little-endian int16 |
| 9 | 2 | Laser X, signed little-endian int16 |
| 11 | 2 | Laser Y, signed little-endian int16 |
| 13 | 1 | Sum of bytes 0-12, low eight bits |

MODE1 uses absolute image pixels: origin at the image top-left, X points right,
and Y points down.

### MODE2, message type `02`, 26 bytes

| Offset | Size | Field |
| --- | --- | --- |
| 0 | 2 | Header `AA 55` |
| 2 | 1 | Message type `02` |
| 3 | 1 | bit0 corners valid, bit1 laser valid |
| 4 | 1 | Sequence |
| 5 | 16 | Q1, Q2, Q3, Q4 `(X,Y)`, signed little-endian int16 |
| 21 | 4 | Laser `(X,Y)`, signed little-endian int16 |
| 25 | 1 | Sum of bytes 0-24, low eight bits |

MODE2 uses rectangle-center-relative pixels: X points right and Y points up.

## Task-side usage

`VisionUart_Start()` is already called in `main.c` after USART1 initialization.
A task can consume the decoded snapshots as follows:

```c
VisionUartMode_t mode = VisionUart_GetActiveMode(200U);

if (mode == VISION_UART_MODE1) {
    VisionUartMode1Frame_t frame;
    if (VisionUart_GetMode1Latest(&frame, 200U)) {
        if (frame.target_valid && frame.laser_valid) {
            /* Use frame.target and frame.laser here. */
        }
    }
} else if (mode == VISION_UART_MODE2) {
    VisionUartMode2Frame_t frame;
    if (VisionUart_GetMode2Latest(&frame, 200U)) {
        if (frame.corners_valid && frame.laser_valid) {
            /* frame.quadrant[0..3] are Q1..Q4. */
        }
    }
}
```

`VisionUart_GetValidFrameCount()` and `VisionUart_GetInvalidFrameCount()` are
available for serial-link diagnostics. `VisionUart_GetReceivedByteCount()`
reports every byte delivered by USART1 RX DMA.
