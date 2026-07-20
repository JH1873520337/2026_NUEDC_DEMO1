# UART4 host link

This BSP exposes the STM32 UART4 transmitter for an upper-computer serial
terminal.

- UART4: `115200 8N1`
- TX: `PA0`
- RX: `PA1` (not used by the demo)
- Logic level: 3.3 V
- API: `UART4_HOST_Write()`

Connect `PA0` to the USB-to-UART adapter RX pin and connect both grounds. The
write API is blocking and must only be called from task context. Its timeout is
provided by the caller; the vision demo uses 30 ms.
