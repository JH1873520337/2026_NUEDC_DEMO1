#ifndef VISION_UART_H
#define VISION_UART_H

#include <stdbool.h>
#include <stdint.h>

#include "stm32f4xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#define VISION_UART_MODE1_FRAME_SIZE 14U
#define VISION_UART_MODE2_FRAME_SIZE 26U
#define VISION_UART_MAX_FRAME_SIZE VISION_UART_MODE2_FRAME_SIZE
#define VISION_UART_FLAG_TARGET_VALID 0x01U
#define VISION_UART_FLAG_CORNERS_VALID 0x01U
#define VISION_UART_FLAG_LASER_VALID 0x02U

typedef struct {
    int16_t x;
    int16_t y;
} VisionCoordinate_t;

typedef enum {
    VISION_UART_MODE_NONE = 0,
    VISION_UART_MODE1 = 1,
    VISION_UART_MODE2 = 2
} VisionUartMode_t;

typedef struct {
    uint8_t sequence;
    bool target_valid;
    bool laser_valid;
    VisionCoordinate_t target;
    VisionCoordinate_t laser;
    uint32_t received_at_ms;
} VisionUartMode1Frame_t;

typedef struct {
    uint8_t sequence;
    bool corners_valid;
    bool laser_valid;
    VisionCoordinate_t quadrant[4];
    VisionCoordinate_t laser;
    uint32_t received_at_ms;
} VisionUartMode2Frame_t;

HAL_StatusTypeDef VisionUart_Start(void);
bool VisionUart_GetMode1Latest(
    VisionUartMode1Frame_t *frame,
    uint32_t timeout_ms
);
bool VisionUart_GetMode2Latest(
    VisionUartMode2Frame_t *frame,
    uint32_t timeout_ms
);
VisionUartMode_t VisionUart_GetActiveMode(uint32_t timeout_ms);
uint32_t VisionUart_GetValidFrameCount(void);
uint32_t VisionUart_GetInvalidFrameCount(void);
uint32_t VisionUart_GetReceivedByteCount(void);
void VisionUart_RxCpltCallback(void);
void VisionUart_ErrorCallback(void);

#ifdef __cplusplus
}
#endif

#endif
