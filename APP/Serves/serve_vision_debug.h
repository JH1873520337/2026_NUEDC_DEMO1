#ifndef SERVE_VISION_DEBUG_H
#define SERVE_VISION_DEBUG_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    SERVE_VISION_DEBUG_OK = 0,
    SERVE_VISION_DEBUG_NO_DATA,
    SERVE_VISION_DEBUG_NOT_NEW,
    SERVE_VISION_DEBUG_FORMAT_ERROR,
    SERVE_VISION_DEBUG_UART_ERROR
} Serve_VisionDebug_Status_t;

void Serve_VisionDebug_Init(void);
Serve_VisionDebug_Status_t Serve_VisionDebug_Process(uint32_t now_ms);

#ifdef __cplusplus
}
#endif

#endif
