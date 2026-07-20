#ifndef UART4_HOST_H
#define UART4_HOST_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    UART4_HOST_OK = 0,
    UART4_HOST_INVALID_ARGUMENT,
    UART4_HOST_TIMEOUT,
    UART4_HOST_ERROR
} UART4_HOST_Status_t;

UART4_HOST_Status_t UART4_HOST_Write(
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms
);

#ifdef __cplusplus
}
#endif

#endif
