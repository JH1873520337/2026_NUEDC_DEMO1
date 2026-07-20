#include "UART4_HOST.h"

#include "usart.h"

UART4_HOST_Status_t UART4_HOST_Write(
    const uint8_t *data,
    uint16_t size,
    uint32_t timeout_ms
)
{
    HAL_StatusTypeDef status;

    if (data == NULL || size == 0U) {
        return UART4_HOST_INVALID_ARGUMENT;
    }

    status = HAL_UART_Transmit(&huart4, (uint8_t *)data, size, timeout_ms);
    if (status == HAL_OK) {
        return UART4_HOST_OK;
    }
    if (status == HAL_TIMEOUT) {
        return UART4_HOST_TIMEOUT;
    }
    return UART4_HOST_ERROR;
}
