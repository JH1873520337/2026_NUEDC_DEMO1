#include "serve_vision_debug.h"

#include <stdbool.h>
#include <stdio.h>

#include "UART4_HOST.h"
#include "vision_uart.h"

#define VISION_DEBUG_RX_TIMEOUT_MS 200U
#define VISION_DEBUG_TX_TIMEOUT_MS 30U
#define VISION_DEBUG_LINE_SIZE 224U
#define VISION_DEBUG_WAIT_REPORT_MS 1000U

static VisionUartMode_t last_mode;
static uint8_t last_sequence;
static bool has_last_sequence;
static char output_line[VISION_DEBUG_LINE_SIZE];
static uint32_t last_wait_report_ms;

static Serve_VisionDebug_Status_t SendLine(int length)
{
    UART4_HOST_Status_t status;

    if (length <= 0 || length >= (int)sizeof(output_line)) {
        return SERVE_VISION_DEBUG_FORMAT_ERROR;
    }

    status = UART4_HOST_Write(
        (const uint8_t *)output_line,
        (uint16_t)length,
        VISION_DEBUG_TX_TIMEOUT_MS
    );
    return status == UART4_HOST_OK
        ? SERVE_VISION_DEBUG_OK
        : SERVE_VISION_DEBUG_UART_ERROR;
}

static Serve_VisionDebug_Status_t ProcessMode1(void)
{
    VisionUartMode1Frame_t frame;
    Serve_VisionDebug_Status_t status;
    int length;

    if (!VisionUart_GetMode1Latest(&frame, VISION_DEBUG_RX_TIMEOUT_MS)) {
        return SERVE_VISION_DEBUG_NO_DATA;
    }
    if (has_last_sequence && last_mode == VISION_UART_MODE1 &&
        last_sequence == frame.sequence) {
        return SERVE_VISION_DEBUG_NOT_NEW;
    }

    length = snprintf(
        output_line,
        sizeof(output_line),
        "MODE1 seq=%u target_valid=%u target=(%d,%d) "
        "laser_valid=%u laser=(%d,%d) rx_ok=%lu rx_bad=%lu\r\n",
        (unsigned int)frame.sequence,
        frame.target_valid ? 1U : 0U,
        (int)frame.target.x,
        (int)frame.target.y,
        frame.laser_valid ? 1U : 0U,
        (int)frame.laser.x,
        (int)frame.laser.y,
        (unsigned long)VisionUart_GetValidFrameCount(),
        (unsigned long)VisionUart_GetInvalidFrameCount()
    );
    status = SendLine(length);
    if (status == SERVE_VISION_DEBUG_OK) {
        last_mode = VISION_UART_MODE1;
        last_sequence = frame.sequence;
        has_last_sequence = true;
    }
    return status;
}

static Serve_VisionDebug_Status_t ProcessMode2(void)
{
    VisionUartMode2Frame_t frame;
    Serve_VisionDebug_Status_t status;
    int length;

    if (!VisionUart_GetMode2Latest(&frame, VISION_DEBUG_RX_TIMEOUT_MS)) {
        return SERVE_VISION_DEBUG_NO_DATA;
    }
    if (has_last_sequence && last_mode == VISION_UART_MODE2 &&
        last_sequence == frame.sequence) {
        return SERVE_VISION_DEBUG_NOT_NEW;
    }

    length = snprintf(
        output_line,
        sizeof(output_line),
        "MODE2 seq=%u corners_valid=%u "
        "q1=(%d,%d) q2=(%d,%d) q3=(%d,%d) q4=(%d,%d) "
        "laser_valid=%u laser=(%d,%d) rx_ok=%lu rx_bad=%lu\r\n",
        (unsigned int)frame.sequence,
        frame.corners_valid ? 1U : 0U,
        (int)frame.quadrant[0].x,
        (int)frame.quadrant[0].y,
        (int)frame.quadrant[1].x,
        (int)frame.quadrant[1].y,
        (int)frame.quadrant[2].x,
        (int)frame.quadrant[2].y,
        (int)frame.quadrant[3].x,
        (int)frame.quadrant[3].y,
        frame.laser_valid ? 1U : 0U,
        (int)frame.laser.x,
        (int)frame.laser.y,
        (unsigned long)VisionUart_GetValidFrameCount(),
        (unsigned long)VisionUart_GetInvalidFrameCount()
    );
    status = SendLine(length);
    if (status == SERVE_VISION_DEBUG_OK) {
        last_mode = VISION_UART_MODE2;
        last_sequence = frame.sequence;
        has_last_sequence = true;
    }
    return status;
}

void Serve_VisionDebug_Init(void)
{
    int length;

    last_mode = VISION_UART_MODE_NONE;
    last_sequence = 0U;
    has_last_sequence = false;
    last_wait_report_ms = 0U;
    length = snprintf(
        output_line,
        sizeof(output_line),
        "UART4 READY: waiting USART1 on PB7, 115200 8N1\r\n"
    );
    (void)SendLine(length);
}

Serve_VisionDebug_Status_t Serve_VisionDebug_Process(uint32_t now_ms)
{
    VisionUartMode_t mode;

    mode = VisionUart_GetActiveMode(VISION_DEBUG_RX_TIMEOUT_MS);
    if (mode == VISION_UART_MODE1) {
        return ProcessMode1();
    }
    if (mode == VISION_UART_MODE2) {
        return ProcessMode2();
    }
    if ((uint32_t)(now_ms - last_wait_report_ms) >=
        VISION_DEBUG_WAIT_REPORT_MS) {
        int length = snprintf(
            output_line,
            sizeof(output_line),
            "WAIT USART1 bytes=%lu rx_ok=%lu rx_bad=%lu\r\n",
            (unsigned long)VisionUart_GetReceivedByteCount(),
            (unsigned long)VisionUart_GetValidFrameCount(),
            (unsigned long)VisionUart_GetInvalidFrameCount()
        );
        last_wait_report_ms = now_ms;
        return SendLine(length);
    }
    return SERVE_VISION_DEBUG_NO_DATA;
}
