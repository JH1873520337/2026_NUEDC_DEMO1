#include "vision_uart.h"

#include "usart.h"

#define VISION_UART_HEADER_1 0xAAU
#define VISION_UART_HEADER_2 0x55U
#define VISION_UART_MESSAGE_MODE1 0x01U
#define VISION_UART_MESSAGE_MODE2 0x02U

static uint8_t rx_byte;
static uint8_t rx_frame[VISION_UART_MAX_FRAME_SIZE];
static uint8_t rx_index;
static uint8_t rx_expected_size;
static volatile VisionUartMode1Frame_t latest_mode1_frame;
static volatile VisionUartMode2Frame_t latest_mode2_frame;
static volatile uint32_t mode1_revision;
static volatile uint32_t mode2_revision;
static volatile uint32_t valid_frame_count;
static volatile uint32_t invalid_frame_count;
static volatile uint32_t received_byte_count;
static volatile bool has_mode1_frame;
static volatile bool has_mode2_frame;
static volatile VisionUartMode_t active_mode;
static volatile uint32_t active_mode_received_at_ms;

static int16_t ReadInt16Le(const uint8_t *data)
{
    uint16_t value = (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
    return (int16_t)value;
}

static bool ChecksumValid(const uint8_t *frame, uint8_t frame_size)
{
    uint8_t checksum = 0U;
    uint32_t index;

    for (index = 0U; index < (uint32_t)(frame_size - 1U); ++index) {
        checksum = (uint8_t)(checksum + frame[index]);
    }
    return checksum == frame[frame_size - 1U];
}

static void PublishMode1Frame(const uint8_t *frame)
{
    uint8_t flags = frame[3];

    ++mode1_revision;
    latest_mode1_frame.sequence = frame[4];
    latest_mode1_frame.target_valid =
        (flags & VISION_UART_FLAG_TARGET_VALID) != 0U;
    latest_mode1_frame.laser_valid =
        (flags & VISION_UART_FLAG_LASER_VALID) != 0U;
    latest_mode1_frame.target.x = ReadInt16Le(&frame[5]);
    latest_mode1_frame.target.y = ReadInt16Le(&frame[7]);
    latest_mode1_frame.laser.x = ReadInt16Le(&frame[9]);
    latest_mode1_frame.laser.y = ReadInt16Le(&frame[11]);
    latest_mode1_frame.received_at_ms = HAL_GetTick();
    ++mode1_revision;
    has_mode1_frame = true;
    active_mode = VISION_UART_MODE1;
    active_mode_received_at_ms = latest_mode1_frame.received_at_ms;
    ++valid_frame_count;
}

static void PublishMode2Frame(const uint8_t *frame)
{
    uint32_t index;
    uint32_t offset = 5U;
    uint8_t flags = frame[3];

    ++mode2_revision;
    latest_mode2_frame.sequence = frame[4];
    latest_mode2_frame.corners_valid =
        (flags & VISION_UART_FLAG_CORNERS_VALID) != 0U;
    latest_mode2_frame.laser_valid =
        (flags & VISION_UART_FLAG_LASER_VALID) != 0U;
    for (index = 0U; index < 4U; ++index) {
        latest_mode2_frame.quadrant[index].x = ReadInt16Le(&frame[offset]);
        latest_mode2_frame.quadrant[index].y =
            ReadInt16Le(&frame[offset + 2U]);
        offset += 4U;
    }
    latest_mode2_frame.laser.x = ReadInt16Le(&frame[offset]);
    latest_mode2_frame.laser.y = ReadInt16Le(&frame[offset + 2U]);
    latest_mode2_frame.received_at_ms = HAL_GetTick();
    ++mode2_revision;
    has_mode2_frame = true;
    active_mode = VISION_UART_MODE2;
    active_mode_received_at_ms = latest_mode2_frame.received_at_ms;
    ++valid_frame_count;
}

static void ParseByte(uint8_t byte)
{
    if (rx_index == 0U) {
        if (byte == VISION_UART_HEADER_1) {
            rx_frame[rx_index++] = byte;
        }
        return;
    }

    if (rx_index == 1U) {
        if (byte == VISION_UART_HEADER_2) {
            rx_frame[rx_index++] = byte;
        } else if (byte == VISION_UART_HEADER_1) {
            rx_frame[0] = byte;
        } else {
            rx_index = 0U;
        }
        return;
    }

    if (rx_index == 2U) {
        rx_frame[rx_index++] = byte;
        if (byte == VISION_UART_MESSAGE_MODE1) {
            rx_expected_size = VISION_UART_MODE1_FRAME_SIZE;
        } else if (byte == VISION_UART_MESSAGE_MODE2) {
            rx_expected_size = VISION_UART_MODE2_FRAME_SIZE;
        } else {
            ++invalid_frame_count;
            rx_index = 0U;
            rx_expected_size = 0U;
        }
        return;
    }

    rx_frame[rx_index++] = byte;
    if (rx_index < rx_expected_size) {
        return;
    }

    if (!ChecksumValid(rx_frame, rx_expected_size)) {
        ++invalid_frame_count;
    } else if (rx_frame[2] == VISION_UART_MESSAGE_MODE1) {
        PublishMode1Frame(rx_frame);
    } else if (rx_frame[2] == VISION_UART_MESSAGE_MODE2) {
        PublishMode2Frame(rx_frame);
    } else {
        ++invalid_frame_count;
    }
    rx_index = 0U;
    rx_expected_size = 0U;
}

HAL_StatusTypeDef VisionUart_Start(void)
{
    rx_index = 0U;
    rx_expected_size = 0U;
    has_mode1_frame = false;
    has_mode2_frame = false;
    mode1_revision = 0U;
    mode2_revision = 0U;
    active_mode = VISION_UART_MODE_NONE;
    active_mode_received_at_ms = 0U;
    valid_frame_count = 0U;
    invalid_frame_count = 0U;
    received_byte_count = 0U;
    return HAL_UART_Receive_DMA(&huart1, &rx_byte, 1U);
}

bool VisionUart_GetMode1Latest(
    VisionUartMode1Frame_t *frame,
    uint32_t timeout_ms
)
{
    uint32_t before;
    uint32_t after;

    if (frame == NULL || !has_mode1_frame) {
        return false;
    }

    do {
        before = mode1_revision;
        if ((before & 1U) != 0U) {
            continue;
        }
        frame->sequence = latest_mode1_frame.sequence;
        frame->target_valid = latest_mode1_frame.target_valid;
        frame->laser_valid = latest_mode1_frame.laser_valid;
        frame->target.x = latest_mode1_frame.target.x;
        frame->target.y = latest_mode1_frame.target.y;
        frame->laser.x = latest_mode1_frame.laser.x;
        frame->laser.y = latest_mode1_frame.laser.y;
        frame->received_at_ms = latest_mode1_frame.received_at_ms;
        after = mode1_revision;
    } while (before != after || (after & 1U) != 0U);

    return (uint32_t)(HAL_GetTick() - frame->received_at_ms) <= timeout_ms;
}

bool VisionUart_GetMode2Latest(
    VisionUartMode2Frame_t *frame,
    uint32_t timeout_ms
)
{
    uint32_t before;
    uint32_t after;
    uint32_t index;

    if (frame == NULL || !has_mode2_frame) {
        return false;
    }

    do {
        before = mode2_revision;
        if ((before & 1U) != 0U) {
            continue;
        }
        frame->sequence = latest_mode2_frame.sequence;
        frame->corners_valid = latest_mode2_frame.corners_valid;
        frame->laser_valid = latest_mode2_frame.laser_valid;
        for (index = 0U; index < 4U; ++index) {
            frame->quadrant[index].x = latest_mode2_frame.quadrant[index].x;
            frame->quadrant[index].y = latest_mode2_frame.quadrant[index].y;
        }
        frame->laser.x = latest_mode2_frame.laser.x;
        frame->laser.y = latest_mode2_frame.laser.y;
        frame->received_at_ms = latest_mode2_frame.received_at_ms;
        after = mode2_revision;
    } while (before != after || (after & 1U) != 0U);

    return (uint32_t)(HAL_GetTick() - frame->received_at_ms) <= timeout_ms;
}

VisionUartMode_t VisionUart_GetActiveMode(uint32_t timeout_ms)
{
    VisionUartMode_t mode = active_mode;
    uint32_t received_at_ms = active_mode_received_at_ms;

    if (mode == VISION_UART_MODE_NONE ||
        (uint32_t)(HAL_GetTick() - received_at_ms) > timeout_ms) {
        return VISION_UART_MODE_NONE;
    }
    return mode;
}

uint32_t VisionUart_GetValidFrameCount(void)
{
    return valid_frame_count;
}

uint32_t VisionUart_GetInvalidFrameCount(void)
{
    return invalid_frame_count;
}

uint32_t VisionUart_GetReceivedByteCount(void)
{
    return received_byte_count;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }
    ++received_byte_count;
    ParseByte(rx_byte);
    (void)HAL_UART_Receive_DMA(&huart1, &rx_byte, 1U);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1) {
        return;
    }
    ++invalid_frame_count;
    rx_index = 0U;
    rx_expected_size = 0U;
    (void)HAL_UART_AbortReceive(huart);
    (void)HAL_UART_Receive_DMA(&huart1, &rx_byte, 1U);
}
