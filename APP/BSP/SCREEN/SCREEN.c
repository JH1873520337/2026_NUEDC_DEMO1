#include "SCREEN.h"

#include "usart.h"

#define SCREEN_FRAME_HEADER              0x55U
#define SCREEN_COMMAND_MIN               ((uint8_t)SCREEN_CMD_HOME)
#define SCREEN_COMMAND_MAX               ((uint8_t)SCREEN_CMD_TRACK_START)
#define SCREEN_COMMAND_QUEUE_SIZE        8U
#define SCREEN_COMMAND_QUEUE_INDEX_MASK  (SCREEN_COMMAND_QUEUE_SIZE - 1U)
#define SCREEN_ANGLE_MIN                  (-9999.9f)
#define SCREEN_ANGLE_MAX                  9999.9f
#define SCREEN_TX_BUFFER_SIZE             40U

static const char SCREEN_HorizontalComponent[] = "tHorizontal";
static const char SCREEN_VerticalComponent[] = "tVertical";

typedef enum {
    SCREEN_PARSE_WAIT_HEADER_1 = 0,
    SCREEN_PARSE_WAIT_HEADER_2,
    SCREEN_PARSE_WAIT_COMMAND
} SCREEN_ParseState_t;

typedef struct {
    SCREEN_Command_t command_queue[SCREEN_COMMAND_QUEUE_SIZE];
    volatile uint8_t queue_head;
    volatile uint8_t queue_tail;
    SCREEN_ParseState_t parse_state;
    uint8_t rx_byte;
    volatile uint8_t initialized;
    volatile SCREEN_Stats_t stats;
} SCREEN_Context_t;

static SCREEN_Context_t SCREEN_Context;

static uint8_t SCREEN_IsCommandValid(uint8_t byte)
{
    return (byte >= SCREEN_COMMAND_MIN) && (byte <= SCREEN_COMMAND_MAX);
}

static void SCREEN_QueueCommand(SCREEN_Command_t command)
{
    uint8_t next_head;

    next_head = (SCREEN_Context.queue_head + 1U) &
                SCREEN_COMMAND_QUEUE_INDEX_MASK;
    if (next_head == SCREEN_Context.queue_tail) {
        SCREEN_Context.stats.queue_overflows++;
        return;
    }

    SCREEN_Context.command_queue[SCREEN_Context.queue_head] = command;
    SCREEN_Context.queue_head = next_head;
}

static void SCREEN_ParseByte(uint8_t byte)
{
    switch (SCREEN_Context.parse_state) {
    case SCREEN_PARSE_WAIT_HEADER_1:
        if (byte == SCREEN_FRAME_HEADER) {
            SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_HEADER_2;
        }
        break;

    case SCREEN_PARSE_WAIT_HEADER_2:
        if (byte == SCREEN_FRAME_HEADER) {
            SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_COMMAND;
        } else {
            SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_HEADER_1;
        }
        break;

    case SCREEN_PARSE_WAIT_COMMAND:
        if (SCREEN_IsCommandValid(byte)) {
            SCREEN_QueueCommand((SCREEN_Command_t)byte);
            SCREEN_Context.stats.valid_frames++;
            SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_HEADER_1;
        } else if (byte == SCREEN_FRAME_HEADER) {
            /* 连续出现 0x55 时保留重同步机会，等待后续命令字。 */
            SCREEN_Context.stats.invalid_commands++;
        } else {
            SCREEN_Context.stats.invalid_commands++;
            SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_HEADER_1;
        }
        break;

    default:
        SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_HEADER_1;
        break;
    }
}

static SCREEN_Status_t SCREEN_StartReceive(void)
{
    if (HAL_UART_Receive_IT(&huart5, &SCREEN_Context.rx_byte, 1U) != HAL_OK) {
        return SCREEN_ERROR;
    }

    return SCREEN_OK;
}

static uint16_t SCREEN_AppendString(uint8_t *buffer,
                                    uint16_t index,
                                    const char *text)
{
    while (*text != '\0') {
        buffer[index++] = (uint8_t)*text++;
    }

    return index;
}

static SCREEN_Status_t SCREEN_BuildAngleCommand(const char *component,
                                                 float angle,
                                                 uint8_t *buffer,
                                                 uint16_t *length)
{
    static const char assignment[] = ".txt=\"";
    uint8_t reverse_digits[5];
    int32_t angle_tenths;
    uint32_t absolute_tenths;
    uint32_t integer_part;
    uint8_t digit_count = 0U;
    uint16_t index = 0U;

    if ((component == NULL) || (buffer == NULL) || (length == NULL) ||
        (angle != angle) || (angle < SCREEN_ANGLE_MIN) ||
        (angle > SCREEN_ANGLE_MAX)) {
        return SCREEN_INVALID_ARGUMENT;
    }

    if (angle >= 0.0f) {
        angle_tenths = (int32_t)(angle * 10.0f + 0.5f);
    } else {
        angle_tenths = (int32_t)(angle * 10.0f - 0.5f);
    }

    if (angle_tenths < 0) {
        absolute_tenths = (uint32_t)(-angle_tenths);
    } else {
        absolute_tenths = (uint32_t)angle_tenths;
    }

    index = SCREEN_AppendString(buffer, index, component);
    index = SCREEN_AppendString(buffer, index, assignment);
    if (angle_tenths < 0) {
        buffer[index++] = (uint8_t)'-';
    }

    integer_part = absolute_tenths / 10U;
    do {
        reverse_digits[digit_count++] = (uint8_t)('0' +
                                                  integer_part % 10U);
        integer_part /= 10U;
    } while (integer_part != 0U);

    while (digit_count != 0U) {
        buffer[index++] = reverse_digits[--digit_count];
    }

    buffer[index++] = (uint8_t)'.';
    buffer[index++] = (uint8_t)('0' + absolute_tenths % 10U);
    buffer[index++] = (uint8_t)'\"';
    buffer[index++] = 0xFFU;
    buffer[index++] = 0xFFU;
    buffer[index++] = 0xFFU;

    *length = index;
    return SCREEN_OK;
}

static SCREEN_Status_t SCREEN_SendAngle(const char *component,
                                         float angle,
                                         uint32_t timeout_ms)
{
    HAL_StatusTypeDef hal_status;
    uint8_t buffer[SCREEN_TX_BUFFER_SIZE];
    uint16_t length;

    if (!SCREEN_Context.initialized) {
        return SCREEN_NOT_INITIALIZED;
    }

    if (timeout_ms == 0U) {
        return SCREEN_INVALID_ARGUMENT;
    }

    if (SCREEN_BuildAngleCommand(component, angle, buffer, &length) !=
        SCREEN_OK) {
        return SCREEN_INVALID_ARGUMENT;
    }

    hal_status = HAL_UART_Transmit(&huart5, buffer, length, timeout_ms);
    if (hal_status == HAL_OK) {
        SCREEN_Context.stats.transmitted_commands++;
        return SCREEN_OK;
    }

    SCREEN_Context.stats.transmit_errors++;
    if (hal_status == HAL_BUSY) {
        return SCREEN_BUSY;
    }
    if (hal_status == HAL_TIMEOUT) {
        return SCREEN_TIMEOUT;
    }

    return SCREEN_ERROR;
}

SCREEN_Status_t SCREEN_Init(void)
{
    if (SCREEN_Context.initialized) {
        return SCREEN_OK;
    }

    SCREEN_Context.queue_head = 0U;
    SCREEN_Context.queue_tail = 0U;
    SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_HEADER_1;
    SCREEN_Context.rx_byte = 0U;
    SCREEN_Context.stats.valid_frames = 0U;
    SCREEN_Context.stats.invalid_commands = 0U;
    SCREEN_Context.stats.queue_overflows = 0U;
    SCREEN_Context.stats.uart_errors = 0U;
    SCREEN_Context.stats.receive_restart_errors = 0U;
    SCREEN_Context.stats.transmitted_commands = 0U;
    SCREEN_Context.stats.transmit_errors = 0U;

    SCREEN_Context.initialized = 1U;
    if (SCREEN_StartReceive() != SCREEN_OK) {
        SCREEN_Context.initialized = 0U;
        return SCREEN_ERROR;
    }

    return SCREEN_OK;
}

SCREEN_Status_t SCREEN_GetCommand(SCREEN_Command_t *command)
{
    uint8_t tail;

    if (command == NULL) {
        return SCREEN_INVALID_ARGUMENT;
    }

    if (!SCREEN_Context.initialized) {
        return SCREEN_NOT_INITIALIZED;
    }

    tail = SCREEN_Context.queue_tail;
    if (tail == SCREEN_Context.queue_head) {
        return SCREEN_EMPTY;
    }

    *command = SCREEN_Context.command_queue[tail];
    SCREEN_Context.queue_tail = (tail + 1U) &
                                SCREEN_COMMAND_QUEUE_INDEX_MASK;
    return SCREEN_OK;
}

SCREEN_Status_t SCREEN_SendHorizontalAngle(float angle, uint32_t timeout_ms)
{
    return SCREEN_SendAngle(SCREEN_HorizontalComponent, angle, timeout_ms);
}

SCREEN_Status_t SCREEN_SendVerticalAngle(float angle, uint32_t timeout_ms)
{
    return SCREEN_SendAngle(SCREEN_VerticalComponent, angle, timeout_ms);
}

SCREEN_Status_t SCREEN_SendAngles(float horizontal_angle,
                                  float vertical_angle,
                                  uint32_t timeout_ms)
{
    SCREEN_Status_t status;

    status = SCREEN_SendHorizontalAngle(horizontal_angle, timeout_ms);
    if (status != SCREEN_OK) {
        return status;
    }

    return SCREEN_SendVerticalAngle(vertical_angle, timeout_ms);
}

SCREEN_Status_t SCREEN_GetStats(SCREEN_Stats_t *stats)
{
    if (stats == NULL) {
        return SCREEN_INVALID_ARGUMENT;
    }

    if (!SCREEN_Context.initialized) {
        return SCREEN_NOT_INITIALIZED;
    }

    *stats = SCREEN_Context.stats;
    return SCREEN_OK;
}

void SCREEN_UART_RxCpltCallback(void)
{
    if (!SCREEN_Context.initialized) {
        return;
    }

    SCREEN_ParseByte(SCREEN_Context.rx_byte);

    if (huart5.RxState == HAL_UART_STATE_READY) {
        if (SCREEN_StartReceive() != SCREEN_OK) {
            SCREEN_Context.stats.receive_restart_errors++;
        }
    }
}

void SCREEN_UART_ErrorCallback(void)
{
    if (!SCREEN_Context.initialized) {
        return;
    }

    SCREEN_Context.stats.uart_errors++;
    SCREEN_Context.parse_state = SCREEN_PARSE_WAIT_HEADER_1;

    if (huart5.RxState == HAL_UART_STATE_READY) {
        if (SCREEN_StartReceive() != SCREEN_OK) {
            SCREEN_Context.stats.receive_restart_errors++;
        }
    }
}
