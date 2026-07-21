/**
 * @file GimbalDebug.c
 * @brief USART1 三字节无符号云台调试协议实现。
 */
#include "GimbalDebug.h"

#include "Gimbal.h"
#include "usart.h"

/** 命令队列容量。采用环形队列，实际最多同时保存 7 帧。 */
#define GIMBAL_DEBUG_QUEUE_SIZE        8U

/** 阻塞发送状态文本时允许等待的最长时间。 */
#define GIMBAL_DEBUG_TX_TIMEOUT_MS     100U

/** 单条串口调试运动的最长等待时间，超时后立即停止并输出诊断信息。 */
#define GIMBAL_DEBUG_MOTION_TIMEOUT_MS  5000U

/** 在尚未收到有效命令时，每秒重复发送一次链路存活提示。 */
#define GIMBAL_DEBUG_HEARTBEAT_MS        1000U

/** 逐字节接收状态，用于自动忽略回车、换行和错位数据。 */
#define GIMBAL_DEBUG_RX_WAIT_AXIS        0U
#define GIMBAL_DEBUG_RX_WAIT_DIRECTION   1U
#define GIMBAL_DEBUG_RX_WAIT_ANGLE       2U

/** 一条已经完成接收的三字节命令。 */
typedef struct
{
    uint8_t axis;
    uint8_t direction;
    uint8_t angle_x10;
} GimbalDebug_Command_t;

/** HAL 每次中断接收一个字节；收到后在 HAL 回调中解析并立即重新挂接。 */
static uint8_t g_gimbal_debug_rx_byte;

/** 中断侧正在组装的命令及其接收阶段。 */
static GimbalDebug_Command_t g_gimbal_debug_pending_command;
static volatile uint8_t g_gimbal_debug_rx_stage;

/** 无法组成有效帧的字节数；主循环根据该计数输出一次诊断警告。 */
static volatile uint32_t g_gimbal_debug_invalid_bytes;
static uint32_t g_gimbal_debug_reported_invalid_bytes;

/** 裸机中断诊断计数：总接收字节、完整有效帧和 USART 硬件错误。 */
static volatile uint32_t g_gimbal_debug_received_bytes;
static volatile uint32_t g_gimbal_debug_valid_frames;

/** ISR 写入、主循环读取的单生产者/单消费者环形队列。 */
static GimbalDebug_Command_t g_gimbal_debug_queue[GIMBAL_DEBUG_QUEUE_SIZE];
static volatile uint8_t g_gimbal_debug_head;
static volatile uint8_t g_gimbal_debug_tail;
static volatile uint32_t g_gimbal_debug_dropped_frames;

/** 置 1 表示已经启动一条调试运动，等待其完成后打印角度。 */
static uint8_t g_gimbal_debug_motion_active;

/** 当前调试运动的开始时刻，用于防止滤波/状态异常导致永远没有返回信息。 */
static uint32_t g_gimbal_debug_motion_started_ms;

/** 置 1 表示 GimbalDebug_Init() 已成功执行。 */
static uint8_t g_gimbal_debug_initialized;

/** 上一次存活提示的发送时间；只在尚未收到完整有效帧时使用。 */
static uint32_t g_gimbal_debug_last_heartbeat_ms;

/**
 * @brief 通过 HAL 挂接下一次 USART1 单字节中断接收。
 * @note 本函数不直接访问 USART1 数据寄存器，接收过程完全由 HAL_UART_IRQHandler() 管理。
 */
static HAL_StatusTypeDef GimbalDebug_StartReceive(void)
{
    return HAL_UART_Receive_IT(&huart1, &g_gimbal_debug_rx_byte, 1U);
}

/**
 * @brief 从主循环侧取出一条命令。
 * @retval 1 成功取出；0 队列为空。
 */
static uint8_t GimbalDebug_Dequeue(GimbalDebug_Command_t *command)
{
    uint8_t tail;

    if (command == NULL) {
        return 0U;
    }

    tail = g_gimbal_debug_tail;
    if (tail == g_gimbal_debug_head) {
        return 0U;
    }

    *command = g_gimbal_debug_queue[tail];
    g_gimbal_debug_tail = (uint8_t)((tail + 1U) % GIMBAL_DEBUG_QUEUE_SIZE);
    return 1U;
}

/**
 * @brief 通过 USART1 输出一行调试文本。
 * @note 仅由主循环上下文调用，不会在串口中断中执行阻塞发送。
 */
static void GimbalDebug_PrintText(const char *text)
{
    size_t length;

    if (text == NULL) {
        return;
    }

    length = 0U;
    while (text[length] != '\0') {
        ++length;
    }

    (void)HAL_UART_Transmit(&huart1,
                            (uint8_t *)text,
                            (uint16_t)length,
                            GIMBAL_DEBUG_TX_TIMEOUT_MS);
}

/**
 * @brief 将浮点角度四舍五入为 0.1° 单位的有符号整数。
 * @details 避免使用 printf 的 %f，因而不需要为 newlib 打开浮点格式化支持。
 */
static int32_t GimbalDebug_AngleToTenths(float angle_deg)
{
    float scaled = angle_deg * 10.0f;
    return (int32_t)((scaled >= 0.0f) ? (scaled + 0.5f) : (scaled - 0.5f));
}

/** @brief 向输出缓存追加一个以 0 结尾的常量字符串。 */
static char *GimbalDebug_AppendText(char *destination, const char *source)
{
    while (*source != '\0') {
        *destination = *source;
        ++destination;
        ++source;
    }
    return destination;
}

/**
 * @brief 向输出缓存追加一个 0.1° 单位的角度。
 * @details 云台软件角范围仅为 ±90°，因此无需使用体积和栈开销较大的 printf。
 */
static char *GimbalDebug_AppendAngle(char *destination, int32_t angle_x10)
{
    uint32_t absolute;
    uint32_t integer_part;

    if (angle_x10 < 0) {
        *destination++ = '-';
        absolute = (uint32_t)(-angle_x10);
    } else {
        absolute = (uint32_t)angle_x10;
    }

    integer_part = absolute / 10U;
    if (integer_part >= 10U) {
        *destination++ = (char)('0' + (integer_part / 10U));
    }
    *destination++ = (char)('0' + (integer_part % 10U));
    *destination++ = '.';
    *destination++ = (char)('0' + (absolute % 10U));
    return destination;
}

/**
 * @brief 打印当前云台软件角度。
 * @details 输出示例："[Gimbal] Yaw=2.0 deg, Pitch=1.5 deg"。
 */
static void GimbalDebug_PrintCurrentAngles(void)
{
    const Gimbal_State_t *state = Gimbal_GetState();
    char message[64];
    char *write_position = message;

    if (state == NULL) {
        return;
    }

    write_position = GimbalDebug_AppendText(write_position, "[Gimbal] Yaw=");
    write_position = GimbalDebug_AppendAngle(
        write_position,
        GimbalDebug_AngleToTenths(state->current_yaw_deg));
    write_position = GimbalDebug_AppendText(write_position, " deg, Pitch=");
    write_position = GimbalDebug_AppendAngle(
        write_position,
        GimbalDebug_AngleToTenths(state->current_pitch_deg));
    write_position = GimbalDebug_AppendText(write_position, " deg\r\n");
    *write_position = '\0';

    GimbalDebug_PrintText(message);
}

/**
 * @brief 在主循环上下文立即回显一条已通过格式检查的命令。
 * @details 该 ACK 在运动开始前发送，因此即使舵机未接电、运动状态未结束或机械变化太小，
 *          也能确认 USART1 中断已经正确收到并解析了三个二进制字节。
 */
static void GimbalDebug_PrintCommandAck(const GimbalDebug_Command_t *command)
{
    char message[52];
    char *write_position = message;
    int32_t signed_angle_x10 = (int32_t)command->angle_x10;

    if ((command->direction == 0U) && (signed_angle_x10 != 0)) {
        signed_angle_x10 = -signed_angle_x10;
    }

    write_position = GimbalDebug_AppendText(write_position, "[Gimbal] RX: ");
    write_position = GimbalDebug_AppendText(
        write_position,
        (command->axis == (uint8_t)'Y') ? "Yaw " : "Pitch ");
    if ((command->direction == 1U) && (signed_angle_x10 != 0)) {
        *write_position++ = '+';
    }
    write_position = GimbalDebug_AppendAngle(write_position, signed_angle_x10);
    write_position = GimbalDebug_AppendText(write_position, " deg\r\n");
    *write_position = '\0';

    GimbalDebug_PrintText(message);
}

/**
 * @brief 解析并启动一条相对运动命令。
 * @details 'Y' 只改变 Yaw；'P' 只改变 Pitch。第二字节 0 为负向、1 为正向；第三字节除以 10 得到角度绝对值。
 */
static void GimbalDebug_ExecuteCommand(const GimbalDebug_Command_t *command)
{
    float angle_deg;
    Gimbal_Status_t status;

    if (command == NULL) {
        return;
    }

    /* 先完整校验三个字段，再回显 ACK；这样 RX 信息一定代表命令已被接受。 */
    if (command->direction > 1U) {
        GimbalDebug_PrintText("[Gimbal] ERR: direction must be 0 or 1\r\n");
        return;
    }
    if ((command->axis != (uint8_t)'Y') &&
        (command->axis != (uint8_t)'P')) {
        GimbalDebug_PrintText("[Gimbal] ERR: axis must be 'Y' or 'P'\r\n");
        return;
    }

    GimbalDebug_PrintCommandAck(command);

    /* GimbalDebug 协议第三字节以 0.1° 为单位，先换算为度，再交给公共三参数接口。 */
    angle_deg = (float)command->angle_x10 * 0.1f;
    status = Gimbal_SetRelativeByAxis(command->axis,
                                      command->direction,
                                      angle_deg);

    if (status != GIMBAL_STATUS_OK) {
        GimbalDebug_PrintText("[Gimbal] ERR: movement command rejected\r\n");
        return;
    }

    g_gimbal_debug_motion_active = 1U;
    g_gimbal_debug_motion_started_ms = HAL_GetTick();

    /* 位于限位或角度为 0 时可能无需产生实际 PWM 运动，也应返回当前角度。 */
    if (Gimbal_IsMoving() == 0U) {
        GimbalDebug_PrintCurrentAngles();
        g_gimbal_debug_motion_active = 0U;
    }
}

HAL_StatusTypeDef GimbalDebug_Init(void)
{
    HAL_StatusTypeDef status;

    if (huart1.Instance != USART1) {
        return HAL_ERROR;
    }

    g_gimbal_debug_head = 0U;
    g_gimbal_debug_tail = 0U;
    g_gimbal_debug_dropped_frames = 0U;
    g_gimbal_debug_invalid_bytes = 0U;
    g_gimbal_debug_reported_invalid_bytes = 0U;
    g_gimbal_debug_received_bytes = 0U;
    g_gimbal_debug_valid_frames = 0U;
    g_gimbal_debug_rx_stage = GIMBAL_DEBUG_RX_WAIT_AXIS;
    g_gimbal_debug_motion_active = 0U;
    g_gimbal_debug_motion_started_ms = 0U;
    g_gimbal_debug_initialized = 0U;
    g_gimbal_debug_last_heartbeat_ms = HAL_GetTick();

    /*
     * 仅使用 HAL 标准接收链：HAL_UART_Receive_IT() 开启接收，
     * USART1_IRQHandler() 只调用 HAL_UART_IRQHandler()。本模块不直接读取 SR/DR。
     */
    status = GimbalDebug_StartReceive();
    if (status != HAL_OK) {
        return status;
    }

    g_gimbal_debug_initialized = 1U;
    GimbalDebug_PrintText(
        "[Gimbal] USART1 ready (HAL IT): HEX [Y/P][dir 0/1][angle x10]\r\n");
    GimbalDebug_PrintCurrentAngles();

    return HAL_OK;
}

void GimbalDebug_Update(void)
{
    GimbalDebug_Command_t command;
    uint32_t invalid_bytes;
    uint32_t now_ms;

    if (g_gimbal_debug_initialized == 0U) {
        return;
    }

    /*
     * 初始 ready 信息可能在串口助手打开之前已经发完，因此在收到第一条完整命令前
     * 每秒重复发送 WAIT。若连 WAIT 都看不到，问题位于 TX/端口/固件烧录，而不是协议。
     */
    now_ms = HAL_GetTick();
    if ((g_gimbal_debug_valid_frames == 0U) &&
        ((now_ms - g_gimbal_debug_last_heartbeat_ms) >=
         GIMBAL_DEBUG_HEARTBEAT_MS)) {
        g_gimbal_debug_last_heartbeat_ms = now_ms;
        GimbalDebug_PrintText(
            "[Gimbal] WAIT USART1 HEX: P-0.5 = 50 00 05\r\n");
    }

    /*
     * 若收到 ASCII 字符串“50 00 05”、回车换行或噪声，逐字节解析器会丢弃它们。
     * 主循环侧只输出一次汇总警告，避免在 USART1 ISR 中进行阻塞发送。
     */
    invalid_bytes = g_gimbal_debug_invalid_bytes;
    if (invalid_bytes != g_gimbal_debug_reported_invalid_bytes) {
        g_gimbal_debug_reported_invalid_bytes = invalid_bytes;
        GimbalDebug_PrintText(
            "[Gimbal] WARN: invalid data; use HEX bytes, waiting for 59/50 header\r\n");
    }

    /* 云台驱动本身是非阻塞状态机，必须周期调用才能产生后续的 <=2° 步进。 */
    Gimbal_Update();

    /* 一条串口命令刚刚运动结束：先打印最终软件角度，再允许下一条命令执行。 */
    if ((g_gimbal_debug_motion_active != 0U) &&
        (Gimbal_IsMoving() == 0U)) {
        GimbalDebug_PrintCurrentAngles();
        g_gimbal_debug_motion_active = 0U;
    } else if ((g_gimbal_debug_motion_active != 0U) &&
               ((HAL_GetTick() - g_gimbal_debug_motion_started_ms) >=
                GIMBAL_DEBUG_MOTION_TIMEOUT_MS)) {
        /* 异常情况下不能无限等待，否则用户会误以为串口完全没有收到数据。 */
        Gimbal_Stop();
        GimbalDebug_PrintText("[Gimbal] ERR: movement timeout; stopped\r\n");
        GimbalDebug_PrintCurrentAngles();
        g_gimbal_debug_motion_active = 0U;
    }

    /* 严格串行执行命令，防止新命令在上一条未结束时修改其目标位置。 */
    /*
     * 没有调试命令在执行时立即处理下一帧。若此时矩形等自动轨迹正在运行，
     * Gimbal_SetRelative() 会按云台接口定义取消原轨迹，使串口调试可随时接管。
     */
    if ((g_gimbal_debug_motion_active == 0U) &&
        (GimbalDebug_Dequeue(&command) != 0U)) {
        GimbalDebug_ExecuteCommand(&command);
    }
}

void GimbalDebug_OnUartRxComplete(UART_HandleTypeDef *huart)
{
    uint8_t received_byte;
    uint8_t head;
    uint8_t next_head;

    if ((huart == NULL) || (huart->Instance != USART1)) {
        return;
    }

    received_byte = g_gimbal_debug_rx_byte;
    ++g_gimbal_debug_received_bytes;

    /*
     * 逐字节同步协议：只有 0x59('Y') 或 0x50('P') 才能开始一帧。
     * 回调只解析、入队和重新挂接接收，不执行阻塞发送或舵机运动。
     */
    if (g_gimbal_debug_rx_stage == GIMBAL_DEBUG_RX_WAIT_AXIS) {
        if ((received_byte == (uint8_t)'Y') ||
            (received_byte == (uint8_t)'P')) {
            g_gimbal_debug_pending_command.axis = received_byte;
            g_gimbal_debug_rx_stage = GIMBAL_DEBUG_RX_WAIT_DIRECTION;
        } else {
            ++g_gimbal_debug_invalid_bytes;
        }
    } else if (g_gimbal_debug_rx_stage == GIMBAL_DEBUG_RX_WAIT_DIRECTION) {
        if (received_byte <= 1U) {
            g_gimbal_debug_pending_command.direction = received_byte;
            g_gimbal_debug_rx_stage = GIMBAL_DEBUG_RX_WAIT_ANGLE;
        } else {
            ++g_gimbal_debug_invalid_bytes;
            if ((received_byte == (uint8_t)'Y') ||
                (received_byte == (uint8_t)'P')) {
                g_gimbal_debug_pending_command.axis = received_byte;
                g_gimbal_debug_rx_stage = GIMBAL_DEBUG_RX_WAIT_DIRECTION;
            } else {
                g_gimbal_debug_rx_stage = GIMBAL_DEBUG_RX_WAIT_AXIS;
            }
        }
    } else {
        g_gimbal_debug_pending_command.angle_x10 = received_byte;
        g_gimbal_debug_rx_stage = GIMBAL_DEBUG_RX_WAIT_AXIS;

        head = g_gimbal_debug_head;
        next_head = (uint8_t)((head + 1U) % GIMBAL_DEBUG_QUEUE_SIZE);
        if (next_head != g_gimbal_debug_tail) {
            g_gimbal_debug_queue[head] = g_gimbal_debug_pending_command;
            g_gimbal_debug_head = next_head;
            ++g_gimbal_debug_valid_frames;
        } else {
            ++g_gimbal_debug_dropped_frames;
        }
    }

    /* HAL 完成一个字节后 RxState 已恢复 READY，可以立即挂接下一字节。 */
    if (GimbalDebug_StartReceive() != HAL_OK) {
        ++g_gimbal_debug_dropped_frames;
    }
}

void GimbalDebug_OnUartError(UART_HandleTypeDef *huart)
{
    if ((huart == NULL) || (huart->Instance != USART1)) {
        return;
    }

    /* 丢弃半帧并让 HAL 结束当前接收，然后重新挂接单字节接收。 */
    g_gimbal_debug_rx_stage = GIMBAL_DEBUG_RX_WAIT_AXIS;
    (void)HAL_UART_AbortReceive(huart);
    if (GimbalDebug_StartReceive() != HAL_OK) {
        ++g_gimbal_debug_dropped_frames;
    }
}

uint32_t GimbalDebug_GetDroppedFrameCount(void)
{
    return g_gimbal_debug_dropped_frames;
}