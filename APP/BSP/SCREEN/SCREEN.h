#ifndef SCREEN_H
#define SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    SCREEN_OK = 0,
    SCREEN_ERROR,
    SCREEN_BUSY,
    SCREEN_TIMEOUT,
    SCREEN_EMPTY,
    SCREEN_INVALID_ARGUMENT,
    SCREEN_NOT_INITIALIZED
} SCREEN_Status_t;

typedef enum {
    SCREEN_CMD_HOME = 0xA0,
    SCREEN_CMD_PAUSE = 0xA1,
    SCREEN_CMD_START = 0xA2,
    SCREEN_CMD_SCREEN_EDGE = 0xA3,
    SCREEN_CMD_A4_TARGET = 0xA4,
    SCREEN_CMD_TRACK_START = 0xA5
} SCREEN_Command_t;

typedef struct {
    uint32_t valid_frames;
    uint32_t invalid_commands;
    uint32_t queue_overflows;
    uint32_t uart_errors;
    uint32_t receive_restart_errors;
    uint32_t transmitted_commands;
    uint32_t transmit_errors;
} SCREEN_Stats_t;

/**
 * @brief 初始化串口屏接收，并启动 UART5 单字节中断接收。
 * @return SCREEN_OK 初始化成功。
 * @return SCREEN_ERROR HAL 接收启动失败。
 */
SCREEN_Status_t SCREEN_Init(void);

/**
 * @brief 从接收队列取出一个按键命令。
 * @param command 命令输出指针。
 * @return SCREEN_OK 成功取出命令。
 * @return SCREEN_EMPTY 当前没有待处理命令。
 */
SCREEN_Status_t SCREEN_GetCommand(SCREEN_Command_t *command);

/**
 * @brief 更新串口屏水平角度文本，保留 1 位小数。
 * @param angle 水平角度，允许范围 -9999.9 至 9999.9 度。
 * @param timeout_ms UART 阻塞发送超时时间，必须大于 0。
 */
SCREEN_Status_t SCREEN_SendHorizontalAngle(float angle,
                                           uint32_t timeout_ms);

/**
 * @brief 更新串口屏垂直角度文本，保留 1 位小数。
 */
SCREEN_Status_t SCREEN_SendVerticalAngle(float angle,
                                         uint32_t timeout_ms);

/**
 * @brief 依次更新水平和垂直角度文本。
 * @note 最坏阻塞时间为两倍 timeout_ms。
 */
SCREEN_Status_t SCREEN_SendAngles(float horizontal_angle,
                                  float vertical_angle,
                                  uint32_t timeout_ms);

/**
 * @brief 获取通信统计信息，便于联调定位丢帧和串口错误。
 */
SCREEN_Status_t SCREEN_GetStats(SCREEN_Stats_t *stats);

/**
 * @brief HAL UART 接收完成回调入口，仅由 UART5 HAL 回调调用。
 * @note 本函数运行在中断上下文，只解析和投递事件。
 */
void SCREEN_UART_RxCpltCallback(void);

/**
 * @brief HAL UART 错误回调入口，仅由 UART5 HAL 回调调用。
 */
void SCREEN_UART_ErrorCallback(void);

#ifdef __cplusplus
}
#endif

#endif /* SCREEN_H */
