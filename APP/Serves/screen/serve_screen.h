#ifndef SERVE_SCREEN_H
#define SERVE_SCREEN_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    SERVE_SCREEN_OK = 0,
    SERVE_SCREEN_ERROR,
    SERVE_SCREEN_BUSY,
    SERVE_SCREEN_TIMEOUT,
    SERVE_SCREEN_INVALID_ARGUMENT,
    SERVE_SCREEN_NOT_INITIALIZED,
    SERVE_SCREEN_EMPTY
} Serve_Screen_Status_t;

typedef enum {
    SERVE_SCREEN_CMD_HOME = 0,
    SERVE_SCREEN_CMD_PAUSE,
    SERVE_SCREEN_CMD_START,
    SERVE_SCREEN_CMD_SCREEN_EDGE,
    SERVE_SCREEN_CMD_A4_TARGET,
    SERVE_SCREEN_CMD_TRACK_START
} Serve_Screen_Command_t;

/**
 * @brief 将当前水平和垂直角度更新到串口屏。
 */
Serve_Screen_Status_t Serve_Screen_UpdateAngles(float horizontal_angle,
                                                  float vertical_angle,
                                                  uint32_t timeout_ms);

/** @brief 从串口屏事件队列读取一个按键命令。 */
Serve_Screen_Status_t Serve_Screen_GetCommand(Serve_Screen_Command_t *command);

#ifdef __cplusplus
}
#endif

#endif /* SERVE_SCREEN_H */
