#ifndef TASK1_H
#define TASK1_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 第一题任务运行状态。
 */
typedef enum {
    TASK1_STATE_IDLE = 0,
    TASK1_STATE_RETURNING_CENTER,
    TASK1_STATE_COMPLETED,
    TASK1_STATE_FAULT
} TASK1_State_t;

/**
 * @brief 第一题实时运行信息，可加入 Live Expressions 观察。
 */
typedef struct {
    float pan_angle;       /**< 当前水平轴指令角度，单位：deg。 */
    float tilt_angle;      /**< 当前俯仰轴指令角度，单位：deg。 */
    TASK1_State_t state;   /**< 当前任务状态。 */
} TASK1_Runtime_t;

extern volatile TASK1_Runtime_t TASK1_Runtime;

/**
 * @brief 第一题 FreeRTOS 任务入口。
 *
 * 上电后将云台回中至水平轴 90 度、俯仰轴 90 度的初始位置，
 * 到位后保持不动。本函数不会返回。
 */
void TASK1_Entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK1_H */
