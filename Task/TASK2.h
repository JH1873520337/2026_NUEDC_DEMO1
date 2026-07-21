#ifndef TASK2_H
#define TASK2_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 第二题任务运行状态。
 */
typedef enum {
    TASK2_STATE_IDLE = 0,
    TASK2_STATE_PREPARING,
    TASK2_STATE_READY,
    TASK2_STATE_RUNNING,
    TASK2_STATE_HOLDING_END,
    TASK2_STATE_RETURNING_CENTER,
    TASK2_STATE_COMPLETED,
    TASK2_STATE_FAULT
} TASK2_State_t;

/**
 * @brief 第二题实时运行信息，可加入 Live Expressions 观察。
 */
typedef struct {
    float pan_angle;          /**< 当前水平轴指令角度，单位：deg。 */
    float tilt_angle;         /**< 当前俯仰轴指令角度，单位：deg。 */
    uint32_t elapsed_ms;      /**< 正式绕行已经运行的时间，单位：ms。 */
    uint8_t waypoint_index;   /**< 当前所在路径段的起点序号，范围 0~8。 */
    TASK2_State_t state;      /**< 当前任务状态。 */
} TASK2_Runtime_t;

extern volatile TASK2_Runtime_t TASK2_Runtime;

/**
 * @brief 第二题 FreeRTOS 任务入口。
 *
 * 上电后先移动到左下角，稳定 1 秒后顺时针绕行一圈。到达第 9 个
 * 左下角顶点后短暂停留，再返回 90 度中心零点并保持不动。
 * 本函数不会返回。
 */
void TASK2_Entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK2_H */
