#ifndef TASK_A4_VISION_H
#define TASK_A4_VISION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    TASK_A4_VISION_STATE_WAITING_VISION = 0,
    TASK_A4_VISION_STATE_READY,
    TASK_A4_VISION_STATE_MOVING_TO_START,
    TASK_A4_VISION_STATE_RUNNING,
    TASK_A4_VISION_STATE_HOLDING_END,
    TASK_A4_VISION_STATE_COMPLETED,
    TASK_A4_VISION_STATE_PAUSED,
    TASK_A4_VISION_STATE_VISION_LOST,
    TASK_A4_VISION_STATE_FAULT
} Task_A4Vision_State_t;

typedef struct {
    float pan_angle;
    float tilt_angle;
    float target_x;
    float target_y;
    float laser_x;
    float laser_y;
    float error_x;
    float error_y;
    float pan_pid_output;
    float tilt_pid_output;
    uint32_t elapsed_ms;
    uint32_t valid_frame_count;
    uint32_t invalid_frame_count;
    uint8_t path_segment;
    uint8_t rectangle_sample_count;
    Task_A4Vision_State_t state;
} Task_A4Vision_Runtime_t;

extern volatile Task_A4Vision_Runtime_t Task_A4Vision_Runtime;

/**
 * @brief MODE1 目标点视觉闭环任务。
 *
 * 使用目标点和激光点坐标计算像素误差，通过双轴 PID 修正云台角度。
 * 目标点轨迹由视觉端生成，本任务实时闭环跟随。
 */
void Task_A4Vision_Entry(void *argument);

/** @brief 由任务管理器转发暂停按键；再次调用可继续运行。 */
void Task_A4Vision_RequestPause(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_A4_VISION_H */
