#ifndef TASK_MANAGER_H
#define TASK_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

typedef enum {
    TASK_MANAGER_STATE_IDLE = 0,
    TASK_MANAGER_STATE_TASK1,
    TASK_MANAGER_STATE_TASK2,
    TASK_MANAGER_STATE_A4_VISION,
    TASK_MANAGER_STATE_FAULT
} Task_Manager_State_t;

typedef struct {
    Task_Manager_State_t state;
    uint32_t task1_start_count;
    uint32_t task2_start_count;
    uint32_t a4_vision_start_count;
} Task_Manager_Runtime_t;

extern volatile Task_Manager_Runtime_t Task_Manager_Runtime;

/** @brief 串口屏任务调度入口，本函数不会返回。 */
void Task_Manager_Entry(void *argument);

#ifdef __cplusplus
}
#endif

#endif /* TASK_MANAGER_H */
