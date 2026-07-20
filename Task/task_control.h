#ifndef TASK_CONTROL_H
#define TASK_CONTROL_H

#include "FreeRTOS.h"
#include "task.h"
#include "Gimbal.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The control task is the single owner of the Gimbal service. */
/* Command dispatch and the separate gimbal update task both run every 10 ms. */
#define TASK_CONTROL_PERIOD_MS             10U
/* Eight values absorb short command bursts without hiding a full queue. */
#define TASK_CONTROL_QUEUE_LENGTH          8U
/* Gimbal plus two filtered servo channels need more than the minimal stack. */
#define TASK_CONTROL_STACK_WORDS           512U
#define TASK_CONTROL_PRIORITY              (tskIDLE_PRIORITY + 3U)
#define TASK_CONTROL_MAX_COMMANDS_PER_CYCLE 4U

typedef enum {
    TASK_CONTROL_COMMAND_STOP = 0,
    TASK_CONTROL_COMMAND_START,
    TASK_CONTROL_COMMAND_PAUSE,
    TASK_CONTROL_COMMAND_RESUME,
    TASK_CONTROL_COMMAND_RESET,
    TASK_CONTROL_COMMAND_SET_ABSOLUTE,
    TASK_CONTROL_COMMAND_SET_RELATIVE,
    TASK_CONTROL_COMMAND_SET_VECTOR
} Task_ControlCommandType_t;

typedef struct {
    Task_ControlCommandType_t type;
    float first;
    float second;
} Task_ControlCommand_t;

typedef struct {
    Gimbal_State_t gimbal;
    uint32_t accepted_commands;
    uint32_t dropped_commands;
    uint8_t initialized;
} Task_ControlStatus_t;

BaseType_t Task_Control_Create(void);
void Task_Control_Entry(void *argument);
BaseType_t Task_Control_Enqueue(const Task_ControlCommand_t *command,
                                TickType_t timeout_ticks);
BaseType_t Task_Control_EnqueueFromISR(const Task_ControlCommand_t *command,
                                        BaseType_t *higher_priority_task_woken);
BaseType_t Task_Control_GetStatus(Task_ControlStatus_t *status);
void Task_Control_GimbalUpdate(void);

#ifdef __cplusplus
}
#endif

#endif /* TASK_CONTROL_H */
