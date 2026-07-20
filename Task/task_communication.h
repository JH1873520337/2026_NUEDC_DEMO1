#ifndef TASK_COMMUNICATION_H
#define TASK_COMMUNICATION_H

#include "FreeRTOS.h"
#include "task.h"
#include "task_control.h"

#ifdef __cplusplus
extern "C" {
#endif

/* This task is a protocol-neutral broker. UART/screen drivers remain above it. */
#define TASK_COMMUNICATION_QUEUE_LENGTH  8U
/* The broker only copies a small command, so it does not need a large stack. */
#define TASK_COMMUNICATION_STACK_WORDS   256U
#define TASK_COMMUNICATION_PRIORITY      (tskIDLE_PRIORITY + 2U)
#define TASK_COMMUNICATION_FORWARD_TIMEOUT_MS 10U

BaseType_t Task_Communication_Create(void);
void Task_Communication_Entry(void *argument);
BaseType_t Task_Communication_Submit(const Task_ControlCommand_t *command,
                                     TickType_t timeout_ticks);
BaseType_t Task_Communication_SubmitFromISR(
    const Task_ControlCommand_t *command,
    BaseType_t *higher_priority_task_woken);

typedef struct {
    uint32_t forwarded_commands;
    uint32_t dropped_commands;
} Task_CommunicationStatus_t;

BaseType_t Task_Communication_GetStatus(Task_CommunicationStatus_t *status);

#ifdef __cplusplus
}
#endif

#endif /* TASK_COMMUNICATION_H */
