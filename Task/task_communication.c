#include "task_communication.h"

#include "queue.h"

static QueueHandle_t communication_queue;
static TaskHandle_t communication_task_handle;
static Task_CommunicationStatus_t communication_status;

BaseType_t Task_Communication_Create(void)
{
    if (communication_queue != NULL || communication_task_handle != NULL) {
        return pdPASS;
    }

    communication_queue = xQueueCreate(TASK_COMMUNICATION_QUEUE_LENGTH,
                                       sizeof(Task_ControlCommand_t));
    if (communication_queue == NULL) {
        return pdFAIL;
    }

    communication_status.forwarded_commands = 0U;
    communication_status.dropped_commands = 0U;

    if (xTaskCreate(Task_Communication_Entry,
                    "TaskComm",
                    TASK_COMMUNICATION_STACK_WORDS,
                    NULL,
                    TASK_COMMUNICATION_PRIORITY,
                    &communication_task_handle) != pdPASS) {
        vQueueDelete(communication_queue);
        communication_queue = NULL;
        return pdFAIL;
    }

    return pdPASS;
}

BaseType_t Task_Communication_GetStatus(Task_CommunicationStatus_t *status)
{
    if (status == NULL) {
        return pdFAIL;
    }
    taskENTER_CRITICAL();
    *status = communication_status;
    taskEXIT_CRITICAL();
    return pdPASS;
}

BaseType_t Task_Communication_Submit(const Task_ControlCommand_t *command,
                                     TickType_t timeout_ticks)
{
    if (communication_queue == NULL || command == NULL) {
        return pdFAIL;
    }
    return xQueueSend(communication_queue, command, timeout_ticks);
}

BaseType_t Task_Communication_SubmitFromISR(
    const Task_ControlCommand_t *command,
    BaseType_t *higher_priority_task_woken)
{
    if (communication_queue == NULL || command == NULL ||
        higher_priority_task_woken == NULL) {
        return pdFAIL;
    }
    return xQueueSendFromISR(communication_queue, command,
                             higher_priority_task_woken);
}

void Task_Communication_Entry(void *argument)
{
    Task_ControlCommand_t command;

    (void)argument;
    for (;;) {
        if (xQueueReceive(communication_queue, &command, portMAX_DELAY) == pdPASS) {
            if (Task_Control_Enqueue(
                    &command,
                    pdMS_TO_TICKS(TASK_COMMUNICATION_FORWARD_TIMEOUT_MS)) == pdPASS) {
                communication_status.forwarded_commands++;
            } else {
                /* A full control queue is reported through the status API. */
                communication_status.dropped_commands++;
            }
        }
    }
}
