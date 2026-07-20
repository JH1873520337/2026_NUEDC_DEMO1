#include "task_control.h"

#include "queue.h"
#include "semphr.h"

static QueueHandle_t control_queue;
static TaskHandle_t control_task_handle;
static SemaphoreHandle_t gimbal_mutex;
static Task_ControlStatus_t control_status;

static BaseType_t Task_Control_ProcessCommand(const Task_ControlCommand_t *command)
{
    Gimbal_Status_t result;

    if (command == NULL) {
        return pdFALSE;
    }

    if (xSemaphoreTake(gimbal_mutex, portMAX_DELAY) != pdPASS) {
        control_status.dropped_commands++;
        return pdFALSE;
    }

    switch (command->type) {
    case TASK_CONTROL_COMMAND_STOP:
        Gimbal_Stop();
        result = GIMBAL_STATUS_OK;
        break;
    case TASK_CONTROL_COMMAND_START:
        result = Gimbal_Start();
        break;
    case TASK_CONTROL_COMMAND_PAUSE:
        result = Gimbal_Pause();
        break;
    case TASK_CONTROL_COMMAND_RESUME:
        result = Gimbal_Resume();
        break;
    case TASK_CONTROL_COMMAND_RESET:
        result = Gimbal_ResetToOrigin();
        break;
    case TASK_CONTROL_COMMAND_SET_ABSOLUTE:
        result = Gimbal_SetAbsolute(command->first, command->second);
        break;
    case TASK_CONTROL_COMMAND_SET_RELATIVE:
        result = Gimbal_SetRelative(command->first, command->second);
        break;
    case TASK_CONTROL_COMMAND_SET_VECTOR:
        result = Gimbal_SetVector(command->first, command->second);
        break;
    default:
        result = GIMBAL_STATUS_INVALID_ARGUMENT;
        break;
    }

    (void)xSemaphoreGive(gimbal_mutex);

    if (result != GIMBAL_STATUS_OK) {
        control_status.dropped_commands++;
        return pdFALSE;
    }

    control_status.accepted_commands++;
    return pdTRUE;
}

BaseType_t Task_Control_Create(void)
{
    if (control_queue != NULL || control_task_handle != NULL) {
        return pdPASS;
    }

    control_queue = xQueueCreate(TASK_CONTROL_QUEUE_LENGTH,
                                 sizeof(Task_ControlCommand_t));
    if (control_queue == NULL) {
        return pdFAIL;
    }

    gimbal_mutex = xSemaphoreCreateMutex();
    if (gimbal_mutex == NULL) {
        vQueueDelete(control_queue);
        control_queue = NULL;
        return pdFAIL;
    }

    control_status.accepted_commands = 0U;
    control_status.dropped_commands = 0U;
    control_status.initialized = 0U;

    if (xTaskCreate(Task_Control_Entry,
                    "TaskControl",
                    TASK_CONTROL_STACK_WORDS,
                    NULL,
                    TASK_CONTROL_PRIORITY,
                    &control_task_handle) != pdPASS) {
        vQueueDelete(control_queue);
        vSemaphoreDelete(gimbal_mutex);
        control_queue = NULL;
        gimbal_mutex = NULL;
        return pdFAIL;
    }

    return pdPASS;
}

BaseType_t Task_Control_Enqueue(const Task_ControlCommand_t *command,
                                TickType_t timeout_ticks)
{
    if (control_queue == NULL || command == NULL) {
        return pdFAIL;
    }
    return xQueueSend(control_queue, command, timeout_ticks);
}

BaseType_t Task_Control_EnqueueFromISR(const Task_ControlCommand_t *command,
                                        BaseType_t *higher_priority_task_woken)
{
    if (control_queue == NULL || command == NULL ||
        higher_priority_task_woken == NULL) {
        return pdFAIL;
    }
    return xQueueSendFromISR(control_queue, command, higher_priority_task_woken);
}

BaseType_t Task_Control_GetStatus(Task_ControlStatus_t *status)
{
    if (status == NULL || gimbal_mutex == NULL) {
        return pdFAIL;
    }
    if (xSemaphoreTake(gimbal_mutex, pdMS_TO_TICKS(10U)) != pdPASS) {
        return pdFAIL;
    }
    *status = control_status;
    status->gimbal = *Gimbal_GetState();
    (void)xSemaphoreGive(gimbal_mutex);
    return pdPASS;
}

void Task_Control_GimbalUpdate(void)
{
    if (gimbal_mutex == NULL || control_status.initialized == 0U) {
        return;
    }
    if (xSemaphoreTake(gimbal_mutex, pdMS_TO_TICKS(10U)) == pdPASS) {
        Gimbal_Update();
        (void)xSemaphoreGive(gimbal_mutex);
    }
}

void Task_Control_Entry(void *argument)
{
    TickType_t last_wake;
    Task_ControlCommand_t command;
    uint32_t processed;

    (void)argument;
    if (xSemaphoreTake(gimbal_mutex, portMAX_DELAY) != pdPASS) {
        control_status.initialized = 0U;
        vTaskDelete(NULL);
    }
    if (Gimbal_Init() != GIMBAL_STATUS_OK) {
        (void)xSemaphoreGive(gimbal_mutex);
        control_status.initialized = 0U;
        for (;;) {
            vTaskDelay(pdMS_TO_TICKS(1000U));
        }
    }
    control_status.initialized = 1U;
    (void)xSemaphoreGive(gimbal_mutex);
    last_wake = xTaskGetTickCount();

    for (;;) {
        processed = 0U;
        while (processed < TASK_CONTROL_MAX_COMMANDS_PER_CYCLE &&
               xQueueReceive(control_queue, &command, 0U) == pdPASS) {
            (void)Task_Control_ProcessCommand(&command);
            processed++;
        }

        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));
    }
}
