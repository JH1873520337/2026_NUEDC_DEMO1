#include "task_vision_debug.h"

#include "FreeRTOS.h"
#include "task.h"

#include "serve_vision_debug.h"

#define TASK_VISION_DEBUG_PERIOD_MS 5U

void Task_VisionDebug_Entry(void *argument)
{
    TickType_t last_wake;

    (void)argument;
    Serve_VisionDebug_Init();
    last_wake = xTaskGetTickCount();

    for (;;) {
        (void)Serve_VisionDebug_Process((uint32_t)xTaskGetTickCount());
        vTaskDelayUntil(
            &last_wake,
            pdMS_TO_TICKS(TASK_VISION_DEBUG_PERIOD_MS)
        );
    }
}
