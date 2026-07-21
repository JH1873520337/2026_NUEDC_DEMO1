#include "task_gimbal_test.h"

#include "FreeRTOS.h"
#include "task.h"

#include "serve_gimbal.h"

#define TASK_GIMBAL_TEST_PERIOD_MS       10U
#define TASK_GIMBAL_TEST_ERROR_DELAY_MS  1000U

typedef struct {
    float pan_angle;
    float tilt_angle;
    uint32_t hold_time_ms;
} Task_GimbalTest_Waypoint_t;

static const Serve_Gimbal_Config_t GimbalTest_Config = {
    .pan_min_angle = 45.0f,
    .pan_max_angle = 135.0f,
    .tilt_min_angle = 45.0f,
    .tilt_max_angle = 135.0f,
    .pan_center_angle = 90.0f,
    .tilt_center_angle = 90.0f,
    .max_speed = 45.0f,
    .acceleration = 90.0f,
    .process_period_ms = TASK_GIMBAL_TEST_PERIOD_MS,
};

static const Task_GimbalTest_Waypoint_t GimbalTest_Waypoints[] = {
    {90.0f, 90.0f, 2000U},
    {45.0f, 90.0f, 1000U},
    {135.0f, 90.0f, 1000U},
    {90.0f, 90.0f, 1000U},
    {90.0f, 45.0f, 1000U},
    {90.0f, 135.0f, 1000U},
    {90.0f, 90.0f, 1000U},
    {45.0f, 45.0f, 1000U},
    {135.0f, 135.0f, 1000U},
    {90.0f, 90.0f, 2000U},
};

static void Task_GimbalTest_ErrorLoop(void)
{
    Serve_Gimbal_Stop();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK_GIMBAL_TEST_ERROR_DELAY_MS));
    }
}

void Task_GimbalTest_Entry(void *argument)
{
    Serve_Gimbal_State_t gimbal_state;
    TickType_t last_wake_tick;
    uint32_t now_ms;
    uint32_t hold_start_ms = 0U;
    uint32_t waypoint_index = 0U;
    uint8_t holding = 0U;

    (void)argument;

    if (Serve_Gimbal_Init(&GimbalTest_Config) != SERVE_GIMBAL_OK) {
        Task_GimbalTest_ErrorLoop();
    }

    if (Serve_Gimbal_SetTarget(GimbalTest_Waypoints[0].pan_angle,
                               GimbalTest_Waypoints[0].tilt_angle) !=
        SERVE_GIMBAL_OK) {
        Task_GimbalTest_ErrorLoop();
    }

    last_wake_tick = xTaskGetTickCount();

    for (;;) {
        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (Serve_Gimbal_Process(now_ms) != SERVE_GIMBAL_OK) {
            Task_GimbalTest_ErrorLoop();
        }

        if (Serve_Gimbal_GetState(&gimbal_state) != SERVE_GIMBAL_OK) {
            Task_GimbalTest_ErrorLoop();
        }

        if (gimbal_state.is_moving) {
            holding = 0U;
        } else if (!holding) {
            hold_start_ms = now_ms;
            holding = 1U;
        } else if ((now_ms - hold_start_ms) >=
                   GimbalTest_Waypoints[waypoint_index].hold_time_ms) {
            waypoint_index++;
            if (waypoint_index >=
                (sizeof(GimbalTest_Waypoints) /
                 sizeof(GimbalTest_Waypoints[0]))) {
                waypoint_index = 0U;
            }

            if (Serve_Gimbal_SetTarget(
                    GimbalTest_Waypoints[waypoint_index].pan_angle,
                    GimbalTest_Waypoints[waypoint_index].tilt_angle) !=
                SERVE_GIMBAL_OK) {
                Task_GimbalTest_ErrorLoop();
            }

            holding = 0U;
        }

        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(TASK_GIMBAL_TEST_PERIOD_MS));
    }
}
