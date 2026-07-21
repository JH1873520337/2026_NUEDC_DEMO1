#include "TASK1.h"

#include "FreeRTOS.h"
#include "task.h"

#include "serve_gimbal.h"
#include "serve_screen.h"

#define TASK1_PROCESS_PERIOD_MS  10U
#define TASK1_SCREEN_PERIOD_MS   100U
#define TASK1_SCREEN_TIMEOUT_MS  10U
#define TASK1_ERROR_DELAY_MS     1000U
#define TASK1_CENTER_ANGLE_DEG   90.0f

volatile TASK1_Runtime_t TASK1_Runtime;

static const Serve_Gimbal_Config_t TASK1_GimbalConfig = {
    .pan_min_angle = 0.0f,
    .pan_max_angle = 180.0f,
    .tilt_min_angle = 0.0f,
    .tilt_max_angle = 180.0f,
    .pan_center_angle = TASK1_CENTER_ANGLE_DEG,
    .tilt_center_angle = TASK1_CENTER_ANGLE_DEG,
    .max_speed = 30.0f,
    .acceleration = 60.0f,
    .process_period_ms = TASK1_PROCESS_PERIOD_MS,
};

static void TASK1_ErrorLoop(void)
{
    TASK1_Runtime.state = TASK1_STATE_FAULT;
    Serve_Gimbal_Stop();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK1_ERROR_DELAY_MS));
    }
}

static void TASK1_ProcessScreenCommand(void)
{
    Serve_Screen_Command_t command;
    Serve_Screen_Status_t status;

    for (;;) {
        status = Serve_Screen_GetCommand(&command);
        if (status == SERVE_SCREEN_EMPTY) {
            return;
        }
        if (status != SERVE_SCREEN_OK) {
            return;
        }

        if (command == SERVE_SCREEN_CMD_HOME) {
            if (Serve_Gimbal_Center() != SERVE_GIMBAL_OK) {
                TASK1_ErrorLoop();
            }
            TASK1_Runtime.state = TASK1_STATE_RETURNING_CENTER;
        }
    }
}

static void TASK1_UpdateScreen(uint32_t now_ms,
                               uint32_t *last_screen_update_ms)
{
    if ((now_ms - *last_screen_update_ms) < TASK1_SCREEN_PERIOD_MS) {
        return;
    }

    *last_screen_update_ms = now_ms;

    /* 屏幕通信失败不影响回中，下个刷新周期继续发送。 */
    (void)Serve_Screen_UpdateAngles(TASK1_Runtime.pan_angle,
                                    TASK1_Runtime.tilt_angle,
                                    TASK1_SCREEN_TIMEOUT_MS);
}

void TASK1_Entry(void *argument)
{
    Serve_Gimbal_State_t gimbal_state;
    TickType_t last_wake_tick;
    uint32_t last_screen_update_ms = 0U;
    uint32_t now_ms;

    (void)argument;

    TASK1_Runtime.pan_angle = TASK1_GimbalConfig.pan_center_angle;
    TASK1_Runtime.tilt_angle = TASK1_GimbalConfig.tilt_center_angle;
    TASK1_Runtime.state = TASK1_STATE_IDLE;

    if (Serve_Gimbal_Init(&TASK1_GimbalConfig) != SERVE_GIMBAL_OK) {
        TASK1_ErrorLoop();
    }

    if (Serve_Gimbal_Center() != SERVE_GIMBAL_OK) {
        TASK1_ErrorLoop();
    }

    TASK1_Runtime.state = TASK1_STATE_RETURNING_CENTER;
    last_wake_tick = xTaskGetTickCount();

    for (;;) {
        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        TASK1_ProcessScreenCommand();

        if (Serve_Gimbal_Process(now_ms) != SERVE_GIMBAL_OK) {
            TASK1_ErrorLoop();
        }

        if (Serve_Gimbal_GetState(&gimbal_state) != SERVE_GIMBAL_OK) {
            TASK1_ErrorLoop();
        }

        TASK1_Runtime.pan_angle = gimbal_state.pan_angle;
        TASK1_Runtime.tilt_angle = gimbal_state.tilt_angle;

        if ((TASK1_Runtime.state == TASK1_STATE_RETURNING_CENTER) &&
            !gimbal_state.is_moving) {
            Serve_Gimbal_Stop();
            TASK1_Runtime.state = TASK1_STATE_COMPLETED;
        }

        TASK1_UpdateScreen(now_ms, &last_screen_update_ms);
        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(TASK1_PROCESS_PERIOD_MS));
    }
}
