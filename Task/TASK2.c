#include "TASK2.h"

#include "FreeRTOS.h"
#include "task.h"

#include "serve_gimbal.h"
#include "serve_screen.h"

#define TASK2_PROCESS_PERIOD_MS       10U
#define TASK2_READY_HOLD_MS           1000U
#define TASK2_END_HOLD_MS             1000U
#define TASK2_LAP_DURATION_MS         24000U
#define TASK2_SCREEN_PERIOD_MS        100U
#define TASK2_SCREEN_TIMEOUT_MS       10U
#define TASK2_ERROR_DELAY_MS          1000U

typedef struct {
    float pan_angle;
    float tilt_angle;
} TASK2_Waypoint_t;

/*
 * 标定顺序：左下角 -> 下边中点 -> 右下角 -> 右边中点 -> 右上角
 *          -> 上边中点 -> 左上角 -> 左边中点 -> 左下角顶点。
 */
static const TASK2_Waypoint_t TASK2_Waypoints[] = {
    {76.2f, 103.0f},
    {89.2f, 103.2f},
    {101.4f, 103.8f},
    {102.5f, 90.3f},
    {102.7f, 77.3f},
    {90.8f, 76.4f},
    {77.0f, 75.6f},
    {76.2f, 89.6f},
    {76.2f, 103.0f},
};

#define TASK2_WAYPOINT_COUNT \
    ((uint32_t)(sizeof(TASK2_Waypoints) / sizeof(TASK2_Waypoints[0])))
#define TASK2_SEGMENT_COUNT       (TASK2_WAYPOINT_COUNT - 1U)
#define TASK2_SEGMENT_DURATION_MS (TASK2_LAP_DURATION_MS / \
                                   TASK2_SEGMENT_COUNT)

volatile TASK2_Runtime_t TASK2_Runtime;

static const Serve_Gimbal_Config_t TASK2_GimbalConfig = {
    .pan_min_angle = 0.0f,
    .pan_max_angle = 180.0f,
    .tilt_min_angle = 0.0f,
    .tilt_max_angle = 180.0f,
    .pan_center_angle = 90.0f,
    .tilt_center_angle = 90.0f,
    .max_speed = 30.0f,
    .acceleration = 60.0f,
    .process_period_ms = TASK2_PROCESS_PERIOD_MS,
};

static void TASK2_ErrorLoop(void)
{
    TASK2_Runtime.state = TASK2_STATE_FAULT;
    Serve_Gimbal_Stop();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK2_ERROR_DELAY_MS));
    }
}

static uint8_t TASK2_SetDirect(float pan_angle, float tilt_angle)
{
    if (Serve_Gimbal_SetTargetDirect(pan_angle, tilt_angle) !=
        SERVE_GIMBAL_OK) {
        return 0U;
    }

    TASK2_Runtime.pan_angle = pan_angle;
    TASK2_Runtime.tilt_angle = tilt_angle;
    return 1U;
}

static uint8_t TASK2_ProcessPath(uint32_t elapsed_ms)
{
    const TASK2_Waypoint_t *start;
    const TASK2_Waypoint_t *end;
    uint32_t segment_index;
    uint32_t segment_elapsed_ms;
    float progress;
    float pan_angle;
    float tilt_angle;

    if (elapsed_ms >= TASK2_LAP_DURATION_MS) {
        TASK2_Runtime.elapsed_ms = TASK2_LAP_DURATION_MS;
        TASK2_Runtime.waypoint_index = (uint8_t)(TASK2_WAYPOINT_COUNT - 1U);

        if (!TASK2_SetDirect(TASK2_Waypoints[TASK2_WAYPOINT_COUNT - 1U]
                                 .pan_angle,
                             TASK2_Waypoints[TASK2_WAYPOINT_COUNT - 1U]
                                 .tilt_angle)) {
            return 0U;
        }

        Serve_Gimbal_Stop();
        TASK2_Runtime.state = TASK2_STATE_HOLDING_END;
        return 1U;
    }

    segment_index = elapsed_ms / TASK2_SEGMENT_DURATION_MS;
    segment_elapsed_ms = elapsed_ms % TASK2_SEGMENT_DURATION_MS;
    progress = (float)segment_elapsed_ms /
               (float)TASK2_SEGMENT_DURATION_MS;

    start = &TASK2_Waypoints[segment_index];
    end = &TASK2_Waypoints[segment_index + 1U];
    pan_angle = start->pan_angle +
                (end->pan_angle - start->pan_angle) * progress;
    tilt_angle = start->tilt_angle +
                 (end->tilt_angle - start->tilt_angle) * progress;

    TASK2_Runtime.elapsed_ms = elapsed_ms;
    TASK2_Runtime.waypoint_index = (uint8_t)segment_index;
    return TASK2_SetDirect(pan_angle, tilt_angle);
}

static void TASK2_UpdateScreen(uint32_t now_ms,
                               uint32_t *last_screen_update_ms)
{
    if ((now_ms - *last_screen_update_ms) < TASK2_SCREEN_PERIOD_MS) {
        return;
    }

    *last_screen_update_ms = now_ms;

    /* 屏幕通信失败不影响路径运动，下个刷新周期继续发送。 */
    (void)Serve_Screen_UpdateAngles(TASK2_Runtime.pan_angle,
                                    TASK2_Runtime.tilt_angle,
                                    TASK2_SCREEN_TIMEOUT_MS);
}

void TASK2_Entry(void *argument)
{
    Serve_Gimbal_State_t gimbal_state;
    TickType_t last_wake_tick;
    uint32_t ready_start_ms = 0U;
    uint32_t path_start_ms = 0U;
    uint32_t end_hold_start_ms = 0U;
    uint32_t last_screen_update_ms = 0U;
    uint32_t now_ms;

    (void)argument;

    TASK2_Runtime.pan_angle = TASK2_GimbalConfig.pan_center_angle;
    TASK2_Runtime.tilt_angle = TASK2_GimbalConfig.tilt_center_angle;
    TASK2_Runtime.elapsed_ms = 0U;
    TASK2_Runtime.waypoint_index = 0U;
    TASK2_Runtime.state = TASK2_STATE_IDLE;

    if (Serve_Gimbal_Init(&TASK2_GimbalConfig) != SERVE_GIMBAL_OK) {
        TASK2_ErrorLoop();
    }

    if (Serve_Gimbal_SetTarget(TASK2_Waypoints[0].pan_angle,
                               TASK2_Waypoints[0].tilt_angle) !=
        SERVE_GIMBAL_OK) {
        TASK2_ErrorLoop();
    }

    TASK2_Runtime.pan_angle = TASK2_Waypoints[0].pan_angle;
    TASK2_Runtime.tilt_angle = TASK2_Waypoints[0].tilt_angle;
    TASK2_Runtime.state = TASK2_STATE_PREPARING;
    last_wake_tick = xTaskGetTickCount();

    for (;;) {
        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (Serve_Gimbal_Process(now_ms) != SERVE_GIMBAL_OK) {
            TASK2_ErrorLoop();
        }

        if (TASK2_Runtime.state == TASK2_STATE_PREPARING) {
            if (Serve_Gimbal_GetState(&gimbal_state) != SERVE_GIMBAL_OK) {
                TASK2_ErrorLoop();
            }

            TASK2_Runtime.pan_angle = gimbal_state.pan_angle;
            TASK2_Runtime.tilt_angle = gimbal_state.tilt_angle;
            if (!gimbal_state.is_moving) {
                ready_start_ms = now_ms;
                TASK2_Runtime.state = TASK2_STATE_READY;
            }
        } else if (TASK2_Runtime.state == TASK2_STATE_READY) {
            if ((now_ms - ready_start_ms) >= TASK2_READY_HOLD_MS) {
                path_start_ms = now_ms;
                TASK2_Runtime.state = TASK2_STATE_RUNNING;
            }
        } else if (TASK2_Runtime.state == TASK2_STATE_RUNNING) {
            if (!TASK2_ProcessPath(now_ms - path_start_ms)) {
                TASK2_ErrorLoop();
            }

            if (TASK2_Runtime.state == TASK2_STATE_HOLDING_END) {
                end_hold_start_ms = now_ms;
            }
        } else if (TASK2_Runtime.state == TASK2_STATE_HOLDING_END) {
            if ((now_ms - end_hold_start_ms) >= TASK2_END_HOLD_MS) {
                if (Serve_Gimbal_SetTarget(
                        TASK2_GimbalConfig.pan_center_angle,
                        TASK2_GimbalConfig.tilt_center_angle) !=
                    SERVE_GIMBAL_OK) {
                    TASK2_ErrorLoop();
                }

                TASK2_Runtime.state = TASK2_STATE_RETURNING_CENTER;
            }
        } else if (TASK2_Runtime.state == TASK2_STATE_RETURNING_CENTER) {
            if (Serve_Gimbal_GetState(&gimbal_state) != SERVE_GIMBAL_OK) {
                TASK2_ErrorLoop();
            }

            TASK2_Runtime.pan_angle = gimbal_state.pan_angle;
            TASK2_Runtime.tilt_angle = gimbal_state.tilt_angle;
            if (!gimbal_state.is_moving) {
                Serve_Gimbal_Stop();
                TASK2_Runtime.state = TASK2_STATE_COMPLETED;
            }
        }

        TASK2_UpdateScreen(now_ms, &last_screen_update_ms);
        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(TASK2_PROCESS_PERIOD_MS));
    }
}
