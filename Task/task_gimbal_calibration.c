#include "task_gimbal_calibration.h"

#include "FreeRTOS.h"
#include "task.h"

#include "main.h"
#include "serve_gimbal.h"
#include "serve_screen.h"
#include "tim.h"

#define TASK_GIMBAL_CALIBRATION_PERIOD_MS          10U
#define TASK_GIMBAL_CALIBRATION_BUTTON_DEBOUNCE_MS 200U
#define TASK_GIMBAL_CALIBRATION_ERROR_DELAY_MS     1000U
#define TASK_GIMBAL_CALIBRATION_SCREEN_PERIOD_MS   100U
#define TASK_GIMBAL_CALIBRATION_SCREEN_TIMEOUT_MS  10U

/* 每个 TIM2 编码器计数对应的角度，可根据编码器手感调整。 */
#define TASK_GIMBAL_CALIBRATION_ANGLE_STEP_DEG     0.1f

/* 编码器方向相反时改为 -1.0f。 */
#define TASK_GIMBAL_CALIBRATION_ENCODER_DIRECTION  1.0f

/* 机械结构允许舵机在完整角度范围内进行标定。 */
#define TASK_GIMBAL_CALIBRATION_MIN_ANGLE_DEG      0.0f
#define TASK_GIMBAL_CALIBRATION_MAX_ANGLE_DEG      180.0f
#define TASK_GIMBAL_CALIBRATION_CENTER_ANGLE_DEG   90.0f

volatile Task_GimbalCalibration_State_t GimbalCalibration_State;

static volatile uint8_t GimbalCalibration_AxisSwitchRequested;

static const Serve_Gimbal_Config_t GimbalCalibration_Config = {
    .pan_min_angle = TASK_GIMBAL_CALIBRATION_MIN_ANGLE_DEG,
    .pan_max_angle = TASK_GIMBAL_CALIBRATION_MAX_ANGLE_DEG,
    .tilt_min_angle = TASK_GIMBAL_CALIBRATION_MIN_ANGLE_DEG,
    .tilt_max_angle = TASK_GIMBAL_CALIBRATION_MAX_ANGLE_DEG,
    .pan_center_angle = TASK_GIMBAL_CALIBRATION_CENTER_ANGLE_DEG,
    .tilt_center_angle = TASK_GIMBAL_CALIBRATION_CENTER_ANGLE_DEG,
    .max_speed = 30.0f,
    .acceleration = 60.0f,
    .process_period_ms = TASK_GIMBAL_CALIBRATION_PERIOD_MS,
};

static float Task_GimbalCalibration_ClampAngle(float angle)
{
    if (angle < TASK_GIMBAL_CALIBRATION_MIN_ANGLE_DEG) {
        return TASK_GIMBAL_CALIBRATION_MIN_ANGLE_DEG;
    }
    if (angle > TASK_GIMBAL_CALIBRATION_MAX_ANGLE_DEG) {
        return TASK_GIMBAL_CALIBRATION_MAX_ANGLE_DEG;
    }

    return angle;
}

static void Task_GimbalCalibration_ErrorLoop(void)
{
    Serve_Gimbal_Stop();

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK_GIMBAL_CALIBRATION_ERROR_DELAY_MS));
    }
}

static uint8_t Task_GimbalCalibration_Init(void)
{
    GimbalCalibration_State.pan_angle =
        TASK_GIMBAL_CALIBRATION_CENTER_ANGLE_DEG;
    GimbalCalibration_State.tilt_angle =
        TASK_GIMBAL_CALIBRATION_CENTER_ANGLE_DEG;
    GimbalCalibration_State.encoder_delta = 0;
    GimbalCalibration_State.axis_switch_count = 0U;
    GimbalCalibration_State.selected_axis =
        TASK_GIMBAL_CALIBRATION_AXIS_PAN;
    GimbalCalibration_State.initialized = 0U;
    GimbalCalibration_AxisSwitchRequested = 0U;

    if (Serve_Gimbal_Init(&GimbalCalibration_Config) != SERVE_GIMBAL_OK) {
        return 0U;
    }

    __HAL_TIM_SET_COUNTER(&htim2, 0U);
    if (HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL) != HAL_OK) {
        Serve_Gimbal_Stop();
        return 0U;
    }

    GimbalCalibration_State.initialized = 1U;
    return 1U;
}

static void Task_GimbalCalibration_ProcessButton(uint32_t now_ms,
                                                  uint32_t *last_switch_ms)
{
    if (!GimbalCalibration_AxisSwitchRequested) {
        return;
    }

    GimbalCalibration_AxisSwitchRequested = 0U;
    if ((now_ms - *last_switch_ms) <
        TASK_GIMBAL_CALIBRATION_BUTTON_DEBOUNCE_MS) {
        return;
    }

    *last_switch_ms = now_ms;
    if (GimbalCalibration_State.selected_axis ==
        TASK_GIMBAL_CALIBRATION_AXIS_PAN) {
        GimbalCalibration_State.selected_axis =
            TASK_GIMBAL_CALIBRATION_AXIS_TILT;
    } else {
        GimbalCalibration_State.selected_axis =
            TASK_GIMBAL_CALIBRATION_AXIS_PAN;
    }

    GimbalCalibration_State.axis_switch_count++;
}

static uint8_t Task_GimbalCalibration_ProcessEncoder(uint32_t *last_count)
{
    uint32_t current_count;
    int32_t delta;
    float angle_change;

    current_count = __HAL_TIM_GET_COUNTER(&htim2);
    delta = (int32_t)(current_count - *last_count);
    *last_count = current_count;
    GimbalCalibration_State.encoder_delta = delta;

    if (delta == 0) {
        return 1U;
    }

    angle_change = (float)delta *
                   TASK_GIMBAL_CALIBRATION_ANGLE_STEP_DEG *
                   TASK_GIMBAL_CALIBRATION_ENCODER_DIRECTION;

    if (GimbalCalibration_State.selected_axis ==
        TASK_GIMBAL_CALIBRATION_AXIS_PAN) {
        GimbalCalibration_State.pan_angle =
            Task_GimbalCalibration_ClampAngle(
                GimbalCalibration_State.pan_angle + angle_change);
    } else {
        GimbalCalibration_State.tilt_angle =
            Task_GimbalCalibration_ClampAngle(
                GimbalCalibration_State.tilt_angle + angle_change);
    }

    return Serve_Gimbal_SetTargetDirect(
               GimbalCalibration_State.pan_angle,
               GimbalCalibration_State.tilt_angle) == SERVE_GIMBAL_OK;
}

void Task_GimbalCalibration_Entry(void *argument)
{
    TickType_t last_wake_tick;
    uint32_t last_encoder_count;
    uint32_t last_switch_ms;
    uint32_t last_screen_update_ms;
    uint32_t now_ms;

    (void)argument;

    if (!Task_GimbalCalibration_Init()) {
        Task_GimbalCalibration_ErrorLoop();
    }

    last_encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    last_switch_ms = now_ms -
                     TASK_GIMBAL_CALIBRATION_BUTTON_DEBOUNCE_MS;
    last_screen_update_ms = now_ms -
                            TASK_GIMBAL_CALIBRATION_SCREEN_PERIOD_MS;
    last_wake_tick = xTaskGetTickCount();

    for (;;) {
        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        Task_GimbalCalibration_ProcessButton(now_ms, &last_switch_ms);
        if (!Task_GimbalCalibration_ProcessEncoder(&last_encoder_count)) {
            Task_GimbalCalibration_ErrorLoop();
        }

        if ((now_ms - last_screen_update_ms) >=
            TASK_GIMBAL_CALIBRATION_SCREEN_PERIOD_MS) {
            last_screen_update_ms = now_ms;

            /* 屏幕通信失败不影响标定，下个刷新周期继续发送。 */
            (void)Serve_Screen_UpdateAngles(
                GimbalCalibration_State.pan_angle,
                GimbalCalibration_State.tilt_angle,
                TASK_GIMBAL_CALIBRATION_SCREEN_TIMEOUT_MS);
        }

        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(TASK_GIMBAL_CALIBRATION_PERIOD_MS));
    }
}

void Task_GimbalCalibration_ButtonIRQ(uint16_t gpio_pin)
{
    if (gpio_pin == GIMBAL_AXIS_BUTTON_Pin) {
        GimbalCalibration_AxisSwitchRequested = 1U;
    }
}
