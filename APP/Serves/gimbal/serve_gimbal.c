#include "serve_gimbal.h"

#include "Servo.h"

// SERVE_GIMBAL_DEFAULT_PAN_MIN_ANGLE：水平旋转最小角度，0°
// SERVE_GIMBAL_DEFAULT_PAN_MAX_ANGLE：水平旋转最大角度，180°
// SERVE_GIMBAL_DEFAULT_TILT_MIN_ANGLE：俯仰最小角度，0°
// SERVE_GIMBAL_DEFAULT_TILT_MAX_ANGLE：俯仰最大角度，180°
// SERVE_GIMBAL_DEFAULT_CENTER_ANGLE：舵机中位角度，90°
// SERVE_GIMBAL_DEFAULT_MAX_SPEED：最大运动速度，通常为 60°/s
// SERVE_GIMBAL_DEFAULT_ACCELERATION：运动加速度，通常为 120°/s²
// SERVE_GIMBAL_DEFAULT_PROCESS_PERIOD_MS：云台控制程序的执行周期，每 10 ms 更新一次
#define SERVE_GIMBAL_DEFAULT_PAN_MIN_ANGLE      0.0f
#define SERVE_GIMBAL_DEFAULT_PAN_MAX_ANGLE      180.0f
#define SERVE_GIMBAL_DEFAULT_TILT_MIN_ANGLE     0.0f
#define SERVE_GIMBAL_DEFAULT_TILT_MAX_ANGLE     180.0f
#define SERVE_GIMBAL_DEFAULT_CENTER_ANGLE       90.0f
#define SERVE_GIMBAL_DEFAULT_MAX_SPEED          60.0f
#define SERVE_GIMBAL_DEFAULT_ACCELERATION       120.0f
#define SERVE_GIMBAL_DEFAULT_PROCESS_PERIOD_MS  10U

typedef struct {
    Serve_Gimbal_Config_t config;
    float pan_target_angle;
    float tilt_target_angle;
    uint32_t last_process_ms;
    uint8_t process_started;
    uint8_t initialized;
} Serve_Gimbal_Context_t;

static Serve_Gimbal_Context_t Gimbal_Context;

static const Serve_Gimbal_Config_t Gimbal_DefaultConfig = {
    SERVE_GIMBAL_DEFAULT_PAN_MIN_ANGLE,
    SERVE_GIMBAL_DEFAULT_PAN_MAX_ANGLE,
    SERVE_GIMBAL_DEFAULT_TILT_MIN_ANGLE,
    SERVE_GIMBAL_DEFAULT_TILT_MAX_ANGLE,
    SERVE_GIMBAL_DEFAULT_CENTER_ANGLE,
    SERVE_GIMBAL_DEFAULT_CENTER_ANGLE,
    SERVE_GIMBAL_DEFAULT_MAX_SPEED,
    SERVE_GIMBAL_DEFAULT_ACCELERATION,
    SERVE_GIMBAL_DEFAULT_PROCESS_PERIOD_MS,
};

static uint8_t Serve_Gimbal_IsConfigValid(
    const Serve_Gimbal_Config_t *config)
{
    if (config == NULL) {
        return 0U;
    }

    if ((config->pan_min_angle >= config->pan_max_angle) ||
        (config->tilt_min_angle >= config->tilt_max_angle)) {
        return 0U;
    }

    if ((config->pan_center_angle < config->pan_min_angle) ||
        (config->pan_center_angle > config->pan_max_angle) ||
        (config->tilt_center_angle < config->tilt_min_angle) ||
        (config->tilt_center_angle > config->tilt_max_angle)) {
        return 0U;
    }

    if ((config->max_speed <= 0.0f) ||
        (config->acceleration <= 0.0f) ||
        (config->process_period_ms == 0U)) {
        return 0U;
    }

    return 1U;
}

static uint8_t Serve_Gimbal_IsTargetValid(float pan_angle, float tilt_angle)
{
    const Serve_Gimbal_Config_t *config = &Gimbal_Context.config;

    return (pan_angle >= config->pan_min_angle) &&
           (pan_angle <= config->pan_max_angle) &&
           (tilt_angle >= config->tilt_min_angle) &&
           (tilt_angle <= config->tilt_max_angle);
}

Serve_Gimbal_Status_t Serve_Gimbal_Init(const Serve_Gimbal_Config_t *config)
{
    const Serve_Gimbal_Config_t *selected_config = config;

    if (selected_config == NULL) {
        selected_config = &Gimbal_DefaultConfig;
    }

    if (!Serve_Gimbal_IsConfigValid(selected_config)) {
        return SERVE_GIMBAL_INVALID_ARGUMENT;
    }

    if (Servo_Init() != HAL_OK) {
        return SERVE_GIMBAL_ERROR;
    }

    Gimbal_Context.config = *selected_config;
    Gimbal_Context.pan_target_angle = selected_config->pan_center_angle;
    Gimbal_Context.tilt_target_angle = selected_config->tilt_center_angle;
    Gimbal_Context.last_process_ms = 0U;
    Gimbal_Context.process_started = 0U;
    Gimbal_Context.initialized = 1U;

    Servo_SetGimbalAngle_Direct(selected_config->pan_center_angle,
                               selected_config->tilt_center_angle);

    return SERVE_GIMBAL_OK;
}

Serve_Gimbal_Status_t Serve_Gimbal_SetTarget(float pan_angle,
                                              float tilt_angle)
{
    if (!Gimbal_Context.initialized) {
        return SERVE_GIMBAL_NOT_INITIALIZED;
    }

    if (!Serve_Gimbal_IsTargetValid(pan_angle, tilt_angle)) {
        return SERVE_GIMBAL_INVALID_ARGUMENT;
    }

    Gimbal_Context.pan_target_angle = pan_angle;
    Gimbal_Context.tilt_target_angle = tilt_angle;

    Servo_SetGimbalAngle_Smooth(pan_angle,
                                tilt_angle,
                                Gimbal_Context.config.max_speed,
                                Gimbal_Context.config.acceleration);

    return SERVE_GIMBAL_OK;
}

Serve_Gimbal_Status_t Serve_Gimbal_SetTargetDirect(float pan_angle,
                                                    float tilt_angle)
{
    if (!Gimbal_Context.initialized) {
        return SERVE_GIMBAL_NOT_INITIALIZED;
    }

    if (!Serve_Gimbal_IsTargetValid(pan_angle, tilt_angle)) {
        return SERVE_GIMBAL_INVALID_ARGUMENT;
    }

    Gimbal_Context.pan_target_angle = pan_angle;
    Gimbal_Context.tilt_target_angle = tilt_angle;
    Servo_SetGimbalAngle_Direct(pan_angle, tilt_angle);

    return SERVE_GIMBAL_OK;
}

Serve_Gimbal_Status_t Serve_Gimbal_SetMotionProfile(float max_speed,
                                                     float acceleration)
{
    if (!Gimbal_Context.initialized) {
        return SERVE_GIMBAL_NOT_INITIALIZED;
    }

    if ((max_speed <= 0.0f) || (acceleration <= 0.0f)) {
        return SERVE_GIMBAL_INVALID_ARGUMENT;
    }

    Gimbal_Context.config.max_speed = max_speed;
    Gimbal_Context.config.acceleration = acceleration;

    return SERVE_GIMBAL_OK;
}

Serve_Gimbal_Status_t Serve_Gimbal_Center(void)
{
    if (!Gimbal_Context.initialized) {
        return SERVE_GIMBAL_NOT_INITIALIZED;
    }

    return Serve_Gimbal_SetTarget(Gimbal_Context.config.pan_center_angle,
                                  Gimbal_Context.config.tilt_center_angle);
}

Serve_Gimbal_Status_t Serve_Gimbal_Process(uint32_t now_ms)
{
    uint32_t elapsed_ms;

    if (!Gimbal_Context.initialized) {
        return SERVE_GIMBAL_NOT_INITIALIZED;
    }

    if (!Gimbal_Context.process_started) {
        Gimbal_Context.last_process_ms = now_ms;
        Gimbal_Context.process_started = 1U;
        Servo_Loop_Process();
        return SERVE_GIMBAL_OK;
    }

    elapsed_ms = now_ms - Gimbal_Context.last_process_ms;
    if (elapsed_ms < Gimbal_Context.config.process_period_ms) {
        return SERVE_GIMBAL_OK;
    }

    Gimbal_Context.last_process_ms +=
        (elapsed_ms / Gimbal_Context.config.process_period_ms) *
        Gimbal_Context.config.process_period_ms;
    Servo_Loop_Process();

    return SERVE_GIMBAL_OK;
}

Serve_Gimbal_Status_t Serve_Gimbal_GetState(Serve_Gimbal_State_t *state)
{
    if (!Gimbal_Context.initialized) {
        return SERVE_GIMBAL_NOT_INITIALIZED;
    }

    if (state == NULL) {
        return SERVE_GIMBAL_INVALID_ARGUMENT;
    }

    state->pan_angle = Servo_State[SERVO_PAN].current_angle;
    state->tilt_angle = Servo_State[SERVO_TILT].current_angle;
    state->pan_target_angle = Gimbal_Context.pan_target_angle;
    state->tilt_target_angle = Gimbal_Context.tilt_target_angle;
    state->is_moving = Servo_State[SERVO_PAN].is_moving ||
                       Servo_State[SERVO_TILT].is_moving;

    return SERVE_GIMBAL_OK;
}

void Serve_Gimbal_Stop(void)
{
    if (!Gimbal_Context.initialized) {
        return;
    }

    Servo_StopAll();
    Gimbal_Context.pan_target_angle = Servo_State[SERVO_PAN].current_angle;
    Gimbal_Context.tilt_target_angle = Servo_State[SERVO_TILT].current_angle;
}
