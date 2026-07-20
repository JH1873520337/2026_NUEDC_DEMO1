/**
  ******************************************************************************
  * @file    Servo.c
  * @brief   Dual-axis servo gimbal driver
  ******************************************************************************
  */

#include "Servo.h"

#include <math.h>

const Servo_Config_t Servo_Config[SERVO_COUNT] = {
    {&htim1, TIM_CHANNEL_3, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
     SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US}, /* PE13: pan */
    {&htim1, TIM_CHANNEL_4, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE,
     SERVO_MIN_PULSE_US, SERVO_MAX_PULSE_US}, /* PE14: tilt */
};

Servo_State_t Servo_State[SERVO_COUNT];

static float Servo_ClampFloat(float value, float min_value, float max_value)
{
    if (value < min_value) {
        return min_value;
    }
    if (value > max_value) {
        return max_value;
    }
    return value;
}

static uint8_t Servo_IsValid(Servo_Id_t servo_id)
{
    return ((uint32_t)servo_id < SERVO_COUNT) &&
           (Servo_Config[servo_id].htim != NULL);
}

static float Servo_ClampAngle(Servo_Id_t servo_id, float angle)
{
    const Servo_Config_t *config = &Servo_Config[servo_id];

    return Servo_ClampFloat(angle, config->min_angle, config->max_angle);
}

static uint32_t Servo_AngleToPulse(Servo_Id_t servo_id, float angle)
{
    const Servo_Config_t *config = &Servo_Config[servo_id];
    float pulse;

    angle = Servo_ClampAngle(servo_id, angle);
    pulse = (float)config->min_pulse_us +
            (angle - config->min_angle) *
            (float)(config->max_pulse_us - config->min_pulse_us) /
            (config->max_angle - config->min_angle);

    return (uint32_t)(pulse + 0.5f);
}

static void Servo_WriteAngle(Servo_Id_t servo_id, float angle)
{
    const Servo_Config_t *config = &Servo_Config[servo_id];
    uint32_t pulse = Servo_AngleToPulse(servo_id, angle);

    /* TIM1 计数频率为 1 MHz，因此比较值与脉宽 us 数值相同。 */
    __HAL_TIM_SET_COMPARE(config->htim, config->channel, pulse);
}

static void Servo_ResetState(Servo_Id_t servo_id, float angle)
{
    Servo_State_t *state = &Servo_State[servo_id];

    state->current_angle = angle;
    state->target_angle = angle;
    state->current_velocity = 0.0f;
    state->max_velocity = 0.0f;
    state->acceleration = 0.0f;
    state->filter_time = SERVO_MIN_FILTER_TIME;
    state->last_tick = HAL_GetTick();
    state->is_moving = 0U;
}

HAL_StatusTypeDef Servo_Init(void)
{
    uint32_t i;

    for (i = 0U; i < SERVO_COUNT; i++) {
        Servo_Id_t servo_id = (Servo_Id_t)i;
        const Servo_Config_t *config = &Servo_Config[i];

        if (!Servo_IsValid(servo_id)) {
            return HAL_ERROR;
        }

        Servo_ResetState(servo_id, SERVO_CENTER_ANGLE);
        Servo_WriteAngle(servo_id, SERVO_CENTER_ANGLE);

        if (HAL_TIM_PWM_Start(config->htim, config->channel) != HAL_OK) {
            Servo_StopAll();
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

void Servo_SetAngle_Direct(Servo_Id_t servo_id, float angle)
{
    Servo_State_t *state;

    if (!Servo_IsValid(servo_id)) {
        return;
    }

    angle = Servo_ClampAngle(servo_id, angle);
    state = &Servo_State[servo_id];
    Servo_ResetState(servo_id, angle);
    state->last_tick = HAL_GetTick();
    Servo_WriteAngle(servo_id, angle);
}

void Servo_SetGimbalAngle_Direct(float pan_angle, float tilt_angle)
{
    Servo_SetAngle_Direct(SERVO_PAN, pan_angle);
    Servo_SetAngle_Direct(SERVO_TILT, tilt_angle);
}

void Servo_SetAngle_Smooth(Servo_Id_t servo_id, float target_angle,
                           float max_speed, float accel)
{
    Servo_State_t *state;
    float distance;

    if (!Servo_IsValid(servo_id)) {
        return;
    }

    target_angle = Servo_ClampAngle(servo_id, target_angle);
    max_speed = fabsf(max_speed);
    accel = fabsf(accel);

    if (max_speed <= 0.0f) {
        Servo_SetAngle_Direct(servo_id, target_angle);
        return;
    }

    state = &Servo_State[servo_id];
    state->target_angle = target_angle;
    state->max_velocity = max_speed;
    state->acceleration = accel;
    state->filter_time = (accel > 0.0f) ? (max_speed / accel) : 0.20f;
    state->filter_time = Servo_ClampFloat(state->filter_time,
                                          SERVO_MIN_FILTER_TIME,
                                          SERVO_MAX_FILTER_TIME);

    distance = fabsf(state->target_angle - state->current_angle);
    if (distance > SERVO_ARRIVAL_ANGLE) {
        if (!state->is_moving) {
            state->last_tick = HAL_GetTick();
        }
        state->is_moving = 1U;
    } else {
        Servo_SetAngle_Direct(servo_id, target_angle);
    }
}

void Servo_SetGimbalAngle_Smooth(float pan_angle, float tilt_angle,
                                 float max_speed, float accel)
{
    Servo_SetAngle_Smooth(SERVO_PAN, pan_angle, max_speed, accel);
    Servo_SetAngle_Smooth(SERVO_TILT, tilt_angle, max_speed, accel);
}

static void Servo_ProcessOne(Servo_Id_t servo_id, uint32_t current_tick)
{
    Servo_State_t *state = &Servo_State[servo_id];
    float dt;
    float error;
    float distance;
    float target_velocity;
    float alpha;
    float step;

    if (!state->is_moving) {
        return;
    }

    dt = (float)(current_tick - state->last_tick) / 1000.0f;
    if (dt <= 0.0f) {
        return;
    }
    if (dt > 0.05f) {
        dt = 0.02f;
    }
    state->last_tick = current_tick;

    error = state->target_angle - state->current_angle;
    distance = fabsf(error);
    if (distance <= SERVO_ARRIVAL_ANGLE &&
        fabsf(state->current_velocity) <= (state->max_velocity * 0.02f + 0.1f)) {
        state->current_angle = state->target_angle;
        state->current_velocity = 0.0f;
        state->is_moving = 0U;
        Servo_WriteAngle(servo_id, state->current_angle);
        return;
    }

    target_velocity = error / state->filter_time;
    target_velocity = Servo_ClampFloat(target_velocity,
                                       -state->max_velocity,
                                       state->max_velocity);
    alpha = dt / (state->filter_time + dt);
    state->current_velocity +=
        (target_velocity - state->current_velocity) * alpha;

    step = state->current_velocity * dt;
    if (fabsf(step) >= distance) {
        state->current_angle = state->target_angle;
        state->current_velocity = 0.0f;
        state->is_moving = 0U;
    } else {
        state->current_angle += step;
    }

    Servo_WriteAngle(servo_id, state->current_angle);
}

void Servo_Loop_Process(void)
{
    uint32_t i;
    uint32_t current_tick = HAL_GetTick();

    for (i = 0U; i < SERVO_COUNT; i++) {
        Servo_ProcessOne((Servo_Id_t)i, current_tick);
    }
}

void Servo_Stop(Servo_Id_t servo_id)
{
    Servo_State_t *state;

    if (!Servo_IsValid(servo_id)) {
        return;
    }

    state = &Servo_State[servo_id];
    state->target_angle = state->current_angle;
    state->current_velocity = 0.0f;
    state->is_moving = 0U;
}

void Servo_StopAll(void)
{
    uint32_t i;

    for (i = 0U; i < SERVO_COUNT; i++) {
        Servo_Stop((Servo_Id_t)i);
    }
}
