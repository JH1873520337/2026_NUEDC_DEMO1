/**
  ******************************************************************************
  * @file    Servo.h
  * @brief   Dual-axis servo gimbal driver
  ******************************************************************************
  */

#ifndef SERVO_H
#define SERVO_H

#ifdef __cplusplus
extern "C" {
#endif

#include "tim.h"

#define SERVO_COUNT             2U

#define SERVO_MIN_ANGLE         0.0f
#define SERVO_MAX_ANGLE         180.0f
#define SERVO_CENTER_ANGLE      90.0f
#define SERVO_MIN_PULSE_US      500U
#define SERVO_MAX_PULSE_US      2500U

#define SERVO_ARRIVAL_ANGLE     0.05f
#define SERVO_MIN_FILTER_TIME   0.02f
#define SERVO_MAX_FILTER_TIME   1.50f

typedef enum {
    SERVO_PAN = 0,
    SERVO_TILT,
} Servo_Id_t;

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float min_angle;
    float max_angle;
    uint16_t min_pulse_us;
    uint16_t max_pulse_us;
} Servo_Config_t;

typedef struct {
    float current_angle;
    float target_angle;
    float current_velocity;
    float max_velocity;
    float acceleration;
    float filter_time;
    uint32_t last_tick;
    uint8_t is_moving;
} Servo_State_t;

extern const Servo_Config_t Servo_Config[SERVO_COUNT];
extern Servo_State_t Servo_State[SERVO_COUNT];

HAL_StatusTypeDef Servo_Init(void);
void Servo_SetAngle_Direct(Servo_Id_t servo_id, float angle);
void Servo_SetGimbalAngle_Direct(float pan_angle, float tilt_angle);
void Servo_SetAngle_Smooth(Servo_Id_t servo_id, float target_angle,
                           float max_speed, float accel);
void Servo_SetGimbalAngle_Smooth(float pan_angle, float tilt_angle,
                                 float max_speed, float accel);
void Servo_Loop_Process(void);
void Servo_Stop(Servo_Id_t servo_id);
void Servo_StopAll(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVO_H */
