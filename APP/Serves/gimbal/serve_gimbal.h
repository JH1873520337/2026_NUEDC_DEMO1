#ifndef SERVE_GIMBAL_H
#define SERVE_GIMBAL_H

/**
 * @file serve_gimbal.h
 * @brief 二维舵机云台服务接口。
 *
 * 本模块负责管理水平轴（Pan）和俯仰轴（Tilt）的安全角度范围、
 * 平滑运动参数、目标角度及运行状态。使用前必须先调用
 * Serve_Gimbal_Init()，并在任务中周期调用 Serve_Gimbal_Process()。
 *
 * @note 本模块没有内部互斥锁，所有接口默认在同一个任务上下文中调用。
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 云台接口执行结果。
 */
typedef enum {
    SERVE_GIMBAL_OK = 0,             /**< 操作成功。 */
    SERVE_GIMBAL_ERROR,              /**< 底层舵机初始化或控制发生错误。 */
    SERVE_GIMBAL_INVALID_ARGUMENT,   /**< 输入配置、角度、运动参数或指针无效。 */
    SERVE_GIMBAL_NOT_INITIALIZED     /**< 云台尚未完成初始化。 */
} Serve_Gimbal_Status_t;

/**
 * @brief 云台工作参数配置。
 *
 * 角度单位均为度（deg），速度单位为度每秒（deg/s），加速度单位为
 * 度每秒平方（deg/s^2）。最小角度必须小于最大角度，中位角必须位于
 * 对应轴的安全角度范围内。
 */
typedef struct {
    float pan_min_angle;          /**< 水平轴允许到达的最小角度，单位：deg。 */
    float pan_max_angle;          /**< 水平轴允许到达的最大角度，单位：deg。 */
    float tilt_min_angle;         /**< 俯仰轴允许到达的最小角度，单位：deg。 */
    float tilt_max_angle;         /**< 俯仰轴允许到达的最大角度，单位：deg。 */
    float pan_center_angle;       /**< 水平轴回中时的目标角度，单位：deg。 */
    float tilt_center_angle;      /**< 俯仰轴回中时的目标角度，单位：deg。 */
    float max_speed;              /**< 平滑运动的最大角速度，单位：deg/s，必须大于 0。 */
    float acceleration;          /**< 平滑运动的角加速度，单位：deg/s^2，必须大于 0。 */
    uint32_t process_period_ms;   /**< 底层运动计算的执行周期，单位：ms，必须大于 0。 */
} Serve_Gimbal_Config_t;

/**
 * @brief 云台当前运行状态。
 *
 * 当前角度来自底层舵机驱动保存的开环指令状态，并非编码器或传感器
 * 测得的真实机械角度。
 */
typedef struct {
    float pan_angle;          /**< 水平轴当前指令角度，单位：deg。 */
    float tilt_angle;         /**< 俯仰轴当前指令角度，单位：deg。 */
    float pan_target_angle;   /**< 水平轴最终目标角度，单位：deg。 */
    float tilt_target_angle;  /**< 俯仰轴最终目标角度，单位：deg。 */
    uint8_t is_moving;        /**< 运动标志：0 表示两轴均停止，非 0 表示至少一轴正在运动。 */
} Serve_Gimbal_State_t;

/**
 * @brief 初始化二维舵机云台。
 *
 * 初始化底层舵机驱动，保存配置参数，并将水平轴和俯仰轴直接设置到
 * 各自的中位角。调用其他云台接口前必须先调用本函数。
 *
 * @param config 云台配置指针；传入 NULL 时使用模块内置的默认配置。
 * @return SERVE_GIMBAL_OK 初始化成功。
 * @return SERVE_GIMBAL_INVALID_ARGUMENT 配置参数不符合要求。
 * @return SERVE_GIMBAL_ERROR 底层舵机初始化失败。
 */
Serve_Gimbal_Status_t Serve_Gimbal_Init(const Serve_Gimbal_Config_t *config);

/**
 * @brief 控制云台平滑移动到目标角度。
 *
 * 使用当前配置的最大速度和加速度规划两轴运动。目标角度超出配置的
 * 安全范围时直接返回错误，不会自动限幅。下发目标后仍需周期调用
 * Serve_Gimbal_Process() 才能持续执行平滑运动。
 *
 * @param pan_angle 水平轴目标角度，单位：deg。
 * @param tilt_angle 俯仰轴目标角度，单位：deg。
 * @return SERVE_GIMBAL_OK 目标设置成功。
 * @return SERVE_GIMBAL_INVALID_ARGUMENT 目标角度超出安全范围。
 * @return SERVE_GIMBAL_NOT_INITIALIZED 云台尚未初始化。
 */
Serve_Gimbal_Status_t Serve_Gimbal_SetTarget(float pan_angle,
                                              float tilt_angle);

/**
 * @brief 不经过平滑规划，直接设置云台角度。
 *
 * 该接口会立即更新两路舵机输出，适合初始化、校准和调试。带负载时
 * 频繁调用可能产生突变和机械冲击。目标角度超出安全范围时不会输出。
 *
 * @param pan_angle 水平轴目标角度，单位：deg。
 * @param tilt_angle 俯仰轴目标角度，单位：deg。
 * @return SERVE_GIMBAL_OK 角度设置成功。
 * @return SERVE_GIMBAL_INVALID_ARGUMENT 目标角度超出安全范围。
 * @return SERVE_GIMBAL_NOT_INITIALIZED 云台尚未初始化。
 */
Serve_Gimbal_Status_t Serve_Gimbal_SetTargetDirect(float pan_angle,
                                                    float tilt_angle);

/**
 * @brief 修改后续平滑运动使用的速度和加速度。
 *
 * 新参数只对之后通过 Serve_Gimbal_SetTarget() 下发的运动生效，不会
 * 重新规划已经开始执行的运动。
 *
 * @param max_speed 最大角速度，单位：deg/s，必须大于 0。
 * @param acceleration 角加速度，单位：deg/s^2，必须大于 0。
 * @return SERVE_GIMBAL_OK 运动参数设置成功。
 * @return SERVE_GIMBAL_INVALID_ARGUMENT 速度或加速度不大于 0。
 * @return SERVE_GIMBAL_NOT_INITIALIZED 云台尚未初始化。
 */
Serve_Gimbal_Status_t Serve_Gimbal_SetMotionProfile(float max_speed,
                                                     float acceleration);

/**
 * @brief 使用当前运动参数平滑返回配置的中位角。
 *
 * @return SERVE_GIMBAL_OK 回中目标设置成功。
 * @return SERVE_GIMBAL_NOT_INITIALIZED 云台尚未初始化。
 */
Serve_Gimbal_Status_t Serve_Gimbal_Center(void);

/**
 * @brief 周期执行云台平滑运动计算。
 *
 * 应在任务循环中持续调用。函数内部根据 process_period_ms 判断是否
 * 到达实际处理周期，调用间隔不足时会直接返回且不会重复更新舵机。
 * 本函数不阻塞，建议传入 HAL_GetTick() 获取的系统毫秒时间。
 *
 * @param now_ms 当前系统时间，单位：ms，允许 uint32_t 自然溢出回绕。
 * @return SERVE_GIMBAL_OK 本次处理完成或尚未到达处理周期。
 * @return SERVE_GIMBAL_NOT_INITIALIZED 云台尚未初始化。
 *
 * @note 默认只在任务上下文中调用，不要在中断服务函数中调用。
 */
Serve_Gimbal_Status_t Serve_Gimbal_Process(uint32_t now_ms);

/**
 * @brief 读取云台当前角度、目标角度和运动状态。
 *
 * @param state 状态输出指针，不能为 NULL。
 * @return SERVE_GIMBAL_OK 状态读取成功。
 * @return SERVE_GIMBAL_INVALID_ARGUMENT state 为 NULL。
 * @return SERVE_GIMBAL_NOT_INITIALIZED 云台尚未初始化。
 */
Serve_Gimbal_Status_t Serve_Gimbal_GetState(Serve_Gimbal_State_t *state);

/**
 * @brief 立即停止两轴当前的平滑运动。
 *
 * 停止后保持当前 PWM 输出位置，并将当前位置记录为新的目标角度。
 * 若云台尚未初始化，本函数不执行任何操作。
 */
void Serve_Gimbal_Stop(void);

#ifdef __cplusplus
}
#endif

#endif /* SERVE_GIMBAL_H */
