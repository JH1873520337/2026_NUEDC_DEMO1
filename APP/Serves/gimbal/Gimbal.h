/**
 * @file Gimbal.h
 * @brief 基于两个 DS3115 舵机的二维云台服务层接口。
 *
 * 坐标约定：
 * - Yaw 软件角：-90~90°，0° 为初始化正前方，正方向由机械安装决定；
 * - Pitch 软件角：-90~90°，0° 为初始化正前方，正方向由机械安装决定；
 * - 舵机机械角 = 云台软件角 + 90°。
 *
 * Gimbal_Update() 内部完成最多 2° 的双轴分段、调用 Servo_SetAngles()，
 * 并调用 Servo_Update() 推进一阶低通和卡尔曼滤波。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Servo.h"

#include <stdint.h>

/** 云台两轴软件角的安全范围和初始化中心。 */
#define GIMBAL_MIN_ANGLE_DEG       (-90.0f)
#define GIMBAL_MAX_ANGLE_DEG       (90.0f)
#define GIMBAL_CENTER_ANGLE_DEG    (0.0f)

/** 每次 Gimbal_Update() 允许任意一个舵机改变的最大角度。 */
#define GIMBAL_MAX_STEP_DEG        (2.0f)

/** 角度到达判断容差，取舵机 0.1° 分辨率的一半。 */
#define GIMBAL_ARRIVAL_TOLERANCE   (0.05f)

/** 云台公共接口返回状态。 */
typedef enum
{
    GIMBAL_STATUS_OK = 0,              /**< 操作成功。 */
    GIMBAL_STATUS_NOT_INITIALIZED,     /**< 尚未调用 Gimbal_Init()。 */
    GIMBAL_STATUS_INVALID_ARGUMENT,    /**< 输入包含无效数值或零向量。 */
    GIMBAL_STATUS_DRIVER_ERROR,        /**< 底层 Servo/PWM 驱动操作失败。 */
} Gimbal_Status_t;

/** 当前或最近一次运动命令的控制模式。 */
typedef enum
{
    GIMBAL_MODE_STOPPED = 0,  /**< 已由 Gimbal_Stop() 主动停止。 */
    GIMBAL_MODE_ABSOLUTE,     /**< 目标为绝对软件角。 */
    GIMBAL_MODE_RELATIVE,     /**< 目标为相对当前软件角的增量。 */
    GIMBAL_MODE_VECTOR,       /**< 沿输入向量方向运动到首个轴限位。 */
    GIMBAL_MODE_PAUSED,       /**< 暂停并保存原目标，等待继续。 */
    GIMBAL_MODE_RESETTING,    /**< 正在回原点，到达后等待启动。 */
    GIMBAL_MODE_ORIGIN_WAITING, /**< 已到原点，等待启动后恢复原目标。 */
} Gimbal_ControlMode_t;

/**
 * @brief 云台完整软件状态。
 *
 * current_* 来自 Servo 层当前 PWM 软件角减去 90°，并非编码器实测值。
 * target_* 是一次控制命令的最终目标；command_* 是本次分段后下发给
 * Servo 层的中间目标，两者在长距离运动期间通常不同。
 */
typedef struct
{
    float current_yaw_deg;       /**< 当前 Yaw 软件输出角，范围 -90~90°。 */
    float current_pitch_deg;     /**< 当前 Pitch 软件输出角，范围 -90~90°。 */
    float target_yaw_deg;        /**< 当前命令的最终 Yaw 目标。 */
    float target_pitch_deg;      /**< 当前命令的最终 Pitch 目标。 */
    float command_yaw_deg;       /**< 最近一次分段下发的 Yaw 中间目标。 */
    float command_pitch_deg;     /**< 最近一次分段下发的 Pitch 中间目标。 */
    float vector_x;              /**< 向量模式原始 X 分量，映射到 Yaw。 */
    float vector_y;              /**< 向量模式原始 Y 分量，映射到 Pitch。 */
    Gimbal_ControlMode_t mode;   /**< 当前或最近一次控制模式。 */
    uint8_t initialized;         /**< 非 0 表示初始化完成。 */
    uint8_t moving;              /**< 非 0 表示仍在向最终目标运动。 */
    uint8_t limit_reached;       /**< 向量模式到达任意轴限位后置 1。 */
    uint8_t driver_fault;        /**< 底层舵机设置失败后置 1。 */
    uint8_t paused;              /**< 非 0 表示存在暂停后可恢复的目标。 */
    uint8_t reset_in_progress;   /**< 非 0 表示正在执行回原点后续行。 */
    uint8_t resume_pending;      /**< 非 0 表示保存了待恢复的原目标。 */
    uint8_t waiting_start;       /**< 非 0 表示必须收到启动命令才继续。 */
} Gimbal_State_t;

/**
 * @brief 初始化舵机和云台软件坐标。
 *
 * 两个舵机被置于机械角 90°，随后 Yaw、Pitch 软件角均记录为 0°。
 */
Gimbal_Status_t Gimbal_Init(void);

/**
 * @brief 设置绝对软件角目标。
 * @param yaw_deg   Yaw 绝对目标，超出 -90~90° 时自动限幅。
 * @param pitch_deg Pitch 绝对目标，超出 -90~90° 时自动限幅。
 */
Gimbal_Status_t Gimbal_SetAbsolute(float yaw_deg, float pitch_deg);

/**
 * @brief 以调用瞬间的当前位置为基准设置相对运动量。
 *
 * @param delta_yaw_deg   Yaw 相对增量。
 * @param delta_pitch_deg Pitch 相对增量。
 * @note 不是以上一次最终目标为基准，运动过程中重发命令也不会累计旧目标。
 */
Gimbal_Status_t Gimbal_SetRelative(float delta_yaw_deg,
                                   float delta_pitch_deg);

/**
 * @brief 沿平面向量 (X,Y) 的方向运动，直到任意一轴到达 ±90°。
 *
 * X 映射到 Yaw，Y 映射到 Pitch。向量长度不表示距离，只使用其方向比例。
 * 驱动预先计算沿射线首先碰到的边界，并让两轴以同一缩放系数运动。
 *
 * @return 零向量或非有限浮点数返回 GIMBAL_STATUS_INVALID_ARGUMENT。
 */
Gimbal_Status_t Gimbal_SetVector(float x, float y);

/**
 * @brief 推进一次云台运动和底层舵机滤波。
 * @note 当前 FreeRTOS 更新任务每 5 ms 调用一次；使用本接口时不要额外调用 Servo_Update()。
 */
void Gimbal_Update(void);

/** 暂停当前运动并保持当前位置，同时保存最终目标供后续继续。 */
Gimbal_Status_t Gimbal_Pause(void);

/** 启动暂停或原点等待流程中保存的最终目标。 */
Gimbal_Status_t Gimbal_Start(void);

/** Gimbal_Start() 的兼容别名。 */
Gimbal_Status_t Gimbal_Resume(void);

/** 回到云台原点 (0,0)，到达后保持等待 Gimbal_Start()。 */
Gimbal_Status_t Gimbal_ResetToOrigin(void);

/** 立即取消旧目标，冻结在当前 PWM 软件角并保持输出。 */
void Gimbal_Stop(void);

/** 返回非 0 表示云台尚未到达本次命令的最终目标。 */
uint8_t Gimbal_IsMoving(void);

/** 获取云台内部状态的只读指针。 */
const Gimbal_State_t *Gimbal_GetState(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_H */
