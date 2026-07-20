/**
 * @file Gimbal.c
 * @brief 二维云台绝对、相对、向量控制及分段运动实现。
 *
 * 云台层只处理软件角和运动状态，不直接访问定时器。底层映射为：
 *   Yaw 软件角   + 90° -> Servo_Yaw   -> TIM1_CH3
 *   Pitch 软件角 + 90° -> Servo_Pitch -> TIM1_CH4
 *
 * 所有运动均为非阻塞式：控制接口设置最终目标，Gimbal_Update() 每次计算
 * 一个双轴中间目标，并保证任意一轴相对当前位置的变化不超过 2°。
 */
#include "Gimbal.h"

#include <float.h>

/** 小于该值的向量分量按 0 处理，避免极小数导致限位时间异常。 */
#define GIMBAL_VECTOR_EPSILON  (1.0e-6f)

/** 某一轴没有运动分量时，该轴到达限位所需的参数时间视为无穷大。 */
#define GIMBAL_NO_LIMIT_TIME   (FLT_MAX)

/** 云台单实例状态。所有公共接口默认由同一个 RTOS 任务串行调用。 */
static Gimbal_State_t g_gimbal_state;

/** 不依赖 libm 的浮点绝对值函数。 */
static float Gimbal_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 返回两个浮点值中的较小者。 */
static float Gimbal_Min(float first, float second)
{
    return (first < second) ? first : second;
}

/** 返回两个浮点值中的较大者。 */
static float Gimbal_Max(float first, float second)
{
    return (first > second) ? first : second;
}

/** 将角度限制在云台软件安全范围内。 */
static float Gimbal_Clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/**
 * 检查输入是否为有限浮点数。
 * value == value 用于排除 NaN；FLT_MAX 比较用于排除正负无穷大。
 */
static uint8_t Gimbal_IsFinite(float value)
{
    return ((value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX))
               ? 1U
               : 0U;
}

/**
 * 将带符号软件角限制到 -90~90°，并四舍五入到最近的 0.1°。
 * 负数分支在转换为整数前减 0.5，以适配 C 语言向 0 截断的转换规则。
 */
static float Gimbal_QuantizeAngle(float angle_deg)
{
    int32_t angle_tenths;

    angle_deg = Gimbal_Clamp(angle_deg,
                             GIMBAL_MIN_ANGLE_DEG,
                             GIMBAL_MAX_ANGLE_DEG);

    if (angle_deg >= 0.0f) {
        angle_tenths = (int32_t)(angle_deg * 10.0f + 0.5f);
    } else {
        angle_tenths = (int32_t)(angle_deg * 10.0f - 0.5f);
    }

    return (float)angle_tenths * SERVO_ANGLE_RESOLUTION_DEG;
}

/**
 * 从 Servo 层同步当前 PWM 软件角，并转换到以初始化正前方为 0° 的坐标。
 * 例如舵机机械角 120° 对应云台软件角 +30°。
 */
static void Gimbal_SyncCurrentFromServo(void)
{
    g_gimbal_state.current_yaw_deg =
        Gimbal_QuantizeAngle(Servo_GetCurrentAngle(SERVO_YAW) -
                             SERVO_CENTER_ANGLE_DEG);
    g_gimbal_state.current_pitch_deg =
        Gimbal_QuantizeAngle(Servo_GetCurrentAngle(SERVO_PITCH) -
                             SERVO_CENTER_ANGLE_DEG);
}

/** 判断单轴当前角和目标角是否处于同一个 0.1° 分辨率位置。 */
static uint8_t Gimbal_AxisArrived(float current_deg, float target_deg)
{
    return (Gimbal_Abs(current_deg - target_deg) <=
            GIMBAL_ARRIVAL_TOLERANCE)
               ? 1U
               : 0U;
}

/** 只有 Yaw、Pitch 两轴同时到达，整个云台运动才算完成。 */
static uint8_t Gimbal_TargetArrived(void)
{
    return ((Gimbal_AxisArrived(g_gimbal_state.current_yaw_deg,
                                g_gimbal_state.target_yaw_deg) != 0U) &&
            (Gimbal_AxisArrived(g_gimbal_state.current_pitch_deg,
                                g_gimbal_state.target_pitch_deg) != 0U))
               ? 1U
               : 0U;
}

/**
 * 统一建立一次新的最终目标。
 *
 * 最终目标先限幅并量化；中间指令从当前位置开始。若目标等于当前位置，
 * 则无需进入周期运动，直接冻结 Servo 滤波状态以防旧目标继续生效。
 */
static void Gimbal_SetMotionTarget(float yaw_deg,
                                   float pitch_deg,
                                   Gimbal_ControlMode_t mode)
{
    g_gimbal_state.target_yaw_deg = Gimbal_QuantizeAngle(yaw_deg);
    g_gimbal_state.target_pitch_deg = Gimbal_QuantizeAngle(pitch_deg);
    g_gimbal_state.command_yaw_deg = g_gimbal_state.current_yaw_deg;
    g_gimbal_state.command_pitch_deg = g_gimbal_state.current_pitch_deg;
    g_gimbal_state.mode = mode;
    g_gimbal_state.limit_reached = 0U;
    g_gimbal_state.driver_fault = 0U;
    g_gimbal_state.moving = (Gimbal_TargetArrived() == 0U) ? 1U : 0U;

    if (g_gimbal_state.moving == 0U) {
        Servo_StopAll();
    }
}

/**
 * 结束一次运动并冻结底层滤波器。
 * 向量模式的最终目标一定是首先碰到的轴边界，因此完成后置限位标志。
 */
static void Gimbal_FinishMotion(void)
{
    Gimbal_SyncCurrentFromServo();

    /* 到达判断已通过，可将状态吸附到量化后的精确最终目标。 */
    g_gimbal_state.current_yaw_deg = g_gimbal_state.target_yaw_deg;
    g_gimbal_state.current_pitch_deg = g_gimbal_state.target_pitch_deg;
    g_gimbal_state.command_yaw_deg = g_gimbal_state.target_yaw_deg;
    g_gimbal_state.command_pitch_deg = g_gimbal_state.target_pitch_deg;
    g_gimbal_state.moving = 0U;

    if (g_gimbal_state.mode == GIMBAL_MODE_VECTOR) {
        g_gimbal_state.limit_reached = 1U;
    }

    /* 保留 PWM 输出以提供舵机保持力，同时清除滤波历史。 */
    Servo_StopAll();
}

Gimbal_Status_t Gimbal_Init(void)
{
    /* Servo_Init() 会写入 90° 比较值并启动 TIM1_CH3/CH4。 */
    if (Servo_Init() != HAL_OK) {
        g_gimbal_state.initialized = 0U;
        g_gimbal_state.driver_fault = 1U;
        return GIMBAL_STATUS_DRIVER_ERROR;
    }

    /* 机械角 90° 映射为云台软件坐标原点 0°。 */
    g_gimbal_state.current_yaw_deg = GIMBAL_CENTER_ANGLE_DEG;
    g_gimbal_state.current_pitch_deg = GIMBAL_CENTER_ANGLE_DEG;
    g_gimbal_state.target_yaw_deg = GIMBAL_CENTER_ANGLE_DEG;
    g_gimbal_state.target_pitch_deg = GIMBAL_CENTER_ANGLE_DEG;
    g_gimbal_state.command_yaw_deg = GIMBAL_CENTER_ANGLE_DEG;
    g_gimbal_state.command_pitch_deg = GIMBAL_CENTER_ANGLE_DEG;
    g_gimbal_state.vector_x = 0.0f;
    g_gimbal_state.vector_y = 0.0f;
    g_gimbal_state.mode = GIMBAL_MODE_STOPPED;
    g_gimbal_state.initialized = 1U;
    g_gimbal_state.moving = 0U;
    g_gimbal_state.limit_reached = 0U;
    g_gimbal_state.driver_fault = 0U;

    return GIMBAL_STATUS_OK;
}

Gimbal_Status_t Gimbal_SetAbsolute(float yaw_deg, float pitch_deg)
{
    if (g_gimbal_state.initialized == 0U) {
        return GIMBAL_STATUS_NOT_INITIALIZED;
    }
    if ((Gimbal_IsFinite(yaw_deg) == 0U) ||
        (Gimbal_IsFinite(pitch_deg) == 0U)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    /* 新命令以 Servo 当前软件输出为运动起点，而不是旧的中间指令。 */
    Gimbal_SyncCurrentFromServo();
    g_gimbal_state.vector_x = 0.0f;
    g_gimbal_state.vector_y = 0.0f;
    Gimbal_SetMotionTarget(yaw_deg, pitch_deg, GIMBAL_MODE_ABSOLUTE);

    return GIMBAL_STATUS_OK;
}

Gimbal_Status_t Gimbal_SetRelative(float delta_yaw_deg,
                                   float delta_pitch_deg)
{
    if (g_gimbal_state.initialized == 0U) {
        return GIMBAL_STATUS_NOT_INITIALIZED;
    }
    if ((Gimbal_IsFinite(delta_yaw_deg) == 0U) ||
        (Gimbal_IsFinite(delta_pitch_deg) == 0U)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    /* 相对控制严格以调用瞬间的位置为零点，随后统一进行 ±90° 限幅。 */
    Gimbal_SyncCurrentFromServo();
    g_gimbal_state.vector_x = 0.0f;
    g_gimbal_state.vector_y = 0.0f;
    Gimbal_SetMotionTarget(g_gimbal_state.current_yaw_deg + delta_yaw_deg,
                           g_gimbal_state.current_pitch_deg + delta_pitch_deg,
                           GIMBAL_MODE_RELATIVE);

    return GIMBAL_STATUS_OK;
}

Gimbal_Status_t Gimbal_SetVector(float x, float y)
{
    float max_component;
    float direction_yaw;
    float direction_pitch;
    float yaw_limit_time = GIMBAL_NO_LIMIT_TIME;
    float pitch_limit_time = GIMBAL_NO_LIMIT_TIME;
    float limit_time;
    float target_yaw;
    float target_pitch;

    if (g_gimbal_state.initialized == 0U) {
        return GIMBAL_STATUS_NOT_INITIALIZED;
    }
    if ((Gimbal_IsFinite(x) == 0U) || (Gimbal_IsFinite(y) == 0U)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    /*
     * 使用无穷范数 max(|X|,|Y|) 归一化，既保持 X:Y 比例，也使两个方向
     * 分量都位于 [-1,1]。向量长度不参与目标距离计算。
     */
    max_component = Gimbal_Max(Gimbal_Abs(x), Gimbal_Abs(y));
    if (max_component <= GIMBAL_VECTOR_EPSILON) {
        /* 零向量没有运动方向，按立即停止处理。 */
        Gimbal_Stop();
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    Gimbal_SyncCurrentFromServo();
    direction_yaw = x / max_component;
    direction_pitch = y / max_component;

    /*
     * 云台在软件角平面中的射线方程：
     *   yaw(t)   = current_yaw   + direction_yaw   * t
     *   pitch(t) = current_pitch + direction_pitch * t
     *
     * 分别求两轴沿当前方向到达 ±90° 所需的正参数 t，取较小值即为
     * 首先发生的机械限位。分量为 0 的轴不会到达新限位，保持 FLT_MAX。
     */
    if (direction_yaw > GIMBAL_VECTOR_EPSILON) {
        yaw_limit_time =
            (GIMBAL_MAX_ANGLE_DEG - g_gimbal_state.current_yaw_deg) /
            direction_yaw;
    } else if (direction_yaw < -GIMBAL_VECTOR_EPSILON) {
        yaw_limit_time =
            (GIMBAL_MIN_ANGLE_DEG - g_gimbal_state.current_yaw_deg) /
            direction_yaw;
    }

    if (direction_pitch > GIMBAL_VECTOR_EPSILON) {
        pitch_limit_time =
            (GIMBAL_MAX_ANGLE_DEG - g_gimbal_state.current_pitch_deg) /
            direction_pitch;
    } else if (direction_pitch < -GIMBAL_VECTOR_EPSILON) {
        pitch_limit_time =
            (GIMBAL_MIN_ANGLE_DEG - g_gimbal_state.current_pitch_deg) /
            direction_pitch;
    }

    limit_time = Gimbal_Min(yaw_limit_time, pitch_limit_time);
    if ((limit_time == GIMBAL_NO_LIMIT_TIME) || (limit_time < 0.0f)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    /* 沿原向量方向计算首个限位点，最终仍会按 0.1° 量化。 */
    target_yaw = g_gimbal_state.current_yaw_deg +
                 direction_yaw * limit_time;
    target_pitch = g_gimbal_state.current_pitch_deg +
                   direction_pitch * limit_time;

    g_gimbal_state.vector_x = x;
    g_gimbal_state.vector_y = y;
    Gimbal_SetMotionTarget(target_yaw, target_pitch, GIMBAL_MODE_VECTOR);

    /* 如果命令下发时已经位于该方向的边界，则无需运动，直接报告限位。 */
    if (g_gimbal_state.moving == 0U) {
        g_gimbal_state.limit_reached = 1U;
    }

    return GIMBAL_STATUS_OK;
}

void Gimbal_Update(void)
{
    float yaw_error;
    float pitch_error;
    float max_error;
    float step_scale;

    if ((g_gimbal_state.initialized == 0U) ||
        (g_gimbal_state.moving == 0U)) {
        return;
    }

    /* 每个周期都从 Servo 层同步，保证状态基于实际写出的 PWM 软件角。 */
    Gimbal_SyncCurrentFromServo();

    if (Gimbal_TargetArrived() != 0U) {
        Gimbal_FinishMotion();
        return;
    }

    yaw_error = g_gimbal_state.target_yaw_deg -
                g_gimbal_state.current_yaw_deg;
    pitch_error = g_gimbal_state.target_pitch_deg -
                  g_gimbal_state.current_pitch_deg;

    /*
     * 两轴共享同一个 step_scale：
     *   scale = min(1, 2 / max(|yaw_error|, |pitch_error|))
     *
     * 因此较大误差轴每次最多移动 2°，另一轴按相同比例移动，不仅满足
     * “两个舵机每次都不大于 2°”，还可以保持二维轨迹的方向比例。
     */
    max_error = Gimbal_Max(Gimbal_Abs(yaw_error), Gimbal_Abs(pitch_error));
    step_scale = (max_error > GIMBAL_MAX_STEP_DEG)
                     ? (GIMBAL_MAX_STEP_DEG / max_error)
                     : 1.0f;

    g_gimbal_state.command_yaw_deg = Gimbal_QuantizeAngle(
        g_gimbal_state.current_yaw_deg + yaw_error * step_scale);
    g_gimbal_state.command_pitch_deg = Gimbal_QuantizeAngle(
        g_gimbal_state.current_pitch_deg + pitch_error * step_scale);

    /* 云台软件角加 90° 后转换为 Servo 层需要的 0~180° 机械角。 */
    if (Servo_SetAngles(g_gimbal_state.command_yaw_deg +
                            SERVO_CENTER_ANGLE_DEG,
                        g_gimbal_state.command_pitch_deg +
                            SERVO_CENTER_ANGLE_DEG) != HAL_OK) {
        g_gimbal_state.driver_fault = 1U;
        g_gimbal_state.moving = 0U;
        Servo_StopAll();
        return;
    }

    /* 推进一阶低通、卡尔曼滤波并更新 CCR3/CCR4。 */
    Servo_Update();
    Gimbal_SyncCurrentFromServo();

    if (Gimbal_TargetArrived() != 0U) {
        Gimbal_FinishMotion();
    }
}

void Gimbal_Stop(void)
{
    if (g_gimbal_state.initialized == 0U) {
        return;
    }

    /*
     * 先记录 Servo 当前软件输出，再冻结 Servo 的目标和两级滤波器，最后
     * 再同步一次，确保云台当前位置、目标和底层保持角完全一致。
     */
    Gimbal_SyncCurrentFromServo();
    Servo_StopAll();
    Gimbal_SyncCurrentFromServo();

    g_gimbal_state.target_yaw_deg = g_gimbal_state.current_yaw_deg;
    g_gimbal_state.target_pitch_deg = g_gimbal_state.current_pitch_deg;
    g_gimbal_state.command_yaw_deg = g_gimbal_state.current_yaw_deg;
    g_gimbal_state.command_pitch_deg = g_gimbal_state.current_pitch_deg;
    g_gimbal_state.vector_x = 0.0f;
    g_gimbal_state.vector_y = 0.0f;
    g_gimbal_state.mode = GIMBAL_MODE_STOPPED;
    g_gimbal_state.moving = 0U;
    g_gimbal_state.limit_reached = 0U;
}

uint8_t Gimbal_IsMoving(void)
{
    return g_gimbal_state.moving;
}

const Gimbal_State_t *Gimbal_GetState(void)
{
    /* 返回内部只读视图；调用者不应去除 const 后修改状态。 */
    return &g_gimbal_state;
}
