/**
 * @file Servo.c
 * @brief DS3115 双通道舵机驱动实现。
 *
 * 数据通路如下：
 *   用户目标角
 *      -> 0~180° 限幅及 0.1° 量化
 *      -> 一阶低通滤波
 *      -> 一维卡尔曼滤波
 *      -> 0.1° 再量化
 *      -> 机械零位补偿与安装方向反转
 *      -> TIM1 CCR3/CCR4
 *
 * 注意：滤波对象是“角度控制指令”，不是传感器反馈。current_angle_deg
 * 代表当前 PWM 指令角度，无法反映堵转、负载过大等真实机械误差。
 */
#include "Servo.h"

/** 0~180° 按 0.1° 划分后共有 1800 个步进。 */
#define SERVO_TOTAL_ANGLE_STEPS  1800UL

/** 到达判断和滤波器吸附目标时使用的半个最小分辨率容差。 */
#define SERVO_ARRIVAL_TOLERANCE  0.05f

/**
 * 两个舵机的固定硬件表。
 *
 * 比较值使用整数四舍五入公式：
 *   compare = pulse_us * counter_hz / 1000000
 * 当前参数得到：500 us -> 450，2500 us -> 2250。
 */
static const Servo_Config_t g_servo_config[SERVO_COUNT] = {
    /* SERVO_YAW 的状态、滤波器、反向和零位补偿只驱动实际 Yaw 通道 CH4。 */
    {&htim1, SERVO_YAW_PWM_CHANNEL,
     (SERVO_MIN_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL,
     (SERVO_MAX_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL},

    /* SERVO_PITCH 的状态、滤波器、反向和零位补偿只驱动实际 Pitch 通道 CH3。 */
    {&htim1, SERVO_PITCH_PWM_CHANNEL,
     (SERVO_MIN_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL,
     (SERVO_MAX_PULSE_US * SERVO_TIMER_COUNTER_HZ + 500000UL) / 1000000UL},
};

/** 两个通道各自独立的目标、滤波和当前输出状态。 */
static Servo_State_t g_servo_state[SERVO_COUNT];

/** 避免依赖 libm 的轻量绝对值函数。 */
static float Servo_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}

/** 把浮点值限制在 [minimum, maximum] 闭区间。 */
static float Servo_Clamp(float value, float minimum, float maximum)
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
 * 将机械角限制到 0~180°，随后四舍五入到最近的 0.1°。
 * 舵机角不存在负数，因此可以使用无符号整数保存十分之一度。
 */
static float Servo_QuantizeAngle(float angle_deg)
{
    uint32_t angle_tenths;

    angle_deg = Servo_Clamp(angle_deg,
                            SERVO_MIN_ANGLE_DEG,
                            SERVO_MAX_ANGLE_DEG);
    angle_tenths = (uint32_t)(angle_deg * 10.0f + 0.5f);
    return (float)angle_tenths * SERVO_ANGLE_RESOLUTION_DEG;
}

/** 检查逻辑编号是否落在 g_servo_config/g_servo_state 的有效范围内。 */
static uint8_t Servo_IsValid(Servo_Id_t servo_id)
{
    return ((uint32_t)servo_id < SERVO_COUNT) ? 1U : 0U;
}

/**
 * 把 HAL 通道编号转换成 TIMx_CCER 中的通道使能位。
 * Servo_StartPwm() 使用该位判断通道是否已经由 main.c 或其他模块启动。
 */
static uint32_t Servo_ChannelEnableMask(uint32_t channel)
{
    switch (channel) {
    case TIM_CHANNEL_1:
        return TIM_CCER_CC1E;
    case TIM_CHANNEL_2:
        return TIM_CCER_CC2E;
    case TIM_CHANNEL_3:
        return TIM_CCER_CC3E;
    case TIM_CHANNEL_4:
        return TIM_CCER_CC4E;
    default:
        return 0U;
    }
}

/**
 * 将 0~180° 映射到当前通道的 CCR 范围。
 *
 * angle_tenths 为 0~1800；compare_range 为 2250-450=1800，故当前硬件
 * 参数下每增加 0.1°，比较值恰好增加 1。公式仍保留通用整数四舍五入，
 * 以后校准不同舵机最小/最大脉宽时也能正常使用。
 */
static uint32_t Servo_AngleToCompare(Servo_Id_t servo_id, float angle_deg)
{
    const Servo_Config_t *config = &g_servo_config[servo_id];
    uint32_t angle_tenths;
    uint32_t compare_range;
    float zero_trim_deg;

    /*
     * 零位补偿必须放在逻辑角状态之外：上层仍把 90° 当作云台 0°，这里只在
     * 写 CCR 前修正实际脉宽。当前实机复测后的最终补偿为 Yaw -6°、Pitch -2°，
     * 使逻辑中心 90°对应正确机械零位，同时上层软件角仍保持 0°。
     */
    zero_trim_deg = (servo_id == SERVO_YAW)
                        ? SERVO_YAW_ZERO_TRIM_DEG
                        : SERVO_PITCH_ZERO_TRIM_DEG;
    angle_deg = Servo_QuantizeAngle(angle_deg + zero_trim_deg);

    /*
     * 根据实际安装方向在最终 PWM 映射处镜像角度：0°<->180°，90°保持不变。
     * 软件状态仍保存反转前的逻辑角，避免上层云台坐标系和到达判断被硬件方向污染。
     */
    if (((servo_id == SERVO_YAW) && (SERVO_YAW_REVERSED != 0U)) ||
        ((servo_id == SERVO_PITCH) && (SERVO_PITCH_REVERSED != 0U))) {
        angle_deg = SERVO_MIN_ANGLE_DEG + SERVO_MAX_ANGLE_DEG - angle_deg;
    }

    angle_tenths = (uint32_t)(angle_deg * 10.0f + 0.5f);
    compare_range = config->max_compare - config->min_compare;

    return config->min_compare +
           (angle_tenths * compare_range + (SERVO_TOTAL_ANGLE_STEPS / 2UL)) /
               SERVO_TOTAL_ANGLE_STEPS;
}

/** 把指定机械角直接转换并写入对应 TIM1 CCR 寄存器。 */
static void Servo_WriteAngle(Servo_Id_t servo_id, float angle_deg)
{
    const Servo_Config_t *config = &g_servo_config[servo_id];

    __HAL_TIM_SET_COMPARE(config->timer,
                          config->channel,
                          Servo_AngleToCompare(servo_id, angle_deg));
}

/**
 * 启动一个 PWM 通道。
 *
 * 工程原有 main.c 可能已经调用 HAL_TIM_PWM_Start()。重复启动已使能通道
 * 可能返回状态错误，因此先检查 CCER；若通道已使能则直接视为成功。
 */
static HAL_StatusTypeDef Servo_StartPwm(Servo_Id_t servo_id)
{
    const Servo_Config_t *config = &g_servo_config[servo_id];
    uint32_t enable_mask = Servo_ChannelEnableMask(config->channel);

    if ((config->timer == (TIM_HandleTypeDef *)0) ||
        (config->timer->Instance == (TIM_TypeDef *)0) ||
        (enable_mask == 0U)) {
        return HAL_ERROR;
    }

    if ((config->timer->Instance->CCER & enable_mask) != 0U) {
        return HAL_OK;
    }

    return HAL_TIM_PWM_Start(config->timer, config->channel);
}

/**
 * 将一个通道的目标、低通状态、卡尔曼状态和当前输出统一到指定角度。
 * 初始化及重新建立确定状态时使用，避免滤波器从旧历史缓慢收敛。
 */
static void Servo_ResetState(Servo_Id_t servo_id, float angle_deg)
{
    Servo_State_t *state = &g_servo_state[servo_id];

    angle_deg = Servo_QuantizeAngle(angle_deg);
    state->target_angle_deg = angle_deg;
    state->low_pass_angle_deg = angle_deg;
    state->current_angle_deg = angle_deg;

    KalmanFilter_Init(&state->kalman,
                      SERVO_KALMAN_PROCESS_NOISE,
                      SERVO_KALMAN_MEASUREMENT_NOISE,
                      angle_deg,
                      SERVO_KALMAN_INITIAL_COVARIANCE);
    state->initialized = 1U;
}

/** 执行指定舵机的一次“低通 -> 卡尔曼 -> 量化 -> PWM”更新。 */
static void Servo_UpdateOne(Servo_Id_t servo_id)
{
    Servo_State_t *state = &g_servo_state[servo_id];
    float filtered_angle;

    if (state->initialized == 0U) {
        return;
    }

    /*
     * 一阶低通离散公式：
     *   y(k) = y(k-1) + alpha * (target - y(k-1))
     * alpha 越大响应越快，越小则运动更平滑。
     */
    state->low_pass_angle_deg +=
        SERVO_FIRST_ORDER_ALPHA *
        (state->target_angle_deg - state->low_pass_angle_deg);

    /* 将低通结果作为卡尔曼滤波器本周期的“观测值”。 */
    filtered_angle = KalmanFilter_Update(&state->kalman,
                                          state->low_pass_angle_deg);

    /*
     * 两级滤波均足够接近最终目标时，直接吸附到精确目标。
     * 这样可以避免指数滤波无限逼近但永远不严格相等的问题。
     */
    if ((Servo_Abs(state->target_angle_deg - filtered_angle) <=
         SERVO_ARRIVAL_TOLERANCE) &&
        (Servo_Abs(state->target_angle_deg - state->low_pass_angle_deg) <=
         SERVO_ARRIVAL_TOLERANCE)) {
        filtered_angle = state->target_angle_deg;
        state->low_pass_angle_deg = state->target_angle_deg;
        KalmanFilter_Reset(&state->kalman,
                           state->target_angle_deg,
                           SERVO_KALMAN_INITIAL_COVARIANCE);
    }

    /* 最终写入前再次按 0.1° 量化，保证状态值与实际 CCR 分辨率一致。 */
    state->current_angle_deg = Servo_QuantizeAngle(filtered_angle);
    Servo_WriteAngle(servo_id, state->current_angle_deg);
}

HAL_StatusTypeDef Servo_Init(void)
{
    uint32_t index;

    for (index = 0U; index < SERVO_COUNT; ++index) {
        Servo_Id_t servo_id = (Servo_Id_t)index;

        /* 先写中心比较值，再启动 PWM，避免启动瞬间输出旧的 0° 脉宽。 */
        Servo_ResetState(servo_id, SERVO_CENTER_ANGLE_DEG);
        Servo_WriteAngle(servo_id, SERVO_CENTER_ANGLE_DEG);

        if (Servo_StartPwm(servo_id) != HAL_OK) {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngle(Servo_Id_t servo_id, float angle_deg)
{
    if ((Servo_IsValid(servo_id) == 0U) ||
        (g_servo_state[servo_id].initialized == 0U)) {
        return HAL_ERROR;
    }

    /* 设置目标不直接跳变 CCR，平滑运动由后续 Servo_Update() 完成。 */
    g_servo_state[servo_id].target_angle_deg =
        Servo_QuantizeAngle(angle_deg);
    return HAL_OK;
}

HAL_StatusTypeDef Servo_SetAngles(float yaw_angle_deg, float pitch_angle_deg)
{
    if (Servo_SetAngle(SERVO_YAW, yaw_angle_deg) != HAL_OK) {
        return HAL_ERROR;
    }

    return Servo_SetAngle(SERVO_PITCH, pitch_angle_deg);
}

void Servo_Update(void)
{
    uint32_t index;

    /* 两个通道使用同一次函数调用更新，使双轴滤波节拍保持一致。 */
    for (index = 0U; index < SERVO_COUNT; ++index) {
        Servo_UpdateOne((Servo_Id_t)index);
    }
}

void Servo_Stop(Servo_Id_t servo_id)
{
    Servo_State_t *state;

    if ((Servo_IsValid(servo_id) == 0U) ||
        (g_servo_state[servo_id].initialized == 0U)) {
        return;
    }

    state = &g_servo_state[servo_id];

    /*
     * “立即停止”表示不再继续追踪旧目标，而不是关闭 PWM。
     * 保持 PWM 可让舵机继续提供保持力；由于无位置反馈，只能冻结在
     * 当前软件输出角，而无法得知输出轴是否因惯性或外力发生偏差。
     */
    state->target_angle_deg = state->current_angle_deg;
    state->low_pass_angle_deg = state->current_angle_deg;
    KalmanFilter_Reset(&state->kalman,
                       state->current_angle_deg,
                       SERVO_KALMAN_INITIAL_COVARIANCE);
    Servo_WriteAngle(servo_id, state->current_angle_deg);
}

void Servo_StopAll(void)
{
    Servo_Stop(SERVO_YAW);
    Servo_Stop(SERVO_PITCH);
}

float Servo_GetCurrentAngle(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U) {
        return SERVO_CENTER_ANGLE_DEG;
    }

    return g_servo_state[servo_id].current_angle_deg;
}

float Servo_GetTargetAngle(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U) {
        return SERVO_CENTER_ANGLE_DEG;
    }

    return g_servo_state[servo_id].target_angle_deg;
}

uint8_t Servo_IsAtTarget(Servo_Id_t servo_id)
{
    if ((Servo_IsValid(servo_id) == 0U) ||
        (g_servo_state[servo_id].initialized == 0U)) {
        return 0U;
    }

    return (Servo_Abs(g_servo_state[servo_id].target_angle_deg -
                      g_servo_state[servo_id].current_angle_deg) <=
            SERVO_ARRIVAL_TOLERANCE)
               ? 1U
               : 0U;
}

const Servo_State_t *Servo_GetState(Servo_Id_t servo_id)
{
    if (Servo_IsValid(servo_id) == 0U) {
        return (const Servo_State_t *)0;
    }

    return &g_servo_state[servo_id];
}
