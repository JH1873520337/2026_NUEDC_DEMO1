/**
 * @file Gimbal.c
 * @brief 二维云台、矩形扫描轨迹以及视觉逐点闭环修正的实现。
 *
 * 软件角定义：舵机逻辑 90° 为云台 0°；实际 PWM 由 Servo 层叠加机械零位补偿。
 * Yaw/Pitch 软件角均限制在 -90°~90°。
 * 运动由 Gimbal_Update() 非阻塞推进，每次给两个舵机下发的角度增量均不超过 2°。
 * 矩形轨迹每边划分为 10 段（11 个端点），整周共 40 段。每到达一个理论点后，
 * 状态机会尝试消费一帧不超过 150 ms 的视觉光斑坐标，完成至多一次增量修正，
 * 然后继续下一个点；没有新鲜视觉数据时绝不等待。
 */
#include "Gimbal.h"
#include "KalmanFilter.h"

#include <float.h>
#include <math.h>
#include <stddef.h>


#define GIMBAL_VECTOR_EPSILON       (1.0e-6f)


#define GIMBAL_GEOMETRY_EPSILON     (1.0e-5f)


#define GIMBAL_NO_LIMIT_TIME        (FLT_MAX)


#define GIMBAL_RAD_TO_DEG           (57.2957795130823208768f)


#define GIMBAL_RECT_ADVANCE_GUARD   (GIMBAL_RECT_TOTAL_SEGMENTS + 2U)


static Gimbal_State_t g_gimbal_state;


static Gimbal_RectanglePath_t g_rectangle_path;

/**
 * @brief 视觉 X/Y 两通道的私有滤波状态。
 *
 * 原始坐标先经过一阶低通，再分别进入一维卡尔曼滤波器。last_used_sample_count
 * 用于保证一个视觉样本最多只参与一次矩形点修正。
 */
typedef struct
{
    float low_pass_x_cm;
    float low_pass_y_cm;
    KalmanFilter_t kalman_x;
    KalmanFilter_t kalman_y;
    uint32_t last_used_sample_count;
    uint8_t initialized;
} Gimbal_VisionFilter_t;

static Gimbal_VisionFilter_t g_vision_filter;

/** 按极角排序时使用的临时顶点结构。 */
typedef struct
{
    Gimbal_Point2D_t point;
    float polar_angle_rad;
    uint8_t original_index;
} Gimbal_AngledVertex_t;


/** 返回浮点绝对值，避免为简单运算额外依赖库函数。 */
static float Gimbal_Abs(float value)
{
    return (value < 0.0f) ? -value : value;
}


/** 返回两个浮点数中的较小值。 */
static float Gimbal_Min(float first, float second)
{
    return (first < second) ? first : second;
}


/** 返回两个浮点数中的较大值。 */
static float Gimbal_Max(float first, float second)
{
    return (first > second) ? first : second;
}


/** 将 value 限制在 [minimum, maximum] 闭区间内。 */
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


/** 检查输入既不是 NaN，也没有超出有限 float 范围。 */
static uint8_t Gimbal_IsFinite(float value)
{
    return ((value == value) && (value <= FLT_MAX) && (value >= -FLT_MAX))
               ? 1U
               : 0U;
}


/**
 * @brief 将云台软件角限幅到 -90°~90°，并四舍五入到 0.1°。
 * @note 对负数使用减 0.5 后截断，保证正负方向采用对称的四舍五入。
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
 * @brief 从舵机驱动读取逻辑 PWM 软件角，并转换成以逻辑 90° 为零点的云台角。
 * @note Servo 层零位补偿不进入软件角状态；该值仍不是编码器反馈的真实机械角。
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


/** 判断单轴当前角与目标角之差是否在到达容差内。 */
static uint8_t Gimbal_AxisArrived(float current_deg, float target_deg)
{
    return (Gimbal_Abs(current_deg - target_deg) <=
            GIMBAL_ARRIVAL_TOLERANCE)
               ? 1U
               : 0U;
}


/** 仅当 Yaw、Pitch 两轴同时到达目标时返回 1。 */
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
 * @brief 取消矩形状态机和正在执行的视觉修正。
 * @param clear_finished 非 0 时同时清除“上一轮轨迹已完成”标志。
 */
static void Gimbal_CancelRectangle(uint8_t clear_finished)
{
    g_gimbal_state.rectangle_active = 0U;
    g_gimbal_state.rectangle_start_reached = 0U;
    g_gimbal_state.rectangle_correcting = 0U;

    if (clear_finished != 0U) {
        g_gimbal_state.rectangle_finished = 0U;
    }
}


/**
 * @brief 保存新的双轴最终目标，并把分段指令起点设置为当前角度。
 * @note 本函数不直接写 PWM；实际运动由 Gimbal_Update() 按每步不超过 2°推进。
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
 * @brief 完成当前运动，将软件状态吸附到目标并冻结 Servo 滤波器。
 * @note 吸附可消除 0.1° 量化和到达容差造成的微小累计误差。
 */
static void Gimbal_FinishMotion(void)
{
    Gimbal_SyncCurrentFromServo();


    g_gimbal_state.current_yaw_deg = g_gimbal_state.target_yaw_deg;
    g_gimbal_state.current_pitch_deg = g_gimbal_state.target_pitch_deg;
    g_gimbal_state.command_yaw_deg = g_gimbal_state.target_yaw_deg;
    g_gimbal_state.command_pitch_deg = g_gimbal_state.target_pitch_deg;
    g_gimbal_state.moving = 0U;

    if (g_gimbal_state.mode == GIMBAL_MODE_VECTOR) {
        g_gimbal_state.limit_reached = 1U;
    }


    Servo_StopAll();
}


/** 使用勾股定理计算纸面上两个点之间的欧氏距离，单位 cm。 */
static float Gimbal_PointDistance(const Gimbal_Point2D_t *first,
                                  const Gimbal_Point2D_t *second)
{
    float delta_x = second->x_cm - first->x_cm;
    float delta_y = second->y_cm - first->y_cm;

    return sqrtf(delta_x * delta_x + delta_y * delta_y);
}


/**
 * @brief 把任意输入顺序的四个顶点按绕中心的顺时针方向排序。
 * @details 使用 atan2(y,x) 计算极角。根据当前云台安装方向和从激光器朝纸面观察的
 *          实际投影方向，按极角升序排列才对应纸面上的顺时针扫描；排序完成后再
 *          旋转数组，保证调用者给出的 vertices[0] 仍然是轨迹起点。
 */
static uint8_t Gimbal_OrderVerticesClockwise(
    const Gimbal_Point2D_t input[GIMBAL_RECT_VERTEX_COUNT],
    Gimbal_Point2D_t output[GIMBAL_RECT_VERTEX_COUNT])
{
    Gimbal_AngledVertex_t sorted[GIMBAL_RECT_VERTEX_COUNT];
    Gimbal_AngledVertex_t temporary;
    uint8_t first_position = 0U;
    uint8_t index;
    uint8_t scan;

    for (index = 0U; index < GIMBAL_RECT_VERTEX_COUNT; ++index) {
        float radius_squared;

        if ((Gimbal_IsFinite(input[index].x_cm) == 0U) ||
            (Gimbal_IsFinite(input[index].y_cm) == 0U)) {
            return 0U;
        }

        radius_squared = input[index].x_cm * input[index].x_cm +
                         input[index].y_cm * input[index].y_cm;
        if (radius_squared <=
            GIMBAL_GEOMETRY_EPSILON * GIMBAL_GEOMETRY_EPSILON) {

            return 0U;
        }

        sorted[index].point = input[index];
        sorted[index].polar_angle_rad =
            atan2f(input[index].y_cm, input[index].x_cm);
        sorted[index].original_index = index;
    }


    for (index = 1U; index < GIMBAL_RECT_VERTEX_COUNT; ++index) {
        temporary = sorted[index];
        scan = index;
        /*
         * 按极角升序排列。原先使用降序时，实机投影轨迹表现为逆时针；
         * 改为升序后仅反转矩形遍历方向，不改变坐标到角度的换算和视觉修正方向。
         */
        while ((scan > 0U) &&
               (sorted[scan - 1U].polar_angle_rad >
                temporary.polar_angle_rad)) {
            sorted[scan] = sorted[scan - 1U];
            --scan;
        }
        sorted[scan] = temporary;
    }

    for (index = 0U; index < GIMBAL_RECT_VERTEX_COUNT; ++index) {
        if (sorted[index].original_index == 0U) {
            first_position = index;
            break;
        }
    }


    for (index = 0U; index < GIMBAL_RECT_VERTEX_COUNT; ++index) {
        output[index] =
            sorted[(uint8_t)((first_position + index) %
                             GIMBAL_RECT_VERTEX_COUNT)]
                .point;
    }

    return 1U;
}


/**
 * @brief 检查相邻顶点不重合，并通过鞋带公式确认四边形面积非零。
 * @return 1 表示几何数据可用，0 表示退化或无效。
 */
static uint8_t Gimbal_ValidateOrderedVertices(
    const Gimbal_Point2D_t vertices[GIMBAL_RECT_VERTEX_COUNT])
{
    float twice_area = 0.0f;
    uint8_t edge;

    for (edge = 0U; edge < GIMBAL_RECT_EDGE_COUNT; ++edge) {
        uint8_t next = (uint8_t)((edge + 1U) % GIMBAL_RECT_VERTEX_COUNT);

        if (Gimbal_PointDistance(&vertices[edge], &vertices[next]) <=
            GIMBAL_GEOMETRY_EPSILON) {
            return 0U;
        }

        twice_area += vertices[edge].x_cm * vertices[next].y_cm -
                      vertices[next].x_cm * vertices[edge].y_cm;
    }

    return (Gimbal_Abs(twice_area) > GIMBAL_GEOMETRY_EPSILON) ? 1U : 0U;
}


/**
 * @brief 生成四条归一化直线方程及每边等距的 11 个离散点。
 * @details 对边 P0->P1，方程为 A*x+B*y+C=0，并令 sqrt(A²+B²)=1；
 *          离散点 P(k)=P0+(P1-P0)*k/10，k=0..10。
 */
static void Gimbal_BuildRectangleGeometry(void)
{
    uint8_t edge;

    for (edge = 0U; edge < GIMBAL_RECT_EDGE_COUNT; ++edge) {
        const Gimbal_Point2D_t *start = &g_rectangle_path.vertices[edge];
        const Gimbal_Point2D_t *end =
            &g_rectangle_path
                 .vertices[(uint8_t)((edge + 1U) %
                                     GIMBAL_RECT_VERTEX_COUNT)];
        float a = start->y_cm - end->y_cm;
        float b = end->x_cm - start->x_cm;
        float c = start->x_cm * end->y_cm - end->x_cm * start->y_cm;
        float normal_length = sqrtf(a * a + b * b);
        uint8_t point_index;


        g_rectangle_path.lines[edge].a = a / normal_length;
        g_rectangle_path.lines[edge].b = b / normal_length;
        g_rectangle_path.lines[edge].c = c / normal_length;

        for (point_index = 0U;
             point_index < GIMBAL_RECT_POINTS_PER_EDGE;
             ++point_index) {
            float ratio = (float)point_index /
                          (float)GIMBAL_RECT_SEGMENTS_PER_EDGE;

            g_rectangle_path.points[edge][point_index].x_cm =
                start->x_cm + (end->x_cm - start->x_cm) * ratio;
            g_rectangle_path.points[edge][point_index].y_cm =
                start->y_cm + (end->y_cm - start->y_cm) * ratio;
        }
    }
}


/** 把指定矩形离散点换算为 Yaw/Pitch 角度并启动非阻塞运动。 */
static void Gimbal_CommandRectanglePoint(uint8_t edge_index,
                                         uint8_t point_index)
{
    float yaw_deg;
    float pitch_deg;
    const Gimbal_Point2D_t *point =
        &g_rectangle_path.points[edge_index][point_index];


    (void)Gimbal_PointToAngles(point,
                               g_rectangle_path.distance_cm,
                               &yaw_deg,
                               &pitch_deg);

    g_gimbal_state.rectangle_edge_index = edge_index;
    g_gimbal_state.rectangle_point_index = point_index;
    Gimbal_SetMotionTarget(yaw_deg, pitch_deg, GIMBAL_MODE_RECTANGLE);
}


/** 标记 40 段矩形轨迹全部完成，并清除未消费的视觉帧。 */
static void Gimbal_FinishRectangle(void)
{
    g_gimbal_state.rectangle_active = 0U;
    g_gimbal_state.rectangle_finished = 1U;
    g_gimbal_state.rectangle_start_reached = 1U;
    g_gimbal_state.rectangle_correcting = 0U;
    g_gimbal_state.vision_data_fresh = 0U;
    g_vision_filter.last_used_sample_count =
        g_gimbal_state.vision_sample_count;
    g_gimbal_state.rectangle_segments_completed =
        GIMBAL_RECT_TOTAL_SEGMENTS;
    g_gimbal_state.moving = 0U;
}


/**
 * @brief 在当前点处理完成后选择并下发下一个矩形离散点。
 * @details 每条边的 point[10] 与下一条边的 point[0] 是同一顶点，因此换边时
 *          从 point[1] 开始，确保整周正好运动 40 段且不重复停留顶点。
 */
static void Gimbal_AdvanceRectangleAfterTarget(void)
{
    uint8_t guard = 0U;

    while ((g_gimbal_state.rectangle_active != 0U) &&
           (guard < GIMBAL_RECT_ADVANCE_GUARD)) {
        uint8_t next_edge;
        uint8_t next_point;

        ++guard;

        if (g_gimbal_state.rectangle_start_reached == 0U) {

            g_gimbal_state.rectangle_start_reached = 1U;
            next_edge = 0U;
            next_point = 1U;
        } else {
            ++g_gimbal_state.rectangle_segments_completed;

            if (g_gimbal_state.rectangle_segments_completed >=
                GIMBAL_RECT_TOTAL_SEGMENTS) {
                Gimbal_FinishRectangle();
                return;
            }

            if (g_gimbal_state.rectangle_point_index <
                GIMBAL_RECT_SEGMENTS_PER_EDGE) {
                next_edge = g_gimbal_state.rectangle_edge_index;
                next_point =
                    (uint8_t)(g_gimbal_state.rectangle_point_index + 1U);
            } else {

                next_edge =
                    (uint8_t)(g_gimbal_state.rectangle_edge_index + 1U);
                next_point = 1U;
            }
        }

        Gimbal_CommandRectanglePoint(next_edge, next_point);

        if (g_gimbal_state.moving != 0U) {
            return;
        }

    }

    if (g_gimbal_state.rectangle_active != 0U) {

        Gimbal_CancelRectangle(1U);
        g_gimbal_state.driver_fault = 1U;
        g_gimbal_state.mode = GIMBAL_MODE_STOPPED;
        g_gimbal_state.moving = 0U;
        Servo_StopAll();
    }
}


/**
 * @brief 尝试使用一帧新鲜视觉坐标修正刚刚到达的矩形离散点。
 *
 * @return 1 表示已经启动一个非零修正动作，状态机应等待该动作完成；
 *         0 表示无新数据、数据过期、误差已足够小或量化后无需运动，可直接去下一点。
 */
static uint8_t Gimbal_TryStartVisionCorrection(void)
{
    const Gimbal_Point2D_t *desired_point;
    Gimbal_Point2D_t measured_point;
    float desired_yaw_deg;
    float desired_pitch_deg;
    float measured_yaw_deg;
    float measured_pitch_deg;
    float corrected_yaw_deg;
    float corrected_pitch_deg;
    uint32_t now_ms;

    if ((g_gimbal_state.vision_valid == 0U) ||
        (g_gimbal_state.vision_data_fresh == 0U) ||
        (g_gimbal_state.vision_sample_count ==
         g_vision_filter.last_used_sample_count)) {
        return 0U;
    }

    /* 先标记为已消费，确保同一帧不会被后续矩形点重复使用。 */
    g_vision_filter.last_used_sample_count =
        g_gimbal_state.vision_sample_count;
    g_gimbal_state.vision_data_fresh = 0U;

    now_ms = HAL_GetTick();
    if ((uint32_t)(now_ms - g_gimbal_state.vision_last_update_ms) >
        GIMBAL_VISION_DATA_TIMEOUT_MS) {
        return 0U;
    }

    desired_point =
        &g_rectangle_path.points[g_gimbal_state.rectangle_edge_index]
                                [g_gimbal_state.rectangle_point_index];
    measured_point.x_cm = g_gimbal_state.vision_filtered_x_cm;
    measured_point.y_cm = g_gimbal_state.vision_filtered_y_cm;

    g_gimbal_state.vision_error_x_cm =
        desired_point->x_cm - measured_point.x_cm;
    g_gimbal_state.vision_error_y_cm =
        desired_point->y_cm - measured_point.y_cm;

    if ((Gimbal_Abs(g_gimbal_state.vision_error_x_cm) <=
         GIMBAL_VISION_CORRECTION_TOLERANCE_CM) &&
        (Gimbal_Abs(g_gimbal_state.vision_error_y_cm) <=
         GIMBAL_VISION_CORRECTION_TOLERANCE_CM)) {
        return 0U;
    }

    /*
     * 不直接重新下发理论角，而是计算“理论点角度 - 实测光斑角度”，再把该差值
     * 叠加到当前云台角度上。这样能够补偿安装偏差、舵机误差和机构误差。
     */
    if ((Gimbal_PointToAngles(desired_point,
                              g_rectangle_path.distance_cm,
                              &desired_yaw_deg,
                              &desired_pitch_deg) != GIMBAL_STATUS_OK) ||
        (Gimbal_PointToAngles(&measured_point,
                              g_rectangle_path.distance_cm,
                              &measured_yaw_deg,
                              &measured_pitch_deg) != GIMBAL_STATUS_OK)) {
        return 0U;
    }

    corrected_yaw_deg =
        g_gimbal_state.current_yaw_deg +
        (desired_yaw_deg - measured_yaw_deg) *
            GIMBAL_VISION_CORRECTION_GAIN;
    corrected_pitch_deg =
        g_gimbal_state.current_pitch_deg +
        (desired_pitch_deg - measured_pitch_deg) *
            GIMBAL_VISION_CORRECTION_GAIN;

    g_gimbal_state.rectangle_correcting = 1U;
    Gimbal_SetMotionTarget(corrected_yaw_deg,
                           corrected_pitch_deg,
                           GIMBAL_MODE_RECTANGLE);

    if (g_gimbal_state.moving == 0U) {
        /* 修正量经 0.1° 量化后为零，不能让矩形状态机停在 correcting 状态。 */
        g_gimbal_state.rectangle_correcting = 0U;
        return 0U;
    }

    return 1U;
}

/** 到达普通目标或视觉修正目标后的统一状态机入口。 */
static void Gimbal_HandleTargetArrival(void)
{
    Gimbal_FinishMotion();

    if (g_gimbal_state.rectangle_active == 0U) {
        return;
    }

    if (g_gimbal_state.rectangle_correcting != 0U) {
        /*
         * 每个离散点只修正一次。修正运动期间收到的视觉帧对应旧点，在进入
         * 下一点前一并丢弃，避免误用为下一点的测量值。
         */
        g_gimbal_state.rectangle_correcting = 0U;
        ++g_gimbal_state.rectangle_corrections_completed;
        g_vision_filter.last_used_sample_count =
            g_gimbal_state.vision_sample_count;
        g_gimbal_state.vision_data_fresh = 0U;
        Gimbal_AdvanceRectangleAfterTarget();
        return;
    }

    if (Gimbal_TryStartVisionCorrection() != 0U) {
        return;
    }

    Gimbal_AdvanceRectangleAfterTarget();
}

/**
 * @brief 初始化舵机、云台状态、矩形状态和视觉滤波状态。
 * @details Servo_Init() 使用 Yaw=-6°、Pitch=-2° 的最终 PWM 零位补偿，
 *          直接输出实机复测得到的正确机械复位位置；补偿不进入云台软件坐标，
 *          因此本函数返回后 current/target/command 的 Yaw、Pitch 均为 0°。
 */
Gimbal_Status_t Gimbal_Init(void)
{

    if (Servo_Init() != HAL_OK) {
        g_gimbal_state.initialized = 0U;
        g_gimbal_state.driver_fault = 1U;
        return GIMBAL_STATUS_DRIVER_ERROR;
    }


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
    g_gimbal_state.rectangle_active = 0U;
    g_gimbal_state.rectangle_finished = 0U;
    g_gimbal_state.rectangle_start_reached = 0U;
    g_gimbal_state.rectangle_edge_index = 0U;
    g_gimbal_state.rectangle_point_index = 0U;
    g_gimbal_state.rectangle_segments_completed = 0U;
    g_gimbal_state.rectangle_correcting = 0U;
    g_gimbal_state.rectangle_corrections_completed = 0U;

    g_gimbal_state.vision_valid = 0U;
    g_gimbal_state.vision_data_fresh = 0U;
    g_gimbal_state.vision_raw_x_cm = 0.0f;
    g_gimbal_state.vision_raw_y_cm = 0.0f;
    g_gimbal_state.vision_filtered_x_cm = 0.0f;
    g_gimbal_state.vision_filtered_y_cm = 0.0f;
    g_gimbal_state.vision_error_x_cm = 0.0f;
    g_gimbal_state.vision_error_y_cm = 0.0f;
    g_gimbal_state.vision_last_update_ms = 0U;
    g_gimbal_state.vision_sample_count = 0U;

    g_vision_filter.low_pass_x_cm = 0.0f;
    g_vision_filter.low_pass_y_cm = 0.0f;
    g_vision_filter.last_used_sample_count = 0U;
    g_vision_filter.initialized = 0U;

    return GIMBAL_STATUS_OK;
}

/** 启动绝对位置运动；新命令会取消正在运行的矩形轨迹。 */
Gimbal_Status_t Gimbal_SetAbsolute(float yaw_deg, float pitch_deg)
{
    if (g_gimbal_state.initialized == 0U) {
        return GIMBAL_STATUS_NOT_INITIALIZED;
    }
    if ((Gimbal_IsFinite(yaw_deg) == 0U) ||
        (Gimbal_IsFinite(pitch_deg) == 0U)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }


    Gimbal_CancelRectangle(1U);
    Gimbal_SyncCurrentFromServo();
    Servo_StopAll();
    g_gimbal_state.vector_x = 0.0f;
    g_gimbal_state.vector_y = 0.0f;
    Gimbal_SetMotionTarget(yaw_deg, pitch_deg, GIMBAL_MODE_ABSOLUTE);

    return GIMBAL_STATUS_OK;
}

/** 以当前 PWM 软件角为基准启动相对位置运动，并自动限制到 ±90°。 */
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


    Gimbal_CancelRectangle(1U);
    Gimbal_SyncCurrentFromServo();
    Servo_StopAll();
    g_gimbal_state.vector_x = 0.0f;
    g_gimbal_state.vector_y = 0.0f;
    Gimbal_SetMotionTarget(g_gimbal_state.current_yaw_deg + delta_yaw_deg,
                           g_gimbal_state.current_pitch_deg + delta_pitch_deg,
                           GIMBAL_MODE_RELATIVE);

    return GIMBAL_STATUS_OK;
}

/**
 * @brief 根据串口帧常用的“轴、方向、角度”三个字段启动单轴相对运动。
 *
 * @details
 * - axis='Y' 时仅改变 Yaw，axis='P' 时仅改变 Pitch；
 * - direction=0 时角度取负，direction=1 时角度取正；
 * - direction 必须是二进制 0x00/0x01，不能传 ASCII 字符 '0'(0x30)/'1'(0x31)；
 * - angle_deg 是角度绝对值，单位为度，本函数内部负责添加正负号；
 * - 最终调用 Gimbal_SetRelative()，因此仍保留 ±90° 限位和每步不超过 2° 的运动规则。
 */
Gimbal_Status_t Gimbal_SetRelativeByAxis(uint8_t axis,
                                         uint8_t direction,
                                         float angle_deg)
{
    float signed_angle_deg;

    /* 角度参数表示绝对值，不允许调用者同时通过负角度和方向重复指定符号。 */
    if ((Gimbal_IsFinite(angle_deg) == 0U) || (angle_deg < 0.0f)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    /* 串口方向字段约定为原始二进制 0x00/0x01。 */
    if ((direction != GIMBAL_DIRECTION_NEGATIVE) &&
        (direction != GIMBAL_DIRECTION_POSITIVE)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    if ((axis != GIMBAL_AXIS_YAW) && (axis != GIMBAL_AXIS_PITCH)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    signed_angle_deg = angle_deg;
    if (direction == GIMBAL_DIRECTION_NEGATIVE) {
        signed_angle_deg = -signed_angle_deg;
    }

    if (axis == GIMBAL_AXIS_YAW) {
        return Gimbal_SetRelative(signed_angle_deg, 0.0f);
    }

    return Gimbal_SetRelative(0.0f, signed_angle_deg);
}
/**
 * @brief 沿输入向量 (X,Y) 的方向持续运动，直到任意一轴先到达 ±90°。
 * @details 对向量按最大分量归一化，保持 Yaw 与 Pitch 运动比例不变。
 */
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


    max_component = Gimbal_Max(Gimbal_Abs(x), Gimbal_Abs(y));
    if (max_component <= GIMBAL_VECTOR_EPSILON) {
        Gimbal_Stop();
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    Gimbal_CancelRectangle(1U);
    Gimbal_SyncCurrentFromServo();
    Servo_StopAll();
    direction_yaw = x / max_component;
    direction_pitch = y / max_component;


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

    target_yaw = g_gimbal_state.current_yaw_deg +
                 direction_yaw * limit_time;
    target_pitch = g_gimbal_state.current_pitch_deg +
                   direction_pitch * limit_time;

    g_gimbal_state.vector_x = x;
    g_gimbal_state.vector_y = y;
    Gimbal_SetMotionTarget(target_yaw, target_pitch, GIMBAL_MODE_VECTOR);

    if (g_gimbal_state.moving == 0U) {
        g_gimbal_state.limit_reached = 1U;
    }

    return GIMBAL_STATUS_OK;
}

/**
 * @brief 根据纸面坐标和垂直距离 D 计算激光指向角。
 * @details yaw=atan2(X,D)，pitch=atan2(Y,sqrt(D²+X²))，最后量化到 0.1°。
 */
Gimbal_Status_t Gimbal_PointToAngles(const Gimbal_Point2D_t *point,
                                     float distance_cm,
                                     float *yaw_deg,
                                     float *pitch_deg)
{
    float yaw_rad;
    float pitch_rad;
    float horizontal_distance_cm;

    if ((point == NULL) || (yaw_deg == NULL) || (pitch_deg == NULL) ||
        (Gimbal_IsFinite(point->x_cm) == 0U) ||
        (Gimbal_IsFinite(point->y_cm) == 0U) ||
        (Gimbal_IsFinite(distance_cm) == 0U) ||
        (distance_cm <= GIMBAL_GEOMETRY_EPSILON)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }


    horizontal_distance_cm =
        sqrtf(distance_cm * distance_cm + point->x_cm * point->x_cm);
    yaw_rad = atan2f(point->x_cm, distance_cm);
    pitch_rad = atan2f(point->y_cm, horizontal_distance_cm);

    *yaw_deg = Gimbal_QuantizeAngle(yaw_rad * GIMBAL_RAD_TO_DEG);
    *pitch_deg = Gimbal_QuantizeAngle(pitch_rad * GIMBAL_RAD_TO_DEG);

    return GIMBAL_STATUS_OK;
}

/**
 * @brief 接收一帧有效光斑坐标，并依次执行一阶低通与 X/Y 卡尔曼滤波。
 * @note 建议约 20 Hz 调用；未识别到光斑时不要调用本函数。
 */
Gimbal_Status_t Gimbal_SubmitVisionSpot(float spot_x_cm, float spot_y_cm)
{
    if (g_gimbal_state.initialized == 0U) {
        return GIMBAL_STATUS_NOT_INITIALIZED;
    }
    if ((Gimbal_IsFinite(spot_x_cm) == 0U) ||
        (Gimbal_IsFinite(spot_y_cm) == 0U)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    g_gimbal_state.vision_raw_x_cm = spot_x_cm;
    g_gimbal_state.vision_raw_y_cm = spot_y_cm;

    if (g_vision_filter.initialized == 0U) {
        /* 第一帧直接作为初值，避免滤波输出从 (0,0) 缓慢爬向真实坐标。 */
        g_vision_filter.low_pass_x_cm = spot_x_cm;
        g_vision_filter.low_pass_y_cm = spot_y_cm;
        KalmanFilter_Init(&g_vision_filter.kalman_x,
                          GIMBAL_VISION_KALMAN_PROCESS_NOISE,
                          GIMBAL_VISION_KALMAN_MEASURE_NOISE,
                          spot_x_cm,
                          GIMBAL_VISION_KALMAN_INITIAL_COV);
        KalmanFilter_Init(&g_vision_filter.kalman_y,
                          GIMBAL_VISION_KALMAN_PROCESS_NOISE,
                          GIMBAL_VISION_KALMAN_MEASURE_NOISE,
                          spot_y_cm,
                          GIMBAL_VISION_KALMAN_INITIAL_COV);
        g_gimbal_state.vision_filtered_x_cm = spot_x_cm;
        g_gimbal_state.vision_filtered_y_cm = spot_y_cm;
        g_vision_filter.initialized = 1U;
    } else {
        /* 一阶低通抑制单帧跳变，再用卡尔曼滤波降低连续测量噪声。 */
        g_vision_filter.low_pass_x_cm +=
            GIMBAL_VISION_FIRST_ORDER_ALPHA *
            (spot_x_cm - g_vision_filter.low_pass_x_cm);
        g_vision_filter.low_pass_y_cm +=
            GIMBAL_VISION_FIRST_ORDER_ALPHA *
            (spot_y_cm - g_vision_filter.low_pass_y_cm);

        g_gimbal_state.vision_filtered_x_cm =
            KalmanFilter_Update(&g_vision_filter.kalman_x,
                                g_vision_filter.low_pass_x_cm);
        g_gimbal_state.vision_filtered_y_cm =
            KalmanFilter_Update(&g_vision_filter.kalman_y,
                                g_vision_filter.low_pass_y_cm);
    }

    g_gimbal_state.vision_valid = 1U;
    g_gimbal_state.vision_data_fresh = 1U;
    g_gimbal_state.vision_last_update_ms = HAL_GetTick();
    ++g_gimbal_state.vision_sample_count;

    return GIMBAL_STATUS_OK;
}

/**
 * @brief 建立矩形几何数据并启动顺时针、40 段、可视觉修正的非阻塞轨迹。
 * @param vertices 四个顶点坐标，单位 cm；vertices[0] 作为起点。
 * @param distance_cm 激光转轴到纸面中心的垂直距离 D，单位 cm。
 */
Gimbal_Status_t Gimbal_StartRectangle(
    const Gimbal_Point2D_t vertices[GIMBAL_RECT_VERTEX_COUNT],
    float distance_cm)
{
    Gimbal_Point2D_t ordered_vertices[GIMBAL_RECT_VERTEX_COUNT];
    uint8_t index;

    if (g_gimbal_state.initialized == 0U) {
        return GIMBAL_STATUS_NOT_INITIALIZED;
    }
    if ((vertices == NULL) || (Gimbal_IsFinite(distance_cm) == 0U) ||
        (distance_cm <= GIMBAL_GEOMETRY_EPSILON)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }

    if ((Gimbal_OrderVerticesClockwise(vertices, ordered_vertices) == 0U) ||
        (Gimbal_ValidateOrderedVertices(ordered_vertices) == 0U)) {
        return GIMBAL_STATUS_INVALID_ARGUMENT;
    }


    Gimbal_SyncCurrentFromServo();
    Servo_StopAll();
    Gimbal_CancelRectangle(1U);

    for (index = 0U; index < GIMBAL_RECT_VERTEX_COUNT; ++index) {
        g_rectangle_path.vertices[index] = ordered_vertices[index];
    }
    g_rectangle_path.distance_cm = distance_cm;
    Gimbal_BuildRectangleGeometry();

    g_gimbal_state.vector_x = 0.0f;
    g_gimbal_state.vector_y = 0.0f;
    g_gimbal_state.rectangle_active = 1U;
    g_gimbal_state.rectangle_finished = 0U;
    g_gimbal_state.rectangle_start_reached = 0U;
    g_gimbal_state.rectangle_edge_index = 0U;
    g_gimbal_state.rectangle_point_index = 0U;
    g_gimbal_state.rectangle_segments_completed = 0U;
    g_gimbal_state.rectangle_correcting = 0U;
    g_gimbal_state.rectangle_corrections_completed = 0U;
    g_vision_filter.last_used_sample_count =
        g_gimbal_state.vision_sample_count;
    g_gimbal_state.vision_data_fresh = 0U;
    g_gimbal_state.vision_error_x_cm = 0.0f;
    g_gimbal_state.vision_error_y_cm = 0.0f;

    Gimbal_CommandRectanglePoint(0U, 0U);

    if (g_gimbal_state.moving == 0U) {

        Gimbal_AdvanceRectangleAfterTarget();
    }

    return (g_gimbal_state.driver_fault == 0U)
               ? GIMBAL_STATUS_OK
               : GIMBAL_STATUS_DRIVER_ERROR;
}

/** 使用 GIMBAL_DEFAULT_TARGET_DISTANCE_CM（当前为 100 cm）启动矩形轨迹。 */
Gimbal_Status_t Gimbal_StartRectangleDefault(
    const Gimbal_Point2D_t vertices[GIMBAL_RECT_VERTEX_COUNT])
{
    return Gimbal_StartRectangle(vertices,
                                 GIMBAL_DEFAULT_TARGET_DISTANCE_CM);
}

/**
 * @brief 推进一次双轴运动和矩形/视觉状态机。
 * @note 推荐固定每 10 ms 调用一次；函数不包含 HAL_Delay()。
 */
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


    Gimbal_SyncCurrentFromServo();

    if (Gimbal_TargetArrived() != 0U) {
        Gimbal_HandleTargetArrival();
        return;
    }

    yaw_error = g_gimbal_state.target_yaw_deg -
                g_gimbal_state.current_yaw_deg;
    pitch_error = g_gimbal_state.target_pitch_deg -
                  g_gimbal_state.current_pitch_deg;


    max_error = Gimbal_Max(Gimbal_Abs(yaw_error), Gimbal_Abs(pitch_error));
    step_scale = (max_error > GIMBAL_MAX_STEP_DEG)
                     ? (GIMBAL_MAX_STEP_DEG / max_error)
                     : 1.0f;

    g_gimbal_state.command_yaw_deg = Gimbal_QuantizeAngle(
        g_gimbal_state.current_yaw_deg + yaw_error * step_scale);
    g_gimbal_state.command_pitch_deg = Gimbal_QuantizeAngle(
        g_gimbal_state.current_pitch_deg + pitch_error * step_scale);


    if (Servo_SetAngles(g_gimbal_state.command_yaw_deg +
                            SERVO_CENTER_ANGLE_DEG,
                        g_gimbal_state.command_pitch_deg +
                            SERVO_CENTER_ANGLE_DEG) != HAL_OK) {
        g_gimbal_state.driver_fault = 1U;
        g_gimbal_state.moving = 0U;
        Gimbal_CancelRectangle(1U);
        Servo_StopAll();
        return;
    }


    Servo_Update();
    Gimbal_SyncCurrentFromServo();

    if (Gimbal_TargetArrived() != 0U) {
        Gimbal_HandleTargetArrival();
    }
}

/** 立即冻结当前 PWM 软件位置，并取消矩形轨迹和视觉修正。 */
void Gimbal_Stop(void)
{
    if (g_gimbal_state.initialized == 0U) {
        return;
    }


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
    Gimbal_CancelRectangle(1U);
}

/** 普通运动或矩形状态机仍活动时返回 1。 */
uint8_t Gimbal_IsMoving(void)
{

    return ((g_gimbal_state.moving != 0U) ||
            (g_gimbal_state.rectangle_active != 0U))
               ? 1U
               : 0U;
}

/** 返回云台只读运行状态，供调试、遥测和上层状态判断使用。 */
const Gimbal_State_t *Gimbal_GetState(void)
{
    return &g_gimbal_state;
}

/** 返回最近一次生成的矩形顶点、直线方程和 44 个边内点。 */
const Gimbal_RectanglePath_t *Gimbal_GetRectanglePath(void)
{
    return &g_rectangle_path;
}
