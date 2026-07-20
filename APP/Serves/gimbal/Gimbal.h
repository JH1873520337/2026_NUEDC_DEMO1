/**
 * @file Gimbal.h
 * @brief DS3115 二维云台、矩形轨迹和视觉逐点修正公共接口。
 *
 * 坐标约定：纸面中心为 (0,0)，X 向右、Y 向上，单位 cm；Yaw/Pitch
 * 软件角范围均为 -90°~90°。所有运动接口均为非阻塞式，必须周期调用
 * Gimbal_Update() 才会推进舵机和轨迹状态机。
 */
#ifndef GIMBAL_H
#define GIMBAL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "Servo.h"
#include <stdint.h>

#define GIMBAL_MIN_ANGLE_DEG                 (-90.0f)
#define GIMBAL_MAX_ANGLE_DEG                 (90.0f)
#define GIMBAL_CENTER_ANGLE_DEG              (0.0f)
#define GIMBAL_MAX_STEP_DEG                  (2.0f)
#define GIMBAL_ARRIVAL_TOLERANCE             (0.05f)

/** 每边 11 个点，即每边 10 段、整周 40 段。 */
#define GIMBAL_RECT_VERTEX_COUNT             4U
#define GIMBAL_RECT_EDGE_COUNT               4U
#define GIMBAL_RECT_POINTS_PER_EDGE          11U
#define GIMBAL_RECT_SEGMENTS_PER_EDGE        10U
#define GIMBAL_RECT_TOTAL_SEGMENTS           40U

/** 当前项目激光笔到纸面的默认垂直距离 D=100 cm。 */
#define GIMBAL_DEFAULT_TARGET_DISTANCE_CM    (100.0f)

/**
 * 视觉坐标滤波参数。视觉接口按约 20 Hz 输入；先一阶低通，再分别对 X/Y
 * 使用一维卡尔曼滤波。参数均可根据实际抖动大小进行标定。
 */
#define GIMBAL_VISION_FIRST_ORDER_ALPHA      (0.40f)
#define GIMBAL_VISION_KALMAN_PROCESS_NOISE   (0.02f)
#define GIMBAL_VISION_KALMAN_MEASURE_NOISE   (0.15f)
#define GIMBAL_VISION_KALMAN_INITIAL_COV     (1.0f)

/** 20 Hz 数据周期为 50 ms；超过 150 ms 未更新即视为当前点“无视觉数据”。 */
#define GIMBAL_VISION_DATA_TIMEOUT_MS        150U

/** 光斑误差不超过该值时不再产生额外修正动作，单位 cm。 */
#define GIMBAL_VISION_CORRECTION_TOLERANCE_CM (0.30f)

/** 视觉修正增益，1.0 表示按观测到的完整坐标误差进行一次角度补偿。 */
#define GIMBAL_VISION_CORRECTION_GAIN        (1.0f)

typedef enum
{
    GIMBAL_STATUS_OK = 0,
    GIMBAL_STATUS_NOT_INITIALIZED,
    GIMBAL_STATUS_INVALID_ARGUMENT,
    GIMBAL_STATUS_DRIVER_ERROR,
} Gimbal_Status_t;

typedef enum
{
    GIMBAL_MODE_STOPPED = 0,
    GIMBAL_MODE_ABSOLUTE,
    GIMBAL_MODE_RELATIVE,
    GIMBAL_MODE_VECTOR,
    GIMBAL_MODE_RECTANGLE,
} Gimbal_ControlMode_t;

/** 相对于纸面中心的二维坐标，单位 cm。 */
typedef struct
{
    float x_cm;
    float y_cm;
} Gimbal_Point2D_t;

/** 归一化直线一般式 a*x+b*y+c=0。 */
typedef struct
{
    float a;
    float b;
    float c;
} Gimbal_Line2D_t;

/** 自动排序后的四条边、直线方程及每边 11 个等距点。 */
typedef struct
{
    Gimbal_Point2D_t vertices[GIMBAL_RECT_VERTEX_COUNT];
    Gimbal_Line2D_t lines[GIMBAL_RECT_EDGE_COUNT];
    Gimbal_Point2D_t points[GIMBAL_RECT_EDGE_COUNT]
                                  [GIMBAL_RECT_POINTS_PER_EDGE];
    float distance_cm;
} Gimbal_RectanglePath_t;

/** 云台运行状态，可通过 Gimbal_GetState() 只读访问。 */
typedef struct
{
    float current_yaw_deg;
    float current_pitch_deg;
    float target_yaw_deg;
    float target_pitch_deg;
    float command_yaw_deg;
    float command_pitch_deg;
    float vector_x;
    float vector_y;
    Gimbal_ControlMode_t mode;
    uint8_t initialized;
    uint8_t moving;
    uint8_t limit_reached;
    uint8_t driver_fault;

    uint8_t rectangle_active;
    uint8_t rectangle_finished;
    uint8_t rectangle_start_reached;
    uint8_t rectangle_edge_index;
    uint8_t rectangle_point_index;
    uint8_t rectangle_segments_completed;
    uint8_t rectangle_correcting;       /**< 正在执行当前点的视觉补偿动作。 */
    uint8_t rectangle_corrections_completed; /**< 已实际执行的视觉补偿次数。 */

    uint8_t vision_valid;               /**< 至少收到过一帧有效视觉坐标。 */
    uint8_t vision_data_fresh;          /**< 最新数据尚未被矩形当前点消费。 */
    float vision_raw_x_cm;
    float vision_raw_y_cm;
    float vision_filtered_x_cm;
    float vision_filtered_y_cm;
    float vision_error_x_cm;            /**< 当前点期望 X - 视觉光斑 X。 */
    float vision_error_y_cm;            /**< 当前点期望 Y - 视觉光斑 Y。 */
    uint32_t vision_last_update_ms;
    uint32_t vision_sample_count;
} Gimbal_State_t;

/** 初始化两个舵机、云台状态以及视觉 X/Y 滤波器。 */
Gimbal_Status_t Gimbal_Init(void);

Gimbal_Status_t Gimbal_SetAbsolute(float yaw_deg, float pitch_deg);
Gimbal_Status_t Gimbal_SetRelative(float delta_yaw_deg,
                                   float delta_pitch_deg);
Gimbal_Status_t Gimbal_SetVector(float x, float y);

/** 将纸面坐标换算成先 Yaw 后 Pitch 的云台软件角。 */
Gimbal_Status_t Gimbal_PointToAngles(const Gimbal_Point2D_t *point,
                                     float distance_cm,
                                     float *yaw_deg,
                                     float *pitch_deg);

/**
 * 输入一帧视觉检测到的激光光斑坐标，建议由视觉接收任务按约 20 Hz 调用。
 * 调用本函数代表“本帧有有效数据”；没有识别到光斑时不要调用。函数内部
 * 对 X/Y 分别进行一阶低通和卡尔曼滤波，并保存最新时间戳。
 */
Gimbal_Status_t Gimbal_SubmitVisionSpot(float spot_x_cm,
                                        float spot_y_cm);

/** 启动矩形轨迹；四个点共包含 8 个 float，distance_cm 单位为 cm。 */
Gimbal_Status_t Gimbal_StartRectangle(
    const Gimbal_Point2D_t vertices[GIMBAL_RECT_VERTEX_COUNT],
    float distance_cm);

/** 使用默认 D=100 cm 启动矩形轨迹。 */
Gimbal_Status_t Gimbal_StartRectangleDefault(
    const Gimbal_Point2D_t vertices[GIMBAL_RECT_VERTEX_COUNT]);

/** 推荐每 10 ms 调用一次，推进普通运动、矩形轨迹和逐点视觉修正。 */
void Gimbal_Update(void);

/** 立即冻结当前位置并取消尚未完成的矩形轨迹和视觉修正。 */
void Gimbal_Stop(void);

uint8_t Gimbal_IsMoving(void);
const Gimbal_State_t *Gimbal_GetState(void);
const Gimbal_RectanglePath_t *Gimbal_GetRectanglePath(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_H */
