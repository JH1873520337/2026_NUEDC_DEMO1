#include "task_a4_vision.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug_uart.h"
#include "serve_vision_control.h"
#include "vision_uart.h"

#define TASK_A4_VISION_PERIOD_MS       10U
#define TASK_A4_VISION_DEBUG_PERIOD_MS 100U
#define TASK_A4_VISION_DEBUG_TIMEOUT_MS 20U
#define TASK_A4_VISION_FRAME_TIMEOUT_MS 150U
#define TASK_A4_VISION_LOST_TIMEOUT_MS 300U
#define TASK_A4_VISION_ERROR_DELAY_MS  1000U

/* MODE1 使用图像绝对坐标，X/Y 均为正方向时对应光斑向右/向下。 */
#define TASK_A4_VISION_PAN_DIRECTION   (-1.0f)
#define TASK_A4_VISION_TILT_DIRECTION  (-1.0f)

/* 输出为角速度 PID，最终按视觉帧时间积分成舵机角度。 */
#define TASK_A4_VISION_PAN_KP          0.40f
#define TASK_A4_VISION_PAN_KI          0.00f
#define TASK_A4_VISION_PAN_KD          0.10f
#define TASK_A4_VISION_TILT_KP         0.40f
#define TASK_A4_VISION_TILT_KI         0.00f
#define TASK_A4_VISION_TILT_KD         0.10f
#define TASK_A4_VISION_ERROR_FILTER_ALPHA 0.25f
#define TASK_A4_VISION_DEADBAND_PIXEL  3.0f
#define TASK_A4_VISION_PRECISION_ZONE_PIXEL 25.0f
#define TASK_A4_VISION_NEAR_KP_SCALE   0.65f
#define TASK_A4_VISION_INTEGRAL_LIMIT  100.0f
#define TASK_A4_VISION_DERIVATIVE_ALPHA 0.25f
#define TASK_A4_VISION_MAX_SPEED_DEG_S 30.0f
#define TASK_A4_VISION_MIN_TRACKING_SPEED_DEG_S 1.6f
#define TASK_A4_VISION_MAX_STEP_DEG    0.3f
#define TASK_A4_VISION_MAX_TARGET_LEAD_DEG 1.0f
#define TASK_A4_VISION_COORDINATE_MIN  0.0f
#define TASK_A4_VISION_COORDINATE_MAX 4095.0f
#define TASK_A4_VISION_MAX_PIXEL_JUMP  500.0f
#define TASK_A4_VISION_SWAP_IMAGE_AXES 1U
#define TASK_A4_VISION_STARTUP_FRAMES  3U

static const Serve_VisionControl_Config_t Task_A4Vision_Config = {
    .gimbal_config = {
        .pan_min_angle = 50.0f,
        .pan_max_angle = 130.0f,
        .tilt_min_angle = 50.0f,
        .tilt_max_angle = 130.0f,
        .pan_center_angle = 90.0f,
        .tilt_center_angle = 90.0f,
        .max_speed = 60.0f,
        .acceleration = 90.0f,
        .process_period_ms = TASK_A4_VISION_PERIOD_MS,
    },
    .pan_direction = TASK_A4_VISION_PAN_DIRECTION,
    .tilt_direction = TASK_A4_VISION_TILT_DIRECTION,
    .pan_kp_deg_per_second_per_pixel = TASK_A4_VISION_PAN_KP,
    .pan_ki_deg_per_second_per_pixel_second = TASK_A4_VISION_PAN_KI,
    .pan_kd_deg_per_pixel = TASK_A4_VISION_PAN_KD,
    .tilt_kp_deg_per_second_per_pixel = TASK_A4_VISION_TILT_KP,
    .tilt_ki_deg_per_second_per_pixel_second = TASK_A4_VISION_TILT_KI,
    .tilt_kd_deg_per_pixel = TASK_A4_VISION_TILT_KD,
    .error_filter_alpha = TASK_A4_VISION_ERROR_FILTER_ALPHA,
    .deadband_pixel = TASK_A4_VISION_DEADBAND_PIXEL,
    .precision_zone_pixel = TASK_A4_VISION_PRECISION_ZONE_PIXEL,
    .near_target_kp_scale = TASK_A4_VISION_NEAR_KP_SCALE,
    .integral_limit_pixel_second = TASK_A4_VISION_INTEGRAL_LIMIT,
    .derivative_filter_alpha = TASK_A4_VISION_DERIVATIVE_ALPHA,
    .max_speed_deg_per_second = TASK_A4_VISION_MAX_SPEED_DEG_S,
    .min_tracking_speed_deg_per_second =
        TASK_A4_VISION_MIN_TRACKING_SPEED_DEG_S,
    .max_step_deg = TASK_A4_VISION_MAX_STEP_DEG,
    .max_target_lead_deg = TASK_A4_VISION_MAX_TARGET_LEAD_DEG,
    .coordinate_min_pixel = TASK_A4_VISION_COORDINATE_MIN,
    .coordinate_max_pixel = TASK_A4_VISION_COORDINATE_MAX,
    .max_coordinate_jump_pixel = TASK_A4_VISION_MAX_PIXEL_JUMP,
    .swap_image_axes = TASK_A4_VISION_SWAP_IMAGE_AXES,
    .startup_frame_count = TASK_A4_VISION_STARTUP_FRAMES,
    .frame_timeout_ms = TASK_A4_VISION_FRAME_TIMEOUT_MS,
};

volatile Task_A4Vision_Runtime_t Task_A4Vision_Runtime;

static void Task_A4Vision_ErrorLoop(void)
{
    Serve_VisionControl_Stop();
    Task_A4Vision_Runtime.state = TASK_A4_VISION_STATE_FAULT;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(TASK_A4_VISION_ERROR_DELAY_MS));
    }
}

static void Task_A4Vision_UpdateRuntime(void)
{
    Serve_VisionControl_State_t state;

    if (Serve_VisionControl_GetState(&state) !=
        SERVE_VISION_CONTROL_OK) {
        return;
    }

    Task_A4Vision_Runtime.pan_angle = state.pan_angle;
    Task_A4Vision_Runtime.tilt_angle = state.tilt_angle;
    Task_A4Vision_Runtime.target_x = state.target_x;
    Task_A4Vision_Runtime.target_y = state.target_y;
    Task_A4Vision_Runtime.laser_x = state.laser_x;
    Task_A4Vision_Runtime.laser_y = state.laser_y;
    Task_A4Vision_Runtime.error_x = state.error_x;
    Task_A4Vision_Runtime.error_y = state.error_y;
    Task_A4Vision_Runtime.pan_pid_output = state.pan_pid_output;
    Task_A4Vision_Runtime.tilt_pid_output = state.tilt_pid_output;
    Task_A4Vision_Runtime.valid_frame_count = state.valid_frame_count;
    Task_A4Vision_Runtime.invalid_frame_count = state.invalid_frame_count;
}

static void Task_A4Vision_SendDebugData(void)
{
    DebugUart_VisionData_t data;

    data.state = (uint8_t)Task_A4Vision_Runtime.state;
    data.target_x = Task_A4Vision_Runtime.target_x;
    data.target_y = Task_A4Vision_Runtime.target_y;
    data.laser_x = Task_A4Vision_Runtime.laser_x;
    data.laser_y = Task_A4Vision_Runtime.laser_y;
    data.error_x = Task_A4Vision_Runtime.error_x;
    data.error_y = Task_A4Vision_Runtime.error_y;
    data.pan_pid_output = Task_A4Vision_Runtime.pan_pid_output;
    data.tilt_pid_output = Task_A4Vision_Runtime.tilt_pid_output;
    data.received_byte_count = VisionUart_GetReceivedByteCount();
    data.valid_frame_count = VisionUart_GetValidFrameCount();
    data.invalid_frame_count = VisionUart_GetInvalidFrameCount();
    data.active_mode = (uint8_t)VisionUart_GetActiveMode(
        TASK_A4_VISION_FRAME_TIMEOUT_MS);

    (void)DebugUart_SendVisionData(&data,
                                   TASK_A4_VISION_DEBUG_TIMEOUT_MS);
}

void Task_A4Vision_Entry(void *argument)
{
    Serve_VisionControl_Frame_t frame;
    Serve_VisionControl_Status_t status;
    TickType_t last_wake_tick;
    uint32_t now_ms;
    uint32_t last_valid_frame_ms;
    uint32_t last_debug_output_ms;

    (void)argument;

    Task_A4Vision_Runtime.pan_angle = 90.0f;
    Task_A4Vision_Runtime.tilt_angle = 90.0f;
    Task_A4Vision_Runtime.target_x = 0.0f;
    Task_A4Vision_Runtime.target_y = 0.0f;
    Task_A4Vision_Runtime.laser_x = 0.0f;
    Task_A4Vision_Runtime.laser_y = 0.0f;
    Task_A4Vision_Runtime.error_x = 0.0f;
    Task_A4Vision_Runtime.error_y = 0.0f;
    Task_A4Vision_Runtime.pan_pid_output = 0.0f;
    Task_A4Vision_Runtime.tilt_pid_output = 0.0f;
    Task_A4Vision_Runtime.elapsed_ms = 0U;
    Task_A4Vision_Runtime.valid_frame_count = 0U;
    Task_A4Vision_Runtime.invalid_frame_count = 0U;
    Task_A4Vision_Runtime.path_segment = 0U;
    Task_A4Vision_Runtime.rectangle_sample_count = 0U;
    Task_A4Vision_Runtime.state = TASK_A4_VISION_STATE_WAITING_VISION;

    if (Serve_VisionControl_Init(&Task_A4Vision_Config) !=
        SERVE_VISION_CONTROL_OK) {
        Task_A4Vision_ErrorLoop();
    }

    now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    last_valid_frame_ms = now_ms;
    last_debug_output_ms = now_ms - TASK_A4_VISION_DEBUG_PERIOD_MS;
    last_wake_tick = xTaskGetTickCount();

    for (;;) {
        now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        if (Serve_VisionControl_Process(now_ms) !=
            SERVE_VISION_CONTROL_OK) {
            Task_A4Vision_ErrorLoop();
        }

        status = Serve_VisionControl_ReadFrame(&frame);
        if (status == SERVE_VISION_CONTROL_OK) {
            last_valid_frame_ms = now_ms;
            status = Serve_VisionControl_TrackPoint(&frame.target, &frame);
            if ((status != SERVE_VISION_CONTROL_OK) &&
                (status != SERVE_VISION_CONTROL_NO_NEW_FRAME)) {
                Task_A4Vision_ErrorLoop();
            }
            Task_A4Vision_Runtime.state = TASK_A4_VISION_STATE_RUNNING;
        } else if ((now_ms - last_valid_frame_ms) >
                   TASK_A4_VISION_LOST_TIMEOUT_MS) {
            Serve_VisionControl_Stop();
            Task_A4Vision_Runtime.state =
                TASK_A4_VISION_STATE_VISION_LOST;
        }

        Task_A4Vision_UpdateRuntime();
        if ((now_ms - last_debug_output_ms) >=
            TASK_A4_VISION_DEBUG_PERIOD_MS) {
            last_debug_output_ms = now_ms;
            Task_A4Vision_SendDebugData();
        }
        vTaskDelayUntil(&last_wake_tick,
                        pdMS_TO_TICKS(TASK_A4_VISION_PERIOD_MS));
    }
}

void Task_A4Vision_RequestPause(void)
{
    /* 当前视觉任务由默认任务直接运行，暂停功能由后续按键任务接入。 */
}
