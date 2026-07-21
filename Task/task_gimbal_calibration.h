#ifndef TASK_GIMBAL_CALIBRATION_H
#define TASK_GIMBAL_CALIBRATION_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 当前由编码器控制的云台轴。
 */
typedef enum {
    TASK_GIMBAL_CALIBRATION_AXIS_PAN = 0,
    TASK_GIMBAL_CALIBRATION_AXIS_TILT
} Task_GimbalCalibration_Axis_t;

/**
 * @brief 标定任务的实时状态，可加入 Live Expressions 观察。
 */
typedef struct {
    float pan_angle;                          /**< 当前水平轴标定角度，单位：deg。 */
    float tilt_angle;                         /**< 当前俯仰轴标定角度，单位：deg。 */
    int32_t encoder_delta;                    /**< 最近一次读取到的编码器计数增量。 */
    uint32_t axis_switch_count;               /**< 按键有效切换轴的累计次数。 */
    Task_GimbalCalibration_Axis_t selected_axis; /**< 当前选中的控制轴。 */
    uint8_t initialized;                      /**< 标定任务是否初始化成功。 */
} Task_GimbalCalibration_State_t;

/**
 * @brief 标定任务实时状态。
 *
 * 调试时观察此变量：转动编码器调整角度，按下 PE12 按键切换 Pan/Tilt，
 * 激光点对准目标后记录 pan_angle 和 tilt_angle。
 */
extern volatile Task_GimbalCalibration_State_t GimbalCalibration_State;

/**
 * @brief FreeRTOS 云台角度标定任务入口。
 */
void Task_GimbalCalibration_Entry(void *argument);

/**
 * @brief GPIO 外部中断事件入口，由 HAL_GPIO_EXTI_Callback() 调用。
 * @param gpio_pin 产生中断的 GPIO 引脚位掩码。
 *
 * @note 本函数运行在中断上下文，只记录按键事件，不直接操作舵机。
 */
void Task_GimbalCalibration_ButtonIRQ(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif /* TASK_GIMBAL_CALIBRATION_H */
