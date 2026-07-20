/**
 * @file KalmanFilter.h
 * @brief 一维标量卡尔曼滤波器公共接口。
 *
 * 本模块只维护一个标量状态，适合对角度、距离、速度等单变量进行平滑。
 * 实现不使用动态内存，也不依赖 STM32 HAL，可由多个模块分别创建独立实例。
 */
#ifndef KALMAN_FILTER_H
#define KALMAN_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief 一维卡尔曼滤波器的全部运行状态。
 *
 * 每个被滤波的信号都应拥有一个独立的 KalmanFilter_t 实例。舵机驱动中
 * Yaw、Pitch 两个通道各自保存一个实例，避免两个轴之间互相影响。
 */
typedef struct
{
    float process_noise;       /**< 过程噪声 Q；越大时滤波器越相信新观测，响应越快。 */
    float measurement_noise;   /**< 测量噪声 R；越大时输出越平滑，但响应会变慢。 */
    float estimate;            /**< 当前最优估计值 x(k|k)。 */
    float error_covariance;    /**< 当前估计误差协方差 P(k|k)。 */
    float kalman_gain;         /**< 最近一次更新得到的卡尔曼增益 K。 */
    uint8_t initialized;       /**< 非 0 表示参数和状态已经初始化。 */
} KalmanFilter_t;

/**
 * @brief 初始化一个卡尔曼滤波器实例。
 *
 * @param filter                     滤波器实例指针，不能为 NULL。
 * @param process_noise              过程噪声 Q，负值会按 0 处理。
 * @param measurement_noise          测量噪声 R，过小值会被限制为安全最小值。
 * @param initial_estimate           初始估计值。
 * @param initial_error_covariance   初始误差协方差，负值会按 0 处理。
 */
void KalmanFilter_Init(KalmanFilter_t *filter,
                       float process_noise,
                       float measurement_noise,
                       float initial_estimate,
                       float initial_error_covariance);

/**
 * @brief 保留 Q、R 参数，仅重置估计值和协方差。
 *
 * 舵机到达目标或被立即停止时使用该接口，防止旧的滤波历史继续推动输出。
 *
 * @param filter             滤波器实例指针，不能为 NULL。
 * @param estimate           重置后的估计值。
 * @param error_covariance   重置后的误差协方差。
 */
void KalmanFilter_Reset(KalmanFilter_t *filter,
                        float estimate,
                        float error_covariance);

/**
 * @brief 输入一次观测值并执行一维卡尔曼更新。
 *
 * @param filter       滤波器实例指针；为 NULL 时直接返回 measurement。
 * @param measurement 本周期观测值 z(k)。
 * @return 本周期更新后的最优估计值。
 */
float KalmanFilter_Update(KalmanFilter_t *filter, float measurement);

#ifdef __cplusplus
}
#endif

#endif /* KALMAN_FILTER_H */
