/**
 * @file KalmanFilter.c
 * @brief 一维标量卡尔曼滤波器实现。
 *
 * 使用“状态保持不变”的最简系统模型：
 *   x(k|k-1) = x(k-1|k-1)
 *   P(k|k-1) = P(k-1|k-1) + Q
 *
 * 收到观测 z(k) 后执行：
 *   K(k)     = P(k|k-1) / (P(k|k-1) + R)
 *   x(k|k)   = x(k|k-1) + K(k) * (z(k) - x(k|k-1))
 *   P(k|k)   = (1 - K(k)) * P(k|k-1)
 */
#include "KalmanFilter.h"

/* 防止 R 或分母为 0，避免除零及数值异常。 */
#define KALMAN_FILTER_MIN_NOISE       (1.0e-6f)

/* 协方差理论上不能为负数，所有外部输入均限制到该下界。 */
#define KALMAN_FILTER_MIN_COVARIANCE  (0.0f)

/** 将参数限制到指定下界，避免无效噪声或协方差进入滤波公式。 */
static float KalmanFilter_Max(float value, float minimum)
{
    return (value < minimum) ? minimum : value;
}

void KalmanFilter_Init(KalmanFilter_t *filter,
                       float process_noise,
                       float measurement_noise,
                       float initial_estimate,
                       float initial_error_covariance)
{
    /* 空指针时不访问内存，由调用者根据业务决定如何处理初始化失败。 */
    if (filter == (KalmanFilter_t *)0) {
        return;
    }

    /* Q 允许为 0；R 必须大于 0，否则增益计算可能出现除零。 */
    filter->process_noise = KalmanFilter_Max(process_noise, 0.0f);
    filter->measurement_noise =
        KalmanFilter_Max(measurement_noise, KALMAN_FILTER_MIN_NOISE);

    filter->estimate = initial_estimate;
    filter->error_covariance =
        KalmanFilter_Max(initial_error_covariance,
                         KALMAN_FILTER_MIN_COVARIANCE);
    filter->kalman_gain = 0.0f;
    filter->initialized = 1U;
}

void KalmanFilter_Reset(KalmanFilter_t *filter,
                        float estimate,
                        float error_covariance)
{
    if (filter == (KalmanFilter_t *)0) {
        return;
    }

    /* Reset 不修改 process_noise 和 measurement_noise，便于保持调参结果。 */
    filter->estimate = estimate;
    filter->error_covariance =
        KalmanFilter_Max(error_covariance,
                         KALMAN_FILTER_MIN_COVARIANCE);
    filter->kalman_gain = 0.0f;
    filter->initialized = 1U;
}

float KalmanFilter_Update(KalmanFilter_t *filter, float measurement)
{
    float denominator;

    /* NULL 表示调用者不希望保存滤波状态，此时采用透明直通策略。 */
    if (filter == (KalmanFilter_t *)0) {
        return measurement;
    }

    /* 容错初始化：即使上层遗漏 Init，第一次观测也可建立有效状态。 */
    if (filter->initialized == 0U) {
        KalmanFilter_Init(filter, 0.02f, 0.10f, measurement, 1.0f);
    }

    /* 预测阶段：状态值不变，仅让不确定度增加 Q。 */
    filter->error_covariance += filter->process_noise;

    /* 更新阶段分母为预测协方差与测量噪声之和。 */
    denominator = filter->error_covariance + filter->measurement_noise;
    if (denominator <= KALMAN_FILTER_MIN_NOISE) {
        /* 正常参数下不会进入此分支；该保护用于阻止异常浮点参数除零。 */
        filter->kalman_gain = 0.0f;
        return filter->estimate;
    }

    /* 计算增益，并用“观测 - 估计”的残差修正当前估计值。 */
    filter->kalman_gain = filter->error_covariance / denominator;
    filter->estimate += filter->kalman_gain *
                        (measurement - filter->estimate);

    /* 观测融合完成后，按照增益降低剩余估计不确定度。 */
    filter->error_covariance *= (1.0f - filter->kalman_gain);

    return filter->estimate;
}
