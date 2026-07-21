/**
 * @file GimbalDebug.h
 * @brief 使用 USART1 三字节中断协议调试二维云台。
 *
 * 数据帧固定为 3 字节，三个字节均按 uint8_t 解释：
 *   - byte[0]：ASCII 'Y'(0x59) 控制 Yaw，ASCII 'P'(0x50) 控制 Pitch；
 *   - byte[1]：方向，0 表示负方向，1 表示正方向；
 *   - byte[2]：相对转角绝对值的 10 倍，范围 0~255，即 0.0°~25.5°。
 *
 * 例如：{0x50, 0x01, 0x14} 表示 Pitch 正向相对转动 2.0°；
 * {0x50, 0x00, 0x14} 表示 Pitch 负向相对转动 2.0°。
 * USART1 使用 HAL 单字节中断接收，解析器以 Y/P 为帧头自动恢复同步；接收完成回调仅负责
 * 组装并保存命令，然后立即挂接下一字节。真正的云台控制、
 * 状态判断和串口打印均在 GimbalDebug_Update() 中完成，避免在中断中阻塞。
 */
#ifndef GIMBAL_DEBUG_H
#define GIMBAL_DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include <stdint.h>

/** USART1 调试命令固定长度。 */
#define GIMBAL_DEBUG_FRAME_SIZE       3U

/**
 * @brief 初始化 USART1 云台调试接口，并通过 HAL 开启单字节中断接收。
 * @retval HAL_OK 表示接收已启动；HAL_BUSY 表示 USART1 已被其他接收流程占用。
 * @note 调用前必须先执行 MX_USART1_UART_Init() 和 Gimbal_Init()。
 */
HAL_StatusTypeDef GimbalDebug_Init(void);

/**
 * @brief 推进云台运动、处理已接收命令；接收后立即输出 ACK，运动结束后打印软件角度。
 * @note 裸机主循环中每 5~10 ms 调用一次，不能在中断中调用。
 */
void GimbalDebug_Update(void);

/** 在应用的 HAL_UART_RxCpltCallback() 中转发 USART1 接收完成事件。 */
void GimbalDebug_OnUartRxComplete(UART_HandleTypeDef *huart);

/** 在应用的 HAL_UART_ErrorCallback() 中转发 USART1 接收错误事件。 */
void GimbalDebug_OnUartError(UART_HandleTypeDef *huart);

/** @brief 返回因处理不及时而被丢弃的命令帧数量。 */
uint32_t GimbalDebug_GetDroppedFrameCount(void);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_DEBUG_H */