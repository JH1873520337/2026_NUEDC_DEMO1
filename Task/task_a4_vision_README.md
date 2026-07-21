# MODE1 视觉闭环任务

视觉端通过 MODE1 同时发送目标点和当前激光点。主控计算两者的像素误差，
使用 Pan/Tilt 双轴 PID 输出角速度，再按视觉帧间隔积分得到舵机角度指令。

## MODE1 数据要求

- 消息类型为 `0x01`
- `flags` 的 bit0 和 bit1 均为 1，即 `flags = 0x03`
- 目标点和激光点使用图像绝对像素坐标，X 向右、Y 向下
- 校验和为前 13 字节累加和的低 8 位

## PID 参数

参数集中在 `Task/task_a4_vision.c`：

- `TASK_A4_VISION_*_DIRECTION`：舵机方向；当前 Pan、Tilt 均为 `-1.0f`
- `TASK_A4_VISION_SWAP_IMAGE_AXES`：当前为 `1`，图像 Y 控制 Pan，图像 X 控制 Tilt
- `TASK_A4_VISION_*_KP/KI/KD`：双轴 PID 参数
- `TASK_A4_VISION_ERROR_FILTER_ALPHA`：视觉误差低通滤波系数，当前为 `0.25`
- `TASK_A4_VISION_DEADBAND_PIXEL`：到达目标后的像素死区
- `TASK_A4_VISION_MAX_SPEED_DEG_S`：最大角速度
- `TASK_A4_VISION_MAX_STEP_DEG`：每帧最大角度变化
- `TASK_A4_VISION_MAX_TARGET_LEAD_DEG`：平滑目标相对实际舵机角度的最大超前量
- 上电后先连续确认 3 帧，确认期间不下发舵机动作
- 坐标越界或单帧跳变超过 500 像素会被丢弃，避免异常帧触发乱跑

当前默认 `KI = 0`、`KD = 0`，只用低增益 Kp。能够稳定靠近目标后，加入
少量 Ki 消除静差，最后加入少量 Kd 抑制超调。

视觉目标角度使用云台平滑接口下发，底层每 10 ms 插值更新 PWM；当前最大
速度为 30 deg/s、加速度为 60 deg/s^2，避免低帧率视觉数据造成阶梯运动。

视觉数据中断超过 300 ms 时，任务停止云台并进入
`TASK_A4_VISION_STATE_VISION_LOST`；数据恢复后自动重新进入闭环。
