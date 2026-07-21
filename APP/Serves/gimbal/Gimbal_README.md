# Gimbal 二维云台驱动

## 1. 功能概述

本模块位于 `APP/Serves/gimbal`，建立在 `APP/BSP/servo` 的 DS3115 双舵机驱动之上，支持：

- Yaw/Pitch 绝对位置控制；
- Yaw/Pitch 相对位置控制；
- 输入二维向量并沿该方向运动，直到任意一轴到达限制；
- 四顶点矩形顺时针轨迹；
- 每边 11 个等距点、每边 10 段、整周 40 段；
- 每个矩形离散点到达后的视觉闭环修正；
- 无视觉数据时不等待，继续执行正常轨迹；
- 随时通过 `Gimbal_Stop()` 立即冻结当前位置。

所有运动接口均为**非阻塞接口**。必须周期调用 `Gimbal_Update()`，否则只保存目标，不会继续推进舵机。

## 2. 硬件与坐标约定

| 轴 | PWM 通道 | 舵机机械角 | 云台软件角 |
|---|---|---:|---:|
| Yaw | TIM1_CH4，PE14 | 0°~180° | -90°~90° |
| Pitch | TIM1_CH3，PE13 | 0°~180° | -90°~90° |

- 舵机机械角 90° 对应云台软件角 0°；
- 纸面中心为 `(0,0)`；
- X 向右，控制 Yaw；
- Y 向上，控制 Pitch；
- 纸面坐标和距离 D 的单位均为 cm；
- 当前实际安装的两个舵机均在 Servo 层进行 PWM 方向反转，Gimbal 层不需要再次取反；
- 如果视觉图像坐标的 Y 轴向下，调用视觉接口前必须转换为 `y_cm = -y_image_cm`。

## 3. 初始化和周期更新

调用顺序应为：

1. HAL 和系统时钟初始化；
2. GPIO、TIM1 等外设初始化；
3. 调用 `Gimbal_Init()`；
4. 在裸机主循环中按固定 5~10 ms 周期持续调用 `Gimbal_Update()`。

`Gimbal_Init()` 内部会调用 `Servo_Init()`，因此应用层不需要再次单独启动两个 PWM 通道。初始化完成后两个舵机的逻辑角为 90°，Servo 层自动叠加 Yaw -3°、Pitch +5° 的机械零位补偿，云台软件角均记录为 0°。

### 裸机示例

```c
#include "Gimbal.h"

int main(void)
{
    HAL_Init();
    SystemClock_Config();
    MX_GPIO_Init();
    MX_TIM1_Init();

    if (Gimbal_Init() != GIMBAL_STATUS_OK) {
        Error_Handler();
    }

    while (1) {
        Gimbal_Update();
        HAL_Delay(10);     /* 推荐固定 10 ms 周期 */
    }
}
```


> 不要用一个很长的 `HAL_Delay()` 代替周期更新。运动过程中每次双轴指令变化不超过 2°，滤波和分段运动都依赖持续调用 `Gimbal_Update()`。

## 4. 基本控制接口

### 4.1 绝对位置

```c
Gimbal_SetAbsolute(30.0f, -15.0f);
```

表示 Yaw 到 30°、Pitch 到 -15°。超出 -90°~90° 的目标会自动限幅。

### 4.2 相对位置

```c
Gimbal_SetRelative(10.0f, 10.0f);
```

表示以当前软件位置为基准，两轴各增加 10°。函数不会阻塞；调用后仍必须继续运行 `Gimbal_Update()`。
#### 串口三参数单轴相对运动

新增接口：

```c
Gimbal_Status_t Gimbal_SetRelativeByAxis(uint8_t axis,
                                         uint8_t direction,
                                         float angle_deg);
```

三个参数分别为轴、方向和角度绝对值：

- `axis`：`'Y'` 控制 Yaw，`'P'` 控制 Pitch；
- `direction`：二进制 `0x00` 为负方向，二进制 `0x01` 为正方向；
- `angle_deg`：角度绝对值，单位为度。

USART1 三字节接收完成后可以直接调用：

```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        (void)Gimbal_SetRelativeByAxis(mess[0],
                                       mess[1],
                                       (float)mess[2]);
        HAL_UART_Receive_IT(&huart1, mess, 3U);
    }
}
```

例如以 HEX 方式发送：

- `50 00 05`：Pitch -5°；
- `50 01 05`：Pitch +5°；
- `59 00 0A`：Yaw -10°；
- `59 01 0A`：Yaw +10°。

注意方向字节是原始二进制 `00/01`，因此不能写成 `mess[1] == '0'`。字符 `'0'` 的数值是 `0x30`，与二进制 `0x00` 不相等。

如果要等待一次运动完成后再发送下一条相对命令，可使用：

```c
if (Gimbal_IsMoving() == 0U) {
    Gimbal_SetRelative(10.0f, 10.0f);
}
```

### 4.3 向量控制

```c
Gimbal_SetVector(2.0f, 1.0f);
```

云台按 Yaw:Pitch = 2:1 的比例运动，直到 Yaw 或 Pitch 中任意一轴先到达 ±90°。到达限制后，`Gimbal_GetState()->limit_reached` 为 1。

## 5. 矩形轨迹

### 5.1 顶点输入

```c
static const Gimbal_Point2D_t rectangle[4] = {
    {-10.0f,  7.0f},
    { 10.0f,  7.0f},
    { 10.0f, -7.0f},
    {-10.0f, -7.0f},
};
```

输入点的单位为 cm，均是相对纸面中心的坐标。模块会根据当前舵机安装和实际纸面投影方向，按极角升序排列四个顶点，使激光光斑在纸面上顺时针运动，并保持 `rectangle[0]` 为扫描起点。若以后改变舵机安装方向，应重新在实机上确认扫描方向。

### 5.2 默认距离 D=100 cm

当前激光笔转轴到纸面中心的垂直距离默认为 **100 cm**：

```c
Gimbal_StartRectangleDefault(rectangle);
```

等价于：

```c
Gimbal_StartRectangle(rectangle, 100.0f);
```

如果现场测得距离变化，应使用实际值，例如 `Gimbal_StartRectangle(rectangle, 98.5f)`。D 必须大于 0。

### 5.3 直线和离散点

相邻点 `P0(x0,y0)`、`P1(x1,y1)` 构成的边使用直线方程：

```text
A = y0 - y1
B = x1 - x0
C = x0*y1 - x1*y0
A*x + B*y + C = 0
```

模块会把 A、B、C 归一化，使 `sqrt(A²+B²)=1`。每条边生成 11 个等距点：

```text
P(k) = P0 + (P1-P0) * k/10,  k=0...10
```

每边有 10 段，四边共 40 段。换边时不会重复运动公共顶点。

### 5.4 点坐标到角度

对于纸面点 `(X,Y)` 和距离 D：

```text
yaw   = atan2(X, D)
pitch = atan2(Y, sqrt(D² + X²))
```

结果转换为角度后限制到 ±90°，再量化到 0.1°。

## 6. 视觉数据接口（约 20 Hz）

### 6.1 接口定义

```c
Gimbal_Status_t Gimbal_SubmitVisionSpot(float spot_x_cm,
                                        float spot_y_cm);
```

参数是视觉检测到的**激光光斑相对纸面中心的坐标**，单位 cm。建议视觉模块约每 50 ms（20 Hz）提供一次有效数据。

调用本函数表示“本帧确实检测到了有效光斑”。如果当前帧没有检测到光斑，**不要调用**该函数。云台不会等待视觉数据，而是在当前理论点完成后直接进入下一点。

### 6.2 视觉接收示例

```c
void Vision_OnFrame(uint8_t has_spot,
                    float spot_x_cm,
                    float spot_y_image_cm)
{
    if (has_spot != 0U) {
        /* 图像 Y 向下，云台纸面坐标 Y 向上，因此取反。 */
        (void)Gimbal_SubmitVisionSpot(spot_x_cm,
                                      -spot_y_image_cm);
    }
}
```

如果视觉已经直接输出“X 向右、Y 向上”的 cm 坐标，则不需要对 Y 取反。

### 6.3 滤波流程

每次提交有效视觉坐标时，X/Y 两轴分别执行：

1. 一阶低通滤波；
2. 一维卡尔曼滤波；
3. 保存滤波坐标、时间戳和样本编号。

默认参数位于 `Gimbal.h`：

```c
#define GIMBAL_VISION_FIRST_ORDER_ALPHA          0.40f
#define GIMBAL_VISION_KALMAN_PROCESS_NOISE       0.02f
#define GIMBAL_VISION_KALMAN_MEASURE_NOISE       0.15f
#define GIMBAL_VISION_KALMAN_INITIAL_COV          1.0f
#define GIMBAL_VISION_DATA_TIMEOUT_MS             150U
```

20 Hz 的帧周期约为 50 ms。超过 150 ms 没有新帧时，该数据对当前点视为无效，轨迹不会停下来等待。

### 6.4 每个点的修正过程

矩形轨迹到达一个理论点后，状态机按以下顺序处理：

1. 检查是否存在尚未消费的新视觉样本；
2. 检查该样本时间是否不超过 150 ms；
3. 使用滤波后的光斑坐标作为实测点；
4. 计算 `误差 = 理论点坐标 - 实测光斑坐标`；
5. 若 X、Y 误差均不超过 0.30 cm，直接进入下一点；
6. 否则将理论点和实测点分别换算为 Yaw/Pitch；
7. 使用角度差对当前云台角做一次增量补偿；
8. 修正动作完成后进入下一点。

一个离散点最多执行一次视觉修正，防止同一点无限震荡。修正运动本身仍由 `Gimbal_Update()` 拆分为双轴每步不超过 2° 的动作。

修正公式为：

```text
corrected_yaw   = current_yaw
                + (desired_yaw - measured_yaw) * gain
corrected_pitch = current_pitch
                + (desired_pitch - measured_pitch) * gain
```

默认 `gain=1.0`。现场若出现过度修正或来回摆动，可适当减小 `GIMBAL_VISION_CORRECTION_GAIN`，例如 0.6~0.8。

## 7. 完整组合示例

```c
#include "Gimbal.h"

static const Gimbal_Point2D_t rectangle[4] = {
    {-10.0f,  7.0f},
    { 10.0f,  7.0f},
    { 10.0f, -7.0f},
    {-10.0f, -7.0f},
};

void App_Init(void)
{
    if (Gimbal_Init() != GIMBAL_STATUS_OK) {
        Error_Handler();
    }

    /* 使用默认 D=100 cm，启动顺时针矩形轨迹。 */
    if (Gimbal_StartRectangleDefault(rectangle) != GIMBAL_STATUS_OK) {
        Error_Handler();
    }
}

/* 由视觉接收/解析代码约 20 Hz 调用；无光斑时不调用。 */
void App_OnVisionSpot(float x_cm, float y_cm)
{
    (void)Gimbal_SubmitVisionSpot(x_cm, y_cm);
}

/* 由裸机主循环固定周期调用。 */
void App_10msUpdate(void)
{
    Gimbal_Update();
}
```

视觉接口和云台更新接口不要放在长时间阻塞的代码后面。裸机程序中建议在主循环处理视觉数据和浮点滤波；中断只保存原始接收数据并置位，不建议在高优先级中断中执行浮点滤波。

## 8. 状态读取与立即停止

```c
const Gimbal_State_t *state = Gimbal_GetState();

if (state->rectangle_finished != 0U) {
    /* 一圈 40 段已经完成 */
}

/* 紧急停止并保持当前位置 */
Gimbal_Stop();
```

常用状态字段：

| 字段 | 含义 |
|---|---|
| `current_yaw_deg/current_pitch_deg` | 当前 PWM 软件角 |
| `rectangle_active` | 矩形轨迹仍在运行 |
| `rectangle_finished` | 40 段轨迹已完成 |
| `rectangle_segments_completed` | 已完成的理论线段数量 |
| `rectangle_correcting` | 正在执行当前点的视觉补偿 |
| `rectangle_corrections_completed` | 已完成的视觉补偿次数 |
| `vision_filtered_x_cm/y_cm` | 最新滤波光斑坐标 |
| `vision_error_x_cm/y_cm` | 最近一次理论点与光斑的坐标误差 |
| `vision_last_update_ms` | 最近有效视觉帧的 HAL tick |
| `driver_fault` | 舵机驱动写入失败 |

## 9. 调试建议

- 首先只调用 `Gimbal_Init()`，确认两个舵机都位于机械中位 90°；
- 再测试小角度绝对/相对运动，确认 Yaw、Pitch 实际方向正确；
- 确认视觉输出已经换算为相对中心的 cm，而不是像素；
- 用固定光斑测试 `vision_raw_*` 和 `vision_filtered_*`，确认滤波坐标方向正确；
- 轨迹测试时观察 `rectangle_edge_index`、`rectangle_point_index` 和 `rectangle_segments_completed`；
- 若误差始终反向增大，优先检查视觉 X/Y 正方向和相机图像 Y 轴是否取反；
- DS3115 应使用独立、足够电流的电源，并与 MCU 共地。
