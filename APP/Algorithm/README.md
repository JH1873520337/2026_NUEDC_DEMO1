# Algorithm 算法层规范

## 定位

`Algorithm` 负责与硬件和操作系统无关的纯算法，例如滤波、PID、姿态解算、轨迹规划、目标筛选、数据拟合和数学变换。

算法应能在 PC 单元测试环境中使用普通 C 编译器独立编译和验证。

## 依赖边界

```text
Serves -> Algorithm -> C 标准库 / CMSIS-DSP（可选）
```

- 不得包含 STM32 HAL、CubeMX 外设头文件或 BSP 头文件。
- 不得依赖 FreeRTOS，不使用任务、队列、信号量或 RTOS 时间函数。
- 不访问 GPIO、串口、定时器等硬件资源。
- 不依赖 `Serves` 或 `Task`，禁止产生反向依赖。
- 如使用 CMSIS-DSP，应在模块文档中说明版本、数值类型和构建选项。

## 文件与命名

```text
Algorithm/
  algorithm_pid.h
  algorithm_pid.c
  algorithm_filter.h
  algorithm_filter.c
```

- 文件名使用 `algorithm_<module>.h/.c`。
- 公共函数使用 `Algorithm_<Module>_<Action>()`。
- 状态型算法使用上下文结构体，由调用方创建并持有。
- 常量使用 `const` 或宏定义，不散布无法解释的魔法数字。

## 设计规范

- 相同输入和相同状态必须得到可重复结果。
- 所有输入进行空指针、范围、除零和数值有效性检查。
- 避免动态内存分配；缓冲区由调用方提供，或使用固定大小上下文。
- 不使用隐藏的全局可变状态。需要历史数据时保存在显式上下文中。
- 明确采样周期、单位、坐标系、角度制/弧度制和数据范围。
- 浮点算法评估 MCU 执行时间与精度；必要时提供定点实现或限制调用频率。
- 循环必须有确定上限，禁止依赖不确定收敛的无限迭代。

## 推荐接口示例

```c
typedef struct {
    float kp;
    float ki;
    float kd;
    float integral;
    float previous_error;
} Algorithm_PID_t;

bool Algorithm_PID_Init(Algorithm_PID_t *pid,
                        float kp,
                        float ki,
                        float kd);
bool Algorithm_PID_Update(Algorithm_PID_t *pid,
                          float error,
                          float dt_s,
                          float *output);
```

## 验证要求

- 为正常值、边界值、非法值和溢出风险准备测试向量。
- 记录期望误差范围、最大执行时间和 RAM/栈占用。
- 使用真实采样数据回放时，测试数据与算法实现分离。
- 新增 `.c` 文件和包含目录已加入根 `CMakeLists.txt`。
