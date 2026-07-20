# KalmanFilter 一维卡尔曼滤波器

## 文件

- `KalmanFilter.h`：滤波器状态结构体和公共接口。
- `KalmanFilter.c`：一维标量卡尔曼滤波实现。

## 用途

该模块为舵机角度指令提供第二级滤波。舵机驱动先使用一阶低通滤波，再把低通结果作为卡尔曼滤波器的观测量，从而减小指令突变和抖动。

## 接口

```c
KalmanFilter_t filter;

KalmanFilter_Init(&filter,
                  0.02f,  /* 过程噪声 Q */
                  0.10f,  /* 测量噪声 R */
                  90.0f,  /* 初始估计 */
                  1.0f);  /* 初始协方差 */

float result = KalmanFilter_Update(&filter, measurement);
```

`Q` 越大，输出跟随输入越快；`R` 越大，输出越平稳。舵机驱动中的默认参数定义在 `Servo.h`，可根据实际机械负载和调用周期调整。

## 注意事项

- 这是无动态内存、非阻塞的一维滤波器。
- 该模块不包含硬件依赖，可以用于其他传感器或控制量。
- 多任务同时访问同一个 `KalmanFilter_t` 实例时，需要由调用者进行互斥保护。
