# Serves 服务层规范

## 定位

`Serves` 是应用服务层，负责连接上层 RTOS 任务与下层 BSP、算法模块。它把驱动能力和算法能力组织成可复用的业务服务，并维护状态机、数据流、错误恢复和资源仲裁。

目录当前命名为 `Serves`，代码和 CMake 路径应统一沿用该名称，避免同时出现 `Serve`、`Service` 等平行目录。

## 依赖边界

```text
Task
  |
  v
Serves ---> Algorithm
  |
  v
 BSP
```

- 可以依赖 `BSP` 和 `Algorithm`。
- 被 `Task` 调用，但不得反向依赖或包含任务层头文件。
- 除极少数系统级需求外，不直接调用 STM32 HAL/LL；硬件访问必须经过 BSP。
- 不创建永久运行的 RTOS 任务。服务逻辑由 `Task` 调度，或以短时、可返回的接口执行。
- 不把具体任务句柄、优先级和调度周期写入服务层。

## 主要职责

- 组合多个 BSP 驱动，形成完整功能流程。
- 调用算法层处理采样数据，并把结果转换为上层可消费的状态或事件。
- 管理模块状态机、启动/停止、超时、重试和故障降级。
- 统一资源所有权，例如同一 UART、SPI 总线或执行器的访问仲裁。
- 向 Task 提供稳定、与硬件细节无关的服务接口。

## 文件与命名

```text
Serves/
  serve_vision.h
  serve_vision.c
  serve_control.h
  serve_control.c
```

- 文件名使用 `serve_<module>.h/.c`。
- 公共函数使用 `Serve_<Module>_<Action>()`。
- 状态和配置使用上下文结构体集中管理，避免散落的全局变量。
- 一个服务只负责一个清晰领域；过大的服务应按数据采集、控制、通信等职责拆分。

## 接口规范

推荐为服务提供清晰的生命周期：

```c
Serve_Status_t Serve_Vision_Init(const Serve_Vision_Config_t *config);
Serve_Status_t Serve_Vision_Start(void);
Serve_Status_t Serve_Vision_Process(uint32_t now_ms);
Serve_Status_t Serve_Vision_GetResult(Serve_Vision_Result_t *result);
void Serve_Vision_Stop(void);
```

- `Init` 只初始化资源，不隐式启动永久行为。
- `Process` 应有明确执行上限，不阻塞 RTOS 调度。
- 输入输出参数进行空指针、长度和范围检查。
- 错误码应能区分参数错误、忙、超时、底层故障和算法失败。
- 明确每个接口可从任务上下文还是中断上下文调用；默认仅允许任务上下文。
- 跨任务数据通过队列、事件或受保护的快照传递，不暴露可被并发修改的内部指针。

## 提交前检查

- Task 不需要了解 HAL 句柄、GPIO 引脚或器件寄存器。
- BSP 不包含服务层状态或业务判断。
- 状态机包含超时和异常恢复路径。
- 阻塞时间、调用周期和线程安全要求已在头文件中说明。
- 新增 `.c` 文件和包含目录已加入根 `CMakeLists.txt`。
