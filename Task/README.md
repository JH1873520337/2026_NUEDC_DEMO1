# Task RTOS 任务层规范

## 定位

`Task` 存放可直接交给 FreeRTOS 调度的成型任务。任务层负责调度周期、优先级、任务间通信、服务生命周期和系统运行状态，不负责具体硬件驱动或算法实现。

## 依赖边界

```text
FreeRTOS
   |
   v
 Task -> Serves -> BSP
                  -> Algorithm
```

- 原则上只调用 `Serves` 暴露的接口。
- 禁止直接调用 HAL/LL 或访问外设句柄、寄存器和 GPIO。
- 禁止在任务文件中实现可复用算法或器件驱动。
- 不允许 BSP、Algorithm 或 Serves 反向依赖 Task。
- `Core/Src/freertos.c` 仅保留 CubeMX 对象创建和启动入口；具体任务实现放在本目录，减少重新生成代码时的冲突。

## 文件与命名

```text
Task/
  task_control.h
  task_control.c
  task_communication.h
  task_communication.c
```

- 文件名使用 `task_<module>.h/.c`。
- 任务入口使用 `Task_<Module>_Entry(void *argument)`。
- 创建函数使用 `Task_<Module>_Create()`，任务句柄默认保持模块私有。
- 栈大小、优先级、周期和队列深度集中定义，并写明选取依据。

## 调度规范

- 周期任务使用 `vTaskDelayUntil()`，避免用 `vTaskDelay()` 累积周期漂移。
- 事件驱动任务优先阻塞在队列、通知、信号量或事件组上，禁止空转轮询。
- 每个循环都必须存在阻塞点或明确的让出机制。
- 高优先级任务不得执行日志打印、长时间 HAL 阻塞或复杂浮点运算。
- 任务入口不得意外返回；确需退出时先释放资源，再调用 `vTaskDelete(NULL)`。
- 看门狗喂狗应基于任务健康状态，不得掩盖死锁或超时。

## 通信与并发

- 单向数据流优先使用队列或任务通知，状态广播可使用事件组。
- 互斥锁只保护共享资源，临界区保持尽可能短。
- ISR 只使用 `...FromISR()` API，并正确执行 `portYIELD_FROM_ISR()`。
- 明确消息结构体所有权；队列传值时控制结构体大小，传指针时明确缓冲区生命周期。
- 禁止多个任务无保护地读写同一服务状态或 BSP 资源。
- 为队列满、等待超时、服务错误和资源不可用设计恢复策略。

## 推荐任务骨架

```c
void Task_Control_Entry(void *argument)
{
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        (void)Serve_Control_Process();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(TASK_CONTROL_PERIOD_MS));
    }
}
```

## 提交前检查

- 任务优先级不存在无依据的抢占关系或优先级反转风险。
- 栈水位通过 `uxTaskGetStackHighWaterMark()` 等方式实测并保留余量。
- CPU 占用、最坏执行时间和周期抖动满足实时性要求。
- 所有阻塞调用都有合理超时，异常路径不会形成忙循环。
- 新增 `.c` 文件和包含目录已加入根 `CMakeLists.txt`。
