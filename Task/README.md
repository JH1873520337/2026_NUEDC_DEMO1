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

## 当前工程实现

当前任务层包含两个任务：

```text
Task_Communication
    |
    | Task_ControlCommand_t
    v
Task_Control
    |
    v
Gimbal Service
```

### 控制任务

控制任务由 `Task_Control_Create()` 创建，配置如下：

| 项目 | 配置 |
|---|---|
| 任务入口 | `Task_Control_Entry()` |
| 周期 | 10 ms |
| 优先级 | `tskIDLE_PRIORITY + 3` |
| 栈大小 | 512 words |
| 命令队列深度 | 8 |
| 单周期最大处理命令数 | 4 |

该任务负责调用 `Gimbal_Init()` 和消费控制命令。云台服务访问由内部互斥锁串行化，其他任务不得直接调用 `Gimbal` 接口。

控制命令包括：

```c
TASK_CONTROL_COMMAND_STOP
TASK_CONTROL_COMMAND_START
TASK_CONTROL_COMMAND_PAUSE
TASK_CONTROL_COMMAND_RESUME
TASK_CONTROL_COMMAND_RESET
TASK_CONTROL_COMMAND_SET_ABSOLUTE
TASK_CONTROL_COMMAND_SET_RELATIVE
TASK_CONTROL_COMMAND_SET_VECTOR
```

`TASK_CONTROL_COMMAND_RESUME` 是 `TASK_CONTROL_COMMAND_START` 的兼容命令，两者均调用 `Gimbal_Start()`。

### 云台更新任务

`Core/Src/freertos.c` 创建独立的 `gimbalUpdate` 任务，入口为 `gimbal_update()`：

| 项目 | 配置 |
|---|---|
| 更新周期 | 5 ms |
| 延时方式 | `osDelay(5)` |
| 优先级 | `osPriorityAboveNormal` |
| 栈大小 | 256 words |

任务不直接访问 `Gimbal`，而是调用 `Task_Control_GimbalUpdate()`。该接口与控制命令共用互斥锁，确保 `Gimbal_Update()` 不会和暂停、启动、复位或新目标命令并发修改云台状态。

### 通信转发任务

通信任务由 `Task_Communication_Create()` 创建，配置如下：

| 项目 | 配置 |
|---|---|
| 任务入口 | `Task_Communication_Entry()` |
| 调度方式 | 队列阻塞 |
| 优先级 | `tskIDLE_PRIORITY + 2` |
| 栈大小 | 256 words |
| 输入队列深度 | 8 |
| 转发超时 | 10 ms |

该任务只负责把协议层已经解析完成的 `Task_ControlCommand_t` 转发给控制任务，不直接访问 UART、GPIO 或 HAL。串口屏、视觉板或其他命令来源应在各自驱动/服务中完成协议解析，然后调用：

```c
Task_Communication_Submit(&command, timeout_ticks);
```

中断上下文使用：

```c
BaseType_t higher_priority_task_woken = pdFALSE;

Task_Communication_SubmitFromISR(&command,
                                 &higher_priority_task_woken);
portYIELD_FROM_ISR(higher_priority_task_woken);
```

### 暂停与启动

```text
执行原目标
  -> PAUSE
  -> 保存原目标并保持当前位置
  -> 等待 START
  -> 从当前位置继续原目标
```

重复发送 `PAUSE` 不会覆盖第一次保存的目标。未处于等待状态时发送 `START` 会返回无效命令。

### 复位与启动

```text
执行原目标
  -> RESET
  -> 保存原目标
  -> 运动到云台原点 (0,0)
  -> 到达原点并保持
  -> 等待 START
  -> 从原点继续原目标
```

复位途中可以暂停。此时第一次 `START` 只继续完成回原点动作，到达原点后再次等待；第二次 `START` 才恢复复位前保存的原目标。

`STOP`、新的绝对命令、相对命令或向量命令会取消暂停/复位保存的目标。

### 状态与错误统计

`Task_Control_GetStatus()` 返回云台状态、任务初始化状态以及接受/拒绝命令计数。`Task_Communication_GetStatus()` 返回成功转发和丢弃命令计数。队列满、转发超时和服务拒绝不会进入忙循环。

### FreeRTOS 创建入口

`MX_FREERTOS_Init()` 只调用：

```c
Task_Control_Create();
Task_Communication_Create();
osThreadNew(gimbal_update, NULL, &gimbalUpdateTask_attributes);
```

具体任务循环仍保留在 `Task/` 目录，避免 CubeMX 重新生成 `freertos.c` 时覆盖业务实现。

## 提交前检查

- 任务优先级不存在无依据的抢占关系或优先级反转风险。
- 栈水位通过 `uxTaskGetStackHighWaterMark()` 等方式实测并保留余量。
- CPU 占用、最坏执行时间和周期抖动满足实时性要求。
- 所有阻塞调用都有合理超时，异常路径不会形成忙循环。
- 新增 `.c` 文件和包含目录已加入根 `CMakeLists.txt`。
