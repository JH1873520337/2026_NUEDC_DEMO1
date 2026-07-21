#include "task_manager.h"

#include "cmsis_os.h"

#include "TASK1.h"
#include "TASK2.h"
#include "task_a4_vision.h"
#include "serve_gimbal.h"
#include "serve_screen.h"

#define TASK_MANAGER_POLL_PERIOD_MS  10U

volatile Task_Manager_Runtime_t Task_Manager_Runtime;

static osThreadId_t Task_Manager_WorkerHandle;

static const osThreadAttr_t Task_Manager_WorkerAttributes = {
    .name = "taskWorker",
    .stack_size = 256U * 4U,
    .priority = (osPriority_t)osPriorityBelowNormal,
};

static void Task_Manager_StopWorker(void)
{
    if (Task_Manager_WorkerHandle == NULL) {
        return;
    }

    Serve_Gimbal_Stop();
    (void)osThreadTerminate(Task_Manager_WorkerHandle);
    Task_Manager_WorkerHandle = NULL;
}

static void Task_Manager_Start(void (*entry)(void *),
                               Task_Manager_State_t state)
{
    Task_Manager_StopWorker();

    Task_Manager_WorkerHandle = osThreadNew(
        entry, NULL, &Task_Manager_WorkerAttributes);
    if (Task_Manager_WorkerHandle == NULL) {
        Task_Manager_Runtime.state = TASK_MANAGER_STATE_FAULT;
        return;
    }

    Task_Manager_Runtime.state = state;
}

static void Task_Manager_StartA4Vision(void)
{
    Task_Manager_Start(Task_A4Vision_Entry,
                       TASK_MANAGER_STATE_A4_VISION);
    if (Task_Manager_Runtime.state == TASK_MANAGER_STATE_A4_VISION) {
        Task_Manager_Runtime.a4_vision_start_count++;
    }
}

void Task_Manager_Entry(void *argument)
{
    Serve_Screen_Command_t command;

    (void)argument;

    Task_Manager_WorkerHandle = NULL;
    Task_Manager_Runtime.state = TASK_MANAGER_STATE_IDLE;
    Task_Manager_Runtime.task1_start_count = 0U;
    Task_Manager_Runtime.task2_start_count = 0U;
    Task_Manager_Runtime.a4_vision_start_count = 0U;

    for (;;) {
        while (Serve_Screen_GetCommand(&command) == SERVE_SCREEN_OK) {
            switch (command) {
            case SERVE_SCREEN_CMD_HOME:
                Task_Manager_Start(TASK1_Entry, TASK_MANAGER_STATE_TASK1);
                if (Task_Manager_Runtime.state == TASK_MANAGER_STATE_TASK1) {
                    Task_Manager_Runtime.task1_start_count++;
                }
                break;

            case SERVE_SCREEN_CMD_SCREEN_EDGE:
                Task_Manager_Start(TASK2_Entry, TASK_MANAGER_STATE_TASK2);
                if (Task_Manager_Runtime.state == TASK_MANAGER_STATE_TASK2) {
                    Task_Manager_Runtime.task2_start_count++;
                }
                break;

            case SERVE_SCREEN_CMD_START:
            case SERVE_SCREEN_CMD_A4_TARGET:
                /* 当前比赛程序的 START 入口对应基本要求（3）、（4）。 */
                Task_Manager_StartA4Vision();
                break;

            case SERVE_SCREEN_CMD_PAUSE:
                if (Task_Manager_Runtime.state ==
                    TASK_MANAGER_STATE_A4_VISION) {
                    Task_A4Vision_RequestPause();
                }
                break;

            default:
                break;
            }
        }

        osDelay(TASK_MANAGER_POLL_PERIOD_MS);
    }
}
