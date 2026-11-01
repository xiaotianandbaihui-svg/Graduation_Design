#include "task_config.h"
#include "app_tasks.h"

/* 名称      任务入口       栈(字) 优先级 间隔(ms) 句柄
 * GUI: 2048字=8KB；LoRa: 512字=2KB。实板调试时可观察栈余量再调整。 */
TaskHandle_t  TCB_Poinetr[2]={NULL,NULL};/*存放任务句柄*/
AppTaskConfig app_task_config[] = {
    {"GUI",  App_GuiTask,  2048u, 2u, 5u, &TCB_Poinetr[0]},
    {"LoRa", App_LoraTask,  512u, 3u, 5u, &TCB_Poinetr[1]}
};

/*记录当前任务数*/
const UBaseType_t app_task_count = sizeof(app_task_config) / sizeof(app_task_config[0]);

