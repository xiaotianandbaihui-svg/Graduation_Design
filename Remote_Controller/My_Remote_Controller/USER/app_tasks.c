#include "app_tasks.h"
#include "task_config.h"
#include "remote_control.h"
#include "remote_ui.h"
#include "lv_port_disp_template.h"
#include "lv_port_indev_template.h"

/* 单个对齐int的读写在本机是原子的；只保留最近一次非零UART发送结果。 */
static volatile int uart_tx_result;

BaseType_t AppTasks_Create(void)
{
    UBaseType_t i;
    for (i = 0u; i < app_task_count; ++i) {
        AppTaskConfig *config = &app_task_config[i];
        if (xTaskCreate(config->entry, config->name, config->stack_words, config,
                        config->priority, config->handle) != pdPASS) return pdFAIL;
    }
    return pdPASS;
}

/* FreeRTOS为1kHz。lv_tick_inc可在中断调用，其余LVGL操作只在GUI任务执行。 */
void vApplicationTickHook(void)
{
    lv_tick_inc(portTICK_PERIOD_MS);/*给LVGL提供时基*/
}

void App_GuiTask(void *argument)
{
    const AppTaskConfig *config = argument;
    lv_init();
    lv_port_disp_init();  /* 初始化LCD及横屏 */
    lv_port_indev_init(); /* 横屏完成后初始化触摸 */
    RemoteUi_Init();
    for (;;) {
        lv_timer_handler();
        RemoteUi_SetTxResult(uart_tx_result);
        vTaskDelay(pdMS_TO_TICKS(config->period_ms));
    }
}

void App_LoraTask(void *argument)
{
    const AppTaskConfig *config = argument;
    TickType_t wake = xTaskGetTickCount();
    for (;;) {
        int result = RemoteControl_Poll((uint32_t)xTaskGetTickCount() * portTICK_PERIOD_MS);
        if (result) uart_tx_result = result;
        vTaskDelayUntil(&wake, pdMS_TO_TICKS(config->period_ms));
    }
}
