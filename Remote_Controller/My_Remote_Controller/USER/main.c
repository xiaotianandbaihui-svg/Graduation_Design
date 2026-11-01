#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "remote_control.h"
#include "app_tasks.h"

int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);/**/
    SystemCoreClockUpdate();
    delay_init((u8)(SystemCoreClock / 1000000u));/*这里的进行了8分频*/
    uart_init(REMOTE_UART_BAUD); /* LoRa???????USART1 PA9/PA10??printf????? */
    
    RemoteControl_Init(); /* 先初始化共享控制数据，再创建任务 */
    if (AppTasks_Create() != pdPASS) {
        printf("Application task allocation failed\r\n");
        for (;;) {}
    }
    vTaskStartScheduler();/*开启调度器*/
    printf("Scheduler start failed\r\n");
    for (;;) {}
}
