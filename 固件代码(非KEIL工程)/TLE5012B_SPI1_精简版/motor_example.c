#include "motor_example.h"

extern void Delay_us(uint32_t us); /* 使用你已有的微秒延时函数 */

SixStep_Result Motor_Example_RunOnce(void)
{
    SixStep_Calibration cal;
    SixStep_Result result;
    uint32_t i;

    /* 前提：应用已经配置好SPI1和两路CS；此处只补足编码器启动等待。 */
    for (i = 0u; i < 10u; ++i) Delay_us(1000u);

    result = SixStep_Init();
    if (result != SIX_STEP_OK) goto failed;

    /* 50表示5%占空比，只是示例起始值，不代表已验证的电流或启动能力。
     * 成功后cal包含零位和方向，同时已在模块内部生效。 */
    result = SixStep_Calibrate(0u, 50u, &cal);
    if (result != SIX_STEP_OK) goto failed;

    for (i = 0u; i < 20000u; ++i) {
        result = SixStep_Update(0u, 50u, SIX_STEP_FORWARD);
        if (result != SIX_STEP_OK) goto failed;
        /* 实际间隔=本次Update耗时+50us，须测量是否满足电机转速要求。
         * 20000次迭代不是精确1秒；实际运行时间还包含SPI等处理耗时。 */
        Delay_us(50u);
    }

    SixStep_Stop(0u); /* 关断该桥的驱动，滑行停止；不是主动刹车 */
    return SIX_STEP_OK;

failed:
    SixStep_StopAll();
    return result;
}
