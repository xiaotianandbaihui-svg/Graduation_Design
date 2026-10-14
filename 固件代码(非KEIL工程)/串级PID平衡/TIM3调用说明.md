# TIM3置标志、主循环调用PID

按你的约定：各轮以电角度增加方向为正，车体前进对应的轮速符号在外部规定。代码不再做安装方向转换。

## 流程

1. 全局BalancePID balance_pid。
2. main开头Balance_Init，填写三组Kp/Ki/Kd，然后Balance_Start。
3. Board_Init配置已有SPI1/CS/IMU或通信；Motor_init初始化并标定电机。
4. TIM3每5ms只把全局volatile PID_Refresh_Flag置1。
5. 主循环看到标志就清零，读取俯仰角和两轮编码器角速度，计算PID。
6. 命令直接拆成方向和幅值；SVPWM_Update放在if外持续换相。

## 需要加入工程

- balance_main_example.c：复制流程到已有main.c，不要同时链接两个main。
- balance_tim3_example.c/.h：TIM3和命令转换。
- balance_pid.c/.h：纯PID。
- 你现有的svpwm.c/.h、tle5012b.c/.h及标准库依赖。

此主循环例程不需要balance_svpwm_example.c。所有增益默认0，先按5ms周期整定。

对接你已有的两个应用函数：
```c
int Board_Init(void);                    /* 外设初始化，不重配系统时钟 */
int Balance_ReadPitch(float *pitch_deg); /* 前倾为正，单位度 */
```
均返回1成功、0失败。ReadPitch可读取本地姿态结果或板三发来的角度；不要把电机机械角当成机身俯仰角。

## 方向与命令

```c
left_rpm  = left_dps  / 6.0f * SVPWM_GetEncoderSign(0);
right_rpm = right_dps / 6.0f * SVPWM_GetEncoderSign(1);
Balance_Update(&balance_pid, pitch, left_rpm, right_rpm, command);
Balance_CommandToDrive(command, dir, modulation);
SVPWM_Update(dir[0], dir[1], modulation[0], modulation[1]);
```

rpm仍是机械转速，只是正方向与电角度增加一致；没有乘极对数。按直驱1:1，有减速器再换算传动比。

```text
command    = { 300, -200 }
dir        = {  +1,   -1 }
modulation = { 300,  200 }
```

零命令转换为dir=+1、幅值0，驱动停止该路输出。转换函数用于PID给出的±1000命令，不重复限幅。

外环给左右内环同一个目标轮速；你须在外部保证两轮对这个共同目标的方向约定一致。若前倾需要负向轮速追赶，由外环参数极性产生负目标；内环误差始终是目标轮速减实际轮速。驱动实际接收电压调制幅值，不是牛·米单位的转矩。

## 调度

```c
while (1) {
    if (PID_Refresh_Flag) {
        PID_Refresh_Flag = 0;
        /* 读取、PID、拆分命令 */
    }
    SVPWM_Update(dir[0], dir[1], modulation[0], modulation[1]);
}
```

两次PID之间保持幅值，仍持续更新电角度。TIM3不占GPIO；电机PWM继续使用TIM2/TIM4。
72MHz定时器时钟下PSC=72-1、ARR=5000-1，即5ms。只读取现有时钟；开启中断前清除初始化UG的标志。只能保留一个TIM3_IRQHandler。

标志不排队，主循环应及时响应；读取和运算太慢会合并刷新事件、拖慢换相，所以硬件5ms中断不等于实际处理绝对准时。标志在开始处理时清除。

传感器、PID、SPI和SVPWM都由主循环串行调用；本例程中断不调用延时。其他中断也不要并发使用这些驱动或共用SysTick的延时。
出错时停止定时器、PID和双电机，不自动重启。没有新增看门狗或命令超时状态。

已通过真实标准库头文件语法检查及主机模拟，覆盖符号转换、定时标志、主循环处理与持续换相、故障停机。未做实车整定或耗时测量。