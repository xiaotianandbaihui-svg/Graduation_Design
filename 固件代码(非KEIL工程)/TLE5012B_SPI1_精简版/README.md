# TLE5012B：硬件SPI1精简版

六步换相的完整调用说明见同目录`电机驱动使用示例.md`，单次标定、运行和停止示例见`motor_example.c`、`motor_example.h`。示例运行在板1；板3的四路舵机控制代码另在`固件示例/BDS300_TIM3_舵机`目录。

只处理编码器寄存器读写、0～360°机械角和带符号机械角速度。给STM32F103C8T6标准库工程使用，只加入本目录的 `tle5012b.c`、`tle5012b.h`，不要再加入旧版同名文件。

## 你负责的初始化

本代码不调用SPI_Init、SPI_Cmd、RCC配置或定时器配置。调用前，你应已经设置：

- SPI1主机、**16位**、MSB先发、CPOL低、CPHA第二边沿（Mode 1）。
- 双线全双工，软件NSS，内部NSS为高，SPI1已使能。
- PA5=SCK、PA6=MISO、PA7=MOSI，相关GPIO时钟已开启。
- id=0用PA4作CS；id=1用PB1作CS。使用的CS配置推挽输出，初始为高，未选中的编码器CS保持高。
- 编码器上电稳定至少10ms，再调用编码器初始化。
- 默认推挽SSC，SCK不超过8MHz；开漏SSC的时钟限制不同，不能直接照用。

## 五个接口

```c
TLE5012B_Init(id);                       // 清上电状态，确认角度可读
TLE5012B_ReadReg(id, reg, &value);        // 读完整16位寄存器
TLE5012B_WriteReg(id, reg, value);        // 写一个寄存器
TLE5012B_ReadAngle(id, &degrees);         // 直接取得[0,360)机械角，单位度
TLE5012B_ReadAngularSpeed(id, &deg_s);    // 芯片测得的机械角速度，单位度/秒，有正负
```

都返回 `TLE_OK`（0）表示成功，负值表示错误。错误时角度/读数据输出变量保持不变，不要把上一次的值当成本次新读数。

```c
#include "tle5012b.h"

float angle_deg;
uint16_t raw;
TLE_Result result;

/* 此前已经完成你自己的SPI1/GPIO配置，并等待编码器启动。 */
result = TLE5012B_Init(0);
if (result == TLE_OK) {
    result = TLE5012B_ReadAngle(0, &angle_deg);
    /* result==TLE_OK时，angle_deg才是这次读到的机械角。 */
}

/* 需要完整的AVAL寄存器时： */
result = TLE5012B_ReadReg(0, TLE_REG_AVAL, &raw);
/* 成功后，raw & 0x7FFF就是0..32767的角度计数。 */
```

`ReadAngle()`内部公式：`(raw & 0x7FFF) * 360.0f / 32768.0f`。所以0→0°，8192→90°，16384→180°，24576→270°，32767→约359.989°；整圈回到0°，不输出360°。

## 读取机械角速度

```c
float speed_deg_s;
float speed_rad_s;
float speed_rpm;

if (TLE5012B_ReadAngularSpeed(0, &speed_deg_s) == TLE_OK) {
    speed_rad_s = speed_deg_s * 0.01745329252f; // 换成rad/s
    speed_rpm = speed_deg_s / 6.0f;            // 换成转/分钟，保留方向正负
}
```

函数直接读取芯片的ASPD（03h，读取命令0x8031），不需要传采样间隔，不使用相邻两次软件读取角度作差。返回的是机械角速度，不乘电机极对数。正负号沿用编码器报告的方向，不能仅凭安装照片规定为轮子前进/后退。

ASPD低15位是有符号补码：bit14为符号位，bit15是读取状态。函数每次先读MOD1和MOD2取得FIR_MD、PREDICT、ANGRANGE，再读取ASPD，因此一次调用共读3个寄存器，不需要额外缓存初始化。

换算依据为Infineon官方库的 `getAngleSpeed()` 和 `calculateAngleSpeed()`：

```text
机械角速度[度/秒] = signed_ASPD × (360/32768) × (128/ANGRANGE)
                  ÷ (N × 芯片滤波更新周期[秒])

PREDICT=0：N=2；PREDICT=1：N=3。
FIR_MD=0/1/2/3时，标称周期分别为21.3/42.7/85.3/170.6微秒。
```

这些周期使用芯片内部时钟的标称值；真实速度比例精度也受该时钟误差影响。使用未知频率外部时钟、DSPU暂停或ANGRANGE为0时返回 `TLE_CONFIG_ERROR`，避免输出错误换算值。运行过程中不要在另一处并发改变这些配置。

参考：[Infineon官方TLE5012b.cpp](https://github.com/Infineon/TLE5012-Magnetic-Angle-Sensor/blob/master/src/corelib/TLE5012b.cpp)。数据手册Rev.2.0第27页表4-10列出常规FIR_MD=1/2/3的滤波更新周期；FIR_MD=0的21.3微秒取自上述官方实现。

## 代码中保留了哪些必要步骤

SPI1收发直接调用 `SPI_I2S_SendData()`、`SPI_I2S_ReceiveData()`，每次一个16位字。仍需等待TXE、RXNE、BSY；标准库接收函数只取出接收寄存器内容，不会自行产生读取时钟。等待有次数上限，不依赖你的毫秒延时函数。

CS拉低后发送命令。读取角度的命令是 `0x8021`，然后收16位数据、16位安全字，最后拉高CS。写操作则发送命令和数据，再接收安全字。

**TLE5012B的芯片DATA是双向线。** 本例在发送命令/写入数据时使PA7输出，接收时将PA7改成浮空输入，避免争用；SPI1的配置不变。PA6始终用于接收。

该做法适用于MOSI经串联电阻接DATA、MISO从DATA引出的接法。你的成品模块内部如何把DATA引成MOSI/MISO，仍需确认；若有独立缓冲/方向电路，应按模块电路调整 `DATA_Output()`。模块供电5V也不自动代表通信脚与3.3V兼容。

代码保留CRC和安全字检查；CRC涉及字节运算不代表SPI改成8位。芯片复位、系统错误、接口错误或角度无效统一返回 `TLE_STATUS_ERROR`。首次读STAT允许历史状态，之后读角度要求全部正常。

`SSC_Gap()`只用短NOP循环保证最小通信间隔，不使用定时器。函数使用同一SPI总线时不得并发调用；发生SPI超时后，外设恢复由应用处理，本代码不重新配置SPI1。

## 关于写寄存器

仅获取绝对机械角，不需要改写配置寄存器。

`WriteReg()`是单个寄存器的SSC写操作，不负责配置策略或EEPROM烧写。寄存器编号传未左移的地址，例如MOD2是`0x08`，不是`0x80`。读支持00h..11h；写接受ACSTAT（01h）及05h..0Fh，具体可写位和保留位仍应按芯片寄存器说明设置。

如果修改08h..0Fh等受配置CRC覆盖的内容，并启用了相应检查，还需要按手册维护配置CRC；本精简驱动不自动执行这些额外配置步骤。这里的配置CRC与每帧通信CRC不同。写函数报错也不代表器件一定未写入，必要时读回确认。

编码器驱动本身只处理编码器通信和数据换算。新增的独立六步换相模块为 `six_step.c`、`six_step.h`，使用它时再把 `six_step.c` 加入标准库工程，接口和引脚表见 `six_step.h`。旧版文件保留在原目录。

## 独立六步换相模块

`six_step.c` 直接调用本目录的 `TLE5012B_ReadAngle()` 和STM32F103标准库。沿用板1旧版网络表：电机1使用TIM2_CH2/3/4（PA1/PA2/PA3），电机2使用TIM4_CH2/3/4（PB7/PB8/PB9）；各自另用3路低侧GPIO，两片DRV8323H共享PB0 ENABLE，MODE必须接地。接线应与实物核对。

先完成原有SPI1、CS初始化和编码器上电等待，再执行 `SixStep_Init()`。之后用 `SixStep_Calibrate()` 自动取得零位和方向，或用 `SixStep_SetCalibration()` 载入真实、有效的旧标定；两者都不自动启动连续转动。标定会驱动轴转动约一圈电角度并检查各步位移，要求两路停止、轴可自由转动、使用限流电源，结束后关闭输出。

运行时周期调用 `SixStep_Update(id, duty, direction)`，`id=0/1`。`duty`为0~1000的无符号千分比，50表示5%，500表示50%，1000表示100%；0滑行停止。方向单独用 `SIX_STEP_FORWARD` 或 `SIX_STEP_REVERSE` 选择，与固定的编码器方向标定参数分开。默认7对极、20kHz PWM；运行接口接受完整0~1000范围，固定矢量标定仍单独限制到1~100。`SixStep_Stop(id)`滑行停止指定电机。编码器错误或检测到nFAULT低会关闭双路并锁存，排查后重新初始化。

换相的角度输入就是现有函数读取的0~360度机械角。先计算 `wrap360(encoder_sign * 7 * (mechanical - zero_deg))`，再依据电角度和方向选择六个驱动矢量。机械一圈有7个电周期、42个换相区间，每个区间约8.571度机械角；不能把整个机械360度只分成6步。

本模块独占TIM2/TIM4，直接调用应用已有的 `Delay_us(uint32_t us)` 微秒延时函数，不配置DWT；延时函数须在SixStep_Init前可用，并且不能占用TIM2/TIM4。模块关闭JTAG并保留SWD，不初始化SPI或注册中断。建议每台电机50~100微秒更新一次，实际执行时间需要测量。停止调用Update不会自动停止PWM；应用需要保证调度并设置必要的超时保护。nFAULT在更新和标定时轮询；没有精确电流限制、电流闭环或转速闭环。运行时不要在其他上下文修改编码器配置或抢占SPI1。

新模块已通过262144组角度/电机/方向组合检查、530521次中间GPIO/PWM状态检查，以及正反方向标定、跨零点、堵转与摆动标定失败、错误输入、故障锁存和立即停止检查。无符号占空比接口另检查了0、1、150、500、999、1000等边界和中间值、100%输出、独立方向切换及越界拒绝。使用真实STM32F1标准库和Cortex-M3工具链完成全部公开接口的编译链接，验证64KB Flash/20KB RAM布局。测试放在 `work/six_step_validation`，不加入Keil工程；未上板测试。

## 验证

2026-10-01：已用Arm Compiler 6.21、真实STM32F1标准库头文件，按Cortex-M3目标通过编译，启用`-Wall -Wextra -Werror`。另在电脑上验证了16位读写顺序、CS及DATA方向、命令字、CRC和状态错误、机械角边界、超时后释放片选及失败时保留输出变量。

尚未连接实物测试。验证用程序放在项目的`work/tle5012b_simple_validation`，不属于要加入Keil的驱动文件。

新增角速度函数已通过672组换算测试，覆盖两只编码器编号、15位有符号边界、bit15读取状态、FIR_MD/PREDICT组合、ANGRANGE缩放，以及失败时保持输出不变。完整驱动已再次通过Cortex-M3标准库编译；保留你已注释的方向切换调用，编译检查仅对现有未使用的`DATA_Output`函数关闭unused-function提示。未更改你的SPI1配置。
