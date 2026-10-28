/*********************************************************************************************************************
* CYT4BB Opensourec Library ���� CYT4BB ��Դ�⣩��һ�����ڹٷ� SDK �ӿڵĵ�������Դ��
* Copyright (c) 2022 SEEKFREE ��ɿƼ�
*
* ���ļ��� CYT4BB ��Դ���һ����
*
* CYT4BB ��Դ�� ���������
* �����Ը���������������ᷢ���� GPL��GNU General Public License���� GNUͨ�ù�������֤��������
* �� GPL �ĵ�3�棨�� GPL3.0������ѡ��ģ��κκ����İ汾�����·�����/���޸���
*
* ����Դ��ķ�����ϣ�����ܷ������ã�����δ�������κεı�֤
* ����û�������������Ի��ʺ��ض���;�ı�֤
* ����ϸ����μ� GPL
*
* ��Ӧ�����յ�����Դ���ͬʱ�յ�һ�� GPL �ĸ���
* ���û�У������<https://www.gnu.org/licenses/>
*
* ����ע����
* ����Դ��ʹ�� GPL3.0 ��Դ����֤Э�� ������������Ϊ���İ汾
* ��������Ӣ�İ��� libraries/doc �ļ����µ� GPL3_permission_statement.txt �ļ���
* ����֤������ libraries �ļ����� �����ļ����µ� LICENSE �ļ�
* ��ӭ��λʹ�ò����������� ���޸�����ʱ���뱣����ɿƼ��İ�Ȩ����������������
*
* �ļ�����          zf_device_imu963ra
* ��˾����          �ɶ���ɿƼ����޹�˾
* �汾��Ϣ          �鿴 libraries/doc �ļ����� version �ļ� �汾˵��
* ��������          IAR 9.40.1
* ����ƽ̨          CYT4BB
* ��������          https://seekfree.taobao.com/
*
* �޸ļ�¼
* ����              ����                ��ע
* 2024-01-12       pudding           first version
* 2026-10-17       BaiXiao           adapt for stm32f103c8t6
********************************************************************************************************************/

#include "zf_device_imu963ra.h"
#include "stm32f10x.h"
#include "Serial.h"
#include "stm32f10x_gpio.h"
#include "Delay.h"

int16_t imu963ra_gyro_x = 0, imu963ra_gyro_y = 0, imu963ra_gyro_z = 0;       // ��������������      GYRO (������)
int16_t imu963ra_acc_x = 0,  imu963ra_acc_y = 0,  imu963ra_acc_z = 0;        // ������ٶȼ�����     ACC  (accelerometer ���ٶȼ�)
int16_t imu963ra_mag_x = 0,  imu963ra_mag_y = 0,  imu963ra_mag_z = 0;        // �������������      MAG  (magnetometer ������)
float imu963ra_transition_factor[3] = {4098, 14.3, 3000};                  // ת��ʵ��ֵ�ı���


/*------------------------����ӿڳ�ʼ�������ͽӿں�����ʼ-------------------------------*/



/* STM32F103��׼��ײ㡣SPI2ģʽ0��8λȫ˫����CS���·���ɺ������ơ�
 * ��֡ʹ����ѯ����ʹ��DMA�����ر��жϣ�ͬһʱ���һ�������ķ��ʡ� */
#define IMU963RA_SPI_WAIT_COUNT 100000u /* ��ʱ��ѯ����������΢�� */
uint8_t imu963ra_spi_error = 1; /* 1=��δ��ʼ����SPI�쳣����ʼ������ */

void imu963ra_gpio_init(void)
{
    GPIO_InitTypeDef gpio;
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
    IMU963RA_CS(1); /* ��Ԥ�����������������ģʽ������CS���������� */
    GPIO_ResetBits(MIU963RA_SCK_PORT, IMU963RA_SPC_PIN);
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_Pin = IMU963RA_CS_PIN;
    gpio.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(IMU963RA_CS_PORT, &gpio);
    gpio.GPIO_Pin = IMU963RA_SPC_PIN;
    gpio.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(MIU963RA_SCK_PORT, &gpio);
    gpio.GPIO_Pin = IMU963RA_SDI_PIN;
    GPIO_Init(MIU963RA_SDI_PORT, &gpio);
    gpio.GPIO_Pin = IMU963RA_SDO_PIN;
    gpio.GPIO_Mode = GPIO_Mode_IN_FLOATING;
    GPIO_Init(MIU963RA_SDO_PORT, &gpio);
}

void imu963ra_spi_init(void)
{
    RCC_ClocksTypeDef clocks;
    SPI_InitTypeDef spi;
    uint16_t divider = 2, prescaler = SPI_BaudRatePrescaler_2;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2, ENABLE);
    SPI_I2S_DeInit(IMU963RA_SPI);
    imu963ra_gpio_init();
    RCC_GetClocksFreq(&clocks);
    /* SPI2ʱ������PCLK1������Ƶ��PCLK1=36MHzʱѡ��/4���õ�9MHz�� */
    while (clocks.PCLK1_Frequency/divider > IMU963RA_SPI_SPEED && divider < 256u) {
        divider *= 2u;
        prescaler += 8u;
    }
    SPI_StructInit(&spi);
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_NSS = SPI_NSS_Soft;
    spi.SPI_BaudRatePrescaler = prescaler;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(IMU963RA_SPI, &spi);
    SPI_Cmd(IMU963RA_SPI, ENABLE);
    imu963ra_spi_error = 0;
}

static uint8_t imu963ra_spi_wait(SPI_TypeDef *spi, uint16_t flag, FlagStatus state)
{
    uint32_t timeout = IMU963RA_SPI_WAIT_COUNT;
    if (imu963ra_spi_error) return 0;
    while (SPI_I2S_GetFlagStatus(spi, flag) != state) {
        if (--timeout == 0) {
            imu963ra_spi_error = 1;
            SPI_Cmd(spi, DISABLE);
            return 0;
        }
    }
    return 1;
}

uint8_t spi_read_write_byte(SPI_TypeDef *spi, uint8_t data)
{
    if (!imu963ra_spi_wait(spi, SPI_I2S_FLAG_TXE, SET)) return 0;
    SPI_I2S_SendData(spi, data);
    if (!imu963ra_spi_wait(spi, SPI_I2S_FLAG_RXNE, SET)) return 0;
    return (uint8_t)SPI_I2S_ReceiveData(spi);
}

static void imu963ra_spi_finish(SPI_TypeDef *spi)
{
    /* TXE����ʾ���ͻ���գ���BSY��������ſ�������CS�� */
    if (imu963ra_spi_wait(spi, SPI_I2S_FLAG_TXE, SET))
        imu963ra_spi_wait(spi, SPI_I2S_FLAG_BSY, RESET);
    if (SPI_I2S_GetFlagStatus(spi, SPI_I2S_FLAG_OVR) == SET) {
        imu963ra_spi_error = 1;
        SPI_Cmd(spi, DISABLE);
    }
}

static void spi_write_8bit_register(SPI_TypeDef *spi, uint8_t reg, uint8_t data)
{
    /* д��ʱҲ���߽������ݣ���ֹOVR�� */
    spi_read_write_byte(spi, reg);
    spi_read_write_byte(spi, data);
    imu963ra_spi_finish(spi);
}

static void spi_read_8bit_registers(SPI_TypeDef *spi, uint8_t reg, uint8_t *data, uint32_t len)
{
    uint32_t i;
    spi_read_write_byte(spi, reg);
    for (i = 0; i < len && !imu963ra_spi_error; ++i)
        data[i] = spi_read_write_byte(spi, 0xFFu); /* �����ֽ��ṩ��ȡʱ�� */
    imu963ra_spi_finish(spi);
    /* ԭ�ϲ�Ϊvoid�ӿڣ�ʧ��ʱ��0����ʹ��δ��ʼ���ֽڣ���������������־�� */
    if (imu963ra_spi_error)
        for (i = 0; i < len; ++i) data[i] = 0;
}

static uint8_t spi_read_8bit_register(SPI_TypeDef *spi, uint8_t reg)
{
    uint8_t data = 0;
    spi_read_8bit_registers(spi, reg, &data, 1);
    return data;
}


/*------------------------����ӿڳ�ʼ�������ͽӿں�������-------------------------------*/


#if IMU963RA_USE_SOFT_IIC
static soft_iic_info_struct imu963ra_iic_struct;

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA д�Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define imu963ra_write_acc_gyro_register(reg,data)       (soft_iic_write_8bit_register(&imu963ra_iic_struct,reg,data))

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA ���Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ���ز���     uint8_t           ����
// ʹ��ʾ��     imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define imu963ra_read_acc_gyro_register(reg)             (soft_iic_sccb_read_register(&imu963ra_iic_struct,reg))

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA ������ �ڲ�����
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ���ݻ�����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
#define imu963ra_read_acc_gyro_registers(reg,data,len)   (soft_iic_read_8bit_registers(&imu963ra_iic_struct,reg,data,len))
#else

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA д�Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ����
// ���ز���     void
// ʹ��ʾ��     imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_write_acc_gyro_register (uint8_t reg, uint8_t data)
{
    IMU963RA_CS(0);
    
    spi_write_8bit_register(IMU963RA_SPI, reg | IMU963RA_SPI_W, data);

    IMU963RA_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA ���Ĵ���
// ����˵��     reg             �Ĵ�����ַ
// ���ز���     uint8_t           ����
// ʹ��ʾ��     imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8_t imu963ra_read_acc_gyro_register (uint8_t reg)
{
    uint8_t data = 0;
    IMU963RA_CS(0);
    
    data = spi_read_8bit_register(IMU963RA_SPI, reg | IMU963RA_SPI_R);
    IMU963RA_CS(1);
    return data;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA ������ �ڲ�����
// ����˵��     reg             �Ĵ�����ַ
// ����˵��     data            ���ݻ�����
// ����˵��     len             ���ݳ���
// ���ز���     void
// ʹ��ʾ��     imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_read_acc_gyro_registers (uint8_t reg, uint8_t *data, uint32_t len)
{
    IMU963RA_CS(0);
    
    spi_read_8bit_registers(IMU963RA_SPI, reg | IMU963RA_SPI_R, data, len);

    IMU963RA_CS(1);
}
#endif

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA ��Ϊ IIC �����������д����
// ����˵��     addr            Ŀ���ַ
// ����˵��     reg             Ŀ��Ĵ���
// ����˵��     data            ����
// ���ز���     uint8_t           1-ʧ�� 0-�ɹ�
// ʹ��ʾ��     imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x80);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8_t imu963ra_write_mag_register (uint8_t addr, uint8_t reg, uint8_t data)
{
    uint8_t return_state = 0;
    uint16_t timeout_count = 0;

    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x00);               // �ӻ�0�������
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 0);              // ���õشżƵ�ַ��ע��������Ҫ����8λ��I2C��ַ�� 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // ��Ҫд��ļĴ�����ַ
    imu963ra_write_acc_gyro_register(IMU963RA_DATAWRITE_SLV0, data);            // ��Ҫд�������
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);             // ���ڵ�һ����������ͨѶ �������� I2C����ʹ��
    
    // �ȴ�ͨѶ�ɹ�
    while(0 == (0x80 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        Delay_ms(2);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA ��Ϊ IIC ����������ƶ�����
// ����˵��     addr            Ŀ���ַ
// ����˵��     reg             Ŀ��Ĵ���
// ���ز���     uint8_t           ��ȡ������
// ʹ��ʾ��     imu963ra_read_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CHIP_ID);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8_t imu963ra_read_mag_register (uint8_t addr, uint8_t reg)
{
    uint16_t timeout_count = 0;

    addr = addr << 1;
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);              // ���õشżƵ�ַ��ע��������Ҫ����8λ��I2C��ַ�� 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // ��Ҫ��ȡ�ļĴ�����ַ
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x01);    
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x4C);             // ���ڵ�һ����������ͨѶ �������� I2C����ʹ��
    
    // �ȴ�ͨѶ�ɹ�
    while(0 == (0x01 & imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER)))
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            break;
        }
        Delay_ms(2);
    }
    
    return (imu963ra_read_acc_gyro_register(IMU963RA_SENSOR_HUB_1));            // ���ض�ȡ��������
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA ��Ϊ IIC ������������Զ�д����
// ����˵��     addr            Ŀ���ַ
// ����˵��     reg             Ŀ��Ĵ���
// ���ز���     void
// ʹ��ʾ��     imu963ra_connect_mag(IMU963RA_MAG_ADDR, IMU963RA_MAG_OUTX_L);
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static void imu963ra_connect_mag (uint8_t addr, uint8_t reg)
{
    addr = addr << 1;
    
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_ADD, addr | 1);              // ���õشżƵ�ַ��ע��������Ҫ����8λ��I2C��ַ�� 0x2C
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_SUBADD, reg);                // ��Ҫ��ȡ�ļĴ�����ַ
    imu963ra_write_acc_gyro_register(IMU963RA_SLV0_CONFIG, 0x06);    
    imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x6C);             // ���ڵ�һ����������ͨѶ �������� I2C����ʹ��
}   


//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA �����Լ� �ڲ�����
// ����˵��     void
// ���ز���     uint8_t           1-�Լ�ʧ�� 0-�Լ�ɹ�
// ʹ��ʾ��     imu963ra_acc_gyro_self_check();
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8_t imu963ra_acc_gyro_self_check (void)
{
    uint8_t return_state = 0;
    uint8_t dat = 0;
    uint16_t timeout_count = 0;

    while(0x6B != dat)                                                          // �ж� ID �Ƿ���ȷ
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        dat = imu963ra_read_acc_gyro_register(IMU963RA_WHO_AM_I);
        Delay_ms(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     IMU963RA �������Լ� �ڲ�����
// ����˵��     void
// ���ز���     uint8_t           1-�Լ�ʧ�� 0-�Լ�ɹ�
// ʹ��ʾ��     imu963ra_mag_self_check();
// ��ע��Ϣ     �ڲ�����
//-------------------------------------------------------------------------------------------------------------------
static uint8_t imu963ra_mag_self_check (void)
{
    uint8_t return_state = 0;
    uint8_t dat = 0;
    uint16_t timeout_count = 0;

    while(0xff != dat)                                                          // �ж� ID �Ƿ���ȷ
    {
        if(IMU963RA_TIMEOUT_COUNT < timeout_count ++)
        {
            return_state = 1;
            break;
        }
        dat = imu963ra_read_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CHIP_ID);
        Delay_ms(10);
    }
    return return_state;
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ IMU963RA ���ٶȼ�����
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     imu963ra_get_acc();
// ��ע��Ϣ     ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_acc (void)
{
    uint8_t dat[6];

    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_A, dat, 6);
    imu963ra_acc_x = (int16_t)(((uint16_t)dat[1]<<8 | dat[0]));
    imu963ra_acc_y = (int16_t)(((uint16_t)dat[3]<<8 | dat[2]));
    imu963ra_acc_z = (int16_t)(((uint16_t)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ IMU963RA ����������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     imu963ra_get_gyro();
// ��ע��Ϣ     ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_gyro (void)
{
    uint8_t dat[6];

    imu963ra_read_acc_gyro_registers(IMU963RA_OUTX_L_G, dat, 6);
    imu963ra_gyro_x = (int16_t)(((uint16_t)dat[1]<<8 | dat[0]));
    imu963ra_gyro_y = (int16_t)(((uint16_t)dat[3]<<8 | dat[2]));
    imu963ra_gyro_z = (int16_t)(((uint16_t)dat[5]<<8 | dat[4]));
}


//-------------------------------------------------------------------------------------------------------------------
// �������     ��ȡ IMU963RA ����������
// ����˵��     void
// ���ز���     void
// ʹ��ʾ��     imu963ra_get_mag();
// ��ע��Ϣ     ִ�иú�����ֱ�Ӳ鿴��Ӧ�ı�������
//-------------------------------------------------------------------------------------------------------------------
void imu963ra_get_mag (void)
{
    uint8_t temp_status;
    uint8_t dat[6];

    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);
    temp_status = imu963ra_read_acc_gyro_register(IMU963RA_STATUS_MASTER);
    if(0x01 & temp_status)
    {
        imu963ra_read_acc_gyro_registers(IMU963RA_SENSOR_HUB_1, dat, 6);
        imu963ra_mag_x = (int16_t)(((uint16_t)dat[1]<<8 | dat[0]));
        imu963ra_mag_y = (int16_t)(((uint16_t)dat[3]<<8 | dat[2]));
        imu963ra_mag_z = (int16_t)(((uint16_t)dat[5]<<8 | dat[4]));
    }
    imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);
}

//-------------------------------------------------------------------------------------------------------------------
// �������     ��ʼ�� IMU963RA
// ����˵��     void
// ���ز���     uint8_t           1-��ʼ��ʧ�� 0-��ʼ���ɹ�
// ʹ��ʾ��     imu963ra_init();
// ��ע��Ϣ
//-------------------------------------------------------------------------------------------------------------------
uint8_t imu963ra_init (void)
{
    uint8_t return_state = 0;
    Delay_ms(10);                                                        // �ϵ���ʱ

#if IMU963RA_USE_SOFT_IIC
    soft_iic_init(&imu963ra_iic_struct, IMU963RA_DEV_ADDR, IMU963RA_SOFT_IIC_DELAY, IMU963RA_SCL_PIN, IMU963RA_SDA_PIN);
#else
    imu963ra_spi_init(); /* �Ѱ���GPIO��ʼ�� */

#endif

    do
    {
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // �ر�HUB�Ĵ�������
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x01);               // ��λ�豸
        Delay_ms(2);
        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // �ر�HUB�Ĵ�������
        if(imu963ra_acc_gyro_self_check())
        {
            printf("%s", "IMU963RA acc and gyro self check error.");
            return_state = 1;
            break;
        }

        imu963ra_write_acc_gyro_register(IMU963RA_INT1_CTRL, 0x03);             // ���������� ���ٶ����ݾ����ж�

        // IMU963RA_CTRL1_XL �Ĵ���
        // ����Ϊ 0x30 ���ٶ�����Ϊ ��2  G    ��ȡ���ļ��ٶȼ����ݳ��� 16393  ����ת��Ϊ��������λ������ ��λ g(m/s^2)
        // ����Ϊ 0x38 ���ٶ�����Ϊ ��4  G    ��ȡ���ļ��ٶȼ����ݳ��� 8197   ����ת��Ϊ��������λ������ ��λ g(m/s^2)
        // ����Ϊ 0x3C ���ٶ�����Ϊ ��8  G    ��ȡ���ļ��ٶȼ����ݳ��� 4098   ����ת��Ϊ��������λ������ ��λ g(m/s^2)
        // ����Ϊ 0x34 ���ٶ�����Ϊ ��16 G    ��ȡ���ļ��ٶȼ����ݳ��� 2049   ����ת��Ϊ��������λ������ ��λ g(m/s^2)
        switch(IMU963RA_ACC_SAMPLE_DEFAULT)
        {
            default:
            {
                printf("%s", "IMU963RA_ACC_SAMPLE_DEFAULT set error.");
                return_state = 1;
            }break;
            case IMU963RA_ACC_SAMPLE_SGN_2G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x30);
                imu963ra_transition_factor[0] = 16393;
            }break;
            case IMU963RA_ACC_SAMPLE_SGN_4G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x38);
                imu963ra_transition_factor[0] = 8197;
            }break;
            case IMU963RA_ACC_SAMPLE_SGN_8G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x3C);
                imu963ra_transition_factor[0] = 4098;
            }break;
            case IMU963RA_ACC_SAMPLE_SGN_16G:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL1_XL, 0x34);
                imu963ra_transition_factor[0] = 2049;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        // IMU963RA_CTRL2_G �Ĵ���
        // ����Ϊ 0x52 ����������Ϊ ��125  dps    ��ȡ�������������ݳ��� 228.6   ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x50 ����������Ϊ ��250  dps    ��ȡ�������������ݳ��� 114.3   ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x54 ����������Ϊ ��500  dps    ��ȡ�������������ݳ��� 57.1    ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x58 ����������Ϊ ��1000 dps    ��ȡ�������������ݳ��� 28.6    ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x5C ����������Ϊ ��2000 dps    ��ȡ�������������ݳ��� 14.3    ����ת��Ϊ��������λ������ ��λΪ ��/s
        // ����Ϊ 0x51 ����������Ϊ ��4000 dps    ��ȡ�������������ݳ��� 7.1     ����ת��Ϊ��������λ������ ��λΪ ��/s
        switch(IMU963RA_GYRO_SAMPLE_DEFAULT)
        {
            default:
            {
                printf("%s", "IMU963RA_GYRO_SAMPLE_DEFAULT set error.");
                return_state = 1;
            }break;
            case IMU963RA_GYRO_SAMPLE_SGN_125DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x52);
                imu963ra_transition_factor[1] = 228.6;
            }break;
            case IMU963RA_GYRO_SAMPLE_SGN_250DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x50);
                imu963ra_transition_factor[1] = 114.3;
            }break;
            case IMU963RA_GYRO_SAMPLE_SGN_500DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x54);
                imu963ra_transition_factor[1] = 57.1;
            }break;
            case IMU963RA_GYRO_SAMPLE_SGN_1000DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x58);
                imu963ra_transition_factor[1] = 28.6;
            }break;
            case IMU963RA_GYRO_SAMPLE_SGN_2000DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x5C);
                imu963ra_transition_factor[1] = 14.3;
            }break;
            case IMU963RA_GYRO_SAMPLE_SGN_4000DPS:
            {
                imu963ra_write_acc_gyro_register(IMU963RA_CTRL2_G, 0x51);
                imu963ra_transition_factor[1] = 7.1;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        imu963ra_write_acc_gyro_register(IMU963RA_CTRL3_C, 0x44);                   // ʹ�����������ֵ�ͨ�˲���
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL4_C, 0x02);                   // ʹ�����ֵ�ͨ�˲���
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL5_C, 0x00);                   // ���ٶȼ�����������������
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL6_C, 0x00);                   // �������ٶȼƸ�����ģʽ �����ǵ�ͨ�˲� 133hz
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL7_G, 0x00);                   // ���������Ǹ�����ģʽ �رո�ͨ�˲�
        imu963ra_write_acc_gyro_register(IMU963RA_CTRL9_XL, 0x01);                  // �ر�I3C�ӿ�

        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x40);           // ����HUB�Ĵ������� �������õشż�
        imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x80);             // ��λI2C����
        Delay_ms(2);                             
        imu963ra_write_acc_gyro_register(IMU963RA_MASTER_CONFIG, 0x00);             // �����λ��־
        Delay_ms(2);
        
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x80);// ��λ���ӵ�����
        Delay_ms(2);
        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL2, 0x00);
        Delay_ms(2);

        if(imu963ra_mag_self_check())
        {
            printf("%s", "IMU963RA mag self check error.");
            return_state = 1;
            break;            
        }

        // IMU963RA_MAG_ADDR �Ĵ���
        // ����Ϊ 0x09 ����������Ϊ 2G   ��ȡ���Ĵ��������ݳ��� 12000   ����ת��Ϊ��������λ������ ��λ G(��˹)
        // ����Ϊ 0x19 ����������Ϊ 8G   ��ȡ���Ĵ��������ݳ��� 3000    ����ת��Ϊ��������λ������ ��λ G(��˹)
        switch(IMU963RA_MAG_SAMPLE_DEFAULT)
        {
            default:
            {
                printf("%s", "IMU963RA_MAG_SAMPLE_DEFAULT set error.");
                return_state = 1;
            }break;
            case IMU963RA_MAG_SAMPLE_2G:
            {
                imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL1, 0x09);
                imu963ra_transition_factor[2] = 12000;
            }break;
            case IMU963RA_MAG_SAMPLE_8G:
            {
                imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_CONTROL1, 0x19);
                imu963ra_transition_factor[2] = 3000;
            }break;
        }
        if(1 == return_state)
        {
            break;
        }

        imu963ra_write_mag_register(IMU963RA_MAG_ADDR, IMU963RA_MAG_FBR, 0x01);
        imu963ra_connect_mag(IMU963RA_MAG_ADDR, IMU963RA_MAG_OUTX_L);

        imu963ra_write_acc_gyro_register(IMU963RA_FUNC_CFG_ACCESS, 0x00);       // �ر�HUB�Ĵ�������

        Delay_ms(20);                                                    // �ȴ������ƻ�ȡ����
    }while(0);
    return return_state;
}
