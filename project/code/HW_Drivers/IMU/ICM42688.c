/*
 * ICM42688.c
 *
 *  Created on: 2024-08-02
 *      Author: ljk
 */
/* MIT License
 *
 * Copyright (c) 2024 ljk
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "ICM42688.h"

#define ICM42688_SPI_BURST_TIMEOUT_US (100U)
#define ICM42688_DWT_UNLOCK_KEY        (0xC5ACCE55U)
#define ICM42688_ID_CHECK_INTERVAL     (100U)
#define ICM42688_FROZEN_FRAME_LIMIT    (100U)

/* zf_driver_spi.c 中的模块表；ICM42688 仅使用独占的 SPI2/SCB9。 */
extern volatile stc_SCB_t *spi_module[4];

/*
 * ICM42688 默认配置（1kHz）
 * 1) 陀螺仪量程 2000dps，覆盖小车常见角速度
 * 2) 加速度量程 16g，覆盖常见动态范围
 * 3) 三阶滤波 + Bandwidth_Factor_2，兼顾噪声与延迟
 * 4) 上电后进入 LN（Low Noise）模式
 */
ICM42688_CONFIG_STRUCT ICM42688_CONFIG = {
    GYRO_2000DPS,
    GYRO_ODR_1000HZ,
    ACC_16G,
    ACC_ODR_1000HZ,
    _3st,
    Bandwidth_Factor_2,
    _3st,
    Bandwidth_Factor_2,
    Bias_On_Chip_Off
};

float Gyro_Sensitivity, Acc_Sensitivity;
ICM42688_RAW_DATA ICM42688_RAW;      /* 原始 LSB 数据 */
ICM42688_real_data ICM42688;         /* 换算后的物理量 */
volatile float ICM42688_Bias_gyro_x = 0;
volatile float ICM42688_Bias_gyro_y = 0;
volatile float ICM42688_Bias_gyro_z = 0;
volatile uint8 ICM42688_Bias_Init_Flag = 0;
volatile uint32 g_icm42688_spi_timeout_count = 0U;
volatile uint32 g_icm42688_invalid_frame_count = 0U;
volatile uint32 g_icm42688_identity_error_count = 0U;

static uint8 s_icm42688_cycle_counter_ready = 0U;
static uint8 s_icm42688_last_raw_valid = 0U;
static uint16 s_icm42688_frozen_frame_count = 0U;
static uint16 s_icm42688_identity_check_divider = 0U;
static ICM42688_RAW_DATA s_icm42688_last_raw = {0};

static uint8 ICM42688_CycleCounter_Init(void)
{
    uint32 start_cycles;

    if ((s_icm42688_cycle_counter_ready != 0U) &&
        ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0U))
    {
        return 1U;
    }

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = ICM42688_DWT_UNLOCK_KEY;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    start_cycles = DWT->CYCCNT;
    __NOP();
    __NOP();
    __NOP();
    __NOP();
    s_icm42688_cycle_counter_ready =
        (DWT->CYCCNT != start_cycles) ? 1U : 0U;
    return s_icm42688_cycle_counter_ready;
}

static void ICM42688_SPI_Recover(void)
{
    volatile stc_SCB_t *module = spi_module[(uint32)ICM42688_SPI];

    module->unCTRL.stcField.u1ENABLED = 0U;
    Cy_SCB_SPI_ClearTxFifo(module);
    Cy_SCB_SPI_ClearRxFifo(module);
    /* SPI2 为 IMU 独占；运行期恢复必须明确重新启用，避免永久离线。 */
    module->unCTRL.stcField.u1ENABLED = 1U;
}

static uint8 ICM42688_SPI_Transfer8_Timeout(const uint8 *write_buffer,
                                            uint8 *read_buffer,
                                            uint32 len,
                                            uint32 timeout_us)
{
    volatile stc_SCB_t *module = spi_module[(uint32)ICM42688_SPI];
    uint32 clock_hz;
    uint32 start_cycles;
    uint32 timeout_cycles;

    if ((write_buffer == NULL) || (read_buffer == NULL) ||
        (len == 0U) || (timeout_us == 0U) ||
        (ICM42688_CycleCounter_Init() == 0U))
    {
        return 0U;
    }

    clock_hz = SystemCoreClock;
    if (clock_hz == 0U)
    {
        clock_hz = system_clock;
    }
    timeout_cycles = (clock_hz / 1000000U) * timeout_us;
    if (timeout_cycles == 0U)
    {
        return 0U;
    }

    /* 初始化寄存器访问使用 16bit，采样突发读取前切回 8bit。 */
    module->unCTRL.u32Register &= 0xFFFF3FFFU;
    module->unTX_CTRL.u32Register &= 0xFFFFFFE0U;
    module->unTX_CTRL.u32Register |= 0x00000007U;
    module->unRX_CTRL.u32Register &= 0xFFFFFFE0U;
    module->unRX_CTRL.u32Register |= 0x00000007U;
    Cy_SCB_SPI_ClearRxFifo(module);

    start_cycles = DWT->CYCCNT;
    while (len > 0U)
    {
        Cy_SCB_WriteTxFifo(module, *write_buffer++);
        while (Cy_SCB_IsTxComplete(module) == 0U)
        {
            if ((uint32)(DWT->CYCCNT - start_cycles) >= timeout_cycles)
            {
                ICM42688_SPI_Recover();
                return 0U;
            }
        }

        while (Cy_SCB_SPI_GetNumInRxFifo(module) == 0U)
        {
            if ((uint32)(DWT->CYCCNT - start_cycles) >= timeout_cycles)
            {
                ICM42688_SPI_Recover();
                return 0U;
            }
        }

        *read_buffer++ = (uint8)module->unRX_FIFO_RD.u32Register;
        len--;
    }

    return 1U;
}

/* 初始化 SPI 与片选 GPIO */
static void ICM42688_SPI_HardWare_Init(void)
{
    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPEED,
             ICM42688_SCK_Pin, ICM42688_MOSI_Pin, ICM42688_MISO_Pin, SPI_CS_NULL);
    gpio_init(ICM42688_CS_Pin, GPO, 1, GPO_PUSH_PULL);
}

/* 发送并接收 16bit SPI 数据 */
static uint16_t SPI_Transfer_16bit(uint16_t dout)
{
    uint16_t return_data;

    gpio_low(ICM42688_CS_Pin);
    spi_transfer_16bit(ICM42688_SPI, &dout, &return_data, 1);
    gpio_high(ICM42688_CS_Pin);

    return return_data;
}

/*
 * 连续读取加速度与陀螺仪原始值
 * 数据顺序：ax ay az gx gy gz
 */
static uint8 ICM42688_Read_Burst(ICM42688_RAW_DATA *raw)
{
    uint8_t tx_buf[13] = {0};
    uint8_t rx_buf[13] = {0};
    ICM42688_RAW_DATA new_raw = {0};

    if (raw == NULL)
    {
        return 0U;
    }

    tx_buf[0] = READ_ACC_X_HIGH;

    gpio_low(ICM42688_CS_Pin);
    if (ICM42688_SPI_Transfer8_Timeout(tx_buf, rx_buf, 13U,
                                      ICM42688_SPI_BURST_TIMEOUT_US) == 0U)
    {
        gpio_high(ICM42688_CS_Pin);
        g_icm42688_spi_timeout_count++;
        return 0U;
    }
    gpio_high(ICM42688_CS_Pin);

    new_raw.acc_x_lsb = (int16)(((uint16)rx_buf[1] << 8) | rx_buf[2]);
    new_raw.acc_y_lsb = (int16)(((uint16)rx_buf[3] << 8) | rx_buf[4]);
    new_raw.acc_z_lsb = (int16)(((uint16)rx_buf[5] << 8) | rx_buf[6]);
    new_raw.gyro_x_lsb = (int16)(((uint16)rx_buf[7] << 8) | rx_buf[8]);
    new_raw.gyro_y_lsb = (int16)(((uint16)rx_buf[9] << 8) | rx_buf[10]);
    new_raw.gyro_z_lsb = (int16)(((uint16)rx_buf[11] << 8) | rx_buf[12]);

    if (((new_raw.acc_x_lsb == 0) &&
         (new_raw.acc_y_lsb == 0) &&
         (new_raw.acc_z_lsb == 0) &&
         (new_raw.gyro_x_lsb == 0) &&
         (new_raw.gyro_y_lsb == 0) &&
         (new_raw.gyro_z_lsb == 0)) ||
        ((new_raw.acc_x_lsb == -1) &&
         (new_raw.acc_y_lsb == -1) &&
         (new_raw.acc_z_lsb == -1) &&
         (new_raw.gyro_x_lsb == -1) &&
         (new_raw.gyro_y_lsb == -1) &&
         (new_raw.gyro_z_lsb == -1)))
    {
        g_icm42688_invalid_frame_count++;
        return 0U;
    }

    if ((s_icm42688_last_raw_valid != 0U) &&
        (memcmp(&new_raw, &s_icm42688_last_raw, sizeof(new_raw)) == 0))
    {
        if (s_icm42688_frozen_frame_count < ICM42688_FROZEN_FRAME_LIMIT)
        {
            s_icm42688_frozen_frame_count++;
        }
        if (s_icm42688_frozen_frame_count >= ICM42688_FROZEN_FRAME_LIMIT)
        {
            g_icm42688_invalid_frame_count++;
            return 0U;
        }
    }
    else
    {
        s_icm42688_last_raw = new_raw;
        s_icm42688_last_raw_valid = 1U;
        s_icm42688_frozen_frame_count = 0U;
    }

    *raw = new_raw;
    return 1U;
}

static uint8 ICM42688_CheckIdentityBounded(void)
{
    uint8 tx_buf[2] = {(uint8)(WHO_AM_I >> 8), 0U};
    uint8 rx_buf[2] = {0U};

    gpio_low(ICM42688_CS_Pin);
    if (ICM42688_SPI_Transfer8_Timeout(tx_buf, rx_buf, 2U,
                                      ICM42688_SPI_BURST_TIMEOUT_US) == 0U)
    {
        gpio_high(ICM42688_CS_Pin);
        g_icm42688_spi_timeout_count++;
        return 0U;
    }
    gpio_high(ICM42688_CS_Pin);

    if (rx_buf[1] != ICM42688_ID)
    {
        g_icm42688_identity_error_count++;
        return 0U;
    }
    return 1U;
}

/* 读取并校验芯片 ID */
static void Find_ICM42688(void)
{
    SPI_Transfer_16bit(WHO_AM_I);
    uint8 ID = (uint8)SPI_Transfer_16bit(WHO_AM_I);

    if (ID != ICM42688_ID)
    {
        printf("ICM42688 not recognized,Read:%d\n", ID);
        while (1)
        {
        }
    }
}

/* 配置陀螺仪量程与输出速率 */
static void ICM42688_SET_GYRO(GYRO_FSR gyro_fsr, GYRO_ODR gyro_odr)
{
    uint8 WRITE_GYRO_CONFIG0 = 0x4F;
    uint8 GYRO_CONFIG0 = 0x00;
    uint8 GYRO_CONFIG_Verify = 0x00;
    uint16 READ_GYRO_CONFIG0 = 0XCF00;

    switch (gyro_fsr)
    {
        case GYRO_15_625DPS:
            GYRO_CONFIG0 |= 0xE0;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_15_625dps;
            break;
        case GYRO_31_25DPS:
            GYRO_CONFIG0 |= 0xC0;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_31_25dps;
            break;
        case GYRO_62_5DPS:
            GYRO_CONFIG0 |= 0xA0;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_62_5dps;
            break;
        case GYRO_125DPS:
            GYRO_CONFIG0 |= 0x80;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_125dps;
            break;
        case GYRO_250DPS:
            GYRO_CONFIG0 |= 0x60;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_250dps;
            break;
        case GYRO_500DPS:
            GYRO_CONFIG0 |= 0x40;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_500dps;
            break;
        case GYRO_1000DPS:
            GYRO_CONFIG0 |= 0x20;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_1000dps;
            break;
        case GYRO_2000DPS:
            GYRO_CONFIG0 |= 0x00;
            Gyro_Sensitivity = SENSITIVITY_ICM42688_GYRO_2000dps;
            break;
    }

    switch (gyro_odr)
    {
        case GYRO_ODR_12_5HZ:
            GYRO_CONFIG0 |= 0x0B;
            break;
        case GYRO_ODR_25HZ:
            GYRO_CONFIG0 |= 0x0A;
            break;
        case GYRO_ODR_50HZ:
            GYRO_CONFIG0 |= 0x09;
            break;
        case GYRO_ODR_100HZ:
            GYRO_CONFIG0 |= 0x08;
            break;
        case GYRO_ODR_200HZ:
            GYRO_CONFIG0 |= 0x07;
            break;
        case GYRO_ODR_500HZ:
            GYRO_CONFIG0 |= 0x0F;
            break;
        case GYRO_ODR_1000HZ:
            GYRO_CONFIG0 |= 0x06;
            break;
        case GYRO_ODR_2000HZ:
            GYRO_CONFIG0 |= 0x05;
            break;
        case GYRO_ODR_4000HZ:
            GYRO_CONFIG0 |= 0x04;
            break;
        case GYRO_ODR_8000HZ:
            GYRO_CONFIG0 |= 0x03;
            break;
        case GYRO_ODR_16000HZ:
            GYRO_CONFIG0 |= 0x02;
            break;
        case GYRO_ODR_32000HZ:
            GYRO_CONFIG0 |= 0x01;
            break;
    }

    SPI_Transfer_16bit(((uint16)WRITE_GYRO_CONFIG0 << 8 | GYRO_CONFIG0));
    GYRO_CONFIG_Verify = (uint8)SPI_Transfer_16bit(READ_GYRO_CONFIG0);

    if (GYRO_CONFIG_Verify != GYRO_CONFIG0)
    {
        printf("Gyroscope set failed!\n");
        while (1)
        {
        }
    }
}

/* 配置加速度计量程与输出速率 */
static void ICM42688_SET_ACC(ACC_FSR acc_fsr, ACC_ODR acc_odr)
{
    uint8 WRITE_ACCEL_CONFIG0 = 0x50;
    uint8 ACCEL_CONFIG0 = 0x00;
    uint8 ACCEL_CONFIG_Verify = 0x00;
    uint16 READ_ACCEL_CONFIG0 = 0XD000;

    switch (acc_fsr)
    {
        case ACC_2G:
            ACCEL_CONFIG0 |= 0x60;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_2G;
            break;
        case ACC_4G:
            ACCEL_CONFIG0 |= 0x40;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_4G;
            break;
        case ACC_8G:
            ACCEL_CONFIG0 |= 0x20;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_8G;
            break;
        case ACC_16G:
            ACCEL_CONFIG0 |= 0x00;
            Acc_Sensitivity = SENSITIVITY_ICM42688_ACC_16G;
            break;
    }

    switch (acc_odr)
    {
        case ACC_ODR_12_5HZ:
            ACCEL_CONFIG0 |= 0x0B;
            break;
        case ACC_ODR_25HZ:
            ACCEL_CONFIG0 |= 0x0A;
            break;
        case ACC_ODR_50HZ:
            ACCEL_CONFIG0 |= 0x09;
            break;
        case ACC_ODR_100HZ:
            ACCEL_CONFIG0 |= 0x08;
            break;
        case ACC_ODR_200HZ:
            ACCEL_CONFIG0 |= 0x07;
            break;
        case ACC_ODR_500HZ:
            ACCEL_CONFIG0 |= 0x0F;
            break;
        case ACC_ODR_1000HZ:
            ACCEL_CONFIG0 |= 0x06;
            break;
        case ACC_ODR_2000HZ:
            ACCEL_CONFIG0 |= 0x05;
            break;
        case ACC_ODR_4000HZ:
            ACCEL_CONFIG0 |= 0x04;
            break;
        case ACC_ODR_8000HZ:
            ACCEL_CONFIG0 |= 0x03;
            break;
        case ACC_ODR_16000HZ:
            ACCEL_CONFIG0 |= 0x02;
            break;
        case ACC_ODR_32000HZ:
            ACCEL_CONFIG0 |= 0x01;
            break;
    }

    SPI_Transfer_16bit(((uint16)WRITE_ACCEL_CONFIG0 << 8 | ACCEL_CONFIG0));
    ACCEL_CONFIG_Verify = (uint8)SPI_Transfer_16bit(READ_ACCEL_CONFIG0);

    if (ACCEL_CONFIG_Verify != ACCEL_CONFIG0)
    {
        printf("Accelerometer set failed!\n");
        while (1)
        {
        }
    }
}

/* 配置陀螺仪/加速度计数字滤波器 */
static void ICM42688_SET_FILTER(Bandwidth_Factor Gyro_Bandwidth_Factor,
                                Filter_Order Gyro_Filter_Order,
                                Bandwidth_Factor Acc_Bandwidth_Factor,
                                Filter_Order Acc_Filter_Order)
{
    uint8 WRITE_GYRO_ACCEL_CONFIG0 = 0X52;
    uint8 WRITE_GYRO_CONFIG1 = 0X51;
    uint8 WRITE_ACCEL_CONFIG1 = 0X53;
    uint8 Gyro_Filter_Ord_Config = 0x00;
    uint8 Acc_Filter_Ord_Config = 0x00;
    uint8 Filter_Config = 0x00;

    switch (Gyro_Bandwidth_Factor)
    {
        case Bandwidth_Factor_2:
            Filter_Config |= 0x00;
            break;
        case Bandwidth_Factor_4:
            Filter_Config |= 0x01;
            break;
        case Bandwidth_Factor_5:
            Filter_Config |= 0x02;
            break;
        case Bandwidth_Factor_8:
            Filter_Config |= 0x03;
            break;
        case Bandwidth_Factor_10:
            Filter_Config |= 0x04;
            break;
        case Bandwidth_Factor_16:
            Filter_Config |= 0x05;
            break;
        case Bandwidth_Factor_20:
            Filter_Config |= 0x06;
            break;
        case Bandwidth_Factor_40:
            Filter_Config |= 0x07;
            break;
        case Low_latency_1:
            Filter_Config |= 0x0E;
            break;
        case Low_Latency_2:
            Filter_Config |= 0x0F;
            break;
    }

    switch (Acc_Bandwidth_Factor)
    {
        case Bandwidth_Factor_2:
            Filter_Config |= 0x00;
            break;
        case Bandwidth_Factor_4:
            Filter_Config |= 0x10;
            break;
        case Bandwidth_Factor_5:
            Filter_Config |= 0x20;
            break;
        case Bandwidth_Factor_8:
            Filter_Config |= 0x30;
            break;
        case Bandwidth_Factor_10:
            Filter_Config |= 0x40;
            break;
        case Bandwidth_Factor_16:
            Filter_Config |= 0x50;
            break;
        case Bandwidth_Factor_20:
            Filter_Config |= 0x60;
            break;
        case Bandwidth_Factor_40:
            Filter_Config |= 0x70;
            break;
        case Low_latency_1:
            Filter_Config |= 0xE0;
            break;
        case Low_Latency_2:
            Filter_Config |= 0xF0;
            break;
    }

    SPI_Transfer_16bit(((uint16)WRITE_GYRO_ACCEL_CONFIG0 << 8 | Filter_Config));

    switch (Gyro_Filter_Order)
    {
        case _1st:
            Gyro_Filter_Ord_Config |= 0x02;
            break;
        case _2st:
            Gyro_Filter_Ord_Config |= 0x06;
            break;
        case _3st:
            Gyro_Filter_Ord_Config |= 0xA0;
            break;
    }
    SPI_Transfer_16bit(((uint16)WRITE_GYRO_CONFIG1 << 8 | Gyro_Filter_Ord_Config));

    switch (Acc_Filter_Order)
    {
        case _1st:
            Acc_Filter_Ord_Config |= 0x02;
            break;
        case _2st:
            Acc_Filter_Ord_Config |= 0x06;
            break;
        case _3st:
            Acc_Filter_Ord_Config |= 0xA0;
            break;
    }
    SPI_Transfer_16bit(((uint16)WRITE_ACCEL_CONFIG1 << 8 | Acc_Filter_Ord_Config));
}

/* 复位 ICM42688（软复位路径） */
static void Rest_ICM42688(void)
{
    uint8 WRITE_PWR_MGMT0 = 0X4E;
    SPI_Transfer_16bit(((uint16)WRITE_PWR_MGMT0 << 8 | 0X00));
}

/* 进入 Low Noise 模式（陀螺仪 + 加速度计） */
static void Set_ICM42688_LN_Mode(void)
{
    uint8 WRITE_PWR_MGMT0 = 0X4E;
    SPI_Transfer_16bit(((uint16)WRITE_PWR_MGMT0 << 8 | 0X0F));
}

/*
 * 读取一次传感器数据，存入 ICM42688（物理量）和 ICM42688_RAW（LSB）
 * 流程：burst 读 13 字节 -> LSB/灵敏度 -> 乘轴符号 -> 扣零偏（如果已标定）
 */
uint8 ICM42688_Get_Data(void)
{
    ICM42688_RAW_DATA new_raw;
    float gyro_x_raw;
    float gyro_y_raw;
    float gyro_z_raw;
    float acc_x_raw;
    float acc_y_raw;
    float acc_z_raw;

    if (ICM42688_Read_Burst(&new_raw) == 0U)
    {
        return 0U;
    }

    s_icm42688_identity_check_divider++;
    if (s_icm42688_identity_check_divider >= ICM42688_ID_CHECK_INTERVAL)
    {
        if (ICM42688_CheckIdentityBounded() == 0U)
        {
            /* 身份异常后逐帧复检，使连续失败能触发上层 5 帧故障门限。 */
            s_icm42688_identity_check_divider =
                ICM42688_ID_CHECK_INTERVAL - 1U;
            return 0U;
        }
        s_icm42688_identity_check_divider = 0U;
    }

    ICM42688_RAW = new_raw;

    /* LSB -> 物理值（dps 或 g） */
    gyro_x_raw = ICM42688_RAW.gyro_x_lsb / Gyro_Sensitivity;
    gyro_y_raw = ICM42688_RAW.gyro_y_lsb / Gyro_Sensitivity;
    gyro_z_raw = ICM42688_RAW.gyro_z_lsb / Gyro_Sensitivity;

    acc_x_raw = ICM42688_RAW.acc_x_lsb / Acc_Sensitivity;
    acc_y_raw = ICM42688_RAW.acc_y_lsb / Acc_Sensitivity;
    acc_z_raw = ICM42688_RAW.acc_z_lsb / Acc_Sensitivity;

    /* 乘轴符号，转为车体坐标系 */
    ICM42688.gyro_x = ICM42688_SIGN_GX * gyro_x_raw;
    ICM42688.gyro_y = ICM42688_SIGN_GY * gyro_y_raw;
    ICM42688.gyro_z = ICM42688_SIGN_GZ * gyro_z_raw;

    ICM42688.acc_x = ICM42688_SIGN_AX * acc_x_raw;
    ICM42688.acc_y = ICM42688_SIGN_AY * acc_y_raw;
    ICM42688.acc_z = ICM42688_SIGN_AZ * acc_z_raw;

    /* 已标定则扣除陀螺零偏 */
    if (ICM42688_Bias_Init_Flag == 1)
    {
        ICM42688.gyro_x -= ICM42688_Bias_gyro_x;
        ICM42688.gyro_y -= ICM42688_Bias_gyro_y;
        ICM42688.gyro_z -= ICM42688_Bias_gyro_z;
    }
    return 1U;
}

/*
 * 陀螺仪静态零偏标定
 * - 标定期间必须保持设备静止！
 * - times 最小 500，建议 1000+，标定耗时约 times/1000 秒
 * - 结果存入 Bias_gyro_x/y/z，并置位 Bias_Init_Flag（只能标定一次）
 */
void ICM42688_Bias_Init(uint32 times)
{
    int64_t sum_gyro_x = 0;
    int64_t sum_gyro_y = 0;
    int64_t sum_gyro_z = 0;
    uint32 i;
    uint32 valid_samples = 0U;

    if (ICM42688_Bias_Init_Flag == 1)
    {
        return;
    }

    if (times < 500U)
    {
        times = 500U;
    }

    for (i = 0U; i < times; i++)
    {
        if (ICM42688_Read_Burst(&ICM42688_RAW) == 0U)
        {
            continue;
        }
        sum_gyro_x += ICM42688_RAW.gyro_x_lsb;
        sum_gyro_y += ICM42688_RAW.gyro_y_lsb;
        sum_gyro_z += ICM42688_RAW.gyro_z_lsb;
        valid_samples++;
        system_delay_us(ICM42688_SAMPLE_INTERVAL_US);
    }

    if (valid_samples == 0U)
    {
        return;
    }

    ICM42688_Bias_gyro_x = ICM42688_SIGN_GX * (((float)sum_gyro_x / (float)valid_samples) / Gyro_Sensitivity);
    ICM42688_Bias_gyro_y = ICM42688_SIGN_GY * (((float)sum_gyro_y / (float)valid_samples) / Gyro_Sensitivity);
    ICM42688_Bias_gyro_z = ICM42688_SIGN_GZ * (((float)sum_gyro_z / (float)valid_samples) / Gyro_Sensitivity);
    ICM42688_Bias_Init_Flag = 1;
}

/* 设置陀螺零偏（dps），enable=0 时关闭零偏补偿。用于从 Flash 恢复标定值 */
void ICM42688_SetGyroBiasDps(float bx, float by, float bz, uint8 enable)
{
    uint32 irq_state = interrupt_global_disable();

    ICM42688_Bias_gyro_x = bx;
    ICM42688_Bias_gyro_y = by;
    ICM42688_Bias_gyro_z = bz;
    ICM42688_Bias_Init_Flag = (enable != 0U) ? 1U : 0U;
    interrupt_global_enable(irq_state);
}

/* 读取当前陀螺零偏（dps）和启用状态，用于保存到 Flash */
void ICM42688_GetGyroBiasDps(float *bx, float *by, float *bz, uint8 *enable)
{
    float local_bx;
    float local_by;
    float local_bz;
    uint8 local_enable;
    uint32 irq_state = interrupt_global_disable();

    local_bx = ICM42688_Bias_gyro_x;
    local_by = ICM42688_Bias_gyro_y;
    local_bz = ICM42688_Bias_gyro_z;
    local_enable = ICM42688_Bias_Init_Flag;
    interrupt_global_enable(irq_state);

    if (bx != NULL)
    {
        *bx = local_bx;
    }
    if (by != NULL)
    {
        *by = local_by;
    }
    if (bz != NULL)
    {
        *bz = local_bz;
    }
    if (enable != NULL)
    {
        *enable = local_enable;
    }
}

/* 初始化 ICM42688 并加载配置 */
void ICM42688_Init(ICM42688_CONFIG_STRUCT *ICM42688_CONFIG)
{
    g_icm42688_spi_timeout_count = 0U;
    g_icm42688_invalid_frame_count = 0U;
    g_icm42688_identity_error_count = 0U;
    s_icm42688_last_raw_valid = 0U;
    s_icm42688_frozen_frame_count = 0U;
    s_icm42688_identity_check_divider = ICM42688_ID_CHECK_INTERVAL - 1U;
    ICM42688_SPI_HardWare_Init();
    Rest_ICM42688();
    system_delay_ms(10);

    Find_ICM42688();
    ICM42688_SET_GYRO((*ICM42688_CONFIG).GYRO_FSR, (*ICM42688_CONFIG).GYRO_ODR);
    ICM42688_SET_ACC((*ICM42688_CONFIG).ACC_FSR, (*ICM42688_CONFIG).ACC_ODR);
    ICM42688_SET_FILTER((*ICM42688_CONFIG).Gyro_Bandwidth_Factor,
                        (*ICM42688_CONFIG).Gyro_Filter_Order,
                        (*ICM42688_CONFIG).Acc_Bandwidth_Factor,
                        (*ICM42688_CONFIG).Acc_Filter_Order);

    system_delay_ms(10);
    Set_ICM42688_LN_Mode();
    system_delay_ms(500);

    /* 清零软件侧零偏状态 */
    ICM42688_Bias_Init_Flag = 0;
    ICM42688_Bias_gyro_x = 0;
    ICM42688_Bias_gyro_y = 0;
    ICM42688_Bias_gyro_z = 0;
}
