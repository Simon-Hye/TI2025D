#include "fdc2214.h"

I2C_HandleTypeDef myhi2c;

uint8_t FDC2214_Init(I2C_HandleTypeDef Mi) {
    myhi2c = Mi;
    uint8_t cof[2];
    uint16_t check[2];

    // RCOUNT: Measurement Time
    cof[0] = 0x34;
    cof[1] = 0xFB;
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, RCOUNT_CH0, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, RCOUNT_CH1, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, RCOUNT_CH2, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, RCOUNT_CH3, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);

    // SETTLECOUNT
    cof[0] = 0x00;
    cof[1] = 0x1B;
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, SETTLECOUNT_CH0, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, SETTLECOUNT_CH1, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, SETTLECOUNT_CH2, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, SETTLECOUNT_CH3, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);

    // CLOCK_DIVIDERS
    cof[0] = 0x20;
    cof[1] = 0x02;
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, CLOCK_DIVIDERS_C_CH0, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, CLOCK_DIVIDERS_C_CH1, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, CLOCK_DIVIDERS_C_CH2, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, CLOCK_DIVIDERS_C_CH3, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);

    // DRIVE CURRENT
    cof[0] = 0x78;
    cof[1] = 0x00;
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, DRIVE_CURRENT_CH0, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, DRIVE_CURRENT_CH1, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, DRIVE_CURRENT_CH2, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, DRIVE_CURRENT_CH3, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);

    // ERROR_CONFIG
    cof[0] = 0x00;
    cof[1] = 0x00;
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, ERROR_CONFIG, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);

    // MUX_CONFIG
    cof[0] = 0xC2;
    cof[1] = 0x0D;
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, MUX_CONFIG, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);

    // CONFIG
    cof[0] = 0x16;
    cof[1] = 0x01;
    HAL_I2C_Mem_Write(&myhi2c, FDC2214, CONFIG, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);

    // 红外检查
    HAL_I2C_Mem_Read(&myhi2c, FDC2214, MANUFACTURER_ID, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    check[0] = (cof[0] << 8) | cof[1];
    HAL_I2C_Mem_Read(&myhi2c, FDC2214, DEVICE_ID, I2C_MEMADD_SIZE_8BIT, cof, 2, 100);
    check[1] = (cof[0] << 8) | cof[1];

    if (check[0] == MANUFACTURER_ID_val && check[1] == DEVICE_ID_val)
        return 0;
    else
        return 1;
}


uint32_t FDC2214_GetCapacitanceData(uint8_t channel) {
    volatile uint32_t data;
    uint8_t i = 0;
    uint8_t pdata[4];
    switch (channel) {
        case 0:
            HAL_I2C_Mem_Read(&myhi2c, FDC2214, DATA_MSB_CH0, I2C_MEMADD_SIZE_8BIT, pdata, 4, 100);
            data = ((pdata[0] << 24 | pdata[1] << 16 | pdata[2] << 8 | pdata[3]) & 0x0fffffff);
            return data;
        case 1:
            HAL_I2C_Mem_Read(&myhi2c, FDC2214, DATA_MSB_CH1, I2C_MEMADD_SIZE_8BIT, pdata, 2, 100);
            HAL_Delay(5);
            HAL_I2C_Mem_Read(&myhi2c, FDC2214, DATA_LSB_CH1, I2C_MEMADD_SIZE_8BIT, pdata + 2, 2, 100);
            HAL_Delay(5);
            data = ((pdata[0] << 24 | pdata[1] << 16 | pdata[2] << 8 | pdata[3]) & 0x0fffffff);
            return data;
        case 2:
            HAL_I2C_Mem_Read(&myhi2c, FDC2214, DATA_MSB_CH2, I2C_MEMADD_SIZE_8BIT, pdata, 2, 100);
            HAL_Delay(5);
            HAL_I2C_Mem_Read(&myhi2c, FDC2214, DATA_LSB_CH2, I2C_MEMADD_SIZE_8BIT, pdata + 2, 2, 100);
            HAL_Delay(5);
            data = ((pdata[0] << 24 | pdata[1] << 16 | pdata[2] << 8 | pdata[3]) & 0x0fffffff);
            return data;
        case 3:
            HAL_I2C_Mem_Read(&myhi2c, FDC2214, DATA_MSB_CH3, I2C_MEMADD_SIZE_8BIT, pdata, 2, 100);
            HAL_Delay(5);
            HAL_I2C_Mem_Read(&myhi2c, FDC2214, DATA_LSB_CH3, I2C_MEMADD_SIZE_8BIT, pdata + 2, 2, 100);
            HAL_Delay(5);
            data = ((pdata[0] << 24 | pdata[1] << 16 | pdata[2] << 8 | pdata[3]) & 0x0fffffff);
            return data;
        default:
            return 0;
    }
}

float FDC2214_Cap_Calculate(uint8_t chx) {
    float Cap;
    uint32_t Data_FDC;
    Data_FDC = FDC2214_GetCapacitanceData(chx);
    Cap = 232021045.248 / (Data_FDC);
    return (Cap * Cap);
}
