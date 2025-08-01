//
// Created by Ellipticer on 2025/7/31.
//
#include "74HC4051.h"

static HC4051 devices[HC4051_MAX_COUNT];
static uint8_t device_count = 0;

/**
 * @brief
 * @param p0p S0的GPIO类型指针
 * @param p0 S0的引脚号
 * @param p1p S1的GPIO类型指针
 * @param p1 S1的引脚号
 * @param p2p S2的GPIO类型指针
 * @param p2 S2的引脚号
 * @param p3p E的GPIO类型指针
 * @param p3 E的GPIO的引脚号
 * @return
 */
int HC4051_init(
    GPIO_TypeDef *p0p, uint16_t p0,
    GPIO_TypeDef *p1p, uint16_t p1,
    GPIO_TypeDef *p2p, uint16_t p2,
    GPIO_TypeDef *p3p, uint16_t p3)
{
    if (device_count >= HC4051_MAX_COUNT)
        return -1; // 设备数量超限

    HC4051 *dev = &devices[device_count];
    dev->S0_port = p0p; dev->S0_pin = p0;
    dev->S1_port = p1p; dev->S1_pin = p1;
    dev->S2_port = p2p; dev->S2_pin = p2;
    dev->E_port  = p3p; dev->E_pin  = p3;
    dev->id = device_count;
    HAL_GPIO_WritePin(dev->E_port, dev->E_pin, GPIO_PIN_SET); // 默认关闭

    return device_count++;
}
/**
 * @brief 关闭所有的开关
 * @param index 设备索引
 */
void HC4051_close(uint8_t index)
{
    if (index >= device_count) return;
    HAL_GPIO_WritePin(devices[index].E_port, devices[index].E_pin, GPIO_PIN_SET);
}
/**
 * @brief 开启某个端口
 * @param portnum 希望开启的开关号（0-7）
 * @param index 设备索引（0-3）
 */
void HC4051_open(uint8_t index,uint8_t portnum)
{
    if (index >= device_count) return;
    HC4051 *dev = &devices[index];

    HAL_GPIO_WritePin(dev->E_port, dev->E_pin, GPIO_PIN_RESET);
    HAL_Delay(3);

    uint8_t value[3] = {
        portnum & 0x01,
        (portnum >> 1) & 0x01,
        (portnum >> 2) & 0x01
    };

    HAL_GPIO_WritePin(dev->S0_port, dev->S0_pin, value[0]);
    HAL_GPIO_WritePin(dev->S1_port, dev->S1_pin, value[1]);
    HAL_GPIO_WritePin(dev->S2_port, dev->S2_pin, value[2]);

    HAL_Delay(3);
}

