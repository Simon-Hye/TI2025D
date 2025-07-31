//
// Created by Ellipticer on 2025/7/31.
//

#ifndef TEST_ONEPORT_74HC4051_H
#define TEST_ONEPORT_74HC4051_H

#define HC4051_MAX_COUNT 4

#include"main.h"

typedef struct {
    GPIO_TypeDef* S0_port;
    uint16_t S0_pin;
    GPIO_TypeDef* S1_port;
    uint16_t S1_pin;
    GPIO_TypeDef* S2_port;
    uint16_t S2_pin;
    GPIO_TypeDef* E_port;
    uint16_t E_pin;
    uint8_t id;
} HC4051;

int HC4051_init(GPIO_TypeDef *p0p, uint16_t p0,
                        GPIO_TypeDef *p1p, uint16_t p1,
                        GPIO_TypeDef *p2p, uint16_t p2,
                        GPIO_TypeDef *p3p, uint16_t p3);
void HC4051_close(uint8_t index);
void HC4051_open(uint8_t index,uint8_t portnum);
#endif //TEST_ONEPORT_74HC4051_H
