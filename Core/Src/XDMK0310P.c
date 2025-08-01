//
// Created by Ellipticer on 2025/8/1.
//
#include "XDMK0310P.h"

uint8_t sendcommond[6] = {0x01, 0xa6, 0x02, 0x8a, 0x8a, 0xbd};
uint8_t data[16] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

float XDMK0310P_getresister(UART_HandleTypeDef *myuart) {
    uint32_t unit = 1;
    uint16_t rawdata = 0;
    float answer;
    HAL_UART_Transmit(myuart, sendcommond, 6, 100);
    HAL_UART_Receive(myuart, data, 16, 2000);
    if (data[4] == 0) {
        unit = 100;
    } else if (data[4] == 1) {
        unit = 10000;
    } else {
        unit = 100000;
    }
    rawdata = ((uint16_t) data[5]) << 8 | data[6];
    answer = (float) rawdata / unit;
    return answer;
}
