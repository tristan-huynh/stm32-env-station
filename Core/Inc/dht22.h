#ifndef DHT22_H
#define DHT22_H

#include "main.h"
#include <stdint.h>

#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_8

void DHT22_Init(TIM_HandleTypeDef *htim);
void DHT22_ReadData(float* temperature, float* humidity);

#endif // DHT22_H