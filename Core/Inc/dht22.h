#ifndef DHT22_H
#define DHT22_H

#include "main.h"

// DHT22 connected to PA8
#define DHT22_PORT GPIOA
#define DHT22_PIN GPIO_PIN_8

void DHT22_Start(void);
uint8_t DHT22_ReadByte(void);
void DHT22_ReadData(float* temperature, float* humidity);

#endif // DHT22_H