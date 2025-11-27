#ifndef FANCONTROL_H
#define FANCONTROL_H

#include "main.h"
#include <stdint.h>

// fan control definitions
#define FANCONTROL_PORT_IN1 GPIOB
#define FANCONTROL_PIN_IN1 GPIO_PIN_10 // D6

void FAN_setSpeed(uint8_t speed); // speed 0-100%
void FAN_Init(void);

#endif