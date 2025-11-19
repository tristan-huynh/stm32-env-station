#ifndef FANCONTROL_H
#define FANCONTROL_H

#include "main.h"
#include <stdint.h>

// fan control definitions
#define FANCONTROL_PORT_IN1 GPIOA
#define FANCONTROL_PIN_IN1 GPIO_PIN_8
#define FANCONTROL_PORT_IN2 GPIOA
#define FANCONTROL_PIN_IN2 GPIO_PIN_9

void FAN_setSpeed(uint8_t speed); // speed 0-100%

#endif