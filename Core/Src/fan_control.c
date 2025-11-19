#include "fan_control.h"

void FAN_setSpeed(uint8_t speed) {
    if (speed == 0) {
        // Stop the fan
        HAL_GPIO_WritePin(FANCONTROL_PORT_IN1, FANCONTROL_PIN_IN1, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(FANCONTROL_PORT_IN2, FANCONTROL_PIN_IN2, GPIO_PIN_RESET);
    } else {
        // Set fan speed (simple on/off control for now)
        HAL_GPIO_WritePin(FANCONTROL_PORT_IN1, FANCONTROL_PIN_IN1, GPIO_PIN_SET);
        HAL_GPIO_WritePin(FANCONTROL_PORT_IN2, FANCONTROL_PIN_IN2, GPIO_PIN_RESET);
    }
}

