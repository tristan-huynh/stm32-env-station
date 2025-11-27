#include "fan_control.h"

extern TIM_HandleTypeDef htim3;



void FAN_Init(void) {
    // HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    
    // HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    //     FAN_setSpeed(0);
    HAL_GPIO_WritePin(FANCONTROL_PORT_IN1, FANCONTROL_PIN_IN1, GPIO_PIN_RESET); //set D6 low
}

void FAN_setSpeed(uint8_t speed) {
    // if (speed == 0) {
    //     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    // } else if (speed <= 100) {
    //         // p = (speed * 65535) / 100
    //     uint32_t pulse = (speed * 65535) / 100;
    //     __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
    //     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
    // }

    // remove pwm
    if (speed == 0) {
        HAL_GPIO_WritePin(FANCONTROL_PORT_IN1, FANCONTROL_PIN_IN1, GPIO_PIN_RESET);
    } else if (speed == 1) {
        HAL_GPIO_WritePin(FANCONTROL_PORT_IN1, FANCONTROL_PIN_IN1, GPIO_PIN_SET);
    }
}