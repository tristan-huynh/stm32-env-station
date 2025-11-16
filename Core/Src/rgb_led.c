#include "rgb_led.h"

static void RGB_PWM(GPIO_TypeDef* port, uint16_t pin, uint8_t duty) {
    if (duty == 0) {
        #if RGB_COMMON_ANODE
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        #else
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        #endif
        return;
    }

    if (duty == 255) {
        #if RGB_COMMON_ANODE
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
        #else
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
        #endif
        return;
    }
    

    #if RGB_COMMON_ANODE
    if (duty > 128) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    }
    #else
    if (duty > 128) {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
    }
    #endif
}

void RGB_Init(void) {
    // Pins are already initialized in MX_GPIO_Init
    RGB_Off();
}

void RGB_SetColor(RGB_Color color) {
    RGB_SetRGB(color.red, color.green, color.blue);
}

void RGB_SetRGB(uint8_t red, uint8_t green, uint8_t blue) {
    RGB_PWM(RGB_RED_PORT, RGB_RED_PIN, red);
    RGB_PWM(RGB_GREEN_PORT, RGB_GREEN_PIN, green);
    RGB_PWM(RGB_BLUE_PORT, RGB_BLUE_PIN, blue);
}

void RGB_Off(void) {
    RGB_SetRGB(0, 0, 0);
}