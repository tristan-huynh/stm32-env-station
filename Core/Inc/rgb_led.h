#ifndef RGB_LED_H
#define RGB_LED_H

#include "main.h"

#define RGB_RED_PORT    GPIOC
#define RGB_RED_PIN     GPIO_PIN_7   // D10

#define RGB_BLUE_PORT   GPIOA
#define RGB_BLUE_PIN    GPIO_PIN_9   // D9

#define RGB_GREEN_PORT  GPIOB
#define RGB_GREEN_PIN   GPIO_PIN_6   // D8

// Anode (1) or Cathode (0)
#define RGB_COMMON_ANODE 0

typedef struct {
    uint8_t red;
    uint8_t green;
    uint8_t blue;
} RGB_Color;

// Predefined colors
#define RGB_COLOR_OFF       {0, 0, 0}
#define RGB_COLOR_RED       {255, 0, 0}
#define RGB_COLOR_GREEN     {0, 255, 0}
#define RGB_COLOR_BLUE      {0, 0, 255}
#define RGB_COLOR_YELLOW    {255, 255, 0}

void RGB_Init(void);
void RGB_SetColor(RGB_Color color);
void RGB_SetRGB(uint8_t red, uint8_t green, uint8_t blue);
void RGB_Off(void);

#endif 