#include "dht22.h"

// local handler
static TIM_HandleTypeDef *dht22_timer = NULL;

// initalize w/hardware timer
void DHT22_Init(TIM_HandleTypeDef *htim) {
    dht22_timer = htim;
    HAL_TIM_Base_Start(dht22_timer);
}

// use the pass hardware hardware timer to get delay in ms
static void delay_us(uint16_t us) {
    __HAL_TIM_SET_COUNTER(dht22_timer, 0);
    while (__HAL_TIM_GET_COUNTER(dht22_timer) < us);
}

// We need these functions because the pin switches between input and output
static void DHT22_SetPinOutput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);
}

static void DHT22_SetPinInput(void) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    GPIO_InitStruct.Pin = DHT22_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DHT22_PORT, &GPIO_InitStruct);
}

// Send start signal to DHT22
static void DHT22_Start(void) {
    DHT22_SetPinOutput();
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_SET);   // Start high
    delay_us(250);
    HAL_GPIO_WritePin(DHT22_PORT, DHT22_PIN, GPIO_PIN_RESET); // Pull low
    HAL_Delay(18); // Hold low for 18ms (spec says at least 1ms, more is safer)
    
    DHT22_SetPinInput(); // Release pin (pull-up will pull it high)
}
// Read one byte from DHT22
uint8_t DHT22_ReadByte(void) {
    uint8_t byte = 0;
    uint16_t timeout;
    
    for (int i = 0; i < 8; i++) {
        // Wait for the pin to go high (start of bit transmission, ~50us low)
        timeout = 0;
        while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET) {
            delay_us(1);
            if (++timeout > 100) return 0; // Timeout
        }
        
        // Wait 30us to check if bit is 0 or 1
        delay_us(30);
        
        // If pin is still high, it's a '1' (70us high), otherwise it's a '0' (26-28us high)
        if (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
            byte |= (1 << (7 - i)); // Set bit
        }
        
        // Wait for pin to go low (end of bit transmission)
        timeout = 0;
        while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
            delay_us(1);
            if (++timeout > 100) return 0; // Timeout
        }
    }
    
    return byte;
}

void DHT22_ReadData(float* temperature, float* humidity) {
    uint8_t data[5] = {0};
    uint8_t checksum = 0;

    // everything here is based off the chinese datasheet
    
    // Send start signal
    DHT22_Start();
    
    // wait for dht22 response (should pull low for 80us then high for 80us)
    uint16_t timeout = 0;
    
    // wait for DHT22 to pull low (20-40us)
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
        delay_us(1);
        if (++timeout > 100) {
            *temperature = -999.0f;
            *humidity = -999.0f;
            return; // timeout waiting for response
        }
    }

    // wait for DHT22 to pull high (80us)
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_RESET) {
        delay_us(1);
        if (++timeout > 100) {
            *temperature = -999.0f;
            *humidity = -999.0f;
            return; // timeout if the response response low
        }
    }

    // wait for DHT22 to pull low again (80us)
    timeout = 0;
    while (HAL_GPIO_ReadPin(DHT22_PORT, DHT22_PIN) == GPIO_PIN_SET) {
        delay_us(1);
        if (++timeout > 100) {
            *temperature = -999.0f;
            *humidity = -999.0f;
            return; // timeout if response is high
        }
    }
    
    // read 5 bytes of data
    for (int i = 0; i < 5; i++) {
        data[i] = DHT22_ReadByte();
    }
    
    // checksum
    checksum = data[0] + data[1] + data[2] + data[3];
    if (checksum != data[4]) {
        *temperature = -999.0f;
        *humidity = -999.0f;
        return;
    }
    
    // calculate humidity based of of the first 2 bytes
    *humidity = ((data[0] << 8) | data[1]) / 10.0f;
    
    // calculate temperature based off bytes 3 and 4
    uint16_t temp_raw = (data[2] << 8) | data[3];
    if (temp_raw & 0x8000) { // Negative temperature
        temp_raw &= 0x7FFF;
        *temperature = -(temp_raw / 10.0f);
    } else {
        *temperature = temp_raw / 10.0f;
    }
}
