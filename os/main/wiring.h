typedef enum mode { OUTPUT, INPUT, TRIGGER_RISING, TRIGGER_FALLING, TRIGGER_RISING_FALLING} Mode;
#define HIGH true
#define LOW false

#include "stdint.h"

#define GPIO_PA0 0x0
#define GPIO_PA1 0x1
#define GPIO_PA2 0x2
#define GPIO_PA3 0x3
#define GPIO_PA4 0x4
#define GPIO_PA5 0x5
#define GPIO_PA6 0x6
#define GPIO_PA7 0x7
#define GPIO_PA8 0x8
#define GPIO_PA9 0x9
#define GPIO_PA10 0xA
#define GPIO_PA11 0xB
#define GPIO_PA12 0xC
#define GPIO_PA13 0xD
#define GPIO_PA14 0xE
#define GPIO_PA15 0xF

#define GPIO_PB0 0x10
#define GPIO_PB1 0x11
#define GPIO_PB2 0x12
#define GPIO_PB3 0x13
#define GPIO_PB4 0x14
#define GPIO_PB5 0x15
#define GPIO_PB6 0x16
#define GPIO_PB7 0x17
#define GPIO_PB8 0x18
#define GPIO_PB9 0x19
#define GPIO_PB10 0x1A
#define GPIO_PB11 0x1B
#define GPIO_PB12 0x1C
#define GPIO_PB13 0x1D
#define GPIO_PB14 0x1E
#define GPIO_PB15 0x1F

#define GPIO_PC0 0x20
#define GPIO_PC1 0x21
#define GPIO_PC2 0x22
#define GPIO_PC3 0x23
#define GPIO_PC4 0x24
#define GPIO_PC5 0x25
#define GPIO_PC6 0x26
#define GPIO_PC7 0x27
#define GPIO_PC8 0x28
#define GPIO_PC9 0x29
#define GPIO_PC10 0x2A
#define GPIO_PC11 0x2B
#define GPIO_PC12 0x2C
#define GPIO_PC13 0x2D
#define GPIO_PC14 0x2E
#define GPIO_PC15 0x2F

#define GPIO_PD0 0x30
#define GPIO_PD1 0x31
#define GPIO_PD2 0x32
#define GPIO_PD3 0x33
#define GPIO_PD4 0x34
#define GPIO_PD5 0x35
#define GPIO_PD6 0x36
#define GPIO_PD7 0x37
#define GPIO_PD8 0x38
#define GPIO_PD9 0x39
#define GPIO_PD10 0x3A
#define GPIO_PD11 0x3B
#define GPIO_PD12 0x3C
#define GPIO_PD13 0x3D
#define GPIO_PD14 0x3E
#define GPIO_PD15 0x3F


void digitalWrite(uint16_t pin, bool value);
bool digitalRead(uint16_t pin);
void pinMode(uint16_t pin, Mode mode);
void delayMicroseconds(uint32_t us);
void enableTiming();
uint32_t micros();
