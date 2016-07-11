#ifdef __cplusplus
extern "C" {
#endif
#ifndef WIRING_MODE
#define WIRING_MODE
typedef enum mode { OUTPUT, INPUT, TRIGGER_RISING, TRIGGER_FALLING, TRIGGER_RISING_FALLING} Mode;
#endif

#define HIGH 1
#define LOW 0

#include "stdint.h"
#include "stm32f4xx.h"

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

#define GPIO_PE0 0x40
#define GPIO_PE1 0x41
#define GPIO_PE2 0x42
#define GPIO_PE3 0x43
#define GPIO_PE4 0x44
#define GPIO_PE5 0x45
#define GPIO_PE6 0x46
#define GPIO_PE7 0x47
#define GPIO_PE8 0x48
#define GPIO_PE9 0x49
#define GPIO_PE10 0x4A
#define GPIO_PE11 0x4B
#define GPIO_PE12 0x4C
#define GPIO_PE13 0x4D
#define GPIO_PE14 0x4E
#define GPIO_PE15 0x4F

#define GPIO_PF0 0x50
#define GPIO_PF1 0x51
#define GPIO_PF2 0x52
#define GPIO_PF3 0x53
#define GPIO_PF4 0x54
#define GPIO_PF5 0x55
#define GPIO_PF6 0x56
#define GPIO_PF7 0x57
#define GPIO_PF8 0x58
#define GPIO_PF9 0x59
#define GPIO_PF10 0x5A
#define GPIO_PF11 0x5B
#define GPIO_PF12 0x5C
#define GPIO_PF13 0x5D
#define GPIO_PF14 0x5E
#define GPIO_PF15 0x5F

#define GPIO_PG0 0x60
#define GPIO_PG1 0x61
#define GPIO_PG2 0x62
#define GPIO_PG3 0x63
#define GPIO_PG4 0x64
#define GPIO_PG5 0x65
#define GPIO_PG6 0x66
#define GPIO_PG7 0x67
#define GPIO_PG8 0x68
#define GPIO_PG9 0x69
#define GPIO_PG10 0x6A
#define GPIO_PG11 0x6B
#define GPIO_PG12 0x6C
#define GPIO_PG13 0x6D
#define GPIO_PG14 0x6E
#define GPIO_PG15 0x6F

#define GPIO_PH0 0x70
#define GPIO_PH1 0x71
#define GPIO_PH2 0x72
#define GPIO_PH3 0x73
#define GPIO_PH4 0x74
#define GPIO_PH5 0x75
#define GPIO_PH6 0x76
#define GPIO_PH7 0x77
#define GPIO_PH8 0x78
#define GPIO_PH9 0x79
#define GPIO_PH10 0x7A
#define GPIO_PH11 0x7B
#define GPIO_PH12 0x7C
#define GPIO_PH13 0x7D
#define GPIO_PH14 0x7E
#define GPIO_PH15 0x7F



void digitalWrite(uint16_t pin, int value);
int digitalRead(uint16_t pin);
void pinMode(uint16_t pin, Mode mode);
void delayMicroseconds(uint32_t us);
void enableTiming();
uint32_t micros();

#ifdef __cplusplus
}
#endif
