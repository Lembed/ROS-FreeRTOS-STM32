#include "stdint.h"

class I2C {
public:
	static void init(uint8_t SCL_pin, uint8_t SDA_pin);
	static void writeByteToRegister(uint8_t slaveAddress, uint8_t reg, uint8_t byte);
	static uint8_t readByteFromRegister(uint8_t slaveAddress, uint8_t reg);
	static uint16_t readWordFromRegisters(uint8_t slaveAddress, uint8_t reg1, uint8_t reg2);
	static void readBytes(uint8_t slaveAddress, uint8_t startReg, uint16_t count, uint8_t* bytes);
};
