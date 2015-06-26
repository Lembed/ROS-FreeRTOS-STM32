#include "i2c.h"
#include "stdlib.h"
#include <stm32f4xx.h>
#include <stm32f4xx_i2c.h>

/* This function issues a start condition and
 * transmits the slave address + R/W bit
 *
 * Parameters:
 * 		I2Cx --> the I2C peripheral e.g. I2C1
 * 		address --> the 7 bit slave address
 * 		direction --> the tranmission direction can be:
 * 						I2C_Direction_Tranmitter for Master transmitter mode
 * 						I2C_Direction_Receiver for Master receiver
 */



#define I2C1_TIMEOUT 0x3000


void I2C_start(I2C_TypeDef* I2Cx, uint8_t address, uint8_t direction){

	uint32_t timeout = I2C1_TIMEOUT;
	// wait until I2C1 is not busy anymore
	while(I2C_GetFlagStatus(I2Cx, I2C_FLAG_BUSY)) {
	    if(timeout!=0) timeout--; else return;
	}

	// Send I2C1 START condition
	I2C_GenerateSTART(I2Cx, ENABLE);

	timeout = I2C1_TIMEOUT;
	// wait for I2C1 EV5 --> Slave has acknowledged start condition
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_MODE_SELECT)) {
	    if(timeout!=0) timeout--; else return;
	}

	// Send slave Address for write
	I2C_Send7bitAddress(I2Cx, address, direction);

	/* wait for I2C1 EV6, check if
	 * either Slave has acknowledged Master transmitter or
	 * Master receiver mode, depending on the transmission
	 * direction
	 */
	if(direction == I2C_Direction_Transmitter){
		timeout = I2C1_TIMEOUT;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
		    if(timeout!=0) timeout--; else return;
		}
	}
	else if(direction == I2C_Direction_Receiver){
		timeout = I2C1_TIMEOUT;
		while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
		    if(timeout!=0) timeout--; else return;
		}
	}
}

/* This function transmits one byte to the slave device
 * Parameters:
 *		I2Cx --> the I2C peripheral e.g. I2C1
 *		data --> the data byte to be transmitted
 */
void I2C_write(I2C_TypeDef* I2Cx, uint8_t data)
{
	I2C_SendData(I2Cx, data);

	uint32_t timeout = I2C1_TIMEOUT;
	// wait for I2C1 EV8_2 --> byte has been transmitted
	while(!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_TRANSMITTED)) {
	    if(timeout!=0) timeout--; else return;
	}
}

/* This function reads one byte from the slave device
 * and acknowledges the byte (requests another byte)
 */
uint8_t I2C_read_ack(I2C_TypeDef* I2Cx){
	// enable acknowledge of recieved data
	I2C_AcknowledgeConfig(I2Cx, ENABLE);

	uint32_t timeout = I2C1_TIMEOUT;
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ) {
	    if(timeout!=0) timeout--; else return 0;
	}
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This function reads one byte from the slave device
 * and doesn't acknowledge the recieved data
 */
uint8_t I2C_read_nack(I2C_TypeDef* I2Cx){
	// disabe acknowledge of received data
	// nack also generates stop condition after last byte received
	// see reference manual for more info
	I2C_AcknowledgeConfig(I2Cx, DISABLE);
	I2C_GenerateSTOP(I2Cx, ENABLE);

	uint32_t timeout = I2C1_TIMEOUT;
	// wait until one byte has been received
	while( !I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED) ) {
	    if(timeout!=0) timeout--; else return 0;
	}
	// read data from I2C data register and return data byte
	uint8_t data = I2C_ReceiveData(I2Cx);
	return data;
}

/* This funtion issues a stop condition and therefore
 * releases the bus
 */
void I2C_stop(I2C_TypeDef* I2Cx){
	// Send I2C1 STOP Condition
	I2C_GenerateSTOP(I2Cx, ENABLE);
}

void I2C::init(uint8_t SCL_pin, uint8_t SDA_pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	I2C_InitTypeDef I2C_InitStruct;

	// enable APB1 peripheral clock for I2C1
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
	// enable clock for SCL and SDA pins
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);

	/* setup SCL and SDA pins
	 * You can connect I2C1 to two different
	 * pairs of pins:
	 * 1. SCL on PB6 and SDA on PB7
	 * 2. SCL on PB8 and SDA on PB9
	 */
	GPIO_InitStruct.GPIO_Pin = (0x1 << (SCL_pin & 0xF)) | (0x1 << (SDA_pin & 0xF)); // we are going to use PB6 and PB7
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;			// set pins to alternate function
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;		// set GPIO speed
	GPIO_InitStruct.GPIO_OType = GPIO_OType_OD;			// set output to open drain --> the line has to be only pulled low, not driven high
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;			// enable pull up resistors
	GPIO_Init(GPIOB, &GPIO_InitStruct);					// init GPIOB
	// Connect I2C1 pins to AF
	GPIO_PinAFConfig((GPIO_TypeDef*)(AHB1PERIPH_BASE + (0x0400 * ((SCL_pin >> 4) & 0xF))), SCL_pin & 0xF, GPIO_AF_I2C1);	// SCL
	GPIO_PinAFConfig((GPIO_TypeDef*)(AHB1PERIPH_BASE + (0x0400 * ((SDA_pin >> 4) & 0xF))), SDA_pin & 0xF, GPIO_AF_I2C1); // SDA

	// configure I2C1
	I2C_InitStruct.I2C_ClockSpeed = 100000; 		// 100kHz
	I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;			// I2C mode
	I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;	// 50% duty cycle --> standard
	I2C_InitStruct.I2C_OwnAddress1 = 0x00;			// own address, not relevant in master mode
	I2C_InitStruct.I2C_Ack = I2C_Ack_Disable;		// disable acknowledge when reading (can be changed later on)
	I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit; // set address length to 7 bit addresses
	I2C_Init(I2C1, &I2C_InitStruct);				// init I2C1

	// enable I2C1
	I2C_Cmd(I2C1, ENABLE);
}

void I2C::writeByteToRegister(uint8_t slaveAddress, uint8_t reg, uint8_t byte)
{
	I2C_start(I2C1, slaveAddress<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1,  reg); // write one byte to the slave
	I2C_write(I2C1, byte); // write another byte to the slave
	I2C_stop(I2C1);
}

uint8_t I2C::readByteFromRegister(uint8_t slaveAddress, uint8_t reg)
{
	I2C_start(I2C1, slaveAddress<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1,  reg); // write one byte to the slave
	I2C_stop(I2C1); // stop the transmission

	I2C_start(I2C1, slaveAddress<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	return I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
}

uint16_t I2C::readWordFromRegisters(uint8_t slaveAddress, uint8_t reg1, uint8_t reg2)
{
	uint8_t received_data[2];
	I2C_start(I2C1, slaveAddress<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1,  reg1); // write one byte to the slave
	I2C_write(I2C1, reg2); // write another byte to the slave
	I2C_stop(I2C1); // stop the transmission

	I2C_start(I2C1, slaveAddress<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	received_data[0] = I2C_read_ack(I2C1); // read one byte and request another byte
	received_data[1] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
	uint16_t val = (received_data[0] & 0xff) | ((received_data[1] & 0xff) << 8);
	return val; // read one byte and don't request another byte, stop transmission
}
extern "C" void os_printf(const char* fmt, ...);
void I2C::readBytes(uint8_t slaveAddress, uint8_t startReg, uint16_t count, uint8_t* bytes)
{
	if (count < 1)
	{
		os_printf("I2C::readBytes error: count must be at least 1!\n");
		return;

	}
	if (bytes == NULL)
	{
		os_printf("I2C::readBytes error: bytes must be non-NULL!\n");
		return;

	}
	I2C_start(I2C1, slaveAddress<<1, I2C_Direction_Transmitter); // start a transmission in Master transmitter mode
	I2C_write(I2C1, startReg); // write one byte to the slave
	//I2C_write(I2C1, count); // write another byte to the slave
	//for (uint16_t i=0; i<count; i++)
		//I2C_write(I2C1,  startReg+i); // write one byte to the slave
	I2C_stop(I2C1); // stop the transmission

	I2C_start(I2C1, slaveAddress<<1, I2C_Direction_Receiver); // start a transmission in Master receiver mode
	uint16_t i=0;
	for (; i<count-1; i++)
		bytes[i] = I2C_read_ack(I2C1); // read one byte and request another byte
	bytes[i] = I2C_read_nack(I2C1); // read one byte and don't request another byte, stop transmission
}
