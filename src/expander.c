/******************************************************************************
        Teenage Engineering oplab_baremetal_usb_lite
        File: expander.c
        Author: Ben Olayinka
        Date: 12 Jul 2019
        Brief: MCP23017 driver
******************************************************************************/

/******************************************************************************
//std
******************************************************************************/
#include <stdbool.h>
#include <inttypes.h>

/******************************************************************************
//local
******************************************************************************/
#include "expander.h"
#include "i2c_adapter.h"
#include "defines.h"

/******************************************************************************
//defines
******************************************************************************/

// registers
#define EXPANDER_IODIRA 0x00
#define EXPANDER_IPOLA 0x02
#define EXPANDER_GPINTENA 0x04
#define EXPANDER_DEFVALA 0x06
#define EXPANDER_INTCONA 0x08
#define EXPANDER_IOCONA 0x0A
#define EXPANDER_GPPUA 0x0C
#define EXPANDER_INTFA 0x0E
#define EXPANDER_INTCAPA 0x10
#define EXPANDER_GPIOA 0x12
#define EXPANDER_OLATA 0x14

#define EXPANDER_IODIRB 0x01
#define EXPANDER_IPOLB 0x03
#define EXPANDER_GPINTENB 0x05
#define EXPANDER_DEFVALB 0x07
#define EXPANDER_INTCONB 0x09
#define EXPANDER_IOCONB 0x0B
#define EXPANDER_GPPUB 0x0D
#define EXPANDER_INTFB 0x0F
#define EXPANDER_INTCAPB 0x11
#define EXPANDER_GPIOB 0x13
#define EXPANDER_OLATB 0x15

#define EXPANDER_INT_ERR 255

/******************************************************************************
//prototypes
******************************************************************************/

typedef struct _expander_t
{
	const expander_id_t expanderId;
	const uint8_t i2cAddress;
	bool isInitialized;
} expander_t;

static expander_t expanders[NUM_EXPANDERS] =
{
		//id, i2caddress, isInit
		{EXPANDER_A, 0b0100000U, false},
		{EXPANDER_B, 0b0100001U, false},
};

/******************************************************************************
//functions
******************************************************************************/

static inline void writeRegister(expander_id_t expanderId, uint8_t reg, uint8_t data)
{
	I2CWriteRegister(expanders[expanderId].i2cAddress, reg, data);
}

static inline uint8_t readRegister(expander_id_t expanderId, uint8_t reg)
{
	uint8_t temp;
	I2CReadRegister(expanders[expanderId].i2cAddress, reg, &temp);
	return temp;
}

static uint8_t getBitFromPin(uint8_t pin)
{
	return pin % 8;
}

uint8_t getRegFromPin(uint8_t pin, uint8_t portA, uint8_t portB)
{
	if(pin < 8)
	{
		return portA;
	}
	else
	{
		return portB;
	}
}

static void updateRegisterBit(expander_id_t expanderId, uint8_t pin, uint8_t val, uint8_t portA, uint8_t portB)
{
	uint8_t reg_val;
	uint8_t reg_addr = getRegFromPin(pin, portA, portB);
	uint8_t bit = getBitFromPin(pin);
	reg_val = readRegister(expanderId, reg_addr);

	bitWrite(reg_val, bit, val);

	writeRegister( expanderId, reg_addr, reg_val);
}

void ExpanderInit(expander_id_t expanderId)
{
	I2CInit();
	writeRegister( expanderId, EXPANDER_IODIRA, 0xff);
	writeRegister( expanderId, EXPANDER_IODIRB, 0xff);
}

void ExpanderPinMode(expander_id_t expanderId, uint8_t pin, uint8_t direction)
{
	updateRegisterBit(expanderId, pin, (direction==INPUT), EXPANDER_IODIRA, EXPANDER_IODIRB);
}

void ExpanderPinInvertInput(expander_id_t expanderId, uint8_t pin, uint8_t invert)
{
	updateRegisterBit(expanderId, pin, invert, EXPANDER_IPOLA, EXPANDER_IPOLB);
}

uint16_t ExpanderReadGPIOAB(expander_id_t expanderId)
{
	uint16_t ba = 0;
	uint8_t a;

	a = readRegister(expanderId, EXPANDER_GPIOA);
	ba = readRegister(expanderId, EXPANDER_GPIOB);

	ba <<= 8;
	ba |= a;
	return ba;
}

uint8_t ExpanderReadGPIOA(expander_id_t expanderId)
{
	uint8_t a;
	a = readRegister(expanderId, EXPANDER_GPIOA);
	return a;
}

uint8_t ExpanderReadGPIOB(expander_id_t expanderId)
{
	uint8_t b;
	b = readRegister(expanderId, EXPANDER_GPIOA);
	return b;
}

void ExpanderWriteGPIOAB(expander_id_t expanderId, uint16_t ba)
{
	writeRegister( expanderId, EXPANDER_GPIOA, (uint8_t)(ba & 0xff));
	writeRegister( expanderId, EXPANDER_GPIOB, (uint8_t)(ba >> 8 & 0xff));
}

void ExpanderWritePin(expander_id_t expanderId, uint8_t pin, uint8_t state)
{
	uint8_t gpio;
	uint8_t bit = getBitFromPin(pin);

	uint8_t reg_addr = getRegFromPin(pin, EXPANDER_OLATA, EXPANDER_OLATB);
	gpio = readRegister(expanderId, reg_addr);

	bitWrite(gpio, bit, state);

	reg_addr = getRegFromPin(pin, EXPANDER_GPIOA, EXPANDER_GPIOB);
	writeRegister( expanderId, reg_addr, gpio);
}

uint8_t ExpanderReadPin(expander_id_t expanderId, uint8_t pin)
{
	uint8_t bit = getBitFromPin(pin);
	uint8_t reg_addr = getRegFromPin(pin, EXPANDER_GPIOA, EXPANDER_GPIOB);
	return (readRegister(expanderId, reg_addr) >> bit) & 0x1;
}

void ExpanderSetPullup(expander_id_t expanderId, uint8_t pin, uint8_t state)
{
	updateRegisterBit(expanderId, pin, state, EXPANDER_GPPUA, EXPANDER_GPPUB);
}

//set mirror to OR interrupts, polarity sets value of interrupt hi or lo
void ExpanderSetupInterrupts(expander_id_t expanderId, uint8_t mirroring, uint8_t openDrain, uint8_t polarity)
{
	//port a
	uint8_t conf = readRegister(expanderId, EXPANDER_IOCONA);
	bitWrite(conf, 6, mirroring);
	bitWrite(conf, 2, openDrain);
	bitWrite(conf, 1, polarity);
	writeRegister(expanderId, EXPANDER_IOCONA, conf);

	//port b
	conf = readRegister(expanderId, EXPANDER_IOCONB);
	bitWrite(conf, 6, mirroring);
	bitWrite(conf, 2, openDrain);
	bitWrite(conf, 1, polarity);
	writeRegister(expanderId, EXPANDER_IOCONB, conf);
}

void ExpanderSetupInterruptPin(expander_id_t expanderId, uint8_t pin, uint8_t mode)
{
	//interrupt compare or change
	updateRegisterBit(expanderId, pin, (mode!=INTERRUPT_CHANGE), EXPANDER_INTCONA, EXPANDER_INTCONB);
	//interrupt rising or falling
	updateRegisterBit(expanderId, pin, (mode==INTERRUPT_FALLING), EXPANDER_DEFVALA, EXPANDER_DEFVALA);
	//enable interrupts
	updateRegisterBit(expanderId, pin, 1, EXPANDER_GPINTENA, EXPANDER_GPINTENB);
}

uint8_t ExpanderGetLastInterruptPin(expander_id_t expanderId)
{
	uint8_t intf;

	intf = readRegister(expanderId, EXPANDER_INTFA);
	for(uint8_t bit = 0; bit < 8; bit ++)
	{
		if (bitRead(intf, bit))
		{
			return bit;
		}
	}

	intf = readRegister(expanderId, EXPANDER_INTFB);
	for(uint8_t bit = 0; bit < 8; bit ++)
	{
		if (bitRead(intf, bit))
		{
			return bit;
		}
	}
	return EXPANDER_INT_ERR;
}

uint8_t ExpanderGetLastInterruptPinValue(expander_id_t expanderId)
{
	uint8_t pin = ExpanderGetLastInterruptPin(expanderId);
	if(EXPANDER_INT_ERR != pin)
	{
		uint8_t cap_reg = getRegFromPin(pin, EXPANDER_INTCAPA, EXPANDER_INTCAPB);
		uint8_t bit = getBitFromPin(pin);
		uint8_t reg_val;
		reg_val = readRegister(expanderId, cap_reg);
		return((reg_val >> bit) & (0x01));
	}
	return EXPANDER_INT_ERR;
}

/* end of file */
