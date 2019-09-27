/******************************************************************************
        Teenage Engineering oplab_baremetal_usb_lite
        File: expander.h
        Author: Ben Olayinka
        Date: 12 Jul 2019
        Brief: MCP23017 driver
******************************************************************************/
#ifndef EXPANDER_EXPANDER_H_
#define EXPANDER_EXPANDER_H_

/******************************************************************************
//std
******************************************************************************/
#include <stdbool.h>
#include <inttypes.h>

#define LOW 0x0
#define HIGH 0x1

#define INPUT 0x0
#define OUTPUT 0x1

typedef enum {
	INTERRUPT_NONE,
	INTERRUPT_CHANGE = 0x1,
	INTERRUPT_FALLING = 0x2,
	INTERRUPT_RISING = 0x3
} interrupt_t;

typedef enum {
	EXPANDER_A,
	EXPANDER_B,
	NUM_EXPANDERS
} expander_id_t;

void ExpanderInit(expander_id_t expanderId);

void ExpanderPinMode(expander_id_t expanderId, uint8_t pin, uint8_t direction);

void ExpanderPinInvertInput(expander_id_t expanderId, uint8_t pin, uint8_t invert);

uint16_t ExpanderReadGPIOAB(expander_id_t expanderId);

uint8_t ExpanderReadGPIOA(expander_id_t expanderId);

uint8_t ExpanderReadGPIOB(expander_id_t expanderId);

void ExpanderWriteGPIOAB(expander_id_t expanderId, uint16_t ba);

void ExpanderWritePin(expander_id_t expanderId, uint8_t pin, uint8_t state);

uint8_t ExpanderReadPin(expander_id_t expanderId, uint8_t pin);

void ExpanderSetPullup(expander_id_t expanderId, uint8_t pin, uint8_t state);

//set mirror to OR interrupts, polarity sets value of interrupt hi or lo
void ExpanderSetupInterrupts(expander_id_t expanderId, uint8_t mirroring, uint8_t openDrain, uint8_t polarity);

void ExpanderSetupInterruptPin(expander_id_t expanderId, uint8_t pin, uint8_t mode);

uint8_t ExpanderGetLastInterruptPin(expander_id_t expanderId);

uint8_t ExpanderGetLastInterruptPinValue(expander_id_t expanderId);

#endif /* EXPANDER_EXPANDER_H_ */
/* end of file */
