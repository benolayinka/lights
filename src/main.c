/*
 * Copyright (c) 2016 Open-RnD Sp. z o.o.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/******************************************************************************
//std
******************************************************************************/
#include <stdbool.h>
#include <inttypes.h>

#include <zephyr.h>
#include <device.h>
#include <drivers/gpio.h>
#include <sys/printk.h>
#include <drivers/pinmux.h>
#include <fsl_port.h>
#include "expander.h"
#include "expanderpwm.h"

void fade()
{
	for(uint32_t pwm = 0; pwm<=0xff; pwm++)
	{
		for(uint32_t cnt = 0; cnt<0xff; cnt++)
		{
			if(cnt == 0)
				ExpanderWriteGPIOAB(EXPANDER_A, 0x0);
			if(pwm == cnt)
				ExpanderWriteGPIOAB(EXPANDER_A, 0xffff);
			k_busy_wait(10);
		}
		k_busy_wait(500);
	}

	//k_busy_wait(1000000);

	for(uint32_t pwm = 0; pwm<=0xff; pwm++)
	{
		for(uint32_t cnt = 0; cnt<0xff; cnt++)
		{
			if(cnt == 0)
				ExpanderWriteGPIOAB(EXPANDER_A, 0xffff);
			if(pwm == cnt)
				ExpanderWriteGPIOAB(EXPANDER_A, 0x0);
			k_busy_wait(10);
		}
		k_busy_wait(500);
	}

	k_busy_wait(1000000);
}

void main(void)
{

	struct device *porta =
		device_get_binding("porta");

	struct device *portb =
		device_get_binding("portb");

	struct device *portc =
		device_get_binding("portc");

	pinmux_pin_set(portb, 23, PORT_PCR_MUX(kPORT_MuxAsGpio));
	pinmux_pin_set(porta, 2, PORT_PCR_MUX(kPORT_MuxAsGpio));
	
	//reset expander
	pinmux_pin_set(portc, 16, PORT_PCR_MUX(kPORT_MuxAsGpio));
	struct device *gpioc;
	gpioc = device_get_binding("GPIO_2");
	gpio_pin_configure(gpioc, 16, GPIO_DIR_OUT);
	//active low
	gpio_pin_write(gpioc, 16, 0);
	gpio_pin_write(gpioc, 16, 1);

	expander_pwm_t pwm = {EXPANDER_A};
	ExpanderPWMInit(&pwm);
	ExpanderPWMSet(&pwm, 8, 0);
	ExpanderPWMSetFadeRate(&pwm, 8, 10, 10);

	// ExpanderInit(EXPANDER_A);
	// ExpanderPinMode(EXPANDER_A, 8, OUTPUT);
	// ExpanderPinMode(EXPANDER_A, 9, OUTPUT);

	while (1) 
	{
		static bool on = false;
		static uint16_t counter = 0;
		counter++;

		if(0==counter)
		{
			printk("loop");
			if(on)
				ExpanderPWMSet(&pwm, 8, 0);
			else
				ExpanderPWMSet(&pwm, 8, 0xff);

			on = !on;
		}
		
		ExpanderPWMUpdate(&pwm);
	}
}