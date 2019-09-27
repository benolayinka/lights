/******************************************************************************
//std
******************************************************************************/
#include <stdbool.h>
#include <inttypes.h>
#include "defines.h"

#include "expanderpwm.h"
#include "expander.h"

uint8_t soft_count = 0xff;

void ExpanderPWMUpdate(expander_pwm_t *expander)
{
	soft_count++;

	if(soft_count == 0)
	{
		for(uint8_t cnl = 0; cnl < EXPANDER_PWM_CHANNELS_MAX; cnl++)
		{
			expander_pwm_channel *channel = &expander->pwmChannels[cnl];

			if(channel->isPWM)
			{
				//if fade value set
				if(channel->fadeUpRate || channel->fadeDownRate)
				{
					int16_t direction = channel->pwmGoal - channel->pwmCurrent;
					int16_t goal = channel->pwmGoal;

					if(direction > 0 && channel->fadeUpRate > 0)
					{
						goal = channel->pwmCurrent + channel->fadeUpRate;
						if(goal > channel->pwmGoal)
						  goal = channel->pwmGoal;
					}
					else if(direction < 0 && channel->fadeDownRate > 0)
					{
						goal = channel->pwmCurrent - channel->fadeDownRate;
						if(goal < channel->pwmGoal)
						  goal = channel->pwmGoal;
					}
					channel->pwmCurrent = goal; 
				}
				//no fade value
				else
				{
					channel->pwmCurrent = channel->pwmGoal;
				}

				if(channel->pwmCurrent)
				{
					ExpanderLedOnInternal(expander, cnl);
				}
			}
		}
	}

	for(uint8_t cnl = 0; cnl < EXPANDER_PWM_CHANNELS_MAX; cnl++)
	{
		expander_pwm_channel *channel = &expander->pwmChannels[cnl];

		if(channel->isPWM)
		{
			expander_pwm_channel *channel = &expander->pwmChannels[cnl];
			if(channel->pwmCurrent != 0xff && channel->pwmCurrent == soft_count)
			{
				ExpanderLedOffInternal(expander, cnl);
			}
		}
	}

	if(expander->ledsDirty)
	{
		ExpanderWriteGPIOAB(expander->expanderId, expander->ledsGPIOState);
	}
}

void ExpanderPWMInit(expander_pwm_t *expander)
{
	ExpanderInit(expander->expanderId);

	for(uint8_t cnl = 0; cnl < EXPANDER_PWM_CHANNELS_MAX; cnl++)
	{
		expander_pwm_channel *channel = &expander->pwmChannels[cnl];
		channel->isPWM = false;
		channel->pin = cnl;
		channel->polarity = POLARITY_DEFAULT;
		channel->pwmGoal = 0;
		channel->pwmCurrent = 0;
		channel->fadeDownRate = 0;
		channel->fadeUpRate = 0;
		ExpanderLedOffInternal(expander, cnl);
	}
}

static void ExpanderPWMLedInit(expander_pwm_t *expander, uint16_t pin)
{
	ExpanderPinMode(expander->expanderId, pin, OUTPUT);
}

void ExpanderPWMSet(expander_pwm_t *expander, uint16_t pin, uint8_t val)
{
	expander_pwm_channel *channel = &expander->pwmChannels[pin];

	channel->isPWM = true;
	channel->pin = pin;
	channel->polarity = POLARITY_DEFAULT;
	channel->pwmGoal = val;
	ExpanderPWMLedInit(expander, pin);
	ExpanderLedOffInternal(expander, pin);
}

void ExpanderPWMSetFadeRate(expander_pwm_t *expander, uint16_t pin, uint8_t fadeUpRate, uint8_t fadeDownRate)
{
	expander_pwm_channel *channel = &expander->pwmChannels[pin];

	channel->isPWM = true;

	if(fadeUpRate > 0)
	{
		channel->fadeUpRate = fadeUpRate;
	}

	if(fadeDownRate > 0)
	{
		channel->fadeDownRate = fadeDownRate;
	}
}

void ExpanderLedOnInternal(expander_pwm_t *expander, uint16_t pin)
{
	uint8_t on_pol = !expander->pwmChannels[pin].polarity;
	bitWrite(expander->ledsGPIOState, pin, on_pol);
	expander->ledsDirty = true;
}

void ExpanderLedOffInternal(expander_pwm_t *expander, uint16_t pin)
{
	uint8_t off_pol = expander->pwmChannels[pin].polarity;
	bitWrite(expander->ledsGPIOState, pin, off_pol);
	expander->ledsDirty = true;
}