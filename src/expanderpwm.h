#ifndef EXPANDER_PWM_H_
#define EXPANDER_PWM_H_

#define EXPANDER_PWM_CHANNELS_MAX 16
#define POLARITY_NON_INVERTED 0
#define POLARITY_INVERTED 1
#define POLARITY_DEFAULT POLARITY_INVERTED

/******************************************************************************
//std
******************************************************************************/
#include <stdbool.h>
#include <inttypes.h>

#include "expander.h"

typedef struct
{
  // hardware I/O port and pin for this channel
	bool isPWM; 
  int8_t pin;
  uint8_t polarity;
  uint8_t pwmGoal;
  uint8_t pwmCurrent;
  uint8_t fadeUpRate;
  uint8_t fadeDownRate;
} expander_pwm_channel;

typedef struct
{
	expander_id_t expanderId;
	uint16_t ledsGPIOState;
	bool ledsDirty;
	expander_pwm_channel pwmChannels[EXPANDER_PWM_CHANNELS_MAX];
} expander_pwm_t;

void ExpanderPWMUpdate(expander_pwm_t *expander);

void ExpanderPWMInit(expander_pwm_t *expander);

void ExpanderPWMSet(expander_pwm_t *expander, uint16_t pin, uint8_t val);

void ExpanderLedOnInternal(expander_pwm_t *expander, uint16_t pin);

void ExpanderLedOffInternal(expander_pwm_t *expander, uint16_t pin);


#endif