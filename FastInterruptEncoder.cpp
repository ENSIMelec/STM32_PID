#include "Arduino.h"
#include "FastInterruptEncoder.h"
#include "main.h"

Encoder::Encoder(int pinA, int pinB, TIM_TypeDef *timer, int16_t *last_ticks, encoder_mode_t mode, uint8_t filter)
{
	_pinA = pinA;
	_pinB = pinB;
	_mode = mode;
	_filter = filter;
	_timer = timer;
	_last_ticks = last_ticks;
}

bool Encoder::init()
{
	pinMode(_pinA, INPUT_PULLUP);
	pinMode(_pinB, INPUT_PULLUP);

	pin_function(digitalPinToPinName(_pinA), pinmap_function(digitalPinToPinName(_pinA), PinMap_TIM));
	pin_function(digitalPinToPinName(_pinB), pinmap_function(digitalPinToPinName(_pinB), PinMap_TIM));
	TIM_Encoder_InitTypeDef sEncoderConfig = {0};

	Encoder_Handle.Init.Period = 65534;
	if (_mode == SINGLE)
	{
		Encoder_Handle.Init.Prescaler = 1;
	}
	else
	{
		Encoder_Handle.Init.Prescaler = 0;
	}
	Encoder_Handle.Init.ClockDivision = 0;
	Encoder_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	Encoder_Handle.Init.RepetitionCounter = 0;
	Encoder_Handle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

	if (_mode == FULLQUAD)
	{
		sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI12;
	}
	else
	{
		sEncoderConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	}

	sEncoderConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sEncoderConfig.IC1Filter = _filter;

	sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_FALLING;
	sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sEncoderConfig.IC2Filter = _filter;

	Encoder_Handle.Instance = _timer;
	enableTimerClock(&Encoder_Handle);
	if (HAL_TIM_Encoder_Init(&Encoder_Handle, &sEncoderConfig) != HAL_OK)
		return 0;
	LL_TIM_SetCounter(Encoder_Handle.Instance, 32767); // set la valeur registre
	HAL_TIM_Encoder_Start(&Encoder_Handle, TIM_CHANNEL_ALL);
	return 1;
}

void Encoder::setInvert(bool invert)
{
	_invert = invert;
}

int16_t Encoder::getTicks()
{
	uint16_t codeur_value = LL_TIM_GetCounter(_timer);
	if (_invert)
		return -static_cast<int16_t>(codeur_value - 32767);
	return static_cast<int16_t>(codeur_value - 32767);
}

void Encoder::resetTicks()
{
	LL_TIM_SetCounter(_timer, 32767);
	*_last_ticks = 0;
}

void Encoder::overflowCallback()
{
	if (__HAL_TIM_GET_FLAG(&Encoder_Handle, TIM_FLAG_UPDATE) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&Encoder_Handle, TIM_IT_UPDATE) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&Encoder_Handle, TIM_IT_UPDATE);
			LL_TIM_SetCounter(_timer, 32767);
			if (_invert)
				*_last_ticks = *_last_ticks + 32767;
			else
				*_last_ticks = *_last_ticks - 32767;
		}
	}
}

/******* FONCTION DE BASE

void Encoder::loop()
{
	int c = LL_TIM_GetCounter((TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(_pinA), PinMap_TIM));
	int change = c - _prevTicks;
	_prevTicks = c;
	if (change > 40000)
	{
		change = 65535 - change;
	}
	else if (change < -40000)
	{
		change = -65535 - change;
	}
	if (_invert)
	{
		_ticks -= change;
	}
	else
	{
		_ticks += change;
	}
}





********/