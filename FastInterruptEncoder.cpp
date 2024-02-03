#include "Arduino.h"
#include "FastInterruptEncoder.h"

Encoder::Encoder(int pinA, int pinB, encoder_mode_t mode, uint8_t filter)
{
	_pinA = pinA;
	_pinB = pinB;
	_mode = mode;
	_filter = filter;
}

bool Encoder::init()
{
	pinMode(_pinA, INPUT_PULLUP);
	pinMode(_pinB, INPUT_PULLUP);

	pin_function(digitalPinToPinName(_pinA), pinmap_function(digitalPinToPinName(_pinA), PinMap_TIM));
	pin_function(digitalPinToPinName(_pinB), pinmap_function(digitalPinToPinName(_pinB), PinMap_TIM));

	TIM_HandleTypeDef Encoder_Handle;
	TIM_Encoder_InitTypeDef sEncoderConfig;

	Encoder_Handle.Init.Period = 65535;
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

	sEncoderConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sEncoderConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sEncoderConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sEncoderConfig.IC2Filter = _filter;

	Encoder_Handle.Instance = (TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(_pinA), PinMap_TIM);
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
	uint16_t codeur_value = LL_TIM_GetCounter((TIM_TypeDef *)pinmap_peripheral(digitalPinToPinName(_pinA), PinMap_TIM));
	if (invert)
		return -static_cast<int16_t>(codeur_value - 32767);
	return static_cast<int16_t>(codeur_value - 32767);
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

void Encoder::resetTicks()
{
	_ticks = 0;
}



********/