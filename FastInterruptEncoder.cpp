#include "Arduino.h"
#include "FastInterruptEncoder.h"
#include "main.h"

Encoder::Encoder(int pinA, int pinB, TIM_TypeDef *timer, encoder_mode_t mode, uint8_t filter)
{
	_pinA = pinA;
	_pinB = pinB;
	_mode = mode;
	_filter = filter;
	_timer = timer;
}

bool Encoder::init()
{
	// Initialisation des pins
	pinMode(_pinA, INPUT_PULLUP);
	pinMode(_pinB, INPUT_PULLUP);

	// Affiliation des pins au timer
	pin_function(digitalPinToPinName(_pinA), pinmap_function(digitalPinToPinName(_pinA), PinMap_TIM));
	pin_function(digitalPinToPinName(_pinB), pinmap_function(digitalPinToPinName(_pinB), PinMap_TIM));

	// Vidage de la structure de configuration
	TIM_Encoder_InitTypeDef sEncoderConfig = {0};

	Encoder_Handle.Init.Period = 65534; // valeur max pour un timer 16 bits

	/*********Configuration des registres du timer*********/
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
	/*******************************************************/

	enableTimerClock(&Encoder_Handle); // Activation de l'horloge du timer
	if (HAL_TIM_Encoder_Init(&Encoder_Handle, &sEncoderConfig) != HAL_OK)
		return 0;
	LL_TIM_SetCounter(Encoder_Handle.Instance, 32767);		 // set la valeur registre
	HAL_TIM_Encoder_Start(&Encoder_Handle, TIM_CHANNEL_ALL); // Activation du timer
	return 1;
}

void Encoder::setInvert(bool invert)
{
	_invert = invert;
}

int16_t Encoder::getTicks()
{
	// récupération de la valeur registre du timer
	uint16_t codeur_value = LL_TIM_GetCounter(_timer);

	// convertion en entier signer sur 16 bits
	// Attention possible améliration en supprimant cette conversion
	if (_invert)
		return -static_cast<int16_t>(codeur_value - 32767);
	return static_cast<int16_t>(codeur_value - 32767);
}

void Encoder::resetTicks()
{
	LL_TIM_SetCounter(_timer, 32767);
}
