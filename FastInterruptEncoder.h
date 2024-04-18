#ifndef __FASTINTERRUPTENCODER_H__
#define __FASTINTERRUPTENCODER_H__

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include "Arduino.h"

typedef enum
{
	SINGLE = 0,
	HALFQUAD = 1,
	FULLQUAD = 2,
} encoder_mode_t;

class Encoder
{
public:
	Encoder(int pinA, int pinB, TIM_TypeDef *timer, int16_t *last_tick, encoder_mode_t mode = SINGLE, uint8_t filter = 0);

	bool init();
	void loop();
	int16_t getTicks();
	void resetTicks();
	void setInvert(bool invert = true);
	void overflowCallback();

private:
	int _pinA;
	int _pinB;
	TIM_TypeDef *_timer;
	encoder_mode_t _mode = SINGLE;
	uint8_t _filter = 0;
	bool _invert = false;
	int16_t *_last_ticks;
	TIM_HandleTypeDef Encoder_Handle;
};

#endif // __FASTINTERRUPTENCODER_H__
