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
	Encoder(int pinA, int pinB, TIM_TypeDef *timer, encoder_mode_t mode = SINGLE, uint8_t filter = 0);
	bool init();
	void loop();
	int16_t getTicks();
	void resetTicks();
	void setInvert(bool invert = true);

private:
	int _pinA;
	int _pinB;
	TIM_TypeDef *_timer;
	encoder_mode_t _mode = SINGLE;
	uint8_t _filter = 0;
	bool _invert = false;
};

#endif // __FASTINTERRUPTENCODER_H__
