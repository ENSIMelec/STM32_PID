#include "main.h"
#include "FastInterruptEncoder.h"
#include "PID.h"
#include <digitalWriteFast.h>

extern Encoder encGauche;
extern Encoder encDroit;
extern PID PID_vitesse_G;
extern PID PID_vitesse_D;
extern PID PID_angle;
extern PID PID_distance;

extern "C"
{
    void TIM2_UP_IRQHandler(void);
    void TIM3_UP_IRQHandler(void);
}

void Update_IT_callback(void);