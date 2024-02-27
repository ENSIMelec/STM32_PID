#include "BoucleOuverte.h"

void Config_PID_Vitesse(void)
{
    unsigned long timeNow = millis();
    if (timeNow - timeSetup < 1000)
    {
        analogWrite(PWM1, 10);
        analogWrite(PWM2, 10);
    }
    if (timeNow - timeSetup < 4000 && timeNow - timeSetup >= 1000)
    {
        analogWrite(PWM1, 100);
        analogWrite(PWM2, 100);
    }
}