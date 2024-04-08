#include "BoucleOuverte.h"

void Config_PID_Vitesse(void)
{
  unsigned long timeNow = millis();
  digitalWrite(2, 1);
  digitalWrite(3, 1);
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
  if (timeNow - timeSetup >= 4000)
  {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
  }
}

void Etalonnage_tic_roue_ligne()
{
}