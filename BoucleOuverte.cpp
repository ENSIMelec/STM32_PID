#include "BoucleOuverte.h"

/* La fonction est à mettre dans la void loop. Elle permet de mettre une vitesse
   sur les moteurs pour vérifier leur vitesse. La vitesse est au début faible afin
   de supprimer toutes les incertitudes du à la mise en mouvement des moteurs.
   En regardant les courbes pour chaque moteur, on peut déterminer l'odre de grandeur
   des valeur de Kp pour chaque moteur.
 */
void Config_PID_Vitesse(void)
{
  unsigned long timeNow = millis();
  digitalWrite(DIR1, 1); // moteur en avnant
  digitalWrite(DIR2, 1);
  if (timeNow - timeSetup < 1000)
  {
    analogWrite(PWM1, 20); // faible vitesse pdt 1s
    analogWrite(PWM2, 20);
  }
  if (timeNow - timeSetup < 4000 && timeNow - timeSetup >= 1000)
  {
    analogWrite(PWM1, 100); // vitesse normale pdt 3s
    analogWrite(PWM2, 100);
  }
  if (timeNow - timeSetup >= 4000)
  {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
  }
}
