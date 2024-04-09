#include "interruption.h"
#include "Odometrie.h"
#include <Arduino.h>

float epsilonDistance = 0.5;
float epsilonAngle = PI / 180;
unsigned int count = 0;
extern float VMax;

/*************************************/
/*****FONCTION ÉCHANTILLONAGE*********/
/*************************************/
void Update_IT_callback(void)
{
  /****Récupération des valeurs des codeurs****/
  int16_t ticks_G = (encGauche.getTicks());
  int16_t ticks_D = (encDroit.getTicks());
  /********************************************/

  /****Calcul des vitesses des moteurs*******/
  vitesse_D = (float)(ticks_D - last_encDroit) * coefVitesseD;
  vitesse_G = (float)(ticks_G - last_encGauche) * coefVitesseG;
  /******************************************/

  /****Calcul de l'angle et de la distance*******/
  angle += (vitesse_G - vitesse_D) * coefAngle;
  distance += (vitesse_D + vitesse_G) / 2 * dt;
  /*********************************************/

  if (abs(distance) >= distanceToDecel && PID_vitesse_G.GetOutputLimitMax() > 50)
  {
    PID_vitesse_G.IncreaseOutputLimits(-1);
    PID_vitesse_D.IncreaseOutputLimits(-1);
  }
  else if (PID_vitesse_G.GetOutputLimitMax() < 250)
  {
    PID_vitesse_G.IncreaseOutputLimits(1);
    PID_vitesse_D.IncreaseOutputLimits(1);
  }

  /*****Calul de PID Angle et Vitesse****/
  if (abs(cmd_angle - angle) < epsilonAngle)
    Output_PID_angle = 0;
  else
  PID_angle.Compute();

  if (abs(cmd_distance - distance) < epsilonDistance ) {
    Output_PID_distance = 0;
    cmd_distance = 0;
    distance = 0;
  }
  else
  PID_distance.Compute();
  /*************************************/

  /***Ajustement Commandes Vitesse****/
  cmd_vitesse_G =   + Output_PID_distance+Output_PID_angle; //+Output_PID_angle
  cmd_vitesse_D = + Output_PID_distance-Output_PID_angle;
  /***********************************/

  /****Calcul des PID Vitesse*******/
  PID_vitesse_G.Compute();
  PID_vitesse_D.Compute();
  /*********************************/

  digitalWriteFast(DIR1, (Output_PID_vitesse_D >= 0));
  digitalWriteFast(DIR2, (Output_PID_vitesse_G >= 0));

  /****Commande des moteurs*******/
  analogWrite(PWM1, abs(Output_PID_vitesse_G));
  analogWrite(PWM2, abs(Output_PID_vitesse_D));
  /*****************************/

  /****Calcul de la position*******/
  update_Position(distance, angle);
  /*******************************/

  /****Sauvegarde des positions*****/
  last_encGauche = ticks_G;
  last_encDroit = ticks_D;
  /********************************/

  Update_IT = true;
  count++;
}
/*************************************/
/*************************************/
/*************************************/
