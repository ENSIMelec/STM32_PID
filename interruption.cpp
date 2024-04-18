#include "interruption.h"
#include "Odometrie.h"
#include <Arduino.h>
#include "main.h"

float epsilonDistance = 0.5;
float epsilonAngle = PI / 180 / 2;
unsigned int interrupt_tick = 0;
/*************************************/
/*****FONCTION ÉCHANTILLONAGE*********/
/*************************************/
void Update_IT_callback(void)
{
  /****Récupération des valeurs des codeurs****/
  int16_t ticks_G = (encGauche.getTicks());
  int16_t ticks_D = (encDroit.getTicks());
  /********************************************/

  if (distance_ok && angle_ok)
  {
    cmd_distance = 0;
    distance = 0;
    reset_last_distance();
    interrupt_tick = 0;
    angle_ok = false;
    distance_ok = false;
  }
  else if (!angle_ok)
  {
    interrupt_tick += 1;
    cmd_angle = angle_command_ramp(interrupt_tick);
  }

  else if (!distance_ok && angle_ok)
  {
    interrupt_tick += 1;
    cmd_distance = distance_command_ramp(interrupt_tick);
  }

  /****Calcul des vitesses des moteurs*******/
  vitesse_D = (float)(ticks_D - last_encDroit) * coefVitesseD;
  vitesse_G = (float)(ticks_G - last_encGauche) * coefVitesseG;
  /******************************************/

  /****Calcul de l'angle et de la distance*******/
  angle += (vitesse_G - vitesse_D) * coefAngle;
  distance += (vitesse_D + vitesse_G) / 2 * dt;
  /*********************************************/

  /*****Calul de PID Angle et Vitesse****/

  if (abs(distance_final - distance) < epsilonDistance)
  {
    Output_PID_distance = 0;
    if (angle_ok)
    {
      distance_ok = true;
      reset_time_distance();
      interrupt_tick = 0;
    }
  }
  else
    PID_distance.Compute();

  if (abs(angle_final - angle) < epsilonAngle)
  {
    Output_PID_angle = 0;
    angle_ok = true;
    reset_time_angle();
    interrupt_tick = 0;
  }
  else
    PID_angle.Compute();
  /*************************************/

  /***Ajustement Commandes Vitesse****/
  cmd_vitesse_G = +Output_PID_distance + Output_PID_angle;
  cmd_vitesse_D = +Output_PID_distance - Output_PID_angle;
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

  // Update_IT = true;
  Serial.print(cmd_angle * 180 / PI, 5);
  Serial.print(" ");
  Serial.println(angle * 180 / PI, 5);
}
/*************************************/
/*************************************/
/*************************************/
