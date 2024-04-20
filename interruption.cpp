#include "interruption.h"
#include "Odometrie.h"
#include <Arduino.h>
#include "main.h"

float epsilonDistance = 2;
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

  if (distance_ok && angle_ok && newCommand.goto_ok || newCommand.rotate_ok && angle_ok)
  {
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_distance.SetMode(AUTOMATIC);
    PID_angle.SetMode(AUTOMATIC);

    cmd_distance = 0;
    distance = 0;
    reset_last_distance();

    interrupt_tick = 0;
    angle_ok = false;
    if (newCommand.goto_ok)
      distance_ok = false;
    newCommand.goto_ok = false;
    newCommand.rotate_ok = false;
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

  if ((abs(distance_final - distance) < epsilonDistance || interrupt_tick >= get_distance_tf()) && angle_ok && !distance_ok)
  {

    distance_ok = true;
    reset_time_distance();
    interrupt_tick = 0;
  }
  else
    PID_distance.Compute();

  if ((abs(angle_final - angle) < epsilonAngle || interrupt_tick >= get_angle_tf()) && !angle_ok)
  {
    angle_ok = true;
    reset_time_angle();
    interrupt_tick = 0;
  }
  else
    PID_angle.Compute();
  /*************************************/

  // si l'erreur dans la distance ou l'angle est trop grande, on ne fait rien
  if (abs(cmd_angle - angle) > 5 * PI / 180 && abs(cmd_distance - distance) > 20)
  {
    Output_PID_vitesse_G = 0;
    Output_PID_vitesse_D = 0;
    Output_PID_distance = 0;
    Output_PID_angle = 0;

    // vérifiacation pour un recalage
    if (-30 < x && x < 30)
    {
      x = 0;
      if (abs(abs(angle) - PI) < abs(angle))
        angle = PI;
      else
        angle = 0;
    }
    if (2970 < x && x < 3030)
    {
      x = 3000;
      if (abs(abs(angle) - PI) < abs(angle))
        angle = PI;
      else
        angle = 0;
    }
    if (-30 < y && y < 30)
    {
      y = 0;
      if (abs(angle - PI / 2) < abs(angle + PI / 2))
        angle = PI / 2;
      else
        angle = -PI / 2;
    }
    if (1970 < y && y < 2030)
    {
      y = 2000;
      if (abs(angle - PI / 2) < abs(angle + PI / 2))
        angle = PI / 2;
      else
        angle = -PI / 2;
    }
  }

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

  if (abs(Output_PID_vitesse_G) > 10)
    analogWrite(PWM1, abs(Output_PID_vitesse_G));
  else
    analogWrite(PWM1, 0);

  if (abs(Output_PID_vitesse_D) > 10)
    analogWrite(PWM2, abs(Output_PID_vitesse_D));
  else
    analogWrite(PWM2, 0);

  /*****************************/

  /****Calcul de la position*******/
  update_Position(distance, angle);
  /*******************************/

  /****Sauvegarde des positions*****/
  last_encGauche = ticks_G;
  last_encDroit = ticks_D;
  /********************************/

  // Update_IT = true;
  Serial.print(cmd_distance, 5);
  Serial.print(" ");
  Serial.print(distance, 5);
  Serial.print(" ");
  Serial.print(cmd_angle, 5);
  Serial.print(" ");
  Serial.println(angle, 5);
}
/*************************************/
/*************************************/
/*************************************/
