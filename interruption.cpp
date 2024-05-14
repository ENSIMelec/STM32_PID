#include "interruption.h"
#include "Odometrie.h"
#include <Arduino.h>
#include "main.h"

float epsilonDistance = 2;         // erreur acceptable pour la distance
float epsilonAngle = PI / 180 / 2; // erreur acceptable pour l'angle
unsigned int interrupt_tick = 0;   // temps écoulé depuis le début de la commande

/************************************************************************************/
/*************************Explication des fonction***********************************/
/************************************************************************************
 * Update_IT_callback: Fonction d'interruption pour l'échantiollnage du PID.
 * Elle est appelé toute les dt secondes par le timer. Cette fonction permet de
 * calculer les vitesses des moteurs, l'angle et la distance parcourue par le robot,
 * ainsi que de calculer les sorties des PID. Pour l'asservissement en postion du robot.
 *************************************************************************************
 * ARU_interrupt: Fonction d'interruption pour l'arrêt d'urgence. Elle est appelé
 * lorque le bouton d'arret d'urgence est appuyé. Cette fonction permet de redémarer
 * la carte de zéro. Un problème éléctronique fait que l'interuption n'était pas appelé
 * lors de l'appui sur le bouton d'arrêt d'urgence.
 ************************************************************************************/
/************************************************************************************/
/************************************************************************************/

/*************************************/
/*****FONCTION ÉCHANTILLONAGE*********/
/*************************************/
void Update_IT_callback(void)
{
  /****Récupération des valeurs des codeurs****/
  int16_t ticks_G = (encGauche.getTicks()); // évite tous changment de valeur des ticks pendant l'interuption
  int16_t ticks_D = (encDroit.getTicks());
  /********************************************/

  if (distance_ok && angle_ok && newCommand.goto_ok || newCommand.rotate_ok && angle_ok)
  {
    change_PID_mode(4); // on met les PID en mode auto au cas ou ils étaient en manuel

    /***Réinitialisation des valeurs*****/

    // Reset de la disatnce
    reset_last_distance();
    cmd_distance = 0;
    distance = 0;

    // Supression de l'erreur entre la commande et l'angle
    cmd_angle = angle;

    // Reset du temps
    interrupt_tick = 0;
    /************************************/

    /*******Verification pour éviter les débordement des timers*****/
    // si les ticks sont trop grand, on les remet à 0
    // cette solution a été implémenté rapidement et nous nous sommes rendu compte
    // que cela entrainait une erreur au niveau du calcul de l'odometrie,
    // avec l'apparition d'un leger décalage de la position.
    // Solution a améliorer !
    if (encDroit.getTicks() > 2500 || encGauche.getTicks() > 2500)
    {
      last_encGauche = -ticks_D + last_encDroit;
      last_encDroit = -ticks_D + last_encDroit;
      encDroit.resetTicks();
      encGauche.resetTicks();
      ticks_G = 0;
      ticks_D = 0;
    }

    angle_ok = false;
    if (newCommand.goto_ok)
      distance_ok = false;
    newCommand.goto_ok = false;
  }
  // Si l'angle n'est pas atteint, on continue de tourner
  else if (!angle_ok)
  {

    interrupt_tick += 1;                            // on incrémente le temps
    cmd_angle = angle_command_ramp(interrupt_tick); // on récupère la commande de l'angle
  }
  // une fois l'angle atteint, on passe à la distance
  else if (!distance_ok && angle_ok)
  {
    interrupt_tick += 1;                                  // on incrémente le temps
    cmd_distance = distance_command_ramp(interrupt_tick); // on récupère la commande de la distance
  }
  /******************************************************************/

  /****Calcul des vitesses des moteurs*******/
  vitesse_D = (float)(ticks_D - last_encDroit) * coefVitesseD;
  vitesse_G = (float)(ticks_G - last_encGauche) * coefVitesseG;
  /******************************************/

  /****Calcul de l'angle et de la distance*******/
  angle += (vitesse_G - vitesse_D) * coefAngle;
  distance += (vitesse_D + vitesse_G) / 2 * dt;
  /*********************************************/

  /*****Calul de PID Angle et Vitesse****/
  // On commence par ajuster l'angle, puis la distance
  // si il a atteint la consigne en distance ou que le temps qu'il doit mettre pour atteindre la consigne est dépassé
  if ((abs(distance_final - distance) < epsilonDistance || interrupt_tick >= get_distance_tf()) && angle_ok && !distance_ok)
  {

    distance_ok = true;            // accepte une nouvelle commande de distance
    arret_lidar++;                 // on incrémente le nombre de fois ou on a pas reçu de commande
    newCommand.distance_final = 0; // on remet la commande de distance à 0, au cas ou on reçoit un restartmove
    newCommand.distance_initial = 0;
    send_new_command_available = true; // on envoie un message pour dire qu'on est prêt à recevoir une nouvelle commande
    reset_time_distance();             // on remet les temps des rampes à 0
    interrupt_tick = 0;                // on remet le temps actuelle à 0
  }
  else
    PID_distance.Compute(); // on calcule la sortie du PID distance

  if ((abs(angle_final - angle) < epsilonAngle || interrupt_tick >= get_angle_tf()) && !angle_ok)
  {
    if (newCommand.rotate_ok)
    {
      newCommand.rotate_ok = false;
      send_new_command_available = true;
    }
    angle_ok = true;
    reset_time_angle();
    interrupt_tick = 0;
  }
  else
    PID_angle.Compute(); // on calcule la sortie du PID angle
  /*************************************/

  // si l'erreur dans la distance ou l'angle est trop grande, on ne fait rien
  // évite l'emballement des roue une fois arriver à sa condition.
  if (((abs(cmd_angle - angle) > 3 * PI / 180) && angle_ok && distance_ok || (abs(cmd_distance - distance) > 10) && distance_ok && angle_ok) && PID_angle.GetMode())
  {
    change_PID_mode(0);
    Output_PID_angle = 0;
    Output_PID_distance = 0;
    Output_PID_vitesse_D = 0;
    Output_PID_vitesse_G = 0;
  }

  /***Ajustement Commandes Vitesse****/
  cmd_vitesse_G = +Output_PID_distance + Output_PID_angle;
  cmd_vitesse_D = +Output_PID_distance - Output_PID_angle;
  /***********************************/

  /****Calcul des PID Vitesse*******/
  PID_vitesse_G.Compute();
  PID_vitesse_D.Compute();
  /*********************************/

  /****Commande de direction des moteurs*******/
  // si la vitesse est positive HIGH, sinon LOW
  digitalWriteFast(DIR1, (Output_PID_vitesse_D >= 0));
  digitalWriteFast(DIR2, (Output_PID_vitesse_G >= 0));
  /********************************************/

  /****Commande des moteurs ajout avec une deadzone*******/
  if (abs(Output_PID_vitesse_G) > 10)
    analogWrite(PWM1, abs(Output_PID_vitesse_G));
  else
    analogWrite(PWM1, 0);

  if (abs(Output_PID_vitesse_D) > 10)
    analogWrite(PWM2, abs(Output_PID_vitesse_D));
  else
    analogWrite(PWM2, 0);
  /********************************************************/

  /****Calcul de la position*******/
  update_Position(distance, angle);
  /*******************************/

  /****Sauvegarde des positions*****/
  last_encGauche = ticks_G;
  last_encDroit = ticks_D;
  /********************************/

  Update_IT++; // incrémente le nombre d'interruption pour l'envoie du debug
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/***********INTERUPTION***************/
/*********ARRET D'URGENCE*************/
void ARU_interrupt()
{
  digitalWrite(LED_BUILTIN, LOW);
  delay(500);
  HAL_NVIC_SystemReset(); // redèmare le programme
}
/*************************************/
/*************************************/
/*************************************/