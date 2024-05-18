#include "move.h"
#include "Odometrie.h"

/*********************************************************************************************************/
/*************************Explication des fonction********************************************************/
/*********************************************************************************************************
 * calculateMovement: Calcul le mouvement à effectuer pour atteindre la position (targetX, targetY)
 * pour cela, l'angle et la distance sont calculés automatiquement en fonction de la position actuelle.
 * La fonction retourne un objet de type MovementResult qui contient les valeurs de l'angle et de la distance.
 *********************************************************************************************************
 * goTo: Fonction qui permet de déplacer le robot à la position en fonction de la structure MovementResult.
 * La focntion prend en charge l'angle et la distance pour déplacer le robot. Elle est à utiliser après
 * calculateMovement.
 *********************************************************************************************************
 * calculate_rotation: Calcul l'angle de rotation à effectuer pour atteindre l'angle voulu et non le
 * déplacement d'angle. La fonction retourne un objet de type MovementResult qui contient les valeurs
 * de l'angle initial et final.
 *********************************************************************************************************
 * rotate: Fonction qui permet de faire tourner le robot à l'angle voulu en fonction de la structure
 * MovementResult. Elle est à utiliser après calculate_rotation.
 *********************************************************************************************************
 * calculate_moveOf: Calcul le déplacement linéaire à effectuer. Le déplacement peut être positif ou négatif.
 * Si le déplacement est positif, le robot avance, si le déplacement est négatif, le robot recule. La fonction
 * retourne un objet de type MovementResult qui contient les valeurs de la distance final.
 *********************************************************************************************************
 * moveOf: Fonction qui permet de déplacer le robot en fonction de la structure MovementResult. Elle est à
 * utiliser après calculate_moveOf.
 *********************************************************************************************************
 * recalage: Fonction qui permet de recalibrer le robot sur les bordures. Le robot avance ou recule
 * en fonction de la direction donnée en paramètre (1 ou -1). Une fois le recalage effectué, le robot
 * s'arrête et envoie le message 'Z' sur le port série pour indiquer qu'il a fini. Pour le moment, le
 * recalage de l'angle est fait via les fonction de communication.
 *********************************************************************************************************
 * change_PID_mode: Fonction qui permet de changer le mode des PID. Le mode peut être manuel ou automatique
 * en fonction de la valeur donnée en paramètre. Le mode 0 désactive tous les PID, le mode 1 active les PID
 * de vitesse et d'angle, mais désactive le PID de distance, le mode 2 active les PID de distance et de vitesse
 * mais désactive le PID d'angle, le mode 3 active les PID de vitesse et désactive le PID de distance et  d'angle
 * et le mode 4 active tous les PID.
 ********************************************************************************************************/
/********************************************************************************************************/
/********************************************************************************************************/

/****************************************************/
/********CALCUL MOUVEMENT X Y************************/
/****************************************************/
MovementResult
calculateMovement(float targetX, float targetY)
{
  MovementResult result;

  // Calcul de l'angle
  float deltaX = targetX - x;                                      // calcul de la différence de position en x
  float deltaY = targetY - y;                                      // calcul de la différence de position en y
  result.angle_initial = angle;                                    // angle initial = angle actuel
  result.distance_initial = distance;                              // distance initial = distance actuel
  result.angle_final = atan2(deltaY, deltaX);                      // calcul de l'angle final
  result.distance_final = sqrt(deltaX * deltaX + deltaY * deltaY); // calcul de la distance final
  return result;                                                   // retourne la structure
}
/****************************************************/
/****************************************************/
/****************************************************/

/****************************************************/
/********INSTRUCTION DE DEPLACEMENT X Y**************/
/****************************************************/
bool goTo(MovementResult mov)
{
  calculate_distance_time(mov.distance_final, mov.speed); // calcul du temps pour la distance
  calculate_angle_time(mov.angle_final, mov.speed);       // calcul du temps pour l'angle
  mov.goto_ok = true;                                     // on indique le type de mouvement
  return true;
}
/****************************************************/
/****************************************************/
/****************************************************/

/****************************************************/
/********CALCUL ROTATION*****************************/
/****************************************************/
MovementResult calculate_rotation(float angle_need)
{
  MovementResult result;           // structure de retour
  result.angle_initial = angle;    // angle initial = angle actuel
  result.angle_final = angle_need; // angle final = angle voulu
  return result;                   // retourne la structure
}
/****************************************************/
/****************************************************/
/****************************************************/

/****************************************************/
/********INSTRUCTION DE ROTATION*********************/
/****************************************************/
bool rotate(MovementResult mov)
{
  calculate_angle_time(mov.angle_final, mov.speed); // calcul du temps pour l'angle
  mov.rotate_ok = true;                             // on indique le type de mouvement
  return true;
}
/****************************************************/
/****************************************************/
/****************************************************/

/****************************************************/
/********CALCUL DEPLACEMENT LINEAIRE*****************/
/****************************************************/
MovementResult calculate_moveOf(float distance_)
{
  MovementResult result;                     // structure de retour
  result.angle_initial = angle;              // angle initial = angle actuel
  result.angle_final = result.angle_initial; // angle final = angle initial
  result.distance_final = distance_;         // distance final = distance à parcourir
  result.distance_initial = 0;               // distance initial = 0
  return result;
}
/****************************************************/
/****************************************************/
/****************************************************/

/****************************************************/
/********INSTRUCTION DE DEPLACEMENT LINEAIRE*********/
/****************************************************/
bool moveOf(MovementResult mov)
{

  calculate_angle_time(mov.angle_final, mov.speed);       // calcul du temps pour l'angle
  calculate_distance_time(mov.distance_final, mov.speed); // calcul du temps pour la distance
  mov.goto_ok = true;                                     // on indique le type de mouvement
  return true;
}
/****************************************************/
/****************************************************/
/****************************************************/

/****************************************************/
/********RECALAGE SUR LES BORDURES*******************/
/****************************************************/
void recalage(int dir)
{
  change_PID_mode(0); // desactivation des PID

  // Changement de la consigne de vitesse en fonction de la direction
  Output_PID_vitesse_G = dir * 50;
  Output_PID_vitesse_D = dir * 50;

  delay(300); // attente pour que les roue se mettent en marche

  // Attente du blocage des roues
  while (abs(vitesse_G) > 0 || abs(vitesse_D) > 0)
    delay(10);

  // Envoi du message de fin
  Serial.println("Z");

  // Arrêt des roues
  Output_PID_vitesse_G = 0;
  Output_PID_vitesse_D = 0;
}
/****************************************************/
/****************************************************/
/****************************************************/

/****************************************************/
/********CHANGEMENT DE MODE DES PID******************/
/****************************************************/
void change_PID_mode(short mode)
{
  switch (mode)
  {
  // Mode manuel
  case 0:
    PID_vitesse_G.SetMode(MANUAL);
    PID_vitesse_D.SetMode(MANUAL);
    PID_angle.SetMode(MANUAL);
    PID_distance.SetMode(MANUAL);
    break;
  // Mode vitesse et angle
  case 1:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(AUTOMATIC);
    PID_distance.SetMode(MANUAL);
    break;
  // Mode vitesse et distance
  case 2:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(MANUAL);
    PID_distance.SetMode(AUTOMATIC);
    break;
  // Mode vitesse
  case 3:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(MANUAL);
    PID_distance.SetMode(MANUAL);
    break;
  // Mode automatique
  case 4:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(AUTOMATIC);
    PID_distance.SetMode(AUTOMATIC);
    break;
  }
}
/****************************************************/
/****************************************************/
/****************************************************/
