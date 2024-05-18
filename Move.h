#ifndef MOVE_H
#define MOVE_H

#include "main.h"
#include <cmath>
#include "PID.h"

/* Variables globales */
extern PID PID_vitesse_G;
extern PID PID_vitesse_D;
extern PID PID_angle;
extern PID PID_distance;

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

MovementResult calculateMovement(float targetX, float targetY);
MovementResult calculate_rotation(float angle_need);
MovementResult calculate_moveOf(float distance_);

bool goTo(MovementResult mov);
bool rotate(MovementResult mov);
bool updateVmax(int new_Vmax);
bool moveOf(MovementResult mov);
void recalage(int dir);
void change_PID_mode(short mode);

#endif
