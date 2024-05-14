#include "main.h"
#include "FastInterruptEncoder.h"
#include "PID.h"
#include <digitalWriteFast.h>
#include "Move.h"

/*****Importation des classe******/
extern Encoder encGauche;
extern Encoder encDroit;
extern PID PID_vitesse_G;
extern PID PID_vitesse_D;
extern PID PID_angle;
extern PID PID_distance;
/***********************************/

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

void Update_IT_callback(void);
void ARU_interrupt();