#include "main.h"

/*********************************************************************************************/
/*************************Explication des fonction********************************************/
/*********************************************************************************************
 * update_Position: met a jour la position du robot en fonction de son angle actuelle
 * et de sa position.
 *********************************************************************************************
 * reset_Position: remet la position du robot au coordonnées (0,0)
 *********************************************************************************************
 * reset_last_distance: remet la distance parcourue sauvegarder à 0
 *********************************************************************************************
 * reset_distance: remet la distance parcourue dans une variable avant
 * de remetre la distance parcourue à 0 lors d'un nouveau déplacement
 *********************************************************************************************
 * calculate_distance_time: calcul les temps de déplacement sauvegarder dans
 * les variables disatnce_t1 et distance_t2. La variable distance_t1 est le temps
 * de fin de la rampe d'accélération et distance_t2 est le temps de début de la rampe
 * de décélération. La fonction prend en paramètre la distance à parcourir et la vitesse
 * max du robot. Le but est d'avoir des vitesses sous formes de trapèze si la distance
 * est assez grande pour atteindre la vitesse max. Sinon cela formera un triangle.
 *********************************************************************************************
 * distance_command_ramp: calcul la commande de distance en fonction du temps. Pour
 * connaitre le temps une variable compte le nombre d'interruption dans la focntion
 * d'interruption. La fonction renvoie la commande de distance. Pour cela, on intégre
 * les déplacement présumer à la vitesse demander.
 *********************************************************************************************
 * calculate_angle_time: similiare à calculate_distance_time mais pour la commande et
 * déplacement d'angle. Cette fois si la fonction prend en paramètre l'angle à laquelle
 * il doit aller et non le déplacement d'angle. Ceci est du au fait que l'angle n'est
 * jamais rénitialiser car normalement contenue entre pi et -pi.
 *********************************************************************************************
 * angle_command_ramp: similaire à distance_command_ramp mais pour les commandes d'angle.
 *********************************************************************************************
 * reset_time_angle: remet les temps de rampe d'angle à 0
 *********************************************************************************************
 * reset_time_distance: remet les temps de rampe de distance à 0
 *********************************************************************************************
 * get_angle_tf: renvoie le temps de fin de déplacement d'angle avec une marge de 100 ms
 *********************************************************************************************
 * get_distance_tf: renvoie le temps de fin de déplacement de distance avec une marge de 100 ms
 *********************************************************************************************
 * obstacle_detection: fonction qui permet de lancer une decélération, lors d'un déplacement.
 * Utile pour arreter le robot si le lidar à detecter un obstacle. Pour cela, le temps de
 * distance_t2 et distance_t1 sont changer à la volé pour permettre la décélération.
 *********************************************************************************************
 * after_obstacle_detection: fonction qui permet de relancer le déplacement après une détection.
 * la fonction relance le déplacement avec la distance restante à effectuer.
 *********************************************************************************************/
/*********************************************************************************************/
/*********************************************************************************************/

bool update_Position(float distance, float angle);

bool reset_Position(void);

bool reset_last_distance(void);

bool reset_distance();

bool calculate_distance_time(float distance_, float Vmax_);

bool calculate_angle_time(float angle_, float Vmax_);

float distance_command_ramp(float interrupt_tick);

float angle_command_ramp(float interrupt_tick);

bool reset_time_angle();

bool reset_time_distance();

float get_angle_tf();

float get_distance_tf();

void obstacle_detection();

void after_obstacle_detection(void);