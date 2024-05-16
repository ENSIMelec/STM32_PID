#include "Odometrie.h"

/*Variables de sauvegarde pour l'odométrie*/
float last_distance = 0;
/***************************************/

/*Variables de temps pour les rampe*/
float distance_t1; // temps de fin de la rampe d'accélération
float distance_t2; // temps de début de la rampe de décélération
float angle_t1;
float angle_t2;
/***********************************/

/*Variable pour calcul des rampes de vitesse*/
float Acc = 1000;   // acceleration linéaire
float distance_lim; // distance limite

float angle_initial = 0;

const float Attenuantion_vit_ang = 0.7;
float VMaxAngulaire; // Vitesse angulaire max
float AccAngulaire;  // Acceleration angulaire
float angle_lim;     // angle limite
/***********************************/

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

/*************************************************/
/************CALCUL ODOMETRIE*********************/
/*************************************************/
bool update_Position(float distance, float angle)
{
    // le += correspond à l'intégral du déplacement
    x += cos(angle) * (distance - last_distance); // mise à jour de la position en x
    y += sin(angle) * (distance - last_distance); // mise à jour de la position en y
    last_distance = distance;
    return true;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/************RENITIALISATION POSITION*************/
/*************************************************/
bool reset_Position(void)
{
    x = 0;
    y = 0;
    last_distance = 0;
    return true;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/****************RESET DISTANCE*******************/
/*************************************************/
bool reset_last_distance(void)
{
    last_distance = 0;
    return true;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/*******SAUVEGARDE DISTANCE APRES RESET***********/
/*************************************************/
bool reset_distance()
{
    last_distance = 0;
    distance = 0;
    return true;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/*********CALCUL DES TEMPS RAMPE DISTANCE*********/
/*************************************************/
bool calculate_distance_time(float distance_, float Vmax_)
{
    distance_final = distance_; // distance à parcourir
    VMax = abs(Vmax_);          // vitesse max
    Acc = abs(Acc);             // Accélération constante positive

    // si la distance est négative alors la vitesse max et l'accélération sont négative
    // pour permettre un déplacement en arrière
    if (distance_final < 0)
    {
        VMax = -VMax;
        Acc = -Acc;
    }

    distance_lim = VMax * VMax / Acc; // distance limite pour atteindre la V max

    // si la distance est inférieur à la distance limite alors on a un triangle
    if (abs(distance_) < abs(distance_lim))
    {
        distance_t1 = sqrt(distance_ / Acc);
        distance_t2 = distance_t1; // les deux temps sont équivalent
        VMax = Acc * distance_t1;  // ajustement de la vitesse max
    }
    else
    {
        distance_t1 = VMax / Acc;                                                         // temps de fin de la rampe d'accélération
        distance_t2 = (distance_ - Acc * distance_t1 * distance_t1) / VMax + distance_t1; // temps de début de la rampe de décélération
    }
    return true;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/******CALCUL DES COMMANDES RAMPE DISTANCE********/
/*************************************************/
float distance_command_ramp(float interrupt_tick)
{
    float t = interrupt_tick * dt; // temps en seconde

    // en phase d'accélération
    if (t < distance_t1)
    {
        return Acc * t * t / 2; // retourne la commande de distance
    }

    // en phase final arriver à la postion final
    else if (t > distance_t2 + distance_t1)
    {
        // retourne la position final calculer. Cela permet de changer à la voler les temps.
        return Acc * distance_t1 * distance_t1 / 2 - Acc * (distance_t1) * (distance_t1) / 2 + VMax * (distance_t2);
    }
    // en phase de décélération
    else if (t > distance_t2)
    {
        return Acc * distance_t1 * distance_t1 / 2 - Acc * (t - distance_t2) * (t - distance_t2) / 2 + VMax * (t - distance_t1);
    }
    // en phase de vitesse constante
    else
    {
        return Acc * distance_t1 * distance_t1 / 2 + VMax * (t - distance_t1);
    }
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/*********CALCUL DES TEMPS RAMPE ANGLE************/
/*************************************************/
bool calculate_angle_time(float angle_, float Vmax_)
{
    angle_initial = angle;
    angle_final = angle_;
    float angle_parcouru = angle_final - angle_initial;
    Vmax_ = abs(Vmax_);
    VMaxAngulaire = Vmax_ / empattementRoueCodeuse * 2;
    AccAngulaire = abs(Acc) / empattementRoueCodeuse * Attenuantion_vit_ang;
    if (angle_parcouru < 0)
    {
        VMaxAngulaire = -VMaxAngulaire;
        AccAngulaire = -AccAngulaire;
    }

    angle_lim = VMaxAngulaire * VMaxAngulaire / AccAngulaire;

    if (abs(angle_parcouru) < abs(angle_lim))
    {
        angle_t1 = sqrt(angle_parcouru / AccAngulaire);
        angle_t2 = angle_t1;
        VMaxAngulaire = AccAngulaire * angle_t1;
    }
    else
    {
        angle_t1 = VMaxAngulaire / AccAngulaire;
        angle_t2 = (angle_parcouru - AccAngulaire * angle_t1 * angle_t1) / VMaxAngulaire + angle_t1;
    }
    return true;
}

/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/******CALCUL DES COMMANDES RAMPE ANGLE************/
/*************************************************/
float angle_command_ramp(float interrupt_tick)
{
    float t = interrupt_tick * dt;
    if (t < angle_t1)
    {
        return AccAngulaire * t * t / 2 + angle_initial;
    }
    else if (t > angle_t2 + angle_t1)
    {
        return angle_final;
    }
    else if (t > angle_t2)
    {
        return AccAngulaire * angle_t1 * angle_t1 / 2 - AccAngulaire * (t - angle_t2) * (t - angle_t2) / 2 + VMaxAngulaire * (t - angle_t1) + angle_initial;
    }
    else
    {
        return AccAngulaire * angle_t1 * angle_t1 / 2 + VMaxAngulaire * (t - angle_t1) + angle_initial;
    }
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/*********RESET TEMPS RAMPE ANGLE*****************/
/*************************************************/
bool reset_time_angle()
{
    angle_t1 = 0;
    angle_t2 = 0;
    return true;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/*********RESET TEMPS RAMPE DISTANCE**************/
/*************************************************/
bool reset_time_distance()
{
    distance_t1 = 0;
    distance_t2 = 0;
    return true;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/****OTBTENIR TEMPS FIN DEPLACEMENT ANGLE*********/
/*************************************************/
float get_angle_tf()
{
    return (angle_t1 + angle_t2) / dt + 10;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/****OTBTENIR TEMPS FIN DEPLACEMENT DISTANCE******/
/*************************************************/
float get_distance_tf()
{
    return (distance_t1 + distance_t2) / dt + 10;
}
/*************************************************/
/*************************************************/
/*************************************************/

/*************************************************/
/********DECELERATION SI DETECTION****************/
/*************************************************/
void obstacle_detection()
{
    float t = interrupt_tick * dt;
    if (t >= (distance_t1 + distance_t2))
        return;
    if (t < distance_t1)
    {
        distance_t1 = interrupt_tick * dt;
        distance_t2 = distance_t1;
        VMax = Acc * distance_t1;
        return;
    }
    distance_t2 = interrupt_tick * dt;
    newCommand.distance_initial = Acc * distance_t1 * distance_t1 / 2 - Acc * (distance_t1) * (distance_t1) / 2 + VMax * (distance_t2);
}
/************************************************/
/************************************************/
/************************************************/

/************************************************/
/*****REDEMARAGE DEPLACEMENT APRES DETECTION*****/
/************************************************/
void after_obstacle_detection(void)
{
    calculate_angle_time(newCommand.angle_final, VMax);
    calculate_distance_time(newCommand.distance_final - newCommand.distance_initial, VMax);
    newCommand.goto_ok = true;
}
/************************************************/
/************************************************/
/************************************************/
