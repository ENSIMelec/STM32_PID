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

bool update_Position(float distance, float angle)
{
    x += cos(angle + angleTot) * (distance - last_distance);
    y += sin(angle + angleTot) * (distance - last_distance);
    last_distance = distance;
    return true;
}

bool reset_Position(void)
{
    x = 0;
    y = 0;
    last_distance = 0;
    return true;
}

bool reset_last_distance(void)
{
    last_distance = 0;
    return true;
}

bool reset_angle(float angle_)
{
    angleTot = angle_;
    return true;
}

bool reset_distance()
{
    last_distance = 0;
    distance = 0;
    return true;
}

bool calculate_distance_time(float distance_, float Vmax_)
{
    distance_final = distance_;
    VMax = abs(Vmax_);
    Acc = abs(Acc);

    if (distance_final < 0)
    {
        VMax = -VMax;
        Acc = -Acc;
    }

    distance_lim = VMax * VMax / Acc;
    if (abs(distance_) < abs(distance_lim))
    {
        distance_t1 = sqrt(distance_ / Acc);
        distance_t2 = distance_t1;
        VMax = Acc * distance_t1;
    }
    else
    {
        distance_t1 = VMax / Acc;
        distance_t2 = (distance_ - Acc * distance_t1 * distance_t1) / VMax + distance_t1;
    }
    return true;
}

float distance_command_ramp(float interrupt_tick)
{
    float t = interrupt_tick * dt;
    if (t < distance_t1)
    {
        return Acc * t * t / 2;
    }
    else if (t > distance_t2 + distance_t1)
    {
        return Acc * distance_t1 * distance_t1 / 2 - Acc * (distance_t1) * (distance_t1) / 2 + VMax * (distance_t2);
    }
    else if (t > distance_t2)
    {
        return Acc * distance_t1 * distance_t1 / 2 - Acc * (t - distance_t2) * (t - distance_t2) / 2 + VMax * (t - distance_t1);
    }
    else
    {
        return Acc * distance_t1 * distance_t1 / 2 + VMax * (t - distance_t1);
    }
}

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

bool reset_time_angle()
{
    angle_t1 = 0;
    angle_t2 = 0;
    return true;
}

bool reset_time_distance()
{
    distance_t1 = 0;
    distance_t2 = 0;
    return true;
}

float get_angle_tf()
{
    return (angle_t1 + angle_t2) / dt + 10;
}

float get_distance_tf()
{
    return (distance_t1 + distance_t2) / dt + 10;
}

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

void after_obstacle_detection(void)
{
    calculate_angle_time(newCommand.angle_final, VMax);
    calculate_distance_time(newCommand.distance_final - newCommand.distance_initial, VMax);
    newCommand.goto_ok = true;
}
