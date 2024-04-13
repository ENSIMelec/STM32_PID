#include "Odometrie.h"

float last_distance = 0;
float angleTot = 0;
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

bool reset_angle(float angle_){
  angleTot = angle_;
}

float acceleration = 1000; // en mm/S-2 pour une augmentation de 1 toutes les 10 ms
int distance_End_Ramp(float distance, float VitesseOutMax)
{
    float distance_Start_Ramp = (VitesseOutMax * VitesseOutMax) / (2 * acceleration);
    return distance - distance_Start_Ramp;
}

int angle_End_Ramp(float angle, float VitesseAngulaireMax)
{
    float angle_Start_Ramp = (VitesseAngulaireMax * VitesseAngulaireMax) / (2 * acceleration);
    return angle - angle_Start_Ramp;
}