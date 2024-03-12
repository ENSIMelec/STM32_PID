#include "Odometrie.h"

bool update_Position(float distance, float angle)
{
    x += cos(angle) * distance;
    y += sin(angle) * distance;
    return true;
}

bool reset_Position(void)
{
    x = 0;
    y = 0;
    return true;
}

float acceleration = 2.05*PI*35; // en mm/S-2 pour une augmentation de 10 toutes les 10 ms
int distance_End_Ramp(float distance, float VitesseOutMax)
{
    float distance_Start_Ramp = (VitesseOutMax * VitesseOutMax) / (2 * acceleration);
    return distance - distance_Start_Ramp;
}