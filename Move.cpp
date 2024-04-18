#include "move.h"
#include "Odometrie.h"

extern

    MovementResult
    calculateMovement(float targetX, float targetY)
{
    MovementResult result;

    // Calcul de l'angle
    float deltaX = targetX - x;
    float deltaY = targetY - y;
    result.angle = atan2(deltaY, deltaX);

    // Calcul de la distance
    result.distance = sqrt(deltaX * deltaX + deltaY * deltaY);

    return result;
}

bool goTo(MovementResult mov, float speed)
{
    calculate_distance_time(mov.distance, speed);
    calculate_angle_time(mov.angle, speed);
    return true;
}

bool rotate(float angle_need, float speed)
{
    calculate_angle_time(angle_need, speed);
    return true;
}

void change_PID_mode(short mode)
{
    switch (mode)
    {
    case 0:
        PID_vitesse_G.setMode(MANUAL);
        PID_vitesse_D.setMode(MANUAL);
        PID_angle.setMode(MANUAL);
        PID_distance.setMode(MANUAL);
        break;
    case 1:
        PID_vitesse_G.setMode(AUTOMATIC);
        PID_vitesse_D.setMode(AUTOMATIC);
        PID_angle.setMode(AUTOMATIC);
        PID_distance.setMode(MANUAL);
        break;
    case 2:
        PID_vitesse_G.setMode(AUTOMATIC);
        PID_vitesse_D.setMode(AUTOMATIC);
        PID_angle.setMode(MANUAL);
        PID_distance.setMode(AUTOMATIC);
        break;
    case 3:
        PID_vitesse_G.setMode(AUTOMATIC);
        PID_vitesse_D.setMode(AUTOMATIC);
        PID_angle.setMode(MANUAL);
        PID_distance.setMode(MANUAL);
        break;
    case 4:
        PID_vitesse_G.setMode(AUTOMATIC);
        PID_vitesse_D.setMode(AUTOMATIC);
        PID_angle.setMode(AUTOMATIC);
        PID_distance.setMode(AUTOMATIC);
        break;
    }
}
