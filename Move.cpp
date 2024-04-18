#include "move.h"
#include "Odometrie.h"
MovementResult calculateMovement(float targetX, float targetY)
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

bool goTo(MovementResult mov, float speed){
  calculate_distance_time(mov.distance, speed);
  calculate_angle_time(mov.angle, speed);
  return true;
}  

bool rotate(float angle_need, float speed){
  calculate_angle_time(angle_need, speed);
  return true;
}

