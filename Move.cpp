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

bool goTo(MovementResult mov){
  angle_ok = false;
  distance_ok = false;
  distanceToDecel = distance_End_Ramp(mov.distance, VMax);
  //angleToDecel = angle_End_Ramp(newCommand.angle, VMax);
  cmd_angle = mov.angle;
  return true;
}  

bool rotate(float angle_need){
  angle_ok = false;
  distance_ok = false;
  cmd_angle = angle_need;
  return true;
}

