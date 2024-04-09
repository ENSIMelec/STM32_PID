#include "move.h"

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
