#include "Odometrie.h"

bool update_Position(distance, angle)
{
    position.x += cos(angle) * distance;
    position.y += sin(angle) * distance;
    return true;
}

bool reset_Position(void)
{
    position.x = 0;
    position.y = 0;
    return true;
}
