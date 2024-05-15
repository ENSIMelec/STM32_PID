#include "move.h"
#include "Odometrie.h"

MovementResult
calculateMovement(float targetX, float targetY)
{
  MovementResult result;

  // Calcul de l'angle
  float deltaX = targetX - x;
  float deltaY = targetY - y;
  result.angle_initial = angle;
  result.distance_initial = distance;
  result.distance_final = 0;
  result.angle_final = atan2(deltaY, deltaX);
  // Calcul de la distance
  result.distance_final = sqrt(deltaX * deltaX + deltaY * deltaY);
  return result;
}

bool goTo(MovementResult mov)
{
  calculate_distance_time(mov.distance_final, mov.speed);
  calculate_angle_time(mov.angle_final, mov.speed);
  mov.goto_ok = true;
  return true;
}

MovementResult calculate_rotation(float angle_need)
{
  MovementResult result;
  result.angle_initial = angle;
  result.angle_final = angle_need;
  return result;
}
MovementResult calculate_moveOf(float distance_)
{
  MovementResult result;
  result.angle_initial = angle;
  result.angle_final = result.angle_initial;
  result.distance_final = distance_;
  result.distance_initial = 0;
  return result;
}

bool rotate(MovementResult mov)
{
  calculate_angle_time(mov.angle_final, mov.speed);
  mov.rotate_ok = true;
  return true;
}

bool moveOf(MovementResult mov)
{
  calculate_angle_time(mov.angle_final, mov.speed);
  calculate_distance_time(mov.distance_final, mov.speed);
  mov.goto_ok = true;
  return true;
}

void recalage(int dir)
{
  change_PID_mode(0);
  Output_PID_vitesse_G = dir * 50;
  Output_PID_vitesse_D = dir * 50;
  delay(300);

  while (abs(vitesse_G) > 0 || abs(vitesse_D) > 0)
    delay(10);

  Serial.println("Z");
  Output_PID_vitesse_G = 0;
  Output_PID_vitesse_D = 0;
}

void change_PID_mode(short mode)
{
  switch (mode)
  {
  case 0:
    PID_vitesse_G.SetMode(MANUAL);
    PID_vitesse_D.SetMode(MANUAL);
    PID_angle.SetMode(MANUAL);
    PID_distance.SetMode(MANUAL);
    break;
  case 1:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(AUTOMATIC);
    PID_distance.SetMode(MANUAL);
    break;
  case 2:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(MANUAL);
    PID_distance.SetMode(AUTOMATIC);
    break;
  case 3:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(MANUAL);
    PID_distance.SetMode(MANUAL);
    break;
  case 4:
    PID_vitesse_G.SetMode(AUTOMATIC);
    PID_vitesse_D.SetMode(AUTOMATIC);
    PID_angle.SetMode(AUTOMATIC);
    PID_distance.SetMode(AUTOMATIC);
    break;
  }
}
