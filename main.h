#include <Arduino.h>

/******Pin********/
#define PWM1 PB6 // PWM4/1 pin D10 donc le Timer4
#define DIR1 PC1 // pin A4
#define PWM2 PA8 // PWM1/1 pin D7 donc le Timer1
#define DIR2 PC0 // pin A5
/******************/

/******ECHANTILLONAGE********/
extern float dt;
extern bool Update_IT;
extern bool Dinverse;
extern bool Ginverse;
/****************************/

/******CONSIGNES PID**********/
extern float cmd_vitesse_G; // commande vitesse moteur gauche en mm/ms
extern float cmd_vitesse_D; // commande vitesse moteur droite en mm/ms
extern float cmd_angle;     // commande angle
extern float cmd_distance;  // commande distance

/*****************************/

/***********Etalonnage Encodeur 1m******/
extern float distance_encoder_gauche;
extern float distance_encoder_droit;
/**************************************/

/********Coef Vitesse ******/
extern float VitesseOutMax; // Vitesse max théorique du moteur en mm/s
extern float coefToPWM;
extern float coefVitesseG;
extern float coefVitesseD;
/**************************/

/********Coef Angle****/
extern float empattementRoueCodeuse;
extern float coefAngle;
/**********************/

/******COEFICIENTS PID************/
extern float Kp_G, Ki_G, Kd_G;                      // coefficients PID vitesse moteur gauche
extern float Kp_D, Ki_D, Kd_D;                      // coefficients PID vitesse moteur droit
extern float Kp_angle, Ki_angle, Kd_angle;          // coefficients PID angle
extern float Kp_distance, Ki_distance, Kd_distance; // coefficients PID distance

/*********************************/

/*****Sauvegarde des positions*****/
extern int16_t last_encGauche;
extern int16_t last_encDroit;
/**********************************/

/******Constante mesuré************/
extern float vitesse_G; // vitesse gauche
extern float vitesse_D; // vitesse droite
extern float angle;     // angle
extern float distance;  // distance
/**********************************/

/******Corection PID************/
extern float Output_PID_vitesse_G; // Valeur sortante du PID vitesse moteur gauche, une PMW donc
extern float Output_PID_vitesse_D; // Valeur sortante du PID vitesse moteur droit, une PMW donc
extern float Output_PID_angle;     // Valeur sortante du PID angle
extern float Output_PID_distance;  // Valeur sortante du PID distance
/*******************************/

extern unsigned long timeSetup;

/******Postion************/
struct Position
{
    float x;
    float y;
};
extern Position position;
/*************************/