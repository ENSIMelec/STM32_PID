
#ifndef MAIN_H
#define MAIN_H

#include <Arduino.h>

/******Pin********/
#define PWM1 PA8 // PWM4/1 pin D10 donc le Timer4
#define DIR1 PA10
#define PWM2 PB6 // PWM1/1 pin D7 donc le Timer1
#define DIR2 PB3
#define CodDB PA0 // codeur droit chanel B
#define CodDA PA1 // codeur droit chanel A
#define CodGB PB4
#define CodGA PB5
#define ARU PA14 // Arret d'urgence
/******************/

/******Mode********/
extern short mode; // 0 stop PID, 1 PID angle ON, 2 PID distance ON, 3 PID Vitesse ON, PID angle et distance ON
extern bool debug; // Mode debug
/******************/

/*****Structure********/
struct MovementResult
{
    float angle_final = 0;      // angle voulu
    float distance_final = 0;   // distance voulu
    float angle_initial = 0;    // angle de départ
    float distance_initial = 0; // distance de départ
    float speed = 500;          // vitesse max par défaut
    bool goto_ok = false;       // mouvement goTo ou moveOf voulu
    bool rotate_ok = false;     // mouvement de rotation voulu
};
/*********************/

/******ECHANTILLONAGE********/
extern float dt;
extern volatile int Update_IT;
/****************************/

/******CONSIGNES PID**********/
extern float cmd_vitesse_G; // commande vitesse moteur gauche en mm/ms
extern float cmd_vitesse_D; // commande vitesse moteur droite en mm/ms
extern float cmd_angle;     // commande angle
extern float cmd_distance;  // commande distance
/*****************************/

/***********Etalonnage Encodeur 1m ou 10 tours******/
extern float distance_encoder_gauche;
extern float distance_encoder_droit;
/***************************************************/

/********Coef Vitesse ******/
extern float VitesseOutMax; // Vitesse max théorique du moteur en mm/s
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
extern int16_t last_encGauche; // sauvegarde de la position de l'encodeur gauche
extern int16_t last_encDroit;  // sauvegarde de la position de l'encodeur droit
extern float x;                // position x
extern float y;                // position y
/**********************************/

/******Constante mesuré************/
extern float vitesse_G; // vitesse gauche
extern float vitesse_D; // vitesse droite
extern float angle;     // angle
extern float distance;  // distance
/**********************************/

/***Variable Rampes Accélération************/
extern bool distance_ok;                // distance atteinte
extern bool angle_ok;                   // angle atteint
extern float angle_final;               // angle final
extern float distance_final;            // distance final
extern unsigned int interrupt_tick;     // mesure du temps pour les rampes
extern MovementResult newCommand;       // commande de déplacement
extern bool send_new_command_available; // nouvelle commande disponible
extern int arret_lidar;                 // 0 = arret en cours, 1 = arreter, 2 = reprise possible du mouvement
extern float VMax;                      // vitesse max
/*******************************************/

/******Corection PID************/
extern float Output_PID_vitesse_G; // Valeur sortante du PID vitesse moteur gauche, une PMW donc
extern float Output_PID_vitesse_D; // Valeur sortante du PID vitesse moteur droit, une PMW donc
extern float Output_PID_angle;     // Valeur sortante du PID angle
extern float Output_PID_distance;  // Valeur sortante du PID distance
/*******************************/

extern unsigned long timeSetup; // temps de setup pour la boucle ouverte

#endif