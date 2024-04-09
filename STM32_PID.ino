
#include "main.h"
#include "PID.h"                  // bibliothèque PID
#include "FastInterruptEncoder.h" // bibliothèque pour les codeurs incrémentaux
#include "SimFirstOrder.h"        // bibliothèque pour la simulation du moteur
#include "Communication.h"
#include "interruption.h"
#include "BoucleOuverte.h"
#include "Odometrie.h"

/******Mode********/
#define DEBUG // mode debug
/******************/
unsigned long timeSetup;

/******ECHANTILLONAGE********/
float dt = 10e-3; // 10ms
bool Update_IT = false;
/****************************/

/******CONSIGNES PID**********/
float cmd_vitesse_G = 0; // commande vitesse moteur gauche en mm/ms
float cmd_vitesse_D = 0; // commande vitesse moteur droite en mm/ms
float cmd_angle = 0;     // commande angle
float cmd_distance = 0;  // commande distance
float distanceToDecel;
float angleToDecel;
float VMax = 1000;
/*****************************/

/***********Etalonnage Encodeur 1m et 10 PI******/
float distance_encoder_gauche = 1000.0 / 5023.0;
// float angle_encoder_droit = 20*PI/(18778.0 + 18824.0);
float distance_encoder_droit = 1000.0 / 4715.0;
// float angle_encoder_droit = 20*PI/(18556.0 + 18639.0);
/**************************************/

/********Coef Vitesse ******/
float VitesseOutMax = 1039.5; // Vitesse max théorique du moteur en mm/s
float coefVitesseG = distance_encoder_gauche / dt;
float coefVitesseD = distance_encoder_droit / dt;
/**************************/

/********Coef Angle****/
float empattementRoueCodeuse = 240;
float coefAngle = dt / empattementRoueCodeuse;
/**********************/

/******COEFICIENTS PID************/
float Kp_G = 100.0 / 525.0, Ki_G = 0, Kd_G = 0.0001;          // coefficients PID vitesse moteur gauche
float Kp_D = 100.0 / 480.0, Ki_D = 0, Kd_D = 0.0001;          // coefficients PID vitesse moteur droit
float Kp_angle = 4000, Ki_angle = 79, Kd_angle = 5;           // coefficients PID angle
float Kp_distance = 50, Ki_distance = 0.9, Kd_distance = 0.5; // coefficients PID distance
/*********************************/

/******Declaration des codeurs************/
// TODO : faire 1 metre avec le robot a la main pour voir combien de tick on fait les codeurs
// et les étalonner

// - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only
Encoder encGauche(CodGA, CodGB, TIM3, HALFQUAD, 250); // PWM2/1 pin A0 et PWM2/2 pin A1 Donc Timer 2 utilisé
Encoder encDroit(CodDA, CodDB, TIM2, HALFQUAD, 250);  // PWM3/1 pin D5 et PWM3/2 pin D4 Donc Timer 3 utilisé
/***************************************/

/*****Sauvegarde des positions*****/
int16_t last_encGauche = 0;
int16_t last_encDroit = 0;
float x = 0;
float y = 0;
/**********************************/

/******Constante mesuré************/
float vitesse_G = 0; // vitesse gauche
float vitesse_D = 0; // vitesse droite
float angle = 0;     // angle
float distance = 0;  // distance
/**********************************/

/******Corection PID************/
float Output_PID_vitesse_G = 0; // Valeur sortante du PID vitesse moteur gauche, une PMW donc
float Output_PID_vitesse_D = 0; // Valeur sortante du PID vitesse moteur droit, une PMW donc
float Output_PID_angle = 0;     // Valeur sortante du PID angle
float Output_PID_distance = 0;  // Valeur sortante du PID distance
/*******************************/

/******Declaration des PID************/
PID PID_vitesse_G(&vitesse_G, &Output_PID_vitesse_G, &cmd_vitesse_G, dt, Kp_G, Ki_G, Kd_G, DIRECT);
PID PID_vitesse_D(&vitesse_D, &Output_PID_vitesse_D, &cmd_vitesse_D, dt, Kp_D, Ki_D, Kd_D, DIRECT);
PID PID_angle(&angle, &Output_PID_angle, &cmd_angle, dt, Kp_angle, Ki_angle, Kd_angle, DIRECT);
PID PID_distance(&distance, &Output_PID_distance, &cmd_distance, dt, Kp_distance, Ki_distance, Kd_distance, DIRECT);
/*************************************/

/********************************************/
/********************************************/
/********************************************/

/*************************************/
/*****SETUP***************************/
/*************************************/
void setup()
{
  /**********INITIALISATION COMMUNICATION SÉRIE*/
  Serial.begin(115200); // Par défaut utilisation de USART1
  /*********************************************/
  Serial.println("Serial OK");

#ifdef DEBUG
  /****************************/
  /********MODE DEBUG***********/
  if (encDroit.init() && encGauche.init())
  {
    Serial.println("-Encoder Initialization OK");
  }
  else
  {
    Serial.println("-Encoder Initialization Failed");
    while (1)
      ;
  }

  pinMode(LED_BUILTIN, OUTPUT);    // Configure la broche de la LED comme sortie
  digitalWrite(LED_BUILTIN, HIGH); // Allume LED Confirmation d'initialisation
  /***************************************/

  /****************************/
  /****************************/
#else
  /****************************/
  /********MODE NORMAL*********/
  if (!encDroit.init() && !encGauche.init())
  {
    while (1)
      ; // encoder initialization failed
  }
  /****************************/
  /****************************/
#endif

  /******Initialisation des PINs****/
  pinMode(DIR1, OUTPUT); // PA_3 = pin D0
  pinMode(DIR2, OUTPUT); // PA_2 = pin D1
  pinMode(PWM1, OUTPUT); // PWM4/1 pin D10 donc le Timer4
  pinMode(PWM2, OUTPUT); // PWM1/1 pin D7 donc le Timer1
  /*********************************/

  delay(5000);
  /******Activation des PID************/
  encDroit.resetTicks();
  encGauche.resetTicks();
  /***********************************/

  /******Initialisation de l'interruption pour l'échantillonnage************/
  TIM_TypeDef *Instance = TIM6;
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(1 / dt, HERTZ_FORMAT);
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
  /**************************************************************************/

  PID_angle.SetOutputLimits(-200, 200, 50);
  PID_distance.SetOutputLimits(-1000, 1000, 50);
  PID_vitesse_D.SetOutputLimits(-1, 1, 40);
  PID_vitesse_G.SetOutputLimits(-1, 1, 40);

  PID_angle.SetMode(AUTOMATIC);
  PID_distance.SetMode(AUTOMATIC);
  PID_vitesse_D.SetMode(AUTOMATIC);
  PID_vitesse_G.SetMode(AUTOMATIC);
  cmd_angle = 0;
  cmd_distance = 2000;
  distanceToDecel = distance_End_Ramp(cmd_distance, VMax);
  angleToDecel = angle_End_Ramp(cmd_angle, VMax);
  timeSetup = millis();
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*****LOOP**************************/
/*************************************/
void loop()
{
  if (Update_IT)
  {
    sendData();
  }

  // Config_PID_Vitesse();
}
/*************************************/
/*************************************/
/*************************************/

// ! ||--------------------------------------------------------------------------------||
// ! ||                                   INFORMATION                                  ||
// ! ||--------------------------------------------------------------------------------||
/*313 rpm max 63 mm diamètre roue
 * donc vitesse mm s = 63 *PI * 313 / 60 = 1039.5 mm/s
 * donc 50 % vmax = 519.75 mm/s
 *     10 % vmax = 10.395 mm/s
 *     5 % de 255 = 13 et 50 % de 255 = 128*/
