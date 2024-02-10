
#include "PID.h"                  // bibliothèque PID
#include "FastInterruptEncoder.h" // bibliothèque pour les codeurs incrémentaux
#include "SimFirstOrder.h"        // bibliothèque pour la simulation du moteur

/******Pin********/
#define PWM1 = PB6 // PWM4/1 pin D10 donc le Timer4
#define DIR1 = PC1 // pin A4
#define PWM2 = PA8 // PWM1/1 pin D7 donc le Timer1
#define DIR2 = PC0 // pin A5
/******************/

/******Mode********/
#define DEBUG // mode debug
/******************/
unsigned long timeSetup;

/******ECHANTILLONAGE********/
float dt = 10e-3; // 10ms
bool Update_IT = false;
bool Dinverse = false;
bool Ginverse = false;
/****************************/

/******CONSIGNES PID**********/
float cmd_vitesse_G = 0; // commande vitesse moteur gauche en mm/ms
float cmd_vitesse_D = 0; // commande vitesse moteur droite en mm/ms
float cmd_angle = 0;     // commande angle
float cmd_distance = 0;  // commande distance
/*****************************/

/***********Etalonnage Encodeur 1m******/
const float distance_encoder_gauche = PI * 35 / 512; // 1000/4991;
const float distance_encoder_droit = PI * 35 / 512;  // 1000/4715;
/**************************************/

/********Coef Vitesse ******/
float VitesseOutMax = 1039.5; // Vitesse max théorique du moteur en mm/s
const float coefToPWM = 255 / VitesseOutMax;
const float coefVitesseG = distance_encoder_gauche * coefToPWM / dt;
const float coefVitesseD = distance_encoder_droit * coefToPWM / dt;
/**************************/

/********Coef Angle****/
const float empattementRoueCodeuse = 239;
const float coefAngle = 1 / empattementRoueCodeuse;
/**********************/

/******COEFICIENTS PID************/
float Kp_G = 1, Ki_G = 0, Kd_G = 0;                      // coefficients PID vitesse moteur gauche
float Kp_D = 1.1, Ki_D = 0, Kd_D = 0;                    // coefficients PID vitesse moteur droit
float Kp_angle = 0, Ki_angle = 0, Kd_angle = 0;          // coefficients PID angle
float Kp_distance = 0, Ki_distance = 0, Kd_distance = 0; // coefficients PID distance
/*********************************/

/******Declaration des codeurs************/
// TODO : faire 1 metre avec le robot a la main pour voir combien de tick on fait les codeurs
// et les étalonner

// - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only
Encoder encGauche(PA0, PA1, TIM2, SINGLE, 250); // PWM2/1 pin A0 et PWM2/2 pin A1 Donc Timer 2 utilisé
Encoder encDroit(PB5, PB4, TIM3, SINGLE, 250);  // PWM3/1 pin D5 et PWM3/2 pin D4 Donc Timer 3 utilisé
/***************************************/

/*****Sauvegarde des positions*****/
int16_t last_encGauche = 0;
int16_t last_encDroit = 0;
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
/*****FONCTION LECTURE SANS BLOCAGE***/
/*************************************/
String nonBlockingReadStringUntil(char terminator)
{
  String result = "";
  while (Serial.available() > 0)
  {
    char c = Serial.read();
    if (c == terminator)
    {
      break;
    }
    result += c;
  }
  return result;
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*****FONCTION RÉCÉPTION DONNÉES******/
/*************************************/
void serialEvent()
{
  String input = nonBlockingReadStringUntil('\n');
  switch (input[0])
  {
  case 'C':
    sscanf(input.c_str(), "C%f:%f", &cmd_vitesse_G, &cmd_vitesse_D);
    break;
  default:
    Serial.println("NC");
    break;
  }
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*****FONCTION ÉCHANTILLONAGE*********/
/*************************************/
void Update_IT_callback(void)
{
  /****Récupération des valeurs des codeurs****/
  int16_t ticks_G = (encGauche.getTicks());
  int16_t ticks_D = (encDroit.getTicks());
  /********************************************/

  /****Calcul des vitesses des moteurs*******/
  if (Dinverse)
    vitesse_D = -(float)(ticks_D - last_encDroit) * coefVitesseD;
  else
    vitesse_D = (float)(ticks_D - last_encDroit) * coefVitesseD;
  if (Ginverse)
    vitesse_G = -(float)(ticks_G - last_encGauche) * coefVitesseG;
  else
    vitesse_G = (float)(ticks_G - last_encGauche) * coefVitesseG;
  /******************************************/

  /****Calcul de l'angle et de la distance*******/
  angle += (ticks_G - ticks_D) * coefAngle;
  // distance = (vitesse_D + vitesse_G) / 2 * dt;
  /*********************************************/

  /*****Calul de PID Angle et Vitesse****/
  // PID_angle.Compute();
  //  PID_distance.Compute();
  /*************************************/

  /***Ajustement Commandes Vitesse****/
  // cmd_vitesse_G = Output_PID_angle + Output_PID_distance;
  // cmd_vitesse_D = -Output_PID_angle + Output_PID_distance;
  /***********************************/

  /****Calcul des PID Vitesse*******/
  PID_vitesse_G.Compute();
  PID_vitesse_D.Compute();
  /*********************************/

  /****Commande des moteurs*******/
  analogWrite(PB6, Output_PID_vitesse_G);
  analogWrite(PA8, Output_PID_vitesse_D);
  /*****************************/

  /****Sauvegarde des positions*****/
  last_encGauche = ticks_G;
  last_encDroit = ticks_D;
  /********************************/
  Update_IT = true;
}
/*************************************/
/*************************************/
/*************************************/

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

  /*******Correction de la direction*******/
  Serial.println("Etalonnage ecodeur");
  while (encGauche.getTicks() > -255 && encGauche.getTicks() < 255 || encDroit.getTicks() > -255 && encDroit.getTicks() < 255)
    ;
  if (encGauche.getTicks() < 0)
    encGauche.setInvert(true);
  if (encDroit.getTicks() < 0)
    encDroit.setInvert(true);
  pinMode(LED_BUILTIN, OUTPUT);    // Configure la broche de la LED comme sortie
  digitalWrite(LED_BUILTIN, HIGH); // Allume la LED
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
  // Serial.println("Etalonnage ecodeur");
  // while (encDroit.getTicks() > -200 && encDroit.getTicks() < 200)
  //   ;
  // if (encDroit.getTicks() < -200)
  //   encDroit.setInvert(true);
  // pinMode(LED_BUILTIN, OUTPUT);    // Configure la broche de la LED comme sortie
  // digitalWrite(LED_BUILTIN, HIGH); // Allume la LED

  /****************************/
  /****************************/
#endif

  /******Initialisation des PINs****/
  pinMode(A4, OUTPUT);  // PA_3 = pin D0
  pinMode(A3, OUTPUT);  // PA_2 = pin D1
  pinMode(PB6, OUTPUT); // PWM4/1 pin D10 donc le Timer4
  pinMode(PA8, OUTPUT); // PWM1/1 pin D7 donc le Timer1
  // encGauche.setInvert(); // Inverser le sens de rotation du codeur
  /*********************************/

  /******Configuration des moteurs************/
  digitalWrite(A4, HIGH);
  digitalWrite(A3, HIGH);
  /*******************************************/
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
  timeSetup = millis();
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*****LOOP****************************/
/*************************************/
void loop()
{
  if (Update_IT)
  {
    Serial.print("A"); // Valeur du codeur Gauche
    Serial.println(last_encGauche);
    Serial.print("B"); // Valeur du codeur Droit
    Serial.println(last_encDroit);
    Serial.print("C"); // Vitesse réel moteur Gauche
    Serial.println(vitesse_G, 5);
    Serial.print("D"); // Vitesse réel moteur Droit
    Serial.println(vitesse_D, 5);
    Serial.print("E"); // Sortie du PID vitesse moteur Gauche
    Serial.println(Output_PID_vitesse_G, 5);
    Serial.print("F"); // Sortie du PID vitesse moteur Droit
    Serial.println(Output_PID_vitesse_D, 5);
    Serial.print("G"); // Consigne de vitesse moteur Gauche
    Serial.println(cmd_vitesse_G, 5);
    Serial.print("H"); // Consigne de vitesse moteur Droit
    Serial.println(cmd_vitesse_D, 5);
    // Serial.print("I"); // Consigne de vitesse moteur Droit
    // Serial.println(angle);

    Update_IT = false;
  }
  unsigned long time = millis() - timeSetup; // Temps écoulé en millisecondes

  if (time >= 0 && time < 5000)
  {
    digitalWrite(A4, HIGH);
    Ginverse = false;
    // Mettre la commande moteur à 10% de la Vmax après 10 secondes
    PID_vitesse_G.SetMode(AUTOMATIC); // turn the PID on
    PID_vitesse_D.SetMode(AUTOMATIC); // turn the PID on
    cmd_vitesse_G = 100;
    cmd_vitesse_D = 100;
  }
  else if (time >= 5000 && time < 9300)
  {
    digitalWrite(A3, LOW);
    Dinverse = true;
    cmd_vitesse_D = 40;
    cmd_vitesse_G = 40;
  }
  else if (time >= 9300 && time < 14300)
  {
    digitalWrite(A3, HIGH);
    Dinverse = false;
    // Mettre la commande moteur à 10% de la Vmax après 10 secondes
    PID_vitesse_G.SetMode(AUTOMATIC); // turn the PID on
    PID_vitesse_D.SetMode(AUTOMATIC); // turn the PID on
    cmd_vitesse_G = 100;
    cmd_vitesse_D = 100;
  }
  else if (time >= 14300 && time < 18300)
  {
    digitalWrite(A4, LOW);
    Ginverse = true;
    cmd_vitesse_D = 40;
    cmd_vitesse_G = 40;
  }
  else if (time >= 18500)
  {
    timeSetup = millis();
  }
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
