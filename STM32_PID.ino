#include "PID.h"

float dt = 10e-3; // 10ms

float cmd_vitesse_G = 0; //commande vitesse moteur gauche / consigne en mm/ms
float cmd_vitesse_D = 0; //commande vitesse moteur droite / consigne en mm/ms

float Kp_G = 6.378, Ki_G = 0.06385, Kd_G = 0; //coefficients PID vitesse moteur gauche
float Kp_D = 5.718, Ki_D = 0.0376, Kd_D = 0; //coefficients PID vitesse moteur droit

#define DEBUG // commenter pour ne pas utiliser le mode debug

// #define useSimulation // commenter pour ne pas utiliser la simulation du moteur;

#ifdef useSimulation
  #include "SimFirstOrder.h"

  float process_sim_motor = 0; // variable qui va stocker la valeur de sortie de la simulation du moteur si on lui donne juste la consigne
  float Output_Sim_PID_vitesse_G= 0; // variable qui stocke la sortie du PID simulé pour le moteur Gauche
  float process_sim_motor_PID = 0; // variable qui va stocker la valeur de sortie de la simulation du moteur part rapport au PID simulé

  float K_Motor_G = 1;
  float tau_Motor_G = 1;
  SimFirstOrder simFirstOrder_G(dt, tau_Motor_G, K_Motor_G); // first order system for motor Gauche
  SimFirstOrder simFirstOrder_G2(dt, tau_Motor_G, K_Motor_G); // first order system simulation of motor Gauche

  PID Sim_PID_vitesse_G(&process_sim_motor_PID, &Output_Sim_PID_vitesse_G, &cmd_vitesse_G, dt, Kp_G, Ki_G, Kd_G, DIRECT);
#else
  #include "FastInterruptEncoder.h"

  int32_t last_encGauche = 0; //int32_t car c'est comme ca dans la librairy des Encoder
  int32_t last_encDroit = 0;
  // TODO : faire 1 metre avec le robot a la main pour voir combien de tick on fait les codeurs et donc comme on sait que 512 ticks font un tour alors on en déduit le diametre des codeurs
  float distance_encoder = 35 * PI / 512; // Constante pour convertir les ticks en mm. On sait que le codeur fait 512 tick pour un tour de roue et que une roue fait 35mm de diamètre
  
  // - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only 
  Encoder encGauche(PA0, PA1, SINGLE, 250); // PWM2/1 pin A0 et PWM2/2 pin A1 Donc Timer 2 utilisé
  Encoder encDroit(PB4, PB5, SINGLE, 250); // PWM3/1 pin D5 et PWM3/2 pin D4 Donc Timer 3 utilisé

  float vitesse_G = 0; //vitesse réel moteur gauche
  float vitesse_D = 0; //vitesse réel moteur droite

  float Output_PID_vitesse_G = 0; // Valeur sortante du PID vitesse moteur gauche, une PMW donc
  float Output_PID_vitesse_D = 0; // Valeur sortante du PID vitesse moteur droit, une PMW donc

  PID PID_vitesse_G(&vitesse_G, &Output_PID_vitesse_G, &cmd_vitesse_G, dt, Kp_G, Ki_G, Kd_G, DIRECT);
  PID PID_vitesse_D(&vitesse_D, &Output_PID_vitesse_D, &cmd_vitesse_D, dt, Kp_D, Ki_D, Kd_D, DIRECT);
#endif

void setup()
{
  Serial.begin(115200); // Par défaut utilisation de USART1 donc attention aux pin avec du USART1
  // USART1->CR1 |= USART_CR1_RXNEIE; // activer l'interruption par caractère spécial
  // USART1->CR1 |= USART_CR1_CMIE; // activer l'interruption par caractère spécial
  // USART1->CR2 |= USART_CR2_LBDIE; // activer l'interruption par caractère spécial
  // USART1->CR2 |= 0x0C; // choisir le caractère spécial, par exemple 0x0C pour le retour chariot
  // NVIC_EnableIRQ(USART1_IRQn); // activer l'interruption par caractère spécial
  #ifdef useSimulation
    Sim_PID_vitesse_G.SetMode(AUTOMATIC); //turn the PID on
  #else
    #ifdef DEBUG
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
    #else
      if (!encDroit.init() && !encGauche.init())
      {
        while (1) //encoder initialization failed
          ;
      }
    #endif

    pinMode(PA3,OUTPUT); // PA_3 = pin D0
    pinMode(PA2,OUTPUT); // PA_2 = pin D1
    pinMode(PB6,OUTPUT); // PWM4/1 pin D10 donc le Timer4
    pinMode(PA8,OUTPUT); // PWM1/1 pin D7 donc le Timer1

    digitalWrite(PA3,HIGH); // PA_3 = pin D0 TODO : fix le pourquoi du comment ca ne marche pas
    digitalWrite(PA2,HIGH); // PA_2 = pin D1

    PID_vitesse_G.SetMode(AUTOMATIC); //turn the PID on
    PID_vitesse_D.SetMode(AUTOMATIC); //turn the PID on
  #endif

  TIM_TypeDef *Instance = TIM6; 
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(1/dt, HERTZ_FORMAT); // 100 Hz -> 10ms
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}

// void serialEvent() {
//   String input = Serial.readStringUntil('\n');
//   if (input.startsWith("C")) { // C20.2930:33.2930 en mm/s
//     float G, D;
//     sscanf(input.c_str(), "C%f:%f", &G, &D);
//     cmd_vitesse_G = G; // on multiplie par dt pour avoir la consigne en mm/ms
//     cmd_vitesse_D = D;
//   }
// }

void Update_IT_callback(void)
{

  #ifdef useSimulation
    // Utilisation de la simulation SimFirstOrder qui donc simule le comportement du moteur Gauche selon la consigne donnée en gros on voit le comportement du moteur sans PID
    process_sim_motor = simFirstOrder_G.process(cmd_vitesse_G);
    Serial.print("Simu : ");
    Serial.println(process_sim_motor,5);
    // Ici c'est la simulation du comportement de mon PID sur le moteur Gauche 
    Sim_PID_vitesse_G.Compute(); // on calcule la sortie du PID
    Serial.print("Output_PID_vitesse_G : ");
    Serial.println(Output_Sim_PID_vitesse_G,5);
    process_sim_motor_PID = simFirstOrder_G2.process(Output_Sim_PID_vitesse_G); // on applique cette sortie sur la simulation du moteur
    Serial.print("Sim_PID_V_G : ");
    Serial.println(process_sim_motor_PID,5);
  #else
    encGauche.loop();
    encDroit.loop();

    vitesse_G = -(float)(encGauche.getTicks() - last_encGauche) * distance_encoder / dt; // d/dt, problème pour le codeur gauche, il est inversé donc quand le robot avance le codeur gauche décrémente tandis que le droit incrémente
    vitesse_D = (float)(encDroit.getTicks() - last_encDroit) * distance_encoder / dt;

    PID_vitesse_G.Compute();
    PID_vitesse_D.Compute();

    analogWrite(PB6,Output_PID_vitesse_G); // PWM4/1 pin D10 donc le Timer4
    analogWrite(PA8,Output_PID_vitesse_D); // PWM1/1 pin D7 donc le Timer1
    
    last_encGauche = encGauche.getTicks();
    last_encDroit = encDroit.getTicks();

    #ifdef DEBUG
      Serial.print("A"); // Valeur du codeur Gauche
      Serial.println(encGauche.getTicks());
      Serial.print("B"); // Valeur du codeur Droit
      Serial.println(encDroit.getTicks());
      Serial.print("C"); // Vitesse réel moteur Gauche
      Serial.println(vitesse_G,5);
      Serial.print("D"); // Vitesse réel moteur Droit
      Serial.println(vitesse_D,5);
      Serial.print("E"); // Sortie du PID vitesse moteur Gauche
      Serial.println(Output_PID_vitesse_G,5);
      Serial.print("F"); // Sortie du PID vitesse moteur Droit
      Serial.println(Output_PID_vitesse_D,5);
      Serial.print("G"); // Consigne de vitesse moteur Gauche
      Serial.println(cmd_vitesse_G,5);
      Serial.print("H"); // Consigne de vitesse moteur Droit
      Serial.println(cmd_vitesse_D,5);
    #endif
  #endif
}
//313 rpm max 
//63 mm diamètre roue
//donc vitesse mm/s = 63 * PI * 313 / 60 = 1039.5 mm/s
//donc vitesse mm/ms = 1039.5 / 1000 = 1.0395 mm/ms
//donc 50 % vmax = 0.51975 mm/ms
//donc 10 % vmax = 0.10395 mm/ms

// 5 % de 255 = 13
// 50 % de 255 = 128
void loop() {
  unsigned long time = millis();  // Temps écoulé en millisecondes
  if (time >= 10000 && time < 11000) {
    // Mettre la commande moteur à 10% de la Vmax après 10 secondes
    cmd_vitesse_G = 0.10395;
    cmd_vitesse_D = 0.10395;
  } else if (time >= 11000 && time < 13000) {
    // Mettre la commande moteur à 50% de la Vmax après 11 secondes
    cmd_vitesse_G = 0.51975;
    cmd_vitesse_D = 0.51975;
  } else {
    // Mettre la commande moteur à 0% de la Vmax avant 10 secondes
    cmd_vitesse_G = 0;
    cmd_vitesse_D = 0;
  }
}