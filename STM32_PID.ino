#include "PID.h"

float dt = 10e-3; // 10ms

float cmd_vitesse_G = 10; //commande vitesse moteur gauche / consigne
float cmd_vitesse_D = 10; //commande vitesse moteur droite / consigne

float Kp_G = 2, Ki_G = 10, Kd_G = 0; //coefficients PID vitesse moteur gauche
float Kp_D = 2, Ki_D = 10, Kd_D = 0; //coefficients PID vitesse moteur droit

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
  Serial.begin(115200); // Par défaut utilisation de USART1 donc attention aux pin avec du UART1

  #ifdef useSimulation
    Sim_PID_vitesse_G.SetMode(AUTOMATIC); //turn the PID on
  #else
    #ifdef DEBUG
      if (encDroit.init() && encGauche.init())
      {
        Serial.println("Encoder Initialization OK");
      }
      else
      {
        Serial.println("Encoder Initialization Failed");
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
    PID_vitesse_G.SetMode(AUTOMATIC); //turn the PID on
    PID_vitesse_D.SetMode(AUTOMATIC); //turn the PID on

    digitalWrite(PA3,LOW); // PA_3 = pin D0
    digitalWrite(PA2,LOW); // PA_2 = pin D1
  #endif

  // // Configuration de l'interruption pour déclencher sur la réception de données
  // USART1->CR1 |= USART_CR1_RXNEIE;
  // NVIC_EnableIRQ(USART1_IRQn);

  TIM_TypeDef *Instance = TIM6; 
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(1/dt, HERTZ_FORMAT); // 100 Hz -> 10ms
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}

// extern "C" void USART1_IRQHandler() {
//   if (USART1->SR & USART_SR_RXNE) {
//     char receivedData = USART1->DR;

//     // Afficher la donnée reçue
//     Serial.println(receivedData);

//     // Autres actions...
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

    vitesse_G = -(float)(encGauche.getTicks() - last_encGauche)/dt; // d/dt, problème pour le codeur gauche, il est inversé donc quand le robot avance le codeur gauche décrémente tandis que le droit incrémente
    vitesse_D = (float)(encDroit.getTicks() - last_encDroit)/dt;

    PID_vitesse_G.Compute();
    PID_vitesse_D.Compute();

    analogWrite(PB6,Output_PID_vitesse_G); // PWM4/1 pin D10 donc le Timer4
    analogWrite(PA8,Output_PID_vitesse_D); // PWM1/1 pin D7 donc le Timer1
    
    last_encGauche = encGauche.getTicks();
    last_encDroit = encDroit.getTicks();

    #ifdef DEBUG
      Serial.print("Gauche: ");
      Serial.println(encGauche.getTicks());
      Serial.print("Droit: ");
      Serial.println(encDroit.getTicks());
      Serial.print("V_G : ");
      Serial.println(vitesse_G,5);
      Serial.print("V_D : ");
      Serial.println(vitesse_D,5);
      Serial.print("PID_V_G : ");
      Serial.println(Output_PID_vitesse_G,5);
      Serial.print("PID_V_D : ");
      Serial.println(Output_PID_vitesse_D,5);
    #endif
  #endif
}

void loop() {}