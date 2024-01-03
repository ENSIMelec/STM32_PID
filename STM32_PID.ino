#include "FastInterruptEncoder.h"
#include "DiscretePID.h"

Encoder encDroit(PA0, PA1, SINGLE, 250); // - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only
Encoder encGauche(PB4, PB5, SINGLE, 250);

float cmd_vitesse_G = 0; //commande vitesse moteur gauche
float cmd_vitesse_D = 0; //commande vitesse moteur droite

float dt = 10e-3; // 10ms

float vitesse_G = 0; //vitesse réel moteur gauche
float vitesse_D = 0; //vitesse réel moteur droite

DiscretePID pid_vitesse_G(dt, 2, 10, 0); //Classe permettant le calcul du PID vitesse moteur gauche
DiscretePID pid_vitesse_D(dt, 2, 10, 0); //Classe permettant le calcul du PID vitesse moteur droite

float PID_vitesse_G = 0; // Valeur sortante du PID vitesse moteur gauche qui va donc etre injecté dans celui ci
float PID_vitesse_D = 0; // Valeur sortante du PID vitesse moteur droit qui va donc etre injecté dans celui ci


bool useSimulation = false;
  #include "SimFirstOrder.h"
  DiscretePID Sim_pid_vitesse_G(dt, 2, 10, 0);
  float Sim_PID_vitesse_G= 0;
  float process_sim = 0;
  float K_Motor_G = 1;
  float tau_Motor_G = 1;
  SimFirstOrder FirstOrder_G(dt, tau_Motor_G, K_Motor_G); // first order system for motor Gauche
  SimFirstOrder simFirstOrder(dt, tau_Motor_G, K_Motor_G); // first order system simulation of motor Gauche

void setup()
{
  Serial.begin(115200);

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

  TIM_TypeDef *Instance = TIM4;
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(1/dt, HERTZ_FORMAT); // 100 Hz
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}

void Update_IT_callback(void)
{
  encDroit.loop();
  encGauche.loop();
  Serial.print("Droit: ");
  Serial.println(encDroit.getTicks());
  Serial.print("Gauche: ");
  Serial.println(encGauche.getTicks());

  if (useSimulation)
    {
    // Utilisation de la simulation SimFirstOrder qui donc simule le comportement du moteur Gauche selon la consigne donnée
    process_sim = simFirstOrder.process(cmd_vitesse_G);
    Serial.print("Simu : ");
    Serial.println(process_sim,5);
    // Ici c'est la simulation du comportement de mon PID sur le moteur Gauche car ducoup je donne mon erreur lié à la différence entre mon PID et la consigne que j'injecte dans la simulation de mon moteur ce qui me donne ce que fait théoriquement mon moteur
    Sim_PID_vitesse_G = FirstOrder_G.process(Sim_pid_vitesse_G.process(cmd_vitesse_G - Sim_PID_vitesse_G));
    Serial.print("Sim_PID_V_G : ");
    Serial.println(PID_vitesse_G,5);
  }
  vitesse_G = encGauche.getTicks()/dt;
  vitesse_D = encDroit.getTicks()/dt;
  Serial.print("V_G : ");
  Serial.println(vitesse_G,5);
  Serial.print("V_D : ");
  Serial.println(vitesse_D,5);

  PID_vitesse_G = pid_vitesse_G.process(cmd_vitesse_G - vitesse_G);
  PID_vitesse_D = pid_vitesse_D.process(cmd_vitesse_D - vitesse_D);
  Serial.print("PID_V_G : ");
  Serial.println(PID_vitesse_G,5);
  Serial.print("PID_V_D : ");
  Serial.println(PID_vitesse_D,5);
  
}

void loop() {}