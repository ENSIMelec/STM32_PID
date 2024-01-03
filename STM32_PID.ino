#include "FastInterruptEncoder.h"
#include "PID.h"

Encoder encDroit(PA0, PA1, SINGLE, 250); // - Example for STM32, check datasheet for possible Timers for Encoder mode. TIM_CHANNEL_1 and TIM_CHANNEL_2 only
Encoder encGauche(PB4, PB5, SINGLE, 250);

float dt = 10e-3; // 10ms

float cmd_vitesse_G = 0; //commande vitesse moteur gauche / consigne
float cmd_vitesse_D = 0; //commande vitesse moteur droite / consigne

float Kp_G = 2, Ki_G = 10, Kd_G = 0; //coefficients PID vitesse moteur gauche
float Kp_D = 1, Ki_D = 1, Kd_D = 1; //coefficients PID vitesse moteur droit

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
  float vitesse_G = 0; //vitesse réel moteur gauche
  float vitesse_D = 0; //vitesse réel moteur droite

  float Output_PID_vitesse_G = 0; // Valeur sortante du PID vitesse moteur gauche, une PMW donc
  float Output_PID_vitesse_D = 0; // Valeur sortante du PID vitesse moteur droit, une PMW donc

  PID PID_vitesse_G(&vitesse_G, &Output_PID_vitesse_G, &cmd_vitesse_G, dt, Kp_G, Ki_G, Kd_G, DIRECT);
  PID PID_vitesse_D(&vitesse_D, &Output_PID_vitesse_D, &cmd_vitesse_D, dt, Kp_D, Ki_D, Kd_D, DIRECT);
#endif

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

  #ifdef useSimulation
    Sim_PID_vitesse_G.SetMode(AUTOMATIC); //turn the PID on
  #else
    PID_vitesse_G.SetMode(AUTOMATIC); //turn the PID on
    PID_vitesse_D.SetMode(AUTOMATIC); //turn the PID on
  #endif

  TIM_TypeDef *Instance = TIM4;
  HardwareTimer *MyTim = new HardwareTimer(Instance);
  MyTim->setOverflow(1/dt, HERTZ_FORMAT); // 100 Hz -> 10ms
  MyTim->attachInterrupt(Update_IT_callback);
  MyTim->resume();
}

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
    encDroit.loop();
    encGauche.loop();
    Serial.print("Droit: ");
    Serial.println(encDroit.getTicks());
    Serial.print("Gauche: ");
    Serial.println(encGauche.getTicks());

    vitesse_G = encGauche.getTicks()/dt;
    vitesse_D = encDroit.getTicks()/dt;
    Serial.print("V_G : ");
    Serial.println(vitesse_G,5);
    Serial.print("V_D : ");
    Serial.println(vitesse_D,5);

    PID_vitesse_G.Compute();
    PID_vitesse_D.Compute();
    Serial.print("PID_V_G : ");
    Serial.println(Output_PID_vitesse_G,5);
    Serial.print("PID_V_D : ");
    Serial.println(Output_PID_vitesse_D,5);
  #endif
}

void loop() {}