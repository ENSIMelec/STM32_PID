#include <Arduino.h>
#include "main.h"
#include "Odometrie.h"

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
/*******Envoie des données************/
/*************************************/
void sendData()
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
    Serial.print("I"); // angle mesurer
    Serial.println(angle,5);
    Serial.print("J"); // angle PID
    Serial.println(Output_PID_angle);
    Serial.print("K");
    Serial.println(cmd_angle, 5);
    Serial.print("L"); // distance mesurer
    Serial.println(distance, 5);
    Serial.print("M"); // distance PID
    Serial.println(Output_PID_distance, 5);
    Serial.print("O"); // cmd distance
    Serial.println(cmd_distance, 5);
    Serial.print("X"); // position x
    Serial.println(x);
    Serial.print("Y"); // position y
    Serial.println(y);

    Update_IT = false;
}
/*************************************/
/*************************************/
/*************************************/
