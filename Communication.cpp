#include <Arduino.h>
#include "main.h"
#include "Odometrie.h"

char inputBuffer[1024] = "\0"; // Buffer pour stocker les données entrantes
int sizeBuffer = 0;			   // Taille du buffer

/*************************************/
/******FONCTION AFFICHAGE USAGE*******/
/*************************************/
void printUsage()
{
	char printUsage[2048] = "Usage : \r\n - asserv wheelcalib \r\n - asserv enablemotor [0|1] \r\n - asserv enablepolar [0|1] \r\n - asserv coders \r\n - asserv reset \r\n - asserv motorspeed [r|l] [speed] \r\n -------------- \r\n - asserv wheelspeedstep [r|l] [speed] [step time] \r\n -------------- \r\n - asserv robotfwspeedstep [speed] [step time] \r\n - asserv robotangspeedstep [speed] [step time] \r\n - asserv speedcontrol [r|l] [Kp] [Ki] \r\n - asserv angleacc delta_speed \r\n- asserv distacc delta_speed \r\n ------------------- \r\n - asserv addangle angle_rad \r\n - asserv anglereset \r\n - asserv anglecontrol Kp \r\n------------------- \r\n - asserv adddist mm \r\n - asserv distreset \r\n - asserv distcontrol Kp \r\n -------------- \r\n - asserv addgoto X Y \r\n - asserv gototest \r\n -------------- \r\n - asserv pll freq \r\n";
	Serial.print(printUsage);
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/********FONCTION INTERPRÉTEUSE*******/
/*************************************/
void asservCommandUSB(int argc, char **argv)
{
	Serial.print("asserv ");
	for (int i = 0; i < argc; i++)
	{
		Serial.print(argv[i]);
		Serial.print(" ");
	}
	Serial.println(" ");
	if (argc == 0)
	{
		printUsage();
		return;
	}
	else if (!strcmp(argv[0], "wheelspeedstep"))
	{
		char side = *argv[1];
		float speedGoal = atof(argv[2]);
		int time = atoi(argv[3]);

		// float speedRight = speedGoal;
		// float speedLeft = 0;
		// if (side == 'l')
		// {
		// 	speedLeft = speedGoal;
		// 	speedRight = 0;
		// }

		// mainAsserv->setWheelsSpeed(speedRight, speedLeft);
		// chThdSleepMilliseconds(time);
		// mainAsserv->setWheelsSpeed(0, 0);
	}
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/**FONCTION CONVERSION EN ARGC ARGV***/
/*************************************/
void usbSerialCallback(char *buffer, uint32_t size)
{
	if (size > 0)
	{
		/*
		 *  On transforme la commande recu dans une version argv/argc
		 *    de manière a utiliser les commandes shell déjà définie...
		 */
		bool prevWasSpace = false;
		char *firstArg = buffer;
		int nb_arg = 0;
		char *argv[10];
		for (uint32_t i = 0; i < size; i++)
		{
			if (prevWasSpace && buffer[i] != ' ')
			{
				argv[nb_arg++] = &buffer[i];
			}
			if (buffer[i] == ' ' || buffer[i] == '\r' || buffer[i] == '\n')
			{
				prevWasSpace = true;
				buffer[i] = '\0';
			}
			else
			{
				prevWasSpace = false;
			}
		}

		// On évite de faire appel au shell si le nombre d'arg est mauvais ou si la
		// 1ière commande est mauvaise...
		if (nb_arg > 0 && !strcmp(firstArg, "asserv"))
		{
			asservCommandUSB(nb_arg, argv);
		}
		else
		{
			printUsage();
		}
	}
}
/*************************************/
/*************************************/
/*************************************/

/*************************************/
/*****FONCTION RÉCÉPTION DONNÉES******/
/*************************************/
void serialEvent()
{
	char c = Serial.read();
	if (c == '\n' || c == '\r' || c == '\0')
	{
		usbSerialCallback(inputBuffer, sizeBuffer);
		memset(inputBuffer, '\0', sizeBuffer); // On vide le buffer
		sizeBuffer = 0;
	}
	else
	{
		inputBuffer[sizeBuffer] = c;
		sizeBuffer++;
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
	Serial.println(angle, 5);
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
	Serial.print("P"); // angle ok
	Serial.println(angle_ok);
	Serial.print("Q"); // distance ok
	Serial.println(distance_ok);
	Serial.print("X"); // position x
	Serial.println(x);
	Serial.print("Y"); // position y
	Serial.println(y);

	Update_IT = false;
}
/*************************************/
/*************************************/
/*************************************/
