#include <Arduino.h>
#include "main.h"
#include "Odometrie.h"
#include "Move.h"
#include "interrupt.h"

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
	// On vérifie le nombre d'argument
	if (argc == 0)
	{
		printUsage();
		return;
	}
	else if (!strcmp(argv[0], "enable"))
	{
		if (argv[1] == "all")
		{
			change_PID_mode(4); // enable de tout l'asservissement
		}
		else if (argv[1] == "angle")
		{
			// enable de l'asservissement d'angle
		}
		else if (argv[1] == "distance")
		{
			// enable de l'asservissement de distance
		}
		else if (argv[1] == "vitesse")
			{
				if (argv[2] == "all")
				{
					// enable de l'asservissement de vitesse droite et gauche
				}
				else if (argv[2] == "gauche")
				{
					// enable de l'asservissement de vitesse gauche
				}
				else if (argv[2] == "droite")
				{
					// enable de l'asservissement de vitesse droite
				}
			}
	}
	else if (!strcmp(argv[0], "disable"))
	{
		if (argv[1] == "all")
		{
			// disable de tout l'asservissement
		}
		else if (argv[1] == "angle")
		{
			// disable de l'asservissement d'angle
		}
		else if (argv[1] == "distance")
		{
			// disable de l'asservissement de distance
		}
		else if (argv[1] == "vitesse")
			{
				if (argv[2] == "all")
				{
					// disable de l'asservissement de vitesse droite et gauche
				}
				else if (argv[2] == "gauche")
				{
					// disable de l'asservissement de vitesse gauche
				}
				else if (argv[2] == "droite")
				{
					// disable de l'asservissement de vitesse droite
				}
			}
	}
	else if (!strcmp(argv[0], "reset"))
	{
		if (argv[1] == "all")
		{
			ARU_interrupt();
		}
		else if (argv[1] == "angle")
		{
			angle = 0; // reset de l'asservissement d'angle
		}
		else if (argv[1] == "distance")
		{
			reset_distance(); // reset de l'asservissement de distance
		}
		else if (argv[1] == "vitesse"):
			{
				if (argv[2] == "all")
				{
					// reset de l'asservissement de vitesse droite et gauche
				}
				else if (argv[2] == "gauche")
				{
					// reset de l'asservissement de vitesse gauche
				}
				else if (argv[2] == "droite")
				{
					// reset de l'asservissement de vitesse droite
				}
			}
	}
	else if (!strcmp(argv[0], "goto"))
	{
		if (argc < 3)
		{
			return;
		}
		float x = atof(argv[1]);
		float y = atof(argv[2]);
		float speed;
		if (argc > 4)
		{
			speed = atof(argv[3]);
		}
		else
		{
			speed = 500;
		}
		newCommand = calculateMovement(x, y);
		if (argc > 5)
		{
			newCommand.recalage = true;
		}
		goTo(newCommand, speed);
		newCommand.goto_ok = true;
	}
	else if (!strcmp(argv[0], "rotate"))
	{
		float angle_ = atof(argv[1]);
		newCommand = calculate_rotation(angle_);
		rotate(newCommand, 500);
		newCommand.rotate_ok = true;
	}
	else if (!strcmp(argv[0], "moveof"))
	{
		if (argc < 2)
		{
			return;
		}
		float distance_ = atof(argv[1]);
		float speed;
		if (argc > 3)
		{
			speed = atof(argv[2]);
		}
		else
		{
			speed = 500;
		}
		newCommand = calculate_moveOf(distance_);
		if (argc > 4)
		{
			newCommand.recalage = true;
		}
		moveOf(newCommand, speed);
		newCommand.goto_ok = true;
	}
	else if (!strcmp(argv[0], "stopmove"))
	{
		obstacle_detection();
	}
	else if (!strcmp(argv[0], "restartmove"))
	{
		after_obstacle_detection();
	}
	else if (!strcmp(argv[0], "setxy"))
	{
		if (argc < 2)
		{
			return;
		}
		x = atof(argv[1]);
		y = atof(argv[2]);
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