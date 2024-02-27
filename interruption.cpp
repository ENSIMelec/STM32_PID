#include "interruption.h"
#include "Odometrie.h"

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
    vitesse_D = (float)(ticks_D - last_encDroit) * coefVitesseD;
    vitesse_G = (float)(ticks_G - last_encGauche) * coefVitesseG;
    /******************************************/

    /****Calcul de l'angle et de la distance*******/
    angle += (vitesse_G + vitesse_D) * coefAngle;
    distance = (vitesse_D + vitesse_G) / 2 * dt;
    /*********************************************/

    /****Calcul de la position*******/
    update_Position(distance, angle);
    /*******************************/

    /*****Calul de PID Angle et Vitesse****/
    PID_angle.Compute();
    PID_distance.Compute();
    /*************************************/

    /***Ajustement Commandes Vitesse****/
    // cmd_vitesse_G = Output_PID_angle + Output_PID_distance;
    // cmd_vitesse_D = Output_PID_angle + Output_PID_distance;
    /***********************************/

    /****Calcul des PID Vitesse*******/
    PID_vitesse_G.Compute();
    PID_vitesse_D.Compute();
    /*********************************/

    digitalWriteFast(DIR1, Output_PID_vitesse_D < 0 ? LOW : HIGH);
    digitalWriteFast(DIR2, Output_PID_vitesse_G < 0 ? LOW : HIGH);

    /****Commande des moteurs*******/
    analogWrite(PWM1, Output_PID_vitesse_G);
    analogWrite(PWM2, Output_PID_vitesse_D);
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
