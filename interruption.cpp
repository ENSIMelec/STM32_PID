#include "interruption.h"

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
    angle += (vitesse_G + vitesse_D) * coefAngle;
    // distance = (vitesse_D + vitesse_G) / 2 * dt;
    /*********************************************/

    /*****Calul de PID Angle et Vitesse****/
    PID_angle.Compute();
    PID_distance.Compute();
    /*************************************/

    /***Ajustement Commandes Vitesse****/
    cmd_vitesse_G = Output_PID_angle; //+ Output_PID_distance;
    cmd_vitesse_D = Output_PID_angle; //+ Output_PID_distance;
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
