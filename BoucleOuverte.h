#include "main.h"

/* La fonction est à mettre dans la void loop. Elle permet de mettre une vitesse
   sur les moteurs pour vérifier leur vitesse. La vitesse est au début faible afin
   de supprimer toutes les incertitudes du à la mise en mouvement des moteurs.
   En regardant les courbes pour chaque moteur, on peut déterminer l'odre de grandeur
   des valeur de Kp pour chaque moteur.
 */
void Config_PID_Vitesse(void);
