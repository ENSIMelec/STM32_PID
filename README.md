# Code Asservissement (ENSIM'ELEC 2024)
## Introduction
Ce code est créé pour les cartes STM32 NUCLEO-64 F446RE/F411RE. Il gére tous les déplacements et asservissements du robot ainsi que son odométrie. Il a été réalisé via Arduino grâce à la librarie STM32duino installable comme ceci: [STM32duino](https://github.com/stm32duino/Arduino_Core_STM32/wiki/Getting-Started).

Nous avons choisi de la faire sur une carte embarqué car un traitement temps réel est important pour les PID (asservissement), c'est pour cela que nous avons décidé de géré l'intégrité des déplacement sur un MCU. 
Une communication série est utilisé afin de communiqué avec un système externe qui lui donnera des ordres de déplacement.  
En règle général notre code fonctionne via des interruptions.

**PS**: si vous souhaiter reprendre ce projet, je cous conseille de le convertir en projet plateformIO qui est un plugin à VScode permettant de programmé du code Arduino mais de mannière plus profesionnelle et simple pour de gros projet comme celui-ci. 

## Schèma de fonctionnement
![schéma du système](./img/schema_PID.png) 
Notre  système est un asservissement en cascade. En effet, les asservissement d'angle et de déplacement contrôlent les asservissemnts de vitesse qui eux même contrôlent la vitesse des moteurs. Le but est de pouvoir indiqué un angle et une distance que ces asservissement vont essayer de garder. Par exemple si le robot parcour une distance trop élevé il retourne en arrière pour rester sur sa bonne position. L'asservissement vitesse permet d'obtenir une vitesse similaire entre les deux moteur. En effet, si un moteur doit supporter une charge plus grande que l'autre, sa consigne doit donc être plus élevé pour que leur vitesse réel soit identique. 
Au niveau de l'odométrie les calculs sont assez simple, pour obtenir des coordonées polaire. la distance parcourue par chaque roue est obtenue par les roues codeuse.

## Encodeurs 
(réf: Bourns ENS1J-B28-L002 )  
Les encodeurs utilisé sont des encodeurs de quadratures. Ce type d'encodeur requiert sur un système classique des interruptions afin de compter le nombre de tick sur chaque roues ([petit tuto sur arudino](https://www.locoduino.org/spip.php?article82) pour mieux comprendre).
Cependant dans notre cas pour optimiser le systèmes au lieu d'utiliser des interruptions, nous avons délégué cette tâche au TIMER, d'ou l'importance que les encodeurs soit de quadratures.  

**ATTENTION**   
lors de la coupe nous avons rencontré un problème avec le débordement du TIMER ce cas n'avait malheuresmsent pas été étudié. Il a été néanmmoins partiellement réglé afin de rendre le robot fonctionelle mais à améliorer.

## PID

### C'est quoi ?
Un PID pour Proportionelle Intégral Dérivation est un ensemble d'algorithme dont le but est de corrigé et/ou de limité l'erreur entre ce qui est mesuré et ce qui est demandé. Le PID va de lui même contrôler la commande à cette fin. 
Le PID forme donc une boucle entre l'entrée et la sortie, on peut le remarquer sur le schèma d'ensemble du système. Pour minimiser l'erreur vous avez 3 types de coeficient (Ki, Kp, Kd).
#### Proportionelle Kp
Le coeficient proportionelle permet de minimiser l'erreur mais gardera toujours une erreur constante. Plus on l'augmente plus l'erreur constante diminue mais plus il va rendre le système instable. A un certain moment il rend le système tellement instable qu'il ne fait qu'osciller. Dans notre cas si on prends le PID qui doit contrôler la position le robot va aller en avant puis en arrière sans jamais atteindre sa postion. Il faut donc trouver la valeur limite qui va lui permettre soit d'obtenir la valeur limite pour qu'il n'y ai pas d'oscilation ou soit pour qu'il y en ai qu'une seule. Cela dépendra de ce que l'on veut faire.

#### Intégral Ki
Ce coeficient permet de rendre l'erreur complétement nulle. Si la valeur est trop faible cela prendra un temps très long pour corriger l'erreur et s'il est trop élevé cela va engendré des dépassement soit des oscillation suplémentaire. Le but de se coeficient est d'atteindre la consigne le plus rapidement possible.

#### Dérivé Kd
Il permet de limité les effets de l'intégral au niveau des dépassement. Cependant si les courbes ne sont pas très belle et on des sauts la dérivé va engendré des instabilité il faut donc en mettre avec parcimonie.

### Comment on règle ?
On commence par faire le coeficient de proportionalité, on l'augmente jusqu'à ce qu'on atteigne l'instabilité et on le règle un peu plus pas que l'instabilité. Ensuite on fait l'intégral, on en ajoute pour que notre erreur soit la plus petite possible sans obtenir trop d'instabilité. Enfin on peu paufiné avec de la dérivé.
Globalement ça se fait via du test essai.  
  
Pour mieux comprendre les PID allez lire [cette article](https://www.fujielectric.fr/blog/regulation-pid-pour-les-nuls-tout-ce-que-vous-devez-savoir/) qui permet de mieux visualiser les expications.

### Les PID dans le programme
Il y a en tout 4 PID: Vitesse droit et gauche, distance, angle.
Le PID d'angle permet de géré l'angle du robot si on lui donne une consigne d'angle, il va calculer en sortie la vitesse que le robot doit avoir pour atteindre l'angle. Dans notre cas les angles sont formé grâce à la sortie du PID en vitesse positive sur une roue et de l'autre 
négative. Le PID angle contrôle donc la consigne que l'on souhaite obtenir dans les PID vitesse. Le PID distance fonctionne excatement de la même manière que le PID angle mais les deux vitesse sont cette fois-ci positive.  
Les PID Vitesse on pour but de géré la vitesse de chaque roue pour qu'elle aille à la même vitesse. En effet, si j'ajoute plus de poid sur une roue que sur l'autre, une roue ira plus vite que l'autre. Notre PID sert donc à limiter cela.


### **Remarque** 
Le PID angle est mal réglé avec certainement un Ki trop élévé c'est pour cela que le robot oscille en déplacement linéaire.

## Odométrie
Le but de l'odométrie est de calculer la position du robot à chaque moment. Pour cela, on utilise des équations que l'on doit ajuster.  
**Remarque**  
L'odométrie est très mal réglé donnant des coordonées complétement incohérentes.

## Courbe d'accélération
Pour comprendre le fonctionnement lancez et lisez le programme **Simulation_courbe.py**. Il permet de comprendre facilement comment les courbes sont réaliser. Sachant que ce programme ce focalise sur la distance mais il s'agit de la même chose mais en angulaire pour l'angle.

## Comunication Série
La liason série fonctionne avec un cable USB et elle est défini à 115200 bauds. Elle fonctionne via l'interruption sur la liason série.  

**ATTENTION**  
nous avons eu plusieurs problèmes de transimissions et de vitesse de transmission avec la liaison USB série. Je vous conseille d'essayer la liasons série via les I/O qui devrait être plus fiable qu'avec un cable USB. Il faudra donc revoir l'électronique.

#### Exemple de commandes
```plaintext
asserv debug [enable|disable]
asserv recalage [1|0]
asserv [enable|disable|reset] [all|angle|distance]
asserv [enable|disable|reset] vitesse [all|gauche|droite]
asserv set coord [x] [y]
asserv set angle [angle]
asserv set position [x] [y] [angle]
asserv goto [x] [y] [speed]
asserv rotate [angle]
asserv moveof [distance] [speed]
asserv stopmove
asserv restartmove
```