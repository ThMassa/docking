# LAB BOOK projet Guerle-docking

## Octobre 2023

### Lundi 9

Hugo & Guillaume : 
- Tests des modems *Simpulse* : connexions master/slave et pings entre différents ordinateurs connectés sur le réseau. Il faut configurer manuellement l'adresse IP de son ordinateur de sorte à pouvoir se connecter :
    1. Accéder aux paramètres de connexion filaire
    2. Cliquer sur *propriétés*
    3. Onglet *IPV4*
    4. Entrer manuellement l'adresse IP souhaitée (ex: 192.168.0.10) ainsi que le masque de sous-réseau 255.255.255.0
- *Hello_world* : package ROS de base pour tester les publishers et subscribers. Le fichier *Hello_UDP.py* montre un exemple d'implémentation d'un broadcast UDP à l'intérieur d'un node ROS. Le script *hello_receiver_UDP.py* montre le côté receveur.

Kévin & Théo
- Prise en main de la Jetson Nano, installation des drivers de la centrale inertielle, mise en place du GPS, debugage de l'installation ROS (préciser *ROS_LOCALHOST=localhost* et *ROS_MASTERID=qqchose*)

Guillaume & Kevin
- Installation d'Ubuntu 18.04 sur la Raspberry Pi et installation des drivers GPS

### Mardi

Hugo & Théo
- Création node de rassemblement et publication des datas, création roslaunch, test