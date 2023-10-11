# LAB BOOK projet Guerle-docking

## Octobre 2023

### Mercredi 11



### Mardi 10

Hugo & Théo
- Création node de rassemblement et publication des datas, création roslaunch, tests capteurs. roslaunch complet crée pour le dock, reste a gerer les logs et les nodes sur le bateau.

Kevin & Guillaume
- Attention, uniquement sur Windows. Pour avoir la correction RTK. aller sur le site de [centipède](https://docs.centipede.fr/docs/base/). L'idée est de leur fournir un log de 24h d'acquisition de données GPS de notre base, et ils vont calculer la position RTK au centimètre près. Attention à bien brancher l'usb sur le port "Power+GPS" du module GPS et pas sur le port "Power+Xbee"

Guillaume
- J'ai suivi les indications d'installation de centipède. Pour l'instant je n'ai fait que les étapes installation logiciel et paramétrage. A ce stade, la base GPS reçoit des signaux satellites et commence un enregistrement continu de données. Un archivage est fait tous les jours à 4h des dernières 24h de données. Il nous faut donc attendre jeudi matin pour disposer de notre position avec la meilleure position (ce qui se fait avec les dernières étapes données par centipède qui restent à faire). Attention, lors de l'installation de la base, il faut flasher la carte micro SD à chaque fois avant d'alimenter une nouvelle fois la raspberry. De plus, il faut connecter le module GPS en USB à la raspberry et non pas au PC réseau (apparemment). D'ailleurs j'ai choisi comme nom de base RTK le mot GHMR (référence évidente à nos itiniales).

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

