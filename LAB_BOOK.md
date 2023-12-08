# LAB BOOK projet Guerle-docking

## Décembre 2023

### Vendredi 8

Guillaume
- TODO : améliorer la structure de la base gnss (ip66) + trouver un endroit fixe où installer la base gnss + comprendre comment émettre les données gps avec la correction RTK via le module XBEE S2C.

## Octobre 2023

### Jeudi 12

Guillaume
- La commande pour lancer le docker container avec accès au GPS en USB est 

        docker run -it --privileged -e DISPLAY=$DISPLAY -v /dev/ttyACM0:/dev/ttyACM0 --rm name:latest

    J'abandonne l'idée de faire ça dans un conteneur Docker. Je vais installer ros noetic sur mon PC. J'ai suivi les indications de [ArduSimple](https://www.ardusimple.com/how-to-use-ardusimple-rtk-receivers-and-get-gps-data-in-ros/) concernant l'installation de la base RTK. J'ai réussi à installer un workspace ROS noetic avec des nodes permettant d'avoir différentes informations sur le GPS. J'ai également réussi à me connecter depuis l'invite de commande Windows au module GPS en ssh avec :

        ssh basegnss@basegnss.local
        basegnss! # mot de passe

    A partir de ça, j'ai suivi les indications de centipède pour [activer un port série](https://docs.centipede.fr/docs/base/port_serie.html) sur la carte Arduino. J'espère que cela va permettre de lire les données émises sur le(s) port(s) série. Jusqu'ici, je n'avais pas réussi à les décoder sous _putty_. J'ai [téléchargé un des logs de la base RTK](https://docs.centipede.fr/docs/base/positionnement.html) et je l'ai envoyé à IGN pour recevoir sa position plus précisément. Pour docker, il ne faut pas oublier de partager les dossiers nécessaires à notre application pour le conteneur. Depuis Docker Desktop : 

        Parameters >> Resources >> File Sharing



### Mercredi 11

Guillaume
- J'ai récupéré les 5h de logs GPS et je les ai mis sur clé usb. Le mot de passe pour se connecter à http://basegnss.local (sur Windows) est "admin". J'ai soudé des connecteurs sur la carte ardusimple du module GPS pour essayer de streamer les données sur mon PC.

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

