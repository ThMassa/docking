<meta charset="latin-1">

# Docking autonome

### Auteurs
* GARDE Guillaume (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* MASSA Théo (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* HOFMANN Hugo (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* REN Kévin (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).

# Table des matières

- [Docking autonome](#docking-autonome)
    - [Auteurs](#auteurs)
- [Table des matières](#table-des-matières)
  - [Description](#description)
  - [Documentation](#documentation)
  - [Configuration](#configuration)
  - [Requirements](#requirements)
  - [Usage](#usage)
    - [Packages](#packages)
    - [Dock](#dock)
    - [Boat](#boat)
    - [Rover](#rover)
    - [RTK Base](#rtk-base)
  - [Utiliser Docker pour ne pas avoir à installer ROS](#utiliser-docker-pour-ne-pas-avoir-à-installer-ros)
  - [TODO](#todo)

## Description
Le but du projet est d'automatiser le processus de docking d'un drone. Pour cela on doit créer un dock capable d'envoyer sa position GPS et son orientation au drone qui, à partir de ces informations, sera capable de calculer une trajectoire et d'adopter un comportement lui permettant de se docker automatiquement. En plus de cela, il est nécessaire de mettre en place une balise RTK permettant d'obtenir une précision GPS au centimètre.

## Documentation 

<!-- TODO -->
La doc pour la centrale inertielle SBG peut se trouver [ici](https://github.com/SBG-Systems/sbg_ros_driver)

La doc pour le driver GPS sur le dock peut se trouver [ici](https://github.com/KumarRobotics/ublox)

La doc pour le rover est [ici](https://github-docs.readthedocs.io/en/latest/aionio.html)

## Configuration
Toutes les cartes étant sur une installation ubuntu18.04 (ou équivalent), l'ensemble de notre architecture repose sur [ROS Melodic](http://wiki.ros.org/melodic).

## Requirements

<!-- TODO -->

## Usage

### Packages
Pour des raisons pratiques de dépendances ROS différentes (et de capteurs différents) selon qu'on soit sur le dock, le drone ou le rover, il a été décidé de séparer chaque partie en un package ROS différent.

Sur le dock, le package *guerledocking* sera utilisé, sur le drone ce sera *guerleboat* et sur le rover *guerlerover*. Il est important de noter qu'à priori il est impossible de build les packages sur autre chose que l'objet dédié car les dépendances ROS sont spécifiques comme précisé précédemment.

### Dock
Pour utiliser le dock, il suffit de se connecter sur le même réseau (pour l'instant via les modem SIMPULSE) et de se connecter en ssh:

    ssh nvidia@192.168.0.12 
    # MDP : nvidia

Une fois connecté en ssh, il suffit de lancer notre [roslaunch](/catkin_ws/src/guerledocking/launch/launch_dock.launch) :

    roslaunch guerledocking launch_dock.launch

Cette commande lance notre [node ROS](/catkin_ws/src/guerledocking/scripts/node_data_broadcaster.py) ainsi que les drivers du GPS et de la centrale inertielle. Les données GPS et inertielle du dock sont ensuite communiquées via connection UDP au device connecté à l'autre modem. Il y a également une prise de logs de toutes les données GPS et inertielles qui peuvent être utiles à des fins d'analyse.

Pour mettre à jour le package il faut faire:

    scp -r catkin_ws/src/guerledocking nvidia@192.168.0.12:~/catkin_ws/src/ 

Puis dans le ssh :

    cd catkin_ws
    catkin_make

### Boat
Pour utiliser le bateau, plusieurs options s'offrent à nous, principalement pour se connecter dessus. Il est possible de :
- Se connecter sur le wifi local :  M1800_11_SR / MDP: Texys2018
- Se connecter en utilisant la base station : APN_Drone_11 / MDP: IMSolutions2020
- Brancher un dangle WiFi pour être sur le même réseau externe.

Ensuite il suffit de se ssh :

    ssh surcouf@192.168.105.80 # Si wifi local (p-e vérifier adresse IP)

    ssh surcouf@10.0.11.100 # Si base station

Dans tous les cas le mot de passe est : ***surcouf2023***

Une fois connecté, il faut lancer le [roslaunch](/catkin_ws/src/guerleboat/launch/launch_boat.launch) avec differents arguments.

    # Lance tout le système sans logs
    roslaunch guerleboat launch_boat.launch

    # Lance tout le système avec logs
    roslaunch guerleboat launch_boat.launch get_logs:=true

    # Lance le système en simulant le dock (la connexion UDP) en forçant les coordonnées à un point du lac
    roslaunch guerleboat launch_boat.launch simulate_dock:=true

    # Lance le système en simulant le dock (la connexion UDP) en forçant les coordonnées
    roslaunch guerleboat launch_boat.launch simulate_dock:=true latitude:=XXX longitude:=XXX yaw:=XXX

    # Lance le système en simulant le dock (la connexion UDP) grâce à un log précédent
    roslaunch guerleboat launch_boat.launch simulate_dock_with_log:=true bag_played:=chemin/vers/le/bag

Cependant cela n'est pas suffisant. Il faut armer le système mavlink et le mettre en mode *GUIDED*. Pour cela il suffit de faire sur un autre terminal:

    rosrun mavros mavsafety arm
    rosrun mavros mavsys mode -c GUIDED


Pour mettre à jour le package il faut faire (sur sa machine):

    scp -r catkin_ws/src/guerleboat surcouf@10.0.11.100:~/catkin_ws/src/

Puis dans le ssh :

    cd catkin_ws
    catkin_make

### Rover
Pour utiliser le bateau, il est nécessaire de se connecter sur son réseau **AIONio-cb61** avec le mdp : **aionrobotics**

Une fois que c'est fait :

    ssh -X aion@10.0.1.128

Le mdp étant **aion**.

Ensuite dans le ssh :

    roslaunch guerlerover launch_rover.launch

Et comme pour le bateau, sur un autre terminal (*toujours en ssh*) :

    rosrun mavros mavsafety arm
    rosrun mavros mavsys mode -c GUIDED

Pour mettre à jour le package il faut faire (sur sa machine):

    scp -r catkin_ws/src/guerlerover aion@10.0.1.128:~/AIONio_ws/src/

Puis dans le ssh :

    cd AIONio_ws
    catkin build

### RTK Base

Pour installer la base RTK, il suffit de 
- connecter le module Ardusimple à la Raspberry en branchant les deux ports **GPS+Power** et **Xbee+Power**.
- relier la carte Ardusimple à l'antenne RTK à l'aide du câble coaxial de l'installation. 
- alimenter la Raspberry et attendre 2-3 minutes que le module se lance correctement (on peut connecter la Raspberry à un écran pour en avoir le coeur net).

Normalement, l'acquisition se lance toute seule. Attention, pour obtenir une précision centimétrique, il faut que la base enregistre ses données pendant 24h. Les archivages se lancent à 4h du matin tous les jours donc il faut bien prendre en compte ce paramètre pour avoir un log de 24h complet. Il faut ensuite suivre les [instructions](https://docs.centipede.fr/docs/base/positionnement.html) données par Centipède pour communiquer avec IGN.

Si on veut indépendament vérifier les données GPS, on peut le faire (sous linux) en connectant le module Ardusimple en USB et en suivant ce [tuto](https://www.ardusimple.com/how-to-use-ardusimple-rtk-receivers-and-get-gps-data-in-ros/). Sinon, on peut connecter le module Ardusimple en USB (sous Windows) suivre la section 7 des [indications](https://docs.centipede.fr/docs/base/Installation.html) données par Centipède. L'adresse http://basegnss.local permet d'accéder aux logs (sous Windows) de la base et le mot de passe est _admin_. On peut alors entre autre télécharger les fichiers qui nous intéressent. **Update** : il est possible (sous Windows) d'accéder aux logs de la base. Pour cela, il faut être relié en ethernet à la raspberry et se rendre à l'adresse [http:basegnss.local](http:basegnss.local).

Autrement, il est possible de se connecter en ssh à la carte Raspberry, lorsque le module Ardusimple y est relié, en tapant : 

        ssh basegnss@basegnss.local
        # mot de passe : basegnss!

Ceci permet d'accéder aux informations de la carte et éventuellement de débugger.


## Utiliser Docker pour ne pas avoir à installer ROS

Dans le dossier Docker de ce projet, on peut trouver un Dockerfile permettant de construire une image avec Ubuntu 18.04 et ROS Melodic. Une archive donne aussi la version avec Ubuntu 20.04 et ROS Noetic. Pour contruire l'image Docker, il faut se rendre dans le dossier [_Docker_](/Docker/) puis taper : 

        docker build -t name . # remplacer "name" 
        # par le nom voulu pour l'image

Il faut ensuite attendre quelques minutes la création de l'image. Pour lancer le conteneur, il est important de préciser en commande les autorisations souhaitées et les fichiers partagés : 

        docker run -it --privileged -v  "to_share":"destination" --rm "name":latest

avec "name" le nom de l'image créée.

Pour autoriser le conteneur à afficher des fenêtres graphiques, il faut rajouter la commande suivante après "privileged" : 

        -e DISPLAY=$DISPLAY

 et derrière le "-v" : 

        /tmp/.X11-unix:/tmp/.X11-unix

Ce qui donne : 

        docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix "to_share":"destination" --rm "name":latest
        
En cas de refus de la part de l'hôte, on rajoute : 

        xhost +

avant de lancer le conteneur. Mais cette commande est à éviter si possible pour des raisons de sécurité pour l'hôte.


## TODO
- potentiellement passer aux angles d'euler
- regarder l'heading avec dual antenne
- calibrer centrale
- verifier ou mettre bonne convention (ENU)
- baisser les coefficients (surtout vitesse)
- verifier projection
- réessayer launch global
- faire un schéma de l'architecture logicielle