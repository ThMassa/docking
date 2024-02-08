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

Pour installer la base RTK, il y a deux choix : 

1. utiliser la Raspberry et le module ardusimple-ublox
2. utiliser seulement le module ardusimple-ublox

La première méthode nécessite de flasher la carte SD de la Raspberry en suivant les instructions données par centipède pour l'installation d'une base RTK. Leur tuto est destiné à créer une base qui sera déclarée sur leur réseau mais ce n'est pas ce qu'on cherche ici. Il faut s'arrêter dans le tuto avant la déclaration de la base sur leur réseau. Concrètement, en suivant ces [instructions](https://docs.centipede.fr/docs/base/), on va installer sur la Raspberry un programme donnant accès facilement à une architecture de base RTK et régler le Ublox en mode base. Le programme fournit une interface html qui permet de faire des réglages sur la base et de récupérer ses logs en se connectant à http://basegnss.local (il faut être sur le même réseau que la Raspberry). Il est aussi possible de le faire en ssh avec : 

```
ssh basegnss@basegnss.local
basegnss! # mot de passe
```
L'idée avec cette méthode, c'est de laisser l'installation Raspberry + Ublox acquérir des données sur 24 h (les logs commencent ou finissent à 4h du matin donc il faut garder ça en tête), de récupérer les logs au format RINEX, de les transmettre à IGN qui calcule les coordonnées précises au centimètre de la base. Il n'y a alors plus qu'à les rentrer dans la base ou dans le Ublox si on veut enlever la Raspberry (qui ne devrait plus servir à présent puisque le but n'est pas de faire une base Centipède). Le Ublox en mode base se charge du reste et transmet les corrections via son antenne Xbee.

La deuxième méthode diffère car elle ne nécessite pas de Raspberry. L'idée ici est de configure le Ublox base pour qu'il trouve tout seul sa position avec la précision voulue en utilisant les sattelites environnant. L'idéal est de le laisser loger 24 h mais 6 h peuvent suffire pour avoir une précision décimétrique.

Pour régler les Ublox, il faut avoir le logiciel U-Center (idéalement sur Windows). Il faudra ensuite suivre les [instructions du PDF](RTK_Base/GuideRTKBase.pdf) sur le sujet pour correctement régler le rover et la base. De même pour les Xbee qu'on peut également configurer avec XCTU (toujours sous Windows)..

Pour alimenter l'installation, il faudra brancher la Raspberry pour la méthode 1 ou alors directement le Ublox pour la méthode 2. Il faut garder à l'esprit qu'il faut se connecter en série sur le port Power+GPS du Ublox pour configurer et lire le GPS et sur le port Power+Xbee pour configurer et lire la radio Xbee.


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
