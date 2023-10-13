# Docking autonome

### Auteurs
* GARDE Guillaume (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* MASSA Théo (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* HOFMANN Hugo (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* REN Kévin (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).

# Table of Contents

- [Docking autonome](#docking-autonome)
    - [Auteurs](#auteurs)
- [Table of Contents](#table-of-contents)
  - [Description](#description)
  - [Documentation](#documentation)
  - [Configuration](#configuration)
  - [Requirements](#requirements)
  - [Usage](#usage)
    - [Packages](#packages)
    - [Dock](#dock)
    - [Boat](#boat)

## Description
Le but du projet est d'automatiser le processus de docking d'un drone. Pour cela on doit créer un dock capable d'envoyer sa position GPS et son orientation au drone qui, à partir de ces informations, sera capable de calculer une trajectoire et d'adopter un comportement lui permettant de se docker automatiquement. En plus de cela, il est nécessaire de mettre en place une balise RTK permettant d'obtenir une précision GPS au centimètre.

## Documentation 

<!-- TODO -->
La doc pour la centrale inertielle SBG peut se trouver [ici](https://github.com/SBG-Systems/sbg_ros_driver)

La doc pour le driver GPS sur le dock peut se trouver [ici](https://github.com/KumarRobotics/ublox)

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

    roslaunch guerledock launch_dock.launch

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

Une fois connecté, il faut lancer le [roslaunch](/catkin_ws/src/guerleboat/launch/launch_boat.launch).

    roslaunch guerleboat launch_boat.launch

Cependant cela n'est pas suffisant. Il faut armer le système mavlink et le mettre en mode *GUIDED*. Pour cela il suffit de faire sur un autre terminal:

    rosrun mavros mavsafety arm
    rosrun mavros mavsys mode -c GUIDED


Pour mettre à jour le package il faut faire:

    scp -r catkin_ws/src/guerleboat surcouf@10.0.11.100:~/catkin_ws/src/

Puis dans le ssh :

    cd catkin_ws
    catkin_make