# docking

Voir [ce lien](https://ardupilot.org/dev/docs/mavlink-basics.html) pour infos sur mavlink

## Docker
Pour ouvrir le container docker : 

    docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm "ubuntu18":latest

## ROS
On utilise ROS ( pas ROS 2 :( ) attention à bien coder les nodes pour que ce soit compatible ROS Melodic et pas Noetic comme nos versions (à voir si le code codé sur Melodic est compatible sur Noetic mais je pense que oui ya qu'une version d'écart).

Ca a pas franchement l'air trop compliqué de faire les nodes (en tout cas en C++)

Pour utiliser ROS (à faire sur les Jetson je pense), aller dans [/ros_ws] puis faire :

    catkin_make

    # Si sous bash (bouh les anciens)
    source devel/setup.bash

    # Si zsh (<3)
    source devel/setup.zsh

