# Notes pour l'utilisation de Docker

## Rappel pour la construction d'une image

Depuis le répertoire contenant un Dockerfile.txt, on exécute la commande :

    docker build -t name .

Il suffit de remplacer "_name_" par le nom de l'image de travail dans cette commande.

## Rappel pour le lancement d'un conteneur temporaire

Pour lancer un conteneur qui exécute l'image de travail, on exécute les commandes : 

    xhost +
    docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm name:latest

La commande "xhost +" permet au conteneur d'accéder aux ressources de l'ordinateur hôte. Ensuite il faut remplacer "_name_" par le nom de l'image de travail. Il est important de placer le répertoire _/tmp_ dans les fichiers partagés avec le conteneur.