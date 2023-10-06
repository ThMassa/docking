# docking

Voir [ce lien](https://ardupilot.org/dev/docs/mavlink-basics.html) pour infos sur mavlink

## Docker
Pour ouvrir le container docker : 

    docker run -it --privileged -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix --rm "ubuntu18":latest