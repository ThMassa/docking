## Quelques informations sur le module ArduSimple de la base RTK

- https://fr.ardusimple.com/rtk-explained/

- https://fr.ardusimple.com/product/simplertk2b-basic-starter-kit-ip65/

- le module Ardusimple qu'on a est muni de deux ports micro USB. Le premier (power+GPS) permet d'alimenter la carte et de se connecter au ublox. Le second (power+Xbee) permet d'alimenter la carte et de se connecter à la radio Xbee. 

- pour configurer le Xbee, il faut avoir XCTU sur windows. **Note importante : si jamais XTCU refuse de reconnaitre le Xbee, il faut court-circuiter le module en reliant le pin reset au pin GND et maintenir le tout pendant la configuration. Cela devrait régler le problème.**
- pour configurer le ublox, il faut avoir u-center sur windows. FLB a un très bon tuto pour configurer en base ou rover le module.

- pour la base rtk, il faut brancher le module ardusimple à la fois en GPS et en Xbee à la Raspberry en USB.

- pour accéder aux informations et aux logs de la raspberry, il faut se connecter sous windows à la raspberry en ethernet. Ensuite, aller sur http://basegnss.local.



