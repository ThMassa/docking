# Comment prendre les logs sur le rover et les visualiser sur ROS2

Il faut d'abord récupérer les logs *.bag* sur le drone, puis utiliser *rosbags-convert* :

```bash
pip install rosbags
rosbags-convert [filename] --include-topic [topic_name] ...
```

On peut ensuite visualiser les logs avec des outils tels que Plotjuggler.

