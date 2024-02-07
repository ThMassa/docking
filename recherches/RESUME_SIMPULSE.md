# Résumé de la documentation de Simpulse

- Les deux modems sont normalement pré-configurés pour que l'un d'entre eux soit le "master" et l'autre le "slave".

- Le test de connexion des deux modems se fait en deux parties:
    1. On commence par allumer les deux modems tout seuls, et on vérifie les LEDS
    ![test1](imgs/basic_test_standalone.png)
    2. On teste ensuite la connexion avec un ping à l'aide d'un PC connecté en ethernet à l'un des deux modems:
    ![test2](imgs/basic_test_laptop.png)


IP du dock : 192.168.0.12
EDIT : IP dock = 10.0.11.72

Tests de bande passante avec *iPerf* :
- sur un premier node (ex: le dock):

    ```bash
    iperf3 -s
    ```
    Pour initialiser le serveur
- sur un deuxième node (ex: PC de test / bateau) :
    ```bash
    iperf3 -c ADDRESSE_IP_SERVEUR -u
    ```
On peut également changer le temps de test (par défaut, 10s) avec le paramètre `-t`.


## Reconfigurer les addresses IP
Pour changer d'addresse IP sur le rover par exemple ou le drone:
```bash
sudo ifconfig [interface] [IP/mask_length]
```

