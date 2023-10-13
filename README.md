# Docking autonome

### Autors
* GARDE Guillaume (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* MASSA Théo (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* HOFMANN Hugo (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).
* REN Kévin (Promotion [_ENSTA Bretagne_](https://www.ensta-bretagne.fr) 2024 - Spécialité Robotique Autonome).

# Table of Contents

- [Description](#description)
- [Documentation](#documentation)
- [Configuration](#configuration)
- [Requirements](#requirements)
- [Usage](#usage)
    - [Build the package](#build-the-package)

## Description
Le but du projet est d'automatiser le processus de docking d'un drone. Pour cela on doit créer un dock capable d'envoyer sa position GPS et son orientation au drone qui, à partir de ces informations, sera capable de calculer une trajectoire et d'adopter un comportement lui permettant de se docker automatiquement. En plus de cela, il est nécessaire de mettre en place une balise RTK permettant d'obtenir une précision GPS au centimètre.