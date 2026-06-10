# Régulateur de vitesse adaptatif et boucle de rétroaction PID
## TIPE PTSI-PT
### 2024-2026

## Description générale

Ce repo est un projet de recherche motivé par l'épreuve de TIPE du concours Banque PT.

Le projet est de construire une maquette d'un **régulateur de vitesse adaptatif** ou **ACC** (**A**daptive **C**ruise **C**ontrol), et de faire des expériences sur sa boucle de correction pour comparer les différents correcteurs **P**roportionnel (**P**), **P**roportionnel-**I**ntégral (**PI**), **P**roportionnel-**D**érivé (**PD**) et **P**roportionnel-**I**ntégral-**D**érivé (**PID**).

### Problématique

Comment un régulateur de vitesse adaptatif fonctionne-t-il, et comment sa boucle de correction PID se comporte-t-elle ?

### Ancrage au thème *Cycles et boucles*

Ce TIPE se concentre sur la boucle de rétroaction, ou boucle de correction, du système. Le cycle de correction est éffectué toutes les 50 ms.

## Maquette

La maquette est composée d'une voiture RC modifiée et équipée de :

- Moteur brushed RC390
- Pont en H L298N
- Arduino Nano Every
- Arduino Nano Connector Carrier
- Capteur de distance à ultrasons HC-SR04
- Capteur à effet Hall + 3 aimants dans la roue arrière gauche
- Carte microSD branchée dans le Connector Carrier

##### Voici quelques images de la maquette

![Model Overview 3/4 front right](/Images/IMG_8594.jpeg "Model Overview 3/4 front right")

![Model Overview 3/4 front left](/Images/IMG_8592.jpeg "Model Overview 3/4 front left")

![Model Overview face front](/Images/IMG_8593.jpeg "Model Overview face front")

![Electronics Overview](/Images/IMG_8603.jpeg "Electronics Overview")

![Electronics Overview left-side view](/Images/IMG_8602.jpeg "Electronics Overview left-side view")

![Electronics Overview right-side view](/Images/IMG_8601.jpeg "Electronics Overview right-side view")

![Hall effect sensor](/Images/IMG_8600.jpeg "Hall effect sensor")

## Résultats

### Correcteur P

![Courbes expérimentales P](/Pratique/acc_code/courbe_P.png "Courbes expérimentales P")

|  Positif  |  Limites  |
| :-------: | :-------: |
| Kick-start efficace | Erreur statique -0,23 m$\cdot$s<sup>-1</sup> (33%) structurelle sans intégrale |
| :-------: | :-------: |
| Commande stable à 170-180 PWM en régime sans obstacle | Commande saccadée en mode distance(bruit amplifié par Kp) |
| :-------: | :-------: |
|           | t<sub>r<sub>5%</sub></sub> = N/A, la vitesse ne converge jamais vers V<sub>c</sub> |


### Correcteur PI

![Courbes expérimentales PI](/Pratique/acc_code/courbe_PI.png "Courbes expérimentales PI")

### Correcteur PD

![Courbes expérimentales PD](/Pratique/acc_code/courbe_PD.png "Courbes expérimentales PD")

### Correcteur PID

![Courbes expérimentales PID](/Pratique/acc_code/courbe_PID.png "Courbes expérimentales PID")
