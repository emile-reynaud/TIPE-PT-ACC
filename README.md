# Régulateur de vitesse adaptatif et boucle de rétroaction PID
## TIPE PTSI-PT
### 2024-2026

---

## Sommaire

0. [Description générale](desc)
   - [Problématique](prob)
   - [Ancrage au thème *Cycles et boucles*](anc-th)
1. [Maquette](model)
2. [Résultats](results)
   - [Correcteur P](P-results)
   - [Correcteur PI](PI-results)
   - [Correcteur PD](PD-results)
   - [Correcteur PID](PID-results)
3. [Conclusion](concl)
4. [Annexes et bibliographies](annex)

---

## Description générale {desc}

Ce repo est un projet de recherche motivé par l'épreuve de TIPE du concours Banque PT.

Le projet est de construire une maquette d'un **régulateur de vitesse adaptatif** ou **ACC** (**A**daptive **C**ruise **C**ontrol), et de faire des expériences sur sa boucle de correction pour comparer les différents correcteurs **P**roportionnel (**P**), **P**roportionnel-**I**ntégral (**PI**), **P**roportionnel-**D**érivé (**PD**) et **P**roportionnel-**I**ntégral-**D**érivé (**PID**).

### Problématique {prob}

Comment un régulateur de vitesse adaptatif fonctionne-t-il, et comment sa boucle de correction PID se comporte-t-elle ?

### Ancrage au thème *Cycles et boucles* {anc-th}

Ce TIPE se concentre sur la boucle de rétroaction, ou boucle de correction, du système. Le cycle de correction est éffectué toutes les 50 ms.

## Maquette {model}

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

## Architecture du correcteur {arch-corr}

Le correcteur se compose du régulateur PID, mais aussi d'un sytème de *feedforward*.

### Description du feedforward {desc-ff}

Le correcteur, qu'il soit P, PI, PD ou PID, prend une erreur en entrée et renvoie une commande moteur, et il veut faire tendre l'entrée vers 0.
Or, s'il fait cela, la commande qu'il retournera sera elle aussi 0.
Cependant, à 0, le moteur ne maintient pas sa vitesse.

Voilà pour nous avons besoin dans ce sytème d'un **feedforward**.
Le feedforward est simplement un ajout après les correcteur, permettant de corriger cela et faire tendre la commande vers une commande en régime, et non en roue libre.
C'est une sorte d'offset de la commande.

### Identification du moteur {id-mot}

Pour calculer la valeur du feedforward, nous avons besoin d'identifier le moteur et de trouverson gain statique K, sa constante de temps $\tau$, ainsi que son seuil de commande s.

Voici la fonction transfert d'un moteur à courant continu :

$\frac{dv}{dt} + \frac{1}{\tau}\cdot{v(t)} = K\cdot{u(t)}$

## Résultats {results}

### Correcteur P {P-results}

![Courbes expérimentales P](/Pratique/acc_code/courbe_P.png "Courbes expérimentales P")

#### Analyse

##### Positif

- Kick-start efficace
- Commande stable à 170-180 PWM en régime sans obstacle

##### Limites

- Erreur statique -0,23 m⋅s<sup>-1</sup> (33%) structurelle sans intégrale
- Commande saccadée en mode distance (bruit amplifié par Kp)
- t<sub>r<sub>5%</sub></sub> = N/A, la vitesse ne converge jamais vers V<sub>c</sub>

#### Bilan

L'erreur statique de 33 % est irréductible sans terme intégral. Sert de référence comparative.

### Correcteur PI {PI-results}

![Courbes expérimentales PI](/Pratique/acc_code/courbe_PI.png "Courbes expérimentales PI")

#### Analyse

##### Positif

- Erreur statique $rarr; 0 : Ki compense l'erreur
- Aucun dépassement, système très amorti
- Transition entre les modes propres, pas de saut

##### Limites

- t<sub>r<sub>5%</sub></sub> = 7,5 s, le plus lent des correcteurs
- Kp = 15 volontairement faible, peu réactif au perturbations soudaines
- Réinitialisation de l'intégrale au kick, convergence repart à zéro après démarrage

#### Bilan

Le PI atteint la précision au prix de la rapidité. t<sub>r<sub>5%</sub></sub> = 7,5 s est rédhibitoire en conditions réelles. À 0,70 m⋅s<sup>-1</sup>, la voiture parcourt 5 mètres avant d'être à vitesse.

### Correcteur PD {PD-results}

![Courbes expérimentales PD](/Pratique/acc_code/courbe_PD.png "Courbes expérimentales PD")

#### Analyse

##### Positif

- t<sub>r<sub>5%</sub></sub> = 1,5 s, le plus rapide

##### Limites

- Erreur statique −0,13 m⋅s<sup>-1</sup> (19 %), irréductible sans terme intégral
- Dépassement 14 % > norme standard 5 % — non conforme

#### Bilan

Le PD est rapide mais imprécis. Dépassement 14 % dépasse la norme de 5 %. Sans intégrale, l'erreur statique de 19 % persiste indéfiniment. Bon transitoire, mauvais état final.

### Correcteur PID {PID-results}

![Courbes expérimentales PID](/Pratique/acc_code/courbe_PID.png "Courbes expérimentales PID")

#### Analyse

##### Positif

- Erreur statique < 1 % (−0,01 m⋅s<sup>-1</sup>)
- Dépassement 4,8 % < 5 %
- t<sub>r<sub>5%</sub></sub> = 4 s, bon compromis rapidité / stabilité
- Mode distance ACC propre

##### Limites

- Réglage plus complexe : 3 paramètres interdépendants (Kp, Ki, Kd)
- Anti-windup nécessaire : intégrale gelée pendant le freinage en mode distance
- Terme D sensible au bruit Hall

#### Bilan

Seul le PID satisfait simultanément les 3 critères : erreur < 1 %, dépassement 4,8 % ≤ 5 %, t<sub>r<sub>5%</sub></sub> = 4 s. L'architecture FF + PID constitue la solution optimale.

## Conclusion {concl}

#### Limites

- 3 aimants : Δv_freq=1,74 m⋅s<sup>-1</sup> (249% à 0,70 m⋅s<sup>-1</sup>)
- K varie avec batterie → re-identification nécessaire
- HC-SR04 : fausses détections en extérieur
- Zone morte 76/255 → oscillations irréductibles

#### Perspectives

- 12 aimants → Δv_freq divisé par 4
- Encodeur optique → résolution µm
- 2e maquette → obstacle mobile réaliste
- Re-identification auto (K adaptatif)

## Annexes et bibliographies {annex}

#### Bibliographie


1. V.K. Kukkala, J. Tunnell, S. Pasricha, T. Bradley — "Advanced Driver-Assistance Systems: A Path Toward Autonomous Vehicles" — IEEE Consumer Electronics Magazine, vol. 7, n°5, pp. 18-25, 2018. DOI: 10.1109/MCE.2018.2828440
2. J.E. Naranjo, F. Serradilla, F. Nashashibi — "Speed Control Optimization for Autonomous Vehicles with Metaheuristics" — Electronics 2020, 9, 551. DOI: 10.3390/electronics9040551
3. Saravanan G et al. — "Red Panda Optimization Algorithm-Based PID Controller Design for Automobile Cruise Control System" — ICSSEECC 2024. DOI: 10.1109/ICSSEECC61126.2024.10649422
4. J.S. Saputro et al. — "Design of an Adaptive Cruise Control System using PID Control Method on Electric Vehicle Prototypes" — ICAMIMIA 2023. DOI: 10.1109/ICAMIMIA60881.2023.10427856

#### Annexes

Pour voir le code, allez voir ce [fichier](/Pratique/acc_code/AAC_TIPE_v19/AAC_TIPE_v19.py).

Pour une présentation un peu plus détaillée et mise en forme, allez voir la [présentation](/TIPE_AAC_v5.pdf).
