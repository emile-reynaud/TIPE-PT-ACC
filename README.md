# Régulateur de vitesse adaptatif et boucle de rétroaction PID
## TIPE PTSI-PT
### 2024-2026

## Description générale

Ce repo est un projet de recherche motivé par l'épreuve de TIPE du concours Banque PT.

Le projet est de construire une maquette d'un régulateur de vitesse adaptatif ou ACC (Adaptive Cruise Control), et de faire des expériences sur sa boucle de correction pour comparer les différents correcteurs Proportionnel (P), Proportionnel-Intégral (PI), Proportionnel Dérivé (PD) et Proportionnel-Intégral-Dérivé (PID).

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
