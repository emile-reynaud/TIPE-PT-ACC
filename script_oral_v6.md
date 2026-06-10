# Script oral — TIPE AAC v4 + protocole — 15 minutes
> **20 slides. La diapo protocole est la n°8, entre "Architecture" et "Courbe P".**

---

## Diapo 1 — Titre *(30 s)*

« Bonjour. Mon TIPE porte sur la conception et la comparaison expérimentale de correcteurs PID pour un régulateur de vitesse adaptatif — un ACC miniature sur voiture RC. L'ancrage au thème "Cycles et boucles" est la boucle de régulation elle-même, exécutée en cycle discret toutes les 50 ms. »

---

## Diapo 2 — Ancrage et problématique *(45 s)*

« La boucle de régulation est un cycle classique : consigne, correcteur, actionneur, mesure — répété 20 fois par seconde. C'est le cœur de l'ancrage au thème. La problématique : quel correcteur P, PI, PD ou PID associé à un feedforward offre le meilleur compromis stabilité-précision pour un ACC miniature ? »

---

## Diapo 3 — Maquette *(40 s)*

« La maquette : Arduino Nano Every pour le calcul, pont en H L298N pour le moteur, HC-SR04 pour la distance obstacle, capteur Hall à 3 aimants pour la vitesse, microSD pour le log CSV à 10 Hz. »

---

## Diapo 4 — Identification du moteur *(55 s)*

« Le moteur est modélisé par un premier ordre : v = K × (u − seuil). Neuf échelons PWM donnent K = 1,4 m/s, τ = 250 ms, seuil = 0,30. De là, le feedforward U_FF = 204 PWM. Sans ce feedforward, le PID seul s'annulerait quand l'erreur tend vers zéro — le moteur s'arrêterait. C'est l'architecture 2-DOF : FF pour le point de fonctionnement, PID pour les perturbations. »

---

## Diapo 5 — Mesure fréquencemétrique *(35 s)*

« La méthode fréquencemétrique compte les impulsions Hall sur 50 ms — incertitude de 249 % à 0,70 m/s, inutilisable pour le PID. Je l'utilise uniquement pour le log. »

---

## Diapo 6 — Mesure période *(40 s)*

« La méthode période mesure la durée entre deux aimants consécutifs — 155 fois plus précise, incertitude 2,3 %. C'est elle qui alimente le correcteur. Un filtre IIR asymétrique la protège contre les rebonds tout en suivant rapidement les décélérations. »

---

## Diapo 7 — Architecture 2-DOF + mode ACC *(55 s)*

« L'architecture 2-DOF : U_FF positionne le moteur au point d'équilibre, le PID corrige les écarts. En mode distance, j'applique une loi ACC réaliste : cmd = cmd_vitesse − 600 × max(0, Dc − d). Si l'obstacle est loin, freinage nul — la voiture maintient sa vitesse sans accélérer vers l'obstacle. Si l'obstacle est trop proche, freinage proportionnel à l'excès de proximité. »

---

## Diapo 8 — Protocole expérimental *(50 s)*

« Avant de présenter les courbes, voici le protocole commun aux 4 correcteurs. La frise montre 35 secondes divisées en 4 phases. Phase 1 : kick-start de 0 à 3 s — la durée et la puissance varient selon le correcteur. Phase 2 : mode vitesse de 3 à 12 s, sans obstacle — on y mesure l'erreur statique, le dépassement et le tr5%. Phase 3 : mode distance ACC de 12 à 27 s — l'obstacle approche sous la distance de sécurité puis s'éloigne. Phase 4 : retour en mode vitesse de 27 à 35 s. Les paramètres communs sont rappelés en bas. »

---

## Diapo 9 — Correcteur P — courbe *(40 s)*

« Sur la courbe P, Kp = 60. L'erreur statique est immédiatement visible : la voiture se stabilise à 0,47 m/s au lieu de 0,70 — 33 % d'écart. En mode distance, regardez la commande : les saccades caractéristiques dues au bruit amplifié par Kp sans lissage. »

---

## Diapo 10 — Correcteur P — analyse *(35 s)*

« Erreur statique de 33 % structurelle — irréductible sans terme intégral. K_réel ≠ K_nom à cause de la batterie, le feedforward ne compense pas entièrement. Le P sert de référence comparative. »

---

## Diapo 11 — Correcteur PI — courbe *(40 s)*

« PI, Kp = 15 et Ki = 60. La montée progressive vers 0,70 m/s est visible — l'intégrale qui compense le biais en 8 secondes. Système très amorti, aucun dépassement. »

---

## Diapo 12 — Correcteur PI — analyse *(35 s)*

« L'intégrale élimine l'erreur statique, mais tr5% = 7,5 s — à 0,70 m/s la voiture parcourt 5 mètres avant d'atteindre la consigne. Trop lent pour un ACC réel. »

---

## Diapo 13 — Correcteur PD — courbe *(40 s)*

« PD, Kp = 350 et Kd = 10. Très rapide — tr5% ≈ 1,5 s — et le dépassement de 14 % est bien visible. La dérivée est calculée sur la mesure de vitesse, pas sur l'erreur, pour éviter le pic brutal lors des transitions. »

---

## Diapo 14 — Correcteur PD — analyse *(35 s)*

« Rapide mais imprécis : erreur statique de 19 % sans intégrale, et dépassement de 14 % au-delà de la norme de 5 %. Bon transitoire, mauvais état final. »

---

## Diapo 15 — Correcteur PID — courbe *(45 s)*

« PID, Kp = 80, Ki = 40, Kd = 8. Démarrage avec 4,8 % de dépassement, convergence vers 0,70 m/s en 4 secondes, erreur résiduelle sous 1 %. En mode distance, la vitesse reste stable jusqu'à ce que l'obstacle soit trop proche, puis le freinage s'active progressivement — sans aucune accélération vers l'obstacle. »

---

## Diapo 16 — Correcteur PID — analyse *(40 s)*

« Seul le PID satisfait les trois critères simultanément : erreur < 1 %, dépassement 4,8 % ≤ 5 %, tr5% = 4 s. L'anti-windup gèle l'intégrale pendant le freinage en mode distance pour éviter l'accélération parasite au retour. »

---

## Diapo 17 — Comparaison *(40 s)*

« Le tableau résume tout. P : erreur 33 %, éliminatoire. PI : précis mais 7,5 s. PD : rapide mais 14 % hors norme. PID : les trois critères respectés. La réponse à la problématique est claire. »

---

## Diapo 18 — Limites *(30 s)*

« Quatre limites : HC-SR04 résolution 2 cm, K qui varie avec la batterie, oscillations résiduelles liées à la zone morte, et inertie réelle légèrement supérieure au modèle. »

---

## Diapo 19 — Conclusion *(30 s)*

« PID + feedforward 2-DOF : solution optimale, critères validés expérimentalement. Mode distance ACC v19 : freinage seul, sans accélération vers l'obstacle. Ancrage : la boucle fermée est un cycle discret — mesure, calcul, action — répété 20 fois par seconde. »

---

## Diapo 20 — Bibliographie *(15 s)*

« Quatre références. Je reste disponible pour vos questions. »

---

## Chronomètre estimé

| # | Slide | Durée | Cumul |
|---|-------|-------|-------|
| 1 | Titre | 0:30 | 0:30 |
| 2 | Ancrage + Problématique | 0:45 | 1:15 |
| 3 | Maquette | 0:40 | 1:55 |
| 4 | Identification moteur | 0:55 | 2:50 |
| 5 | Méthode fréquencemétrique | 0:35 | 3:25 |
| 6 | Méthode période | 0:40 | 4:05 |
| 7 | Architecture + mode ACC | 0:55 | 5:00 |
| **8** | **Protocole expérimental** | **0:50** | **5:50** |
| 9 | P — courbe | 0:40 | 6:30 |
| 10 | P — analyse | 0:35 | 7:05 |
| 11 | PI — courbe | 0:40 | 7:45 |
| 12 | PI — analyse | 0:35 | 8:20 |
| 13 | PD — courbe | 0:40 | 9:00 |
| 14 | PD — analyse | 0:35 | 9:35 |
| 15 | PID — courbe | 0:45 | 10:20 |
| 16 | PID — analyse | 0:40 | 11:00 |
| 17 | Comparaison | 0:40 | 11:40 |
| 18 | Limites | 0:30 | 12:10 |
| 19 | Conclusion | 0:30 | 12:40 |
| 20 | Bibliographie | 0:15 | 12:55 |
| **Marge** | | **2:05** | **≤ 15:00** |

> 💡 **Si tu es en avance** : développe les diapos d'analyse (10, 12, 14, 16) ou prends plus de temps sur la diapo protocole.
> **Si tu es en retard** : fusionne courbe + analyse oralement en une minute par correcteur, et condense les diapos 5+6 en 30 s : *« fréquencemétrique pour le log, période pour le PID — 155× plus précis »*.
