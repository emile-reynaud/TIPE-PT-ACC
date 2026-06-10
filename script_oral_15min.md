# Script oral — TIPE AAC — 15 minutes chrono
> **Règle d'or : une diapo = 1 minute maximum. Parlez lentement, voix posée.**  
> Chaque section indique la durée cible et les points à ne pas oublier.

---

## DIAPO 1 — Titre *(30 secondes)*

« Bonjour. Mon TIPE porte sur la conception et la comparaison expérimentale de correcteurs PID appliqués à un régulateur de vitesse adaptatif — un ACC miniature construit sur une voiture RC.
L'ancrage au thème "Cycles et boucles" est immédiat : le cœur du système est une boucle de régulation fermée, exécutée en cycle discret toutes les 50 millisecondes. »

---

## DIAPO 2 — Problématique *(50 secondes)*

« La structure classique d'un régulateur en boucle fermée : on compare la consigne à la mesure, l'écart ε alimente le correcteur, qui agit sur le moteur, et le capteur ferme la boucle.

La problématique est : quel correcteur — P, PI, PD ou PID — associé à une commande par anticipation offre le meilleur compromis entre stabilité, précision et temps de réponse pour un ACC miniature ?

L'ancrage au thème est direct : cette boucle est un cycle discret — mesure, calcul, action — répété 20 fois par seconde. »

---

## DIAPO 3 — Maquette *(55 secondes)*

« La maquette embarque cinq composants clés.
Un Arduino Nano Every joue le rôle de calculateur. Un pont en H L298N pilote le moteur RC390 par PWM.
Un HC-SR04 mesure la distance à l'obstacle jusqu'à 1,2 mètre. Trois aimants et un capteur Hall mesurent la vitesse de la roue.
Tout est enregistré sur microSD en CSV à 10 Hz pour analyse post-expérience.

Les consignes : vitesse de croisière 0,70 m/s, distance de sécurité 0,30 m, boucle de contrôle à 50 ms. »

---

## DIAPO 4 — Identification du moteur *(60 secondes)*

« Avant de concevoir le correcteur, il faut modéliser le moteur.
On identifie un modèle de premier ordre : la vitesse d'équilibre est proportionnelle à la commande au-delà d'un seuil de démarrage.
K = 1,40 m/s est le gain statique. Le seuil θ = 0,30 normalisé, soit 76 PWM, est la commande minimale pour vaincre le frottement.

De là, on calcule le feedforward : U_FF = (Vc/K + θ) × 255 = 204 PWM. C'est la commande d'équilibre qui place le moteur directement au bon point de fonctionnement — sans ça, le correcteur P seul aurait toujours une erreur statique.

Cette architecture 2-DOF — feedforward plus PID — est le standard industriel. »

---

## DIAPO 5 — Mesure de vitesse *(55 secondes)*

« On dispose de deux méthodes complémentaires.
La méthode fréquencemétrique compte les impulsions Hall sur 50 ms — elle est précise à haute vitesse mais aveugle à basse vitesse. On l'utilise pour le log uniquement.

La méthode période mesure la durée entre deux aimants consécutifs — 155 fois plus précise à basse vitesse. C'est elle qui alimente le correcteur PID.

Un filtre IIR asymétrique la protège : montée lente (alpha 0,25) pour ignorer les rebonds mécaniques, descente rapide (alpha 0,85) pour suivre les vraies décélérations sans retard.

L'incertitude de 2,3 % se traduit par le bruit visible sur les courbes. »

---

## DIAPO 6 — Architecture + mode ACC *(60 secondes)*

« En mode vitesse, l'architecture est 2-DOF : feedforward plus PID sur l'erreur de vitesse.

En mode distance — quand un obstacle est détecté — voici l'innovation v19 :  
**cmd = cmd_vitesse − K_brake × max(0, Dc − d)**

Si l'obstacle est loin (d ≥ Dc) : le freinage est nul, la voiture maintient exactement sa vitesse de croisière — elle n'accélère pas vers l'obstacle.
Si l'obstacle est trop proche (d < Dc) : le freinage est proportionnel à l'excès de proximité.

Un anti-windup gèle l'intégrale pendant le freinage pour éviter une accélération parasite au retour. »

---

## DIAPO 7 — Correcteur P *(55 secondes)*

« Sur la courbe P, on voit clairement l'erreur statique : la voiture se stabilise à 0,47 m/s au lieu de 0,70 — soit 33% d'écart.

Cette erreur est structurelle : le terme P seul calcule u = Kp × ε. À l'équilibre, ε tend vers une valeur non nulle car le feedforward ne compense pas parfaitement le mismatch entre K nominal et K réel — liée à la décharge de la batterie.

En mode distance, regardez la commande — sans lissage, le bruit de mesure amplifié par Kp=60 crée les saccades caractéristiques visibles dans le quatrième graphe.

Le P seul est clairement insuffisant pour un ACC. »

---

## DIAPO 8 — Correcteur PI *(55 secondes)*

« Le PI introduit l'intégrale, qui s'accumule tant que l'erreur persiste.
Avec Ki = 60, la compensation du mismatch de K prend environ 8 secondes — visible sur la courbe par la montée progressive vers 0,70 m/s.

Le prix à payer : un temps de réponse de 7,5 secondes. Le Kp = 15 est volontairement faible pour maintenir la stabilité malgré le fort Ki — système très amorti, aucun dépassement.

Ce système est précis mais trop lent pour un ACC réel : en 7,5 secondes à 0,70 m/s, la voiture parcourt plus de 5 mètres. »

---

## DIAPO 9 — Correcteur PD *(55 secondes)*

« Le PD — ici avec Kp = 350 et Kd = 10 — est l'opposé du PI : très rapide, imprécis.

Le temps de réponse est inférieur à 2 secondes. Le terme dérivé amortit partiellement la montée, mais on voit un dépassement de 14% — au-dessus de la norme standard de 5%.

L'erreur statique de 0,13 m/s est réduite par rapport au P simple grâce au Kp plus élevé, mais irréductible sans intégrale.

À noter : j'ai implémenté la dérivée sur la mesure de vitesse et non sur l'erreur pour éviter le "derivative kick" — le pic brutal qui apparaît quand la consigne change soudainement. »

---

## DIAPO 10 — Correcteur PID *(60 secondes)*

« Le PID combine les trois termes et donne les meilleurs résultats sur tous les critères.

L'intégrale (Ki = 40) élimine l'erreur statique — la voiture atteint 0,70 m/s.
Le dérivé (Kd = 8) amortit le dépassement à 4,8%, en dessous de la norme standard de 5%.
Le temps de réponse est de 4 secondes — bon compromis.

En mode distance, regardez la vitesse : elle reste constante jusqu'à ce que l'obstacle soit trop proche, puis décroît progressivement. Pas d'accélération vers l'obstacle. À droite du graphe, au retour en mode vitesse, la remontée vers Vc est propre.

C'est le seul correcteur à satisfaire simultanément les trois critères. »

---

## DIAPO 11 — Comparaison synthèse *(50 secondes)*

« Ce tableau résume toutes les performances mesurées.

P : inacceptable — erreur statique de 33%, jamais éliminée.
PI : précis mais trop lent — 7,5 secondes pour atteindre la consigne.
PD : rapide mais imprécis — dépassement de 14%, au-dessus de la norme.
PID : les trois critères sont respectés simultanément.

La réponse à la problématique est donc claire : le PID associé au feedforward 2-DOF est le meilleur compromis. Les correcteurs partiels — P, PI, PD — présentent chacun un défaut rédhibitoire en conditions réelles. »

---

## DIAPO 12 — Innovations v19 *(55 secondes)*

« Pour obtenir ces courbes propres, quatre choix d'implémentation ont été nécessaires.

Premièrement, la dérivée sur la mesure de vitesse — mathématiquement équivalente en régime stable, mais sans le pic brutal au changement de consigne.

Deuxièmement, la continuité de l'intégrale entre les modes vitesse et distance — c'est ce qui supprime l'accélération parasite au passage en mode ACC.

Troisièmement, l'anti-windup spécifique au freinage — sans ça, l'intégrale s'emballe pendant le ralentissement et fait accélérer la voiture après.

Quatrièmement, le kick-start adapté à chaque correcteur pour reproduire les comportements transitoires caractéristiques. »

---

## DIAPO 13 — Limites *(45 secondes)*

« Quatre limites identifiées.
Le HC-SR04 avec 2 cm de résolution crée des oscillations de commande — un LiDAR ToF améliorerait la précision d'un facteur 10.
Le capteur Hall à 3 aimants génère un bruit périodique à 8 Hz — 6 aimants réduiraient l'incertitude de moitié.
La décharge de la batterie fait baisser K de 40% — un observateur adaptatif corrigerait cette dérive.
En ACC réel, la distance de sécurité devrait croître avec la vitesse selon le concept de headway time. »

---

## DIAPO 14 — Conclusion *(40 secondes)*

« En conclusion : l'architecture 2-DOF avec correcteur PID est la solution optimale pour ce système.
Les critères sont validés expérimentalement : erreur statique inférieure à 1%, dépassement de 4,8% conforme à la norme de 5%, temps de réponse de 4 secondes.
Le mode ACC v19 reproduit le comportement d'un vrai régulateur de vitesse adaptatif.
Et l'ancrage au thème est fort : la boucle fermée de régulation est littéralement un cycle discret — mesure, calcul, action — incarnant la notion de "Cycles et boucles". »

---

## DIAPO 15 — Bibliographie *(20 secondes)*

« Les quatre références utilisées couvrent le contexte industriel des ADAS et les aspects techniques du contrôle PID sur Arduino. Je reste disponible pour toute question. »

---

## Chronomètre estimé

| Diapo | Contenu | Durée |
|-------|---------|-------|
| 1 | Titre | 0:30 |
| 2 | Problématique | 1:20 |
| 3 | Maquette | 2:15 |
| 4 | Identification | 3:15 |
| 5 | Mesure vitesse | 4:10 |
| 6 | Architecture + ACC | 5:10 |
| 7 | Correcteur P | 6:05 |
| 8 | Correcteur PI | 7:00 |
| 9 | Correcteur PD | 7:55 |
| 10 | Correcteur PID | 8:55 |
| 11 | Comparaison | 9:45 |
| 12 | Innovations v19 | 10:40 |
| 13 | Limites | 11:25 |
| 14 | Conclusion | 12:05 |
| 15 | Bibliographie | 12:25 |
| **Marge questions** | | **2:35** |
| **TOTAL** | | **≤ 15:00** |

> 💡 **Conseil** : les diapos de courbe (7–10) peuvent se résumer en 45 s si tu as du retard. La comparaison (11) et la conclusion (14) sont les deux points à tenir coûte que coûte.
