# Script oral — TIPE AAC v4 — 15 minutes
> **19 slides, ~45 s par slide en moyenne. Parlez lentement et clairement.**

---

## Diapo 1 — Titre *(30 s)*

« Bonjour. Mon TIPE porte sur la conception et la comparaison expérimentale de correcteurs PID pour un régulateur de vitesse adaptatif — un ACC miniature sur voiture RC. L'ancrage au thème "Cycles et boucles" est la boucle de régulation elle-même, exécutée en cycle discret toutes les 50 ms. »

---

## Diapo 2 — Ancrage et problématique *(50 s)*

« La boucle de régulation est un cycle classique de l'automatique : on compare la consigne de vitesse à la mesure, l'écart alimente le correcteur qui commande le moteur, et le capteur Hall ferme la boucle. Ce cycle se répète 20 fois par seconde — c'est le cœur de l'ancrage au thème.

La problématique : quel correcteur P, PI, PD ou PID associé à un feedforward offre le meilleur compromis stabilité-précision pour un ACC miniature ? »

---

## Diapo 3 — Maquette *(45 s)*

« La maquette embarque cinq éléments. Un Arduino Nano Every joue le rôle du calculateur embarqué. Un pont en H L298N pilote le moteur brushed RC390. Un HC-SR04 mesure la distance obstacle. Trois aimants et un capteur Hall mesurent la vitesse. Tout est enregistré en CSV sur microSD à 10 Hz pour analyse post-expérience. »

---

## Diapo 4 — Identification du moteur *(55 s)*

« Avant de concevoir le correcteur, j'ai identifié le moteur par un modèle de premier ordre : v = K × (u − seuil). J'ai réalisé 9 échelons PWM de 40 à 240 et mesuré K = 1,4 m/s et τ = 250 ms.

Ce modèle sert à calculer le feedforward : U_FF = (Vc/K + seuil) × 255 = 204 PWM. C'est la commande d'équilibre qui place le moteur directement au bon point de fonctionnement. Sans ça, le PID seul s'annulerait quand l'erreur tend vers zéro — et le moteur s'arrêterait. »

---

## Diapo 5 — Mesure de vitesse fréquencemétrique *(40 s)*

« La méthode fréquencemétrique compte les impulsions Hall sur 50 ms. Elle est précise à haute vitesse mais inutilisable en dessous de quelques impulsions par cycle — son incertitude dépasse 249 % à 0,70 m/s. Je l'utilise uniquement pour le log. »

---

## Diapo 6 — Mesure de vitesse — méthode période *(45 s)*

« La méthode période mesure la durée entre deux aimants consécutifs. Son incertitude est 155 fois plus faible : 2,3 % relatif constant. C'est elle qui alimente le correcteur PID. Un filtre exponentiel asymétrique la protège contre les rebonds mécaniques tout en suivant rapidement les décélérations. »

---

## Diapo 7 — Architecture 2-DOF + mode ACC *(60 s)*

« L'architecture retenue est 2-DOF : le feedforward positionne le moteur au point d'équilibre, et le PID corrige les écarts résiduels. La commande totale est U_FF + Kp × ε + Ki × ∫ε dt + Kd × dε/dt.

En mode distance, j'ai adopté une architecture ACC réaliste. Au lieu de réguler sur l'erreur de distance, la commande reste celle du régulateur de vitesse, et on lui soustrait un terme de freinage uniquement si l'obstacle est trop proche : cmd = cmd_vitesse − 600 × max(0, Dc − d). Si l'obstacle est loin, la voiture maintient sa vitesse sans accélérer vers lui. »

---

## Diapo 8 — Correcteur P — courbe *(40 s)*

« Sur la courbe du correcteur P avec Kp = 60, on voit immédiatement l'erreur statique : la voiture se stabilise à 0,47 m/s au lieu de 0,70 — un écart de 33 %. En mode distance, regardez la commande en bas : les saccades caractéristiques dues au bruit de mesure amplifié par Kp sans lissage. »

---

## Diapo 9 — Correcteur P — analyse *(40 s)*

« L'erreur statique de 33 % est structurelle : le terme P seul calcule Kp × ε, et ne peut atteindre Vc que si ε est non nul en permanence. K_réel ≠ K_nom à cause de la décharge batterie — le feedforward ne compense pas entièrement ce biais. Le P sert de référence pour la suite. »

---

## Diapo 10 — Correcteur PI — courbe *(40 s)*

« Le PI avec Kp = 15 et Ki = 60. On voit la montée progressive vers 0,70 m/s — l'intégrale qui s'accumule sur 8 secondes pour compenser le biais de K. Le système est amorti, aucun dépassement. »

---

## Diapo 11 — Correcteur PI — analyse *(40 s)*

« L'intégrale élimine l'erreur statique — elle converge vers 0 — mais au prix d'un temps de réponse de 7,5 secondes. C'est rédhibitoire en conditions réelles : à 0,70 m/s la voiture parcourt 5 mètres avant d'être à vitesse. Kp = 15 volontairement faible garantit la stabilité malgré le fort Ki. »

---

## Diapo 12 — Correcteur PD — courbe *(40 s)*

« Le PD avec Kp = 350 et Kd = 10. La réponse est rapide — tr5% ≈ 1,5 s — et le dépassement de 14 % est clairement visible à la montée. La dérivée est calculée sur la mesure de vitesse et non sur l'erreur, ce qui évite le pic brutal lors des transitions de mode. »

---

## Diapo 13 — Correcteur PD — analyse *(40 s)*

« Le PD est le plus rapide, mais deux défauts : l'erreur statique de 19 % persiste — sans intégrale, elle est irréductible. Et le dépassement de 14 % dépasse la norme industrielle de 5 %. Bon transitoire, mauvais état final. »

---

## Diapo 14 — Correcteur PID — courbe *(45 s)*

« Le PID avec Kp = 80, Ki = 40 et Kd = 8. On voit les trois comportements combinés : démarrage avec léger dépassement de 4,8 %, puis convergence rapide vers 0,70 m/s en ~4 secondes, erreur résiduelle inférieure à 1 %. En mode distance, la vitesse reste stable jusqu'à ce que l'obstacle soit trop proche — puis le freinage s'active progressivement. »

---

## Diapo 15 — Correcteur PID — analyse *(45 s)*

« Seul le PID satisfait les trois critères simultanément. Erreur statique inférieure à 1 % — l'intégrale compense le biais en ~5 s. Dépassement 4,8 %, en dessous de la norme de 5 %. Temps de réponse de 4 secondes, bon compromis entre le PI trop lent et le PD trop imprécis. L'anti-windup gèle l'intégrale pendant le freinage en mode distance pour éviter l'accélération parasite au retour. »

---

## Diapo 16 — Comparaison *(45 s)*

« Ce tableau résume tout. P : erreur de 33 %, éliminatoire. PI : précis mais 7,5 s de réponse. PD : rapide mais 14 % de dépassement hors norme. PID : erreur < 1 %, dépassement 4,8 % ≤ 5 %, tr5% = 4 s. Et en mode distance, le comportement ACC est correct. La réponse à la problématique est claire. »

---

## Diapo 17 — Limites *(35 s)*

« Quatre limites identifiées. Le HC-SR04 a une résolution de 2 cm. K varie avec la batterie, ce qu'un observateur adaptatif pourrait corriger. Les oscillations résiduelles sont liées à la zone morte du moteur. Et l'inertie réelle de la voiture rend τ_réel légèrement supérieur au modèle. »

---

## Diapo 18 — Conclusion *(35 s)*

« En conclusion : l'architecture 2-DOF avec PID est la solution optimale sur cette maquette. Les critères sont validés expérimentalement. Le mode distance v19 reproduit le comportement d'un vrai ACC. Et l'ancrage est fort : la boucle fermée est littéralement un cycle discret — mesure, calcul, action — répété 20 fois par seconde. »

---

## Diapo 19 — Bibliographie *(20 s)*

« Quatre références couvrant le contexte industriel ADAS et les aspects techniques PID sur Arduino. Je reste disponible pour toute question. »

---

## Chronomètre estimé

| # | Slide | Durée | Cumul |
|---|-------|-------|-------|
| 1 | Titre | 0:30 | 0:30 |
| 2 | Ancrage + Problématique | 0:50 | 1:20 |
| 3 | Maquette | 0:45 | 2:05 |
| 4 | Identification moteur | 0:55 | 3:00 |
| 5 | Méthode fréquencemétrique | 0:40 | 3:40 |
| 6 | Méthode période | 0:45 | 4:25 |
| 7 | Architecture + mode ACC | 1:00 | 5:25 |
| 8 | P — courbe | 0:40 | 6:05 |
| 9 | P — analyse | 0:40 | 6:45 |
| 10 | PI — courbe | 0:40 | 7:25 |
| 11 | PI — analyse | 0:40 | 8:05 |
| 12 | PD — courbe | 0:40 | 8:45 |
| 13 | PD — analyse | 0:40 | 9:25 |
| 14 | PID — courbe | 0:45 | 10:10 |
| 15 | PID — analyse | 0:45 | 10:55 |
| 16 | Comparaison | 0:45 | 11:40 |
| 17 | Limites | 0:35 | 12:15 |
| 18 | Conclusion | 0:35 | 12:50 |
| 19 | Bibliographie | 0:20 | 13:10 |
| **Marge** | | **1:50** | **≤ 15:00** |

> 💡 **Si tu es en avance** : développe davantage les diapos d'analyse (9, 11, 13, 15).  
> **Si tu es en retard** : les diapos 5, 6 (méthodes de mesure) peuvent passer en 25 s chacune — dites juste « fréquencemétrique pour le log, période pour le PID car 155× plus précis ».
