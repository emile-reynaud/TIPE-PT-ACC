"""
=============================================================
  SIMULATION DU RÉGULATEUR AAC — TIPE CPGE PT
=============================================================
  Modèle physique simplifié de la voiture télécommandée :
    - moteur brushed RC390 + L298N modélisés comme un système
      du 1er ordre : τ_moteur × dv/dt + v = K_moteur × u
    - frottements visqueux + frottement sec (seuil de démarrage)
    - la commande u est le PWM normalisé entre 0 et 1
      (0 = arrêt, 1 = pleine puissance)

  Le simulateur reproduit EXACTEMENT la logique du code Arduino :
    - même structure P / PI / PD / PID
    - même anti-windup par saturation de l'intégrale
    - même kick-start (impulsion de démarrage)
    - même lissage par rampe
    - même gestion sans obstacle / avec obstacle
    - même quantification Hall (3 aimants) optionnelle

  Usage :
    python simulation_AAC.py              → lance la GUI interactive
    python simulation_AAC.py --batch      → trace les 4 correcteurs d'un coup

  Dépendances : pip install numpy matplotlib
=============================================================
"""

import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Slider, Button, RadioButtons

# ─────────────────────────────────────────────────────────────
#   PARAMÈTRES DU MODÈLE PHYSIQUE
#   À ajuster pour coller à votre voiture réelle.
# ─────────────────────────────────────────────────────────────

# Constante de temps du moteur (s) :
# temps pour atteindre 63 % de la vitesse finale en réponse à un échelon.
# Un RC390 + L298N typique : 0.15 à 0.35 s.
TAU_MOTEUR = 0.1167

# Gain statique du moteur (m/s par unité de PWM normalisé) :
# vitesse finale à PWM = 1.0 (pleine puissance).
# Mesurer : appliquer PWM 255 en continu, mesurer la vitesse de régime.
# Valeur typique pour une RC à ~1,5 km/h max : 0.45 m/s.
K_MOTEUR = 5.7669

# Frottement sec (Coulomb) : force de résistance constante,
# exprimée en m/s/s (décélération à PWM nul).
# Modélise la résistance mécanique au démarrage.
FROTT_SEC_MS2 = 0.8

# Frottement visqueux : proportionnel à la vitesse.
# Coefficient en s⁻¹ (décélération = FROTT_VISC × v).
FROTT_VISC = 0.5

# Seuil de démarrage moteur : PWM normalisé minimal pour
# vaincre les frottements et commencer à tourner.
# Correspond à PWM ≈ 60-80 sur 255.
SEUIL_DEMARRAGE = 0.29

# ─────────────────────────────────────────────────────────────
#   PARAMÈTRES DE SIMULATION (identiques au code Arduino)
# ─────────────────────────────────────────────────────────────
DT          = 0.050        # pas de temps (s) = CONTROL_PERIOD_MS
T_TOTAL     = 10.0         # durée de simulation (s)
PWM_MAX     = 255          # valeur max de la commande Arduino

# Consignes (identiques au code Arduino)
VITESSE_CIBLE_MS   = 0.40  # m/s
DIST_CONSIGNE_M    = 0.30  # m
DIST_URGENCE_M     = 0.10  # m
DIST_MAX_M         = 2.00  # au-delà : pas d'obstacle

# Paramètres kick-start (identiques au code Arduino)
KICK_PWM_NORM      = 200 / PWM_MAX   # normalisé
KICK_DUREE_S       = 0.080           # 80 ms
VITESSE_QUASI_NULLE = 0.03           # m/s
COMMANDE_MIN_KICK  = 5 / PWM_MAX

# Rampe de lissage
RAMPE_PAR_S        = 350 / PWM_MAX   # en unités normalisées/s

# Anti-windup
INTEGRALE_MAX_SI = 100 / PWM_MAX     # normalisé (l'intégrale est en unités de commande)


# ─────────────────────────────────────────────────────────────
#   MODÈLE PHYSIQUE : intégration d'Euler du 1er ordre
# ─────────────────────────────────────────────────────────────

def modele_moteur(v, u_norm, dt):
    """
    Calcule la nouvelle vitesse après un pas de temps dt.

    Équation du modèle :
        τ × dv/dt = K × sat(u) - v - frottements
    Discrétisée en Euler explicite :
        v(k+1) = v(k) + dt/τ × (K × u_eff - v(k)) - frott

    u_norm : commande normalisée entre -1 (freinage max) et +1 (avance max)
    """
    # Saturation de la commande
    u_norm = np.clip(u_norm, -1.0, 1.0)

    if u_norm >= 0:
        # Avance : le moteur produit une force si u dépasse le seuil
        if u_norm > SEUIL_DEMARRAGE:
            u_eff = u_norm
        else:
            u_eff = 0.0  # moteur ne démarre pas encore

        # Force motrice nette (en m/s/s équivalent)
        force = (K_MOTEUR * u_eff - v) / TAU_MOTEUR

        # Frottements (s'opposent au mouvement)
        if v > 0.001:
            force -= FROTT_VISC * v + FROTT_SEC_MS2 * dt / DT * 0.1
        elif u_eff == 0.0:
            force = 0.0  # immobile sans commande : reste à l'arrêt

    else:
        # Freinage actif : on court-circuite le moteur
        # La décélération est proportionnelle à |u| et à la vitesse
        force = -abs(u_norm) * (v / TAU_MOTEUR + FROTT_VISC * v)
        if v < 0.001:
            force = 0.0

    v_new = v + force * dt
    return max(0.0, v_new)  # la voiture ne recule pas dans ce modèle


# ─────────────────────────────────────────────────────────────
#   QUANTIFICATION HALL (optionnelle)
#   Simule l'arrondi à l'entier inférieur du compteur d'aimants
# ─────────────────────────────────────────────────────────────

PERIMETRE_M = np.pi * 0.083
NB_AIMANTS  = 3
PAS_M       = PERIMETRE_M / NB_AIMANTS

def vitesse_quantifiee(v_reelle, dt):
    """
    Simule la mesure de vitesse par méthode fréquencemétrique
    (comptage d'impulsions sur DT). Retourne la vitesse quantifiée.
    """
    distance = v_reelle * dt
    nb_impulsions = int(distance / PAS_M)  # arrondi inférieur = quantification
    return nb_impulsions * PAS_M / dt


# ─────────────────────────────────────────────────────────────
#   CORRECTEUR PID (identique au code Arduino)
# ─────────────────────────────────────────────────────────────

class CorrecteurPID:
    """
    Implémente exactement la logique du code Arduino :
    - termes P, I, D activables indépendamment
    - anti-windup par saturation
    - gel de l'intégrale pendant le kick-start
    - kick-start + rampe de lissage
    """

    def __init__(self, Kp, Ki, Kd, use_I, use_D):
        self.Kp    = Kp / PWM_MAX   # normalisation : gains en unités normalisées
        self.Ki    = Ki / PWM_MAX
        self.Kd    = Kd / PWM_MAX
        self.use_I = use_I
        self.use_D = use_D

        # État interne
        self.integrale         = 0.0
        self.erreur_precedente = 0.0
        self.kick_actif        = False
        self.kick_debut_s      = 0.0
        self.commande_lissee   = 0.0

    def reset(self):
        self.integrale         = 0.0
        self.erreur_precedente = 0.0
        self.kick_actif        = False
        self.kick_debut_s      = 0.0
        self.commande_lissee   = 0.0

    def calculer(self, erreur, dt, v_mesuree, t_courant):
        """
        Calcule la commande normalisée [-1, +1].
        erreur    : consigne - mesure (m/s ou m selon le mode)
        dt        : pas de temps (s)
        v_mesuree : vitesse actuelle (m/s), pour le kick-start
        t_courant : temps courant (s), pour la durée du kick
        """
        # ── Terme P ──────────────────────────────────────────
        u_P = self.Kp * erreur

        # ── Terme I ──────────────────────────────────────────
        u_I = 0.0
        if self.use_I:
            # Gel pendant le kick (même logique qu'Arduino)
            if not self.kick_actif:
                self.integrale += erreur * dt
                self.integrale = np.clip(self.integrale,
                                         -INTEGRALE_MAX_SI, INTEGRALE_MAX_SI)
            u_I = self.Ki * self.integrale

        # ── Terme D ──────────────────────────────────────────
        u_D = 0.0
        if self.use_D:
            if dt > 0:
                derivee = (erreur - self.erreur_precedente) / dt
            else:
                derivee = 0.0
            u_D = self.Kd * derivee
        self.erreur_precedente = erreur

        commande_correcteur = u_P + u_I + u_D

        # ── Kick-start + rampe ────────────────────────────────
        commande_finale = self._appliquer_kick_et_rampe(
            commande_correcteur, dt, v_mesuree, t_courant
        )
        return commande_finale

    def _appliquer_kick_et_rampe(self, cmd_correcteur, dt, v, t):
        # Urgence / freinage : pas de rampe ni de kick
        if cmd_correcteur < 0.0:
            self.kick_actif = False
            self.commande_lissee = cmd_correcteur
            return np.clip(cmd_correcteur, -1.0, 1.0)

        # Kick en cours ?
        if self.kick_actif:
            duree_ecoulee = t - self.kick_debut_s
            if duree_ecoulee < KICK_DUREE_S and v < VITESSE_QUASI_NULLE:
                self.commande_lissee = KICK_PWM_NORM
                return KICK_PWM_NORM
            else:
                # Kick terminé
                self.kick_actif = False
                self.commande_lissee = KICK_PWM_NORM  # rampe part de là

        # Déclencher un nouveau kick si roue à l'arrêt
        if cmd_correcteur > COMMANDE_MIN_KICK and v < VITESSE_QUASI_NULLE:
            self.kick_actif  = True
            self.kick_debut_s = t
            self.commande_lissee = KICK_PWM_NORM
            return KICK_PWM_NORM

        # Rampe de lissage
        cible    = np.clip(cmd_correcteur, 0.0, 1.0)
        delta_max = RAMPE_PAR_S * dt
        delta    = np.clip(cible - self.commande_lissee, -delta_max, delta_max)
        self.commande_lissee += delta

        return np.clip(self.commande_lissee, 0.0, 1.0)


# ─────────────────────────────────────────────────────────────
#   SCÉNARIO D'OBSTACLE
# ─────────────────────────────────────────────────────────────

def distance_obstacle(t, scenario):
    """
    Retourne la distance à l'obstacle en fonction du temps.
    scenario : 'aucun', 'fixe', 'rapprochement', 'eloignement', 'mixte'
    """
    if scenario == 'aucun':
        return -1.0  # pas d'obstacle

    elif scenario == 'fixe':
        # Obstacle fixe à 0,5 m dès t=3 s
        return 0.50 if t >= 3.0 else -1.0

    elif scenario == 'rapprochement':
        # Obstacle qui se rapproche de 2 m à 0,15 m entre t=2 s et t=6 s
        if t < 2.0:
            return -1.0
        elif t < 6.0:
            return max(0.15, 2.0 - (t - 2.0) * 0.46)
        else:
            return 0.15

    elif scenario == 'eloignement':
        # Obstacle à 0,20 m puis qui s'éloigne
        if t < 2.0:
            return -1.0
        elif t < 5.0:
            return min(2.5, 0.20 + (t - 2.0) * 0.77)
        else:
            return -1.0

    elif scenario == 'mixte':
        # Rapprochement puis éloignement
        if t < 1.5:
            return -1.0
        elif t < 4.0:
            return max(0.12, 1.8 - (t - 1.5) * 0.67)
        elif t < 7.0:
            return min(2.5, 0.12 + (t - 4.0) * 0.79)
        else:
            return -1.0

    return -1.0


# ─────────────────────────────────────────────────────────────
#   BOUCLE DE SIMULATION PRINCIPALE
# ─────────────────────────────────────────────────────────────

def simuler(Kp, Ki, Kd, mode, scenario='aucun', quantifier_hall=False):
    """
    Lance une simulation complète et retourne les séries temporelles.

    Retourne un dict avec les clés :
      t, vitesse, commande, erreur, distance, integrale
    """
    use_I = mode in ('PI', 'PID')
    use_D = mode in ('PD', 'PID')

    correcteur = CorrecteurPID(Kp, Ki, Kd, use_I, use_D)

    N  = int(T_TOTAL / DT)
    t  = np.zeros(N)
    v  = np.zeros(N)       # vitesse réelle
    vm = np.zeros(N)       # vitesse mesurée (potentiellement quantifiée)
    u  = np.zeros(N)       # commande normalisée
    e  = np.zeros(N)       # erreur
    d  = np.zeros(N)       # distance obstacle
    ig = np.zeros(N)       # intégrale

    vitesse = 0.0

    for k in range(N):
        t_k = k * DT

        # Mesure de vitesse
        v_mesuree = vitesse_quantifiee(vitesse, DT) if quantifier_hall else vitesse

        # Distance obstacle
        dist = distance_obstacle(t_k, scenario)

        # Logique de régulation (identique au code Arduino)
        if dist > 0.0 and dist < DIST_URGENCE_M:
            # Freinage d'urgence
            erreur   = dist - DIST_CONSIGNE_M
            cmd_corr = -1.0  # freinage max normalisé
            correcteur.integrale = 0.0
            correcteur.erreur_precedente = 0.0
            commande = correcteur._appliquer_kick_et_rampe(cmd_corr, DT, v_mesuree, t_k)

        elif dist < 0.0 or dist >= DIST_MAX_M:
            # Pas d'obstacle : régulation sur la vitesse
            erreur   = VITESSE_CIBLE_MS - v_mesuree
            commande = correcteur.calculer(erreur, DT, v_mesuree, t_k)

        else:
            # Obstacle dans la zone : régulation sur la distance
            erreur   = dist - DIST_CONSIGNE_M
            commande = correcteur.calculer(erreur, DT, v_mesuree, t_k)

        # Intégration du modèle physique
        vitesse = modele_moteur(vitesse, commande, DT)

        # Stockage
        t[k]  = t_k
        v[k]  = vitesse
        vm[k] = v_mesuree
        u[k]  = commande * PWM_MAX   # repasser en PWM pour l'affichage
        e[k]  = erreur
        d[k]  = dist
        ig[k] = correcteur.integrale * PWM_MAX  # en unités PWM pour lisibilité

    return dict(t=t, vitesse=v, vitesse_mesuree=vm,
                commande=u, erreur=e, distance=d, integrale=ig)


# ─────────────────────────────────────────────────────────────
#   TRACÉ : 4 SOUS-GRAPHIQUES
# ─────────────────────────────────────────────────────────────

COULEURS = {'P': '#E24B4A', 'PI': '#185FA5', 'PD': '#3B8C1A', 'PID': '#9B6000'}

def tracer_resultats(ax_list, res, mode, label_suffix=''):
    c = COULEURS.get(mode, '#555')
    ax_v, ax_e, ax_u, ax_d = ax_list

    label = f"{mode}{label_suffix}"

    ax_v.plot(res['t'], res['vitesse'], color=c, lw=1.8, label=label)
    ax_v.axhline(VITESSE_CIBLE_MS, color=c, lw=0.8, ls='--', alpha=0.5)

    ax_e.plot(res['t'], res['erreur'], color=c, lw=1.4, label=label)
    ax_e.axhline(0, color='#aaa', lw=0.6, ls=':')

    ax_u.plot(res['t'], res['commande'], color=c, lw=1.4, label=label)
    ax_u.axhline(0, color='#aaa', lw=0.6, ls=':')

    # Distance : ne tracer que là où il y a un obstacle
    d = res['distance']
    mask = d >= 0
    if mask.any():
        ax_d.plot(res['t'][mask], d[mask], color=c, lw=1.4, label=label)
        ax_d.axhline(DIST_CONSIGNE_M, color=c, lw=0.8, ls='--', alpha=0.5)


def configurer_axes(ax_list, scenario):
    ax_v, ax_e, ax_u, ax_d = ax_list

    ax_v.set_ylabel('Vitesse (m/s)')
    ax_v.set_title('Vitesse mesurée', fontsize=10)
    ax_v.legend(fontsize=8, loc='lower right')
    ax_v.grid(True, alpha=0.25)
    ax_v.set_ylim(-0.05, K_MOTEUR * 1.3)

    ax_e.set_ylabel('Erreur')
    ax_e.set_title('Erreur (consigne − mesure)', fontsize=10)
    ax_e.legend(fontsize=8)
    ax_e.grid(True, alpha=0.25)

    ax_u.set_ylabel('Commande (PWM)')
    ax_u.set_title('Commande moteur', fontsize=10)
    ax_u.legend(fontsize=8)
    ax_u.set_ylim(-270, 270)
    ax_u.axhspan(-270, 0, alpha=0.04, color='red')
    ax_u.grid(True, alpha=0.25)

    ax_d.set_ylabel('Distance (m)')
    ax_d.set_xlabel('Temps (s)')
    ax_d.set_title(f'Distance obstacle — scénario : {scenario}', fontsize=10)
    ax_d.legend(fontsize=8)
    ax_d.grid(True, alpha=0.25)
    if scenario != 'aucun':
        ax_d.axhline(DIST_CONSIGNE_M, color='gray', lw=0.8,
                     ls='--', label='consigne dist')
        ax_d.axhline(DIST_URGENCE_M,  color='red',  lw=0.8,
                     ls=':', label='urgence')


# ─────────────────────────────────────────────────────────────
#   MODE BATCH : compare les 4 correcteurs
# ─────────────────────────────────────────────────────────────

def mode_batch():
    # Paramètres de départ conseillés
    configs = [
        ('P',   50,  0,  0),
        ('PI',  40,  8,  0),
        ('PD',  40,  0,  5),
        ('PID', 35,  6,  4),
    ]

    scenario = 'mixte'

    fig = plt.figure(figsize=(14, 9))
    fig.suptitle('Simulation AAC — Comparaison P / PI / PD / PID\n'
                 f'Modèle : τ={TAU_MOTEUR}s, K={K_MOTEUR}m/s  |  '
                 f'Scénario obstacle : {scenario}',
                 fontsize=11, fontweight='bold')
    gs = gridspec.GridSpec(4, 1, hspace=0.50)
    axes = [fig.add_subplot(gs[i]) for i in range(4)]

    for mode, Kp, Ki, Kd in configs:
        res = simuler(Kp, Ki, Kd, mode, scenario)
        tracer_resultats(axes, res, mode)

    configurer_axes(axes, scenario)
    plt.savefig('simulation_batch.png', dpi=150, bbox_inches='tight')
    print("Figure sauvegardée : simulation_batch.png")
    plt.show()


# ─────────────────────────────────────────────────────────────
#   MODE INTERACTIF : sliders pour régler les gains en temps réel
# ─────────────────────────────────────────────────────────────

def mode_interactif():
    fig = plt.figure(figsize=(14, 10))
    fig.suptitle('Simulation AAC — Réglage interactif des gains PID',
                 fontsize=11, fontweight='bold')

    # Zone de tracé
    gs_plot = gridspec.GridSpec(4, 1, left=0.10, right=0.65,
                                 hspace=0.50, top=0.92, bottom=0.06)
    ax_v = fig.add_subplot(gs_plot[0])
    ax_e = fig.add_subplot(gs_plot[1], sharex=ax_v)
    ax_u = fig.add_subplot(gs_plot[2], sharex=ax_v)
    ax_d = fig.add_subplot(gs_plot[3], sharex=ax_v)
    axes = [ax_v, ax_e, ax_u, ax_d]

    # ── Sliders (à droite) ───────────────────────────────────
    slider_color = '#dce6f0'

    def make_slider(ax_rect, label, valmin, valmax, valinit, step=0.5):
        ax_s = fig.add_axes(ax_rect, facecolor=slider_color)
        s = Slider(ax_s, label, valmin, valmax, valinit=valinit,
                   valstep=step, color='#4a90d9')
        return s

    sl_Kp = make_slider([0.70, 0.80, 0.26, 0.03], 'Kp',  0, 10, 5, 0.1)
    sl_Ki = make_slider([0.70, 0.73, 0.26, 0.03], 'Ki',  0,  10,  0, 0.1)
    sl_Kd = make_slider([0.70, 0.66, 0.26, 0.03], 'Kd',  0,  10,  0, 0.1)

    # ── Sélecteur de mode ────────────────────────────────────
    ax_radio = fig.add_axes([0.70, 0.42, 0.26, 0.20], facecolor='#f0f0f0')
    radio = RadioButtons(ax_radio, ('P', 'PI', 'PD', 'PID'), active=3,
                         activecolor='#4a90d9')
    ax_radio.set_title('Correcteur', fontsize=9, pad=4)

    # ── Sélecteur de scénario ────────────────────────────────
    ax_scen = fig.add_axes([0.70, 0.20, 0.26, 0.18], facecolor='#f0f0f0')
    radio_scen = RadioButtons(ax_scen,
        ('aucun', 'fixe', 'rapprochement', 'eloignement', 'mixte'),
        active=0, activecolor='#e05c3a')
    ax_scen.set_title('Scénario obstacle', fontsize=9, pad=4)

    # ── Checkbox quantification Hall ─────────────────────────
    ax_quant = fig.add_axes([0.70, 0.13, 0.26, 0.05])
    btn_quant = Button(ax_quant, 'Quantif. Hall : OFF',
                       color='#f0f0f0', hovercolor='#d0e8ff')
    quant_state = [False]

    def toggle_quant(event):
        quant_state[0] = not quant_state[0]
        btn_quant.label.set_text(
            f"Quantif. Hall : {'ON' if quant_state[0] else 'OFF'}")
        update(None)
    btn_quant.on_clicked(toggle_quant)

    # ── Bouton export PNG ─────────────────────────────────────
    ax_save = fig.add_axes([0.70, 0.06, 0.26, 0.05])
    btn_save = Button(ax_save, 'Sauvegarder PNG',
                      color='#e8f4e8', hovercolor='#c8e8c8')

    def sauvegarder(event):
        mode   = radio.value_selected
        fname  = f"sim_{mode}_Kp{sl_Kp.val:.0f}_Ki{sl_Ki.val:.1f}_Kd{sl_Kd.val:.1f}.png"
        fig.savefig(fname, dpi=150, bbox_inches='tight')
        print(f"Sauvegardé : {fname}")
    btn_save.on_clicked(sauvegarder)

    # ── Lignes tracées (pour mise à jour efficace) ────────────
    lines = {}   # dictionnaire ligne_name → Line2D

    def update(val):
        Kp     = sl_Kp.val
        Ki     = sl_Ki.val
        Kd     = sl_Kd.val
        mode   = radio.value_selected
        scen   = radio_scen.value_selected
        quant  = quant_state[0]

        res = simuler(Kp, Ki, Kd, mode, scen, quant)
        c   = COULEURS.get(mode, '#555')

        # Effacer et re-tracer
        for ax in axes:
            ax.cla()

        tracer_resultats(axes, res, mode,
                         f'  Kp={Kp:.0f} Ki={Ki:.1f} Kd={Kd:.1f}')
        configurer_axes(axes, scen)

        # Annoter les performances
        v_arr = res['vitesse']
        t_arr = res['t']
        # Temps de réponse à 95 % (premier passage à 95 % de la consigne)
        idx95 = np.where(v_arr >= 0.95 * VITESSE_CIBLE_MS)[0]
        if len(idx95) > 0:
            tr = t_arr[idx95[0]]
            ax_v.axvline(tr, color=c, lw=0.8, ls=':', alpha=0.7)
            ax_v.text(tr + 0.05, 0.05, f'tr={tr:.2f}s',
                      color=c, fontsize=7, va='bottom')

        # Dépassement
        v_max = v_arr.max()
        if v_max > VITESSE_CIBLE_MS * 1.01:
            dep = (v_max - VITESSE_CIBLE_MS) / VITESSE_CIBLE_MS * 100
            ax_v.text(0.98, 0.95, f'Dép. {dep:.1f}%',
                      transform=ax_v.transAxes, ha='right', va='top',
                      color='red', fontsize=8)

        fig.canvas.draw_idle()

    # Connecter les widgets
    sl_Kp.on_changed(update)
    sl_Ki.on_changed(update)
    sl_Kd.on_changed(update)
    radio.on_clicked(update)
    radio_scen.on_clicked(update)

    # Premier tracé
    update(None)
    plt.show()


# ─────────────────────────────────────────────────────────────
#   POINT D'ENTRÉE
# ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Simulation AAC TIPE')
    parser.add_argument('--batch', action='store_true',
                        help='Comparer P/PI/PD/PID sans interface interactive')
    args = parser.parse_args()

    if args.batch:
        mode_batch()
    else:
        mode_interactif()
