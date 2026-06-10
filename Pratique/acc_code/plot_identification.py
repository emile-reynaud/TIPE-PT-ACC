"""
=============================================================
  TRACÉ D'IDENTIFICATION MOTEUR — AAC TIPE CPGE PT
=============================================================
  Lit le fichier texte copié depuis le moniteur série Arduino
  et produit :
    1) Les courbes de réponse indicielle pour chaque échelon
    2) La courbe v_∞(U₀) pour vérifier la linéarité → K
    3) La courbe τ(PWM) pour vérifier la constance de τ
    4) La courbe théorique v(t) = K·U₀·(1−e^{−t/τ}) superposée

  Usage :
    python plot_identification.py serial_log.txt

  Pour obtenir serial_log.txt :
    - Ouvrir le moniteur série Arduino (115200 bauds)
    - Lancer l'identification
    - Copier-coller TOUT le contenu dans un fichier .txt

  Dépendances : pip install numpy matplotlib
=============================================================
"""

import sys
import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from collections import defaultdict


# ─────────────────────────────────────────────────────────────
#   PARSING DU FICHIER LOG
# ─────────────────────────────────────────────────────────────

def parse_log(filepath):
    """
    Lit le fichier log et retourne :
      - data   : dict {pwm_val: {'t': [], 'v_freq': [], 'v_per': []}}
      - resume : dict {pwm_val: {'v_inf': float, 'tau_ms': float, 'K': float}}
      - params : dict avec K_MOTEUR, TAU_MOTEUR, SEUIL conseillés
    """
    data   = defaultdict(lambda: {'t': [], 'v_freq': [], 'v_per': []})
    resume = {}
    params = {}

    pwm_courant = None

    with open(filepath, 'r', encoding='utf-8', errors='replace') as f:
        for line in f:
            line = line.strip()

            # Ligne d'en-tête d'échelon : # --- Echelon PWM = 120 ---
            m = re.match(r'#\s*---\s*Echelon PWM\s*=\s*(\d+)', line)
            if m:
                pwm_courant = int(m.group(1))
                continue

            # Ligne de résultat : # Resultat : v_inf=0.3821 m/s  tau=245.0 ms  K=0.8123
            m = re.match(r'#\s*Resultat\s*:.*v_inf=([0-9.]+).*tau=([0-9.]+|indetermine).*K=([0-9.]+)', line)
            if m and pwm_courant is not None:
                v_inf_val = float(m.group(1))
                tau_val   = float(m.group(2)) if m.group(2) != 'indetermine' else np.nan
                K_val     = float(m.group(3))
                resume[pwm_courant] = {'v_inf': v_inf_val, 'tau_ms': tau_val, 'K': K_val}
                continue

            # Lignes de résumé final
            m = re.match(r'#\s*K_MOTEUR\s*=\s*([0-9.]+)', line)
            if m:
                params['K_MOTEUR'] = float(m.group(1))
                continue

            m = re.match(r'#\s*TAU_MOTEUR\s*=\s*([0-9.]+)', line)
            if m:
                params['TAU_MOTEUR'] = float(m.group(1))
                continue

            m = re.match(r'#\s*SEUIL_DEMARRAGE\s*=\s*([0-9.]+)', line)
            if m:
                params['SEUIL'] = float(m.group(1))
                continue

            # Ligne CSV de données : t_ms,pwm,vitesse_freq_ms,vitesse_per_ms
            # Ignorer les en-têtes
            if line.startswith('t_ms'):
                continue
            # Ignorer les commentaires
            if line.startswith('#') or line == '':
                continue

            parts = line.split(',')
            if len(parts) == 4 and pwm_courant is not None:
                try:
                    t      = float(parts[0])
                    pwm    = int(parts[1])
                    v_freq = float(parts[2])
                    v_per  = float(parts[3])
                    data[pwm_courant]['t'].append(t)
                    data[pwm_courant]['v_freq'].append(v_freq)
                    data[pwm_courant]['v_per'].append(v_per)
                except ValueError:
                    pass

    # Conversion en numpy
    for pwm in data:
        for key in ('t', 'v_freq', 'v_per'):
            data[pwm][key] = np.array(data[pwm][key])

    return data, resume, params


# ─────────────────────────────────────────────────────────────
#   COURBE THÉORIQUE DU 1er ORDRE
# ─────────────────────────────────────────────────────────────

def courbe_theorique(t_arr, K, tau_s, U0):
    """v(t) = K × U₀ × (1 − e^{−t/τ})"""
    return K * U0 * (1 - np.exp(-t_arr / tau_s))


# ─────────────────────────────────────────────────────────────
#   COULEURS PAR PWM
# ─────────────────────────────────────────────────────────────

def couleur_pwm(pwm, pwm_min=40, pwm_max=240):
    """Dégradé du bleu (faible PWM) au rouge (fort PWM)."""
    t = (pwm - pwm_min) / max(pwm_max - pwm_min, 1)
    return plt.cm.RdYlBu_r(t)


# ─────────────────────────────────────────────────────────────
#   TRACÉ PRINCIPAL
# ─────────────────────────────────────────────────────────────

def tracer(data, resume, params):
    pwm_list = sorted(data.keys())
    if not pwm_list:
        print("Aucune donnée trouvée dans le fichier.")
        return

    K_moy   = params.get('K_MOTEUR',   None)
    tau_moy = params.get('TAU_MOTEUR', None)

    fig = plt.figure(figsize=(15, 11))
    fig.suptitle(
        'Identification moteur RC390 — Réponses indicielles\n'
        + (f'K = {K_moy:.4f} m/s/unité  |  τ = {tau_moy*1000:.1f} ms'
           if K_moy and tau_moy else ''),
        fontsize=12, fontweight='bold'
    )

    gs = gridspec.GridSpec(2, 2, hspace=0.45, wspace=0.35,
                           left=0.07, right=0.97, top=0.88, bottom=0.07)

    ax_step  = fig.add_subplot(gs[0, :])   # réponses indicielles (pleine largeur)
    ax_vinf  = fig.add_subplot(gs[1, 0])   # v_∞ vs U₀
    ax_tau   = fig.add_subplot(gs[1, 1])   # τ vs PWM

    # ── 1. Réponses indicielles ───────────────────────────────
    for pwm in pwm_list:
        d  = data[pwm]
        c  = couleur_pwm(pwm, pwm_list[0], pwm_list[-1])
        t  = d['t'] / 1000.0  # ms → s
        # Vitesse affichée : période si dispo, sinon fréquencemétrique
        v  = np.where(d['v_per'] > 0.01, d['v_per'], d['v_freq'])

        ax_step.plot(t, v, color=c, lw=1.5, label=f'PWM={pwm}')

        # Superposition de la courbe théorique si on a K et τ
        if pwm in resume and K_moy and tau_moy:
            r   = resume[pwm]
            U0  = pwm / 255.0
            t_th = np.linspace(0, t[-1] if len(t) > 0 else 3.0, 300)
            v_th = courbe_theorique(t_th, K_moy, tau_moy, U0)
            ax_step.plot(t_th, v_th, color=c, lw=0.8, ls='--', alpha=0.6)

        # Marquer v_∞ estimée
        if pwm in resume and resume[pwm]['v_inf'] > 0.01:
            ax_step.axhline(resume[pwm]['v_inf'], color=c,
                            lw=0.5, ls=':', alpha=0.5)

    # Marquer τ (ligne verticale à t = τ_moy)
    if tau_moy:
        ax_step.axvline(tau_moy, color='gray', lw=1.0, ls='--', alpha=0.7,
                        label=f'τ = {tau_moy*1000:.0f} ms')
        ax_step.text(tau_moy + 0.02, 0.01, f'τ={tau_moy*1000:.0f} ms',
                     color='gray', fontsize=8)

    ax_step.set_xlabel('Temps (s)')
    ax_step.set_ylabel('Vitesse (m/s)')
    ax_step.set_title('Réponses indicielles — trait plein : mesuré, tiret : modèle 1er ordre',
                      fontsize=10)
    ax_step.legend(fontsize=7, ncol=3, loc='lower right')
    ax_step.grid(True, alpha=0.25)
    ax_step.set_xlim(left=0)
    ax_step.set_ylim(bottom=0)

    # ── 2. v_∞ vs U₀ (linéarité → K) ─────────────────────────
    pwm_ok  = [p for p in pwm_list if p in resume and resume[p]['v_inf'] > 0.01]
    U0_arr  = np.array([p / 255.0 for p in pwm_ok])
    vi_arr  = np.array([resume[p]['v_inf'] for p in pwm_ok])

    if len(pwm_ok) >= 2:
        ax_vinf.scatter(U0_arr, vi_arr, color='steelblue', zorder=5, s=50)
        for p, u, v in zip(pwm_ok, U0_arr, vi_arr):
            ax_vinf.annotate(str(p), (u, v), textcoords='offset points',
                             xytext=(4, 4), fontsize=7)

        # Régression linéaire forcée par l'origine : v_∞ = K × U₀
        K_fit = np.dot(U0_arr, vi_arr) / np.dot(U0_arr, U0_arr)
        u_fit = np.linspace(0, U0_arr.max() * 1.05, 100)
        ax_vinf.plot(u_fit, K_fit * u_fit, 'r--', lw=1.5,
                     label=f'Régression : K = {K_fit:.4f} m/s')
        ax_vinf.legend(fontsize=8)

    ax_vinf.set_xlabel('Commande normalisée U₀ = PWM / 255')
    ax_vinf.set_ylabel('v_∞ (m/s)')
    ax_vinf.set_title('Vitesse de régime vs commande\n(linéarité → K)', fontsize=10)
    ax_vinf.grid(True, alpha=0.25)
    ax_vinf.set_xlim(left=0)
    ax_vinf.set_ylim(bottom=0)

    # ── 3. τ vs PWM ────────────────────────────────────────────
    pwm_tau = [p for p in pwm_list if p in resume and not np.isnan(resume[p]['tau_ms'])]
    tau_arr = np.array([resume[p]['tau_ms'] for p in pwm_tau])

    if len(pwm_tau) >= 1:
        ax_tau.scatter(pwm_tau, tau_arr, color='darkorange', zorder=5, s=50)
        for p, tval in zip(pwm_tau, tau_arr):
            ax_tau.annotate(f'{tval:.0f}', (p, tval),
                            textcoords='offset points', xytext=(4, 4), fontsize=7)

        if len(pwm_tau) >= 2:
            tau_mean = np.mean(tau_arr)
            ax_tau.axhline(tau_mean, color='red', lw=1.2, ls='--',
                           label=f'Moyenne : τ = {tau_mean:.1f} ms')
            ax_tau.legend(fontsize=8)

    ax_tau.set_xlabel('PWM')
    ax_tau.set_ylabel('τ (ms)')
    ax_tau.set_title('Constante de temps τ vs PWM\n(idéalement constant)', fontsize=10)
    ax_tau.grid(True, alpha=0.25)

    # ── Résumé console ────────────────────────────────────────
    print("\n=== RÉSUMÉ IDENTIFICATION ===")
    print(f"{'PWM':>5}  {'U₀':>6}  {'v_∞ (m/s)':>10}  {'K (m/s)':>9}  {'τ (ms)':>8}")
    print("-" * 48)
    for p in pwm_list:
        if p in resume:
            r  = resume[p]
            U0 = p / 255.0
            tau_str = f"{r['tau_ms']:.1f}" if not np.isnan(r['tau_ms']) else "  N/A"
            print(f"{p:>5}  {U0:>6.3f}  {r['v_inf']:>10.4f}  {r['K']:>9.4f}  {tau_str:>8}")

    print("\n=== VALEURS POUR simulation_AAC.py ===")
    if K_moy:   print(f"TAU_MOTEUR    = {tau_moy:.4f}   # secondes")
    if tau_moy: print(f"K_MOTEUR      = {K_moy:.4f}   # m/s")
    if 'SEUIL' in params:
        print(f"SEUIL_DEMARRAGE = {params['SEUIL']:.3f}   # normalisé [0-1]")

    plt.savefig('identification_moteur.png', dpi=150, bbox_inches='tight')
    print("\nFigure sauvegardée : identification_moteur.png")
    plt.show()


# ─────────────────────────────────────────────────────────────
#   POINT D'ENTRÉE
# ─────────────────────────────────────────────────────────────

if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage : python plot_identification.py serial_log.txt")
        print("        Copier-coller le moniteur série Arduino dans serial_log.txt")
        sys.exit(1)

    filepath = sys.argv[1]
    data, resume, params = parse_log(filepath)
    tracer(data, resume, params)
