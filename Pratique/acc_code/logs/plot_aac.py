"""
AAC TIPE — Tracé des courbes depuis le fichier CSV
===================================================
Utilisation :
    python plot_aac.py log_PID_Kp-50.0_Ki-8.0_Kd-5.0_000.csv
    python plot_aac.py log_P_Kp-50.0_Ki-8.0_Kd-5.0_000.csv log_PI_Kp-50.0_Ki-8.0_Kd-5.0_001.csv log_PID_Kp-50.0_Ki-8.0_Kd-5.0_002.csv

Dépendances : pip install pandas matplotlib
"""

import sys
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from pathlib import Path


# ── Couleurs par correcteur ───────────────────────────────────
COULEURS = {
    "P":   "#E24B4A",
    "PI":  "#185FA5",
    "PD":  "#3B6D11",
    "PID": "#854F0B",
}


def charger_csv(chemin: str) -> pd.DataFrame:
    """Charge un fichier CSV produit par l'Arduino."""
    df = pd.read_csv(chemin, skiprows=1)

    # Nettoyage : supprimer les éventuelles lignes corrompues
    df = df.dropna()

    # Forcer les types numériques (sécurité si un NaN s'est glissé)
    for col in ["t_s", "vitesse_freq_ms", "vitesse_periode_ms", "consigne_vitesse_ms",
                "distance_m", "consigne_dist_m", "erreur_m", "commande_pwm"]:
        df[col] = pd.to_numeric(df[col], errors="coerce")
    df = df.dropna()

    # Séparer les lignes avec/sans obstacle
    # dist = -1 signifie "pas d'obstacle" (convention du code Arduino)
    df["obstacle"] = df["distance_m"] >= 0.0

    return df


def tracer_fichier(ax_list, df: pd.DataFrame, label: str, couleur: str):
    """
    Trace les 4 courbes (vitesse, distance, erreur, commande)
    pour un fichier CSV donné.
    """
    ax_v, ax_d, ax_e, ax_c = ax_list
    t = df["t_s"]

    # ── Vitesse ───────────────────────────────────────────────
    ax_v.plot(t, df["vitesse_periode_ms"], color=couleur, lw=1.5, label=label, alpha=0.9)
    # Consigne de vitesse (trait pointillé fin, une seule fois)
    ax_v.axhline(df["consigne_vitesse_ms"].iloc[0],
                 color=couleur, lw=1, ls="--", alpha=0.4)

    # ── Distance (seulement quand un obstacle est détecté) ────
    df_obs = df[df["obstacle"]]
    if not df_obs.empty:
        ax_d.plot(df_obs["t_s"], df_obs["distance_m"],
                  color=couleur, lw=1.5, label=label, alpha=0.9)
        ax_d.axhline(df_obs["consigne_dist_m"].iloc[0],
                     color=couleur, lw=1, ls="--", alpha=0.4)

    # ── Erreur ────────────────────────────────────────────────
    ax_e.plot(t, df["erreur_m"], color=couleur, lw=1.2, label=label, alpha=0.9)
    ax_e.axhline(0, color="#888", lw=0.7, ls=":")

    # ── Commande PWM ─────────────────────────────────────────
    ax_c.plot(t, df["commande_pwm"], color=couleur, lw=1.2, label=label, alpha=0.9)
    ax_c.axhline(0, color="#888", lw=0.7, ls=":")


def main():
    if len(sys.argv) < 2:
        print("Usage : python plot_aac.py fichier1.csv [fichier2.csv ...]")
        sys.exit(1)

    fichiers = sys.argv[1:]

    correcteurs = []
    Kp = []
    Ki = []
    Kd = []

    for chemin in fichiers:
        p = Path(chemin)
        if not p.exists():
            print(f"Fichier introuvable : {chemin}")
            continue

        f = open(chemin)

        first_line = f.readline().split(',')
        correcteurs.append(first_line[0].split('=')[1])
        Kp.append(float(first_line[1].split('=')[1]))
        Ki.append(float(first_line[2].split('=')[1]))
        Kd.append(float(first_line[3].split('=')[1]))

    # ── Mise en page : 4 lignes × 1 colonne ─────────────────
    fig = plt.figure(figsize=(12, 10))
    title = f"TIPE AAC\n"
    for i in range(len(correcteurs)):
        title += f"{correcteurs[i]} : Kp={Kp[i]} | Ki={Ki[i]} | Kd={Kd[i]}\n"
    fig.suptitle(title,
                 fontsize=13, fontweight="bold", y=0.98)

    gs = gridspec.GridSpec(4, 1, hspace=0.45)
    ax_v = fig.add_subplot(gs[0])
    ax_d = fig.add_subplot(gs[1], sharex=ax_v)
    ax_e = fig.add_subplot(gs[2], sharex=ax_v)
    ax_c = fig.add_subplot(gs[3], sharex=ax_v)

    # ── Chargement et tracé de chaque fichier ────────────────
    path_index = 0
    for chemin in fichiers:
        p = Path(chemin)
        if not p.exists():
            print(f"Fichier introuvable : {chemin}")
            continue

        df = charger_csv(chemin)

        couleur = COULEURS.get(correcteurs[path_index], "#555555")
        label   = f"Correcteur {correcteurs[path_index]}"

        tracer_fichier([ax_v, ax_d, ax_e, ax_c], df, label, couleur)
        print(f"Chargé : {chemin} ({len(df)} points, correcteur={correcteurs[path_index]})")
        path_index += 1

    # ── Mise en forme des axes ────────────────────────────────
    ax_v.set_ylabel("Vitesse (m/s)")
    ax_v.set_title("Vitesse mesurée vs consigne", fontsize=10)
    ax_v.legend(fontsize=8)
    ax_v.grid(True, alpha=0.3)

    ax_d.set_ylabel("Distance (m)")
    ax_d.set_title("Distance obstacle mesurée vs consigne", fontsize=10)
    ax_d.legend(fontsize=8)
    ax_d.grid(True, alpha=0.3)

    ax_e.set_ylabel("Erreur (m)")
    ax_e.set_title("Erreur (consigne − mesure)", fontsize=10)
    ax_e.legend(fontsize=8)
    ax_e.grid(True, alpha=0.3)

    ax_c.set_ylabel("Commande (PWM)")
    ax_c.set_xlabel("Temps (s)")
    ax_c.set_title("Commande envoyée au moteur", fontsize=10)
    ax_c.legend(fontsize=8)
    ax_c.grid(True, alpha=0.3)
    ax_c.set_ylim(-270, 270)

    # Zones grises pour commande négative (freinage)
    ax_c.axhspan(-270, 0, alpha=0.04, color="red", label="zone freinage")

    # ── Sauvegarde ────────────────────────────────────────────
    # n = 1
    # nom_sortie = "courbes_AAC_000.png"
    # a = Path(nom_sortie)
    # while a.exists():
    #     nom_sortie = "courbes_AAC_" + str(n).zfill(3) + ".png"
    #     n += 1
    #     a = Path(nom_sortie)
    # plt.savefig(nom_sortie, dpi=150, bbox_inches="tight")
    # print(f"\nFigure sauvegardée : {nom_sortie}")
    plt.show()


if __name__ == "__main__":
    main()
