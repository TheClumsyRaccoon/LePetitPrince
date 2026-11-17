import numpy as np
import matplotlib.pyplot as plt
import config as P
from physique import Modele6DDL
from controleur import ControleurRaie


def executer_bilan_pid(temps, hist_cap, hist_prof, hist_vx, hist_cg, hist_cd,
                       cap_cible_deg, prof_cible, vit_cible, t_fuite):
    # Indice juste avant la fuite simulee (t = 10.5 s)
    idx_avant_fuite = int(10.5 / 0.01)

    cap_final = hist_cap[idx_avant_fuite]
    prof_finale = hist_prof[idx_avant_fuite]
    vx_finale = hist_vx[idx_avant_fuite]

    err_cap = abs(cap_final - cap_cible_deg)
    err_prof = abs(prof_finale - prof_cible)
    err_vx = abs(vx_finale - vit_cible)

    # Verification de la coupure moteur apres la fuite (t = 12.0 s)
    idx_apres_fuite = int(12.0 / 0.01)
    moteurs_coupes = (abs(hist_cg[idx_apres_fuite]) < 1e-3) and (abs(hist_cd[idx_apres_fuite]) < 1e-3)

    criteres = [
        ("Asservissement Cap", f"{cap_final:.1f} deg (cible {cap_cible_deg:.1f} deg)", err_cap < 3.0),
        ("Asservissement Profondeur", f"{prof_finale:.2f} m (cible {prof_cible:.2f} m)", err_prof < 0.15),
        ("Asservissement Vitesse", f"{vx_finale:.2f} m/s (cible {vit_cible:.2f} m/s)", err_vx < 0.08),
        ("Securite Fuite (>70% HR)", "Propulseurs coupes a 0" if moteurs_coupes else "Propulseurs actifs !", moteurs_coupes),
    ]

    print("=== Bilan des regulateurs PID (ControleurRaie) ===")
    reussis = 0
    for nom, detail, ok in criteres:
        statut = "[OK]   " if ok else "[ECHEC]"
        print(f"{statut} {nom:<26} -> {detail}")
        if ok:
            reussis += 1
    print(f"=== Resultat : {reussis}/{len(criteres)} criteres valides ===\n")


def tester_et_tracer_controleur():
    sim = Modele6DDL()
    ctrl = ControleurRaie()
    ctrl.mode_vision = False

    cap_cible_deg = 60.0
    ctrl.cap_cible = np.radians(cap_cible_deg)
    ctrl.profondeur_cible = 2.5
    ctrl.vitesse_cible = 0.6

    dt_ctrl = 0.01  # 100 Hz controle
    t_fuite = 11.0  # Simulation d'une voie d'eau a t = 11 s

    temps = []
    hist_cap, hist_prof, hist_vx = [], [], []
    hist_cg, hist_cd, hist_elev, hist_bal = [], [], [], []

    for i in range(1500):  # 15 secondes
        t = i * dt_ctrl
        humidite = 85.0 if t >= t_fuite else 45.0

        etat = {
            "vx": sim.vel_body[0],
            "yaw": sim.att[2],
            "profondeur": sim.profondeur,
            "humidite": humidite,
        }
        cg, cd, eg, ed, bal_cible = ctrl.calculer_commandes(etat, dt_ctrl)

        for _ in range(2):  # 200 Hz physique
            sim.step(cg, cd, eg, ed, bal_cible, 0.005)

        temps.append(t)
        hist_cap.append(np.degrees(sim.att[2]))
        hist_prof.append(sim.profondeur)
        hist_vx.append(sim.vel_body[0])
        hist_cg.append(cg)
        hist_cd.append(cd)
        hist_elev.append(np.degrees(eg))
        hist_bal.append(sim.ballast_ml)

    executer_bilan_pid(
        temps, hist_cap, hist_prof, hist_vx, hist_cg, hist_cd,
        cap_cible_deg, ctrl.profondeur_cible, ctrl.vitesse_cible, t_fuite
    )

    fig, axs = plt.subplots(4, 1, figsize=(10, 9), sharex=True)

    # 1. Cap
    axs[0].plot(temps, hist_cap, "m-", lw=2, label="Cap mesure (deg)")
    axs[0].axhline(cap_cible_deg, color="k", ls="--", label="Consigne (60 deg)")
    axs[0].axvline(t_fuite, color="r", ls=":", label="Alerte fuite (t=11s)")
    axs[0].set_ylabel("Cap (deg)")
    axs[0].set_title("Validation en boucle fermee - ControleurRaie (PID)")
    axs[0].grid(True)
    axs[0].legend(loc="lower right", fontsize=8)

    # 2. Profondeur
    axs[1].plot(temps, hist_prof, "c-", lw=2, label="Profondeur mesuree (m)")
    axs[1].axhline(ctrl.profondeur_cible, color="k", ls="--", label="Consigne (2.5 m)")
    axs[1].axvline(t_fuite, color="r", ls=":")
    axs[1].invert_yaxis()
    axs[1].set_ylabel("Profondeur (m)")
    axs[1].grid(True)
    axs[1].legend(loc="lower right", fontsize=8)

    # 3. Vitesse & Commandes moteurs
    axs[2].plot(temps, hist_vx, "g-", lw=2, label="Vitesse X (m/s)")
    axs[2].plot(temps, hist_cg, "b--", alpha=0.7, label="Cmd Moteur G")
    axs[2].plot(temps, hist_cd, "r--", alpha=0.7, label="Cmd Moteur D")
    axs[2].axvline(t_fuite, color="r", ls=":")
    axs[2].set_ylabel("Vitesse / Gaz")
    axs[2].grid(True)
    axs[2].legend(loc="upper right", fontsize=8)

    # 4. Actionneurs verticaux (Ballast vs Elevons)
    ax4_bis = axs[3].twinx()
    l1 = axs[3].plot(temps, hist_bal, color="orange", lw=2, label="Ballast (mL)")
    l2 = ax4_bis.plot(temps, hist_elev, color="purple", ls="-.", lw=1.5, label="Elevons (deg)")
    axs[3].axvline(t_fuite, color="r", ls=":")
    axs[3].set_ylabel("Ballast (mL)", color="orange")
    ax4_bis.set_ylabel("Elevons (deg)", color="purple")
    axs[3].set_xlabel("Temps (s)")
    axs[3].grid(True)
    lignes = l1 + l2
    axs[3].legend(lignes, [l.get_label() for l in lignes], loc="upper right", fontsize=8)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    tester_et_tracer_controleur()