import numpy as np
import config as P
from physique import Modele6DDL
from controleur import ControleurRaie


def tester_pid():
    sim = Modele6DDL()
    ctrl = ControleurRaie()
    ctrl.mode_vision = False
    ctrl.cap_cible = np.radians(60.0)
    ctrl.profondeur_cible = 2.5
    ctrl.vitesse_cible = 0.6

    dt = 0.01  # 100 Hz controle
    for _ in range(1000):  # 10 secondes
        etat = {
            "vx": sim.vel_body[0],
            "yaw": sim.att[2],
            "profondeur": sim.profondeur,
            "humidite": 45.0,
        }
        cg, cd, eg, ed, bal = ctrl.calculer_commandes(etat, dt)
        for _ in range(2):  # 200 Hz physique
            sim.step(cg, cd, eg, ed, bal, 0.005)

    err_cap_deg = abs(np.degrees(sim.att[2]) - 60.0)
    err_prof_m = abs(sim.profondeur - 2.5)
    print(f"Cap final : {np.degrees(sim.att[2]):.1f} deg (cible 60.0 deg | ecart {err_cap_deg:.1f} deg)")
    print(f"Profondeur finale : {sim.profondeur:.2f} m (cible 2.50 m | ecart {err_prof_m:.2f} m)")
    assert err_cap_deg < 5.0 and err_prof_m < 0.25, "Les PID n'ont pas converge correctement"
    print("[OK] Convergence des PID validee !")


if __name__ == "__main__":
    tester_pid()