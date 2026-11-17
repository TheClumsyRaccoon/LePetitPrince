import numpy as np
import matplotlib.pyplot as plt
import config as P
from physique import Modele6DDL


def test_equilibre_statique():
    sim = Modele6DDL()
    for _ in range(1000):
        sim.step(0.0, 0.0, 0.0, 0.0, P.BALLAST_NEUTRE_ML, P.DT_PHYSIQUE_S)

    assert np.isclose(sim.profondeur, 1.5, atol=1e-3), f"Derive profondeur: {sim.profondeur:.4f} m"
    assert np.linalg.norm(sim.vel_body) < 1e-4, f"Vitesse residuelle: {sim.vel_body}"


def test_flottabilite_ballast():
    sim = Modele6DDL()
    # Ballast plein (100 mL = +50 g) -> doit plonger
    for _ in range(2000): 
        sim.step(0.0, 0.0, 0.0, 0.0, 100.0, P.DT_PHYSIQUE_S)
    assert sim.profondeur > 2.0, f"N'a pas assez plonge: {sim.profondeur:.2f} m"

    # Ballast vide (0 mL = -50 g) -> doit remonter vers la surface
    for _ in range(4000):
        sim.step(0.0, 0.0, 0.0, 0.0, 0.0, P.DT_PHYSIQUE_S)
    assert sim.profondeur < 1.0, f"N'est pas remonte: {sim.profondeur:.2f} m"


def test_stabilite_pendulaire():
    sim = Modele6DDL()
    sim.att[0] = np.radians(30.0)  # Roulis initial de 30 deg
    for _ in range(2000):
        sim.step(0.0, 0.0, 0.0, 0.0, P.BALLAST_NEUTRE_ML, P.DT_PHYSIQUE_S)
    assert abs(sim.att[0]) < np.radians(2.0), f"Roulis non amorti: {np.degrees(sim.att[0]):.2f} deg"


def test_vitesse_croisiere():
    sim = Modele6DDL()
    for _ in range(2000):  # Pleine poussee avant pendant 10 s
        sim.step(1.0, 1.0, 0.0, 0.0, P.BALLAST_NEUTRE_ML, P.DT_PHYSIQUE_S)

    v_theo = np.sqrt((2.0 * P.POUSSEE_MAX_AVANT_N) / P.TRAINEE_LINEAIRE[0])
    assert np.isclose(sim.vel_body[0], v_theo, rtol=0.02), (
        f"Vitesse {sim.vel_body[0]:.2f} m/s != theorie {v_theo:.2f} m/s"
    )


def executer_batterie_tests():
    tests = [
        ("Equilibre statique (50 mL)", test_equilibre_statique),
        ("Flottabilite ballast (plongee/remontee)", test_flottabilite_ballast),
        ("Rappel pendulaire passif", test_stabilite_pendulaire),
        ("Vitesse limite en croisiere", test_vitesse_croisiere),
    ]

    reussis = 0
    print("=== Lancement des tests unitaires physique ===")
    for nom, fn in tests:
        try:
            fn()
            print(f"[OK]    {nom}")
            reussis += 1
        except AssertionError as e:
            print(f"[ECHEC] {nom} -> {e}")
        except Exception as e:
            print(f"[ERREUR] {nom} -> {type(e).__name__}: {e}")

    print(f"=== Bilan : {reussis}/{len(tests)} tests valides ===\n")


def tracer_essai_boucle_ouverte():
    sim = Modele6DDL()
    dt = P.DT_PHYSIQUE_S
    temps, hist_z, hist_vx, hist_yaw = [], [], [], []

    for i in range(3000): 
        t = i * dt
        if t < 5.0:
            sim.step(0.5, 0.5, -0.05, -0.05, 70.0, dt)
        elif t < 10.0:
            sim.step(0.6, 0.3, 0.0, 0.0, 50.0, dt)
        else:
            sim.step(0.4, 0.4, 0.08, 0.08, 30.0, dt)

        temps.append(t)
        hist_z.append(sim.profondeur)
        hist_vx.append(sim.vel_body[0])
        hist_yaw.append(np.degrees(sim.att[2]))

    fig, axs = plt.subplots(3, 1, figsize=(9, 7), sharex=True)
    axs[0].plot(temps, hist_vx, "g-")
    axs[0].set_ylabel("Vitesse X (m/s)")
    axs[0].set_title("Validation en boucle ouverte - Modele 6-DDL")
    axs[0].grid(True)

    axs[1].plot(temps, hist_z, "c-")
    axs[1].invert_yaxis()
    axs[1].set_ylabel("Profondeur (m)")
    axs[1].grid(True)

    axs[2].plot(temps, hist_yaw, "m-")
    axs[2].set_ylabel("Cap / Yaw (deg)")
    axs[2].set_xlabel("Temps (s)")
    axs[2].grid(True)

    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    executer_batterie_tests()
    tracer_essai_boucle_ouverte()