import numpy as np
import matplotlib.pyplot as plt
import config as P
from physique import Modele6DDL


def test_equilibre_statique():
    sim = Modele6DDL()
    for _ in range(1000):  # 5 secondes au repos
        sim.step(0.0, 0.0, 0.0, 0.0, P.BALLAST_NEUTRE_ML, P.DT_PHYSIQUE_S)

    assert np.isclose(sim.profondeur, 1.5, atol=1e-3), f"Derive profondeur: {sim.profondeur}"
    assert np.linalg.norm(sim.vel_body) < 1e-4, "Le robot bouge sans commande"
    print("[OK] Equilibre statique a 50 mL valide")


def test_flottabilite_ballast():
    sim = Modele6DDL()
    # Ballast plein (100 mL = +50 g) -> doit plonger
    for _ in range(2000):  # 10 secondes
        sim.step(0.0, 0.0, 0.0, 0.0, 100.0, P.DT_PHYSIQUE_S)
    assert sim.profondeur > 2.0, f"Le robot n'a pas plonge: {sim.profondeur} m"

    # Ballast vide (0 mL = -50 g) -> doit remonter vers la surface
    for _ in range(4000):  # 20 secondes
        sim.step(0.0, 0.0, 0.0, 0.0, 0.0, P.DT_PHYSIQUE_S)
    assert sim.profondeur < 1.0, f"Le robot n'est pas remonte: {sim.profondeur} m"
    print("[OK] Flottabilite ballast (plongee et remontee) validee")


if __name__ == "__main__":
    test_equilibre_statique()
    test_flottabilite_ballast()
    test_stabilite_pendulaire()
    test_vitesse_croisiere()
    tracer_essai_boucle_ouverte()