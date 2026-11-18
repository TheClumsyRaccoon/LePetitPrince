import cv2
import numpy as np
from direct.task import Task
import config as P
from physique import Modele6DDL
from monde_3d import SceneSousMarine3D
from controleur import ControleurRaie


def lancer_simulation():
    sim = Modele6DDL()
    scene = SceneSousMarine3D()
    ctrl = ControleurRaie()

    # Shortcuts dans la fenètre 3d
    def basculer_mode_vision():
        ctrl.mode_vision = not ctrl.mode_vision
        etat_str = "VISION AUTONOME" if ctrl.mode_vision else "CAP/PROF FIXES"
        print(f"[SIMU] Mode de pilotage : {etat_str}")

    def reinitialiser_robot():
        nonlocal sim
        sim = Modele6DDL()
        ctrl.pid_vitesse.reset()
        ctrl.pid_cap.reset()
        ctrl.pid_profondeur.reset()
        print("[SIMU] Position et regulateurs reinitialises.")

    scene.accept("m", basculer_mode_vision)
    scene.accept("r", reinitialiser_robot)

    print("=== Simulateur 3D 6-DDL — Le Petit Prince ===")
    print("  Souris (clic gauche + glisser / molette) : camera orbitale / zoom")
    print("  [V] : basculer vue externe / vue nez embarquee")
    print("  [M] : basculer mode Vision Autonome / Consigne Fixe")
    print("  [R] : reinitialiser la position de la raie")
    print("  [Q] (dans la fenetre OpenCV) : quitter")

    # Dernieres commandes actionneurs
    cmd_g, cmd_d = 0.0, 0.0
    elev_g, elev_d = 0.0, 0.0
    ballast_cible = P.BALLAST_NEUTRE_ML

    accum_ctrl = 0.0
    accum_vision = 0.0
    dt_ctrl = 1.0 / P.FREQ_CONTROLE_HZ      # 100 Hz
    dt_vision = 1.0 / P.FREQ_VISION_HZ      # 25 Hz
    frame_annotee = np.zeros((P.CAM_HAUTEUR, P.CAM_LARGEUR, 3), dtype=np.uint8)

    def boucle_principale(task):
        nonlocal cmd_g, cmd_d, elev_g, elev_d, ballast_cible
        nonlocal accum_ctrl, accum_vision, frame_annotee

        dt = min(globalClock.getDt(), 0.05)

        # 1. Integration physique (200 Hz)
        nb_pas = max(1, int(round(dt / P.DT_PHYSIQUE_S)))
        dt_sub = dt / nb_pas
        for _ in range(nb_pas):
            sim.step(cmd_g, cmd_d, elev_g, elev_d, ballast_cible, dt_sub)

        # 2. Mise à jour du rendu 3D et de la caméra embarquée
        scene.maj(sim, dt)

        # 3. Boucle vision OpenCV (25 Hz)
        accum_vision += dt
        if accum_vision >= dt_vision:
            frame_bgr = scene.capturer_frame_bgr()
            etat_capteurs = {
                "vx": sim.vel_body[0],
                "yaw": sim.att[2],
                "profondeur": sim.profondeur,
                "humidite": 45.0,
            }
            frame_annotee = ctrl.maj_consignes_vision(frame_bgr, etat_capteurs, accum_vision)

            # Incrustation data sur le retour camera
            mode_txt = "AUTO (VISION)" if ctrl.mode_vision else "MANUEL (FIXE)"
            cv2.putText(
                frame_annotee,
                f"{mode_txt} | Prof: {sim.profondeur:.2f}m (cible {ctrl.profondeur_cible:.2f}m)",
                (10, P.CAM_HAUTEUR - 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 255, 255),
                1,
            )
            cv2.putText(
                frame_annotee,
                f"Vit: {sim.vel_body[0]:.2f}m/s | Cap: {np.degrees(sim.att[2]):.0f} deg | Bal: {sim.ballast_ml:.0f}mL",
                (10, P.CAM_HAUTEUR - 12),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (200, 255, 200),
                1,
            )
            accum_vision = 0.0

        # 4. Boucle d'asservissement PID (100 Hz)
        accum_ctrl += dt
        while accum_ctrl >= dt_ctrl:
            etat_capteurs = {
                "vx": sim.vel_body[0],
                "yaw": sim.att[2],
                "profondeur": sim.profondeur,
                "humidite": 45.0,
            }
            cmd_g, cmd_d, elev_g, elev_d, ballast_cible = ctrl.calculer_commandes(
                etat_capteurs, dt_ctrl
            )
            accum_ctrl -= dt_ctrl

        # Affichage de la fenetre OpenCV
        cv2.imshow("Vision Embarquee - Le Petit Prince", frame_annotee)
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            cv2.destroyAllWindows()
            scene.userExit()
            return Task.done

        return Task.cont

    scene.taskMgr.add(boucle_principale, "boucle_simulation_complete")
    scene.run()


if __name__ == "__main__":
    lancer_simulation()