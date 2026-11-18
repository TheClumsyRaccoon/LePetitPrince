import cv2
import numpy as np
from direct.task import Task
from panda3d.core import ClockObject
import config as P
from physique import Modele6DDL
from monde_3d import SceneSousMarine3D
from controleur import ControleurRaie


class Simulation:
    """Orchestrateur temps reel reliant physique 6-DDL, rendu 3D et controleur."""

    def __init__(self):
        self.clock = ClockObject.getGlobalClock()
        self.sim = Modele6DDL()
        self.scene = SceneSousMarine3D()
        self.ctrl = ControleurRaie()

        # Cadences
        self.dt_phys = P.DT_PHYSIQUE_S              #(200 Hz)
        self.dt_ctrl = 0.01    #(100 Hz)
        self.dt_vision = 0.04    # (25 Hz)
        self.pas_par_ctrl = max(1, round(self.dt_ctrl / self.dt_phys))

        # Etat interne de la boucle
        self.accum_phys = 0.0
        self.accum_vision = 0.0
        self.n_pas = 0
        self.humidite_sim = 45.0

        self._init_commandes()
        self._configurer_raccourcis()

    def _init_commandes(self):
        self.cmd_g = 0.0
        self.cmd_d = 0.0
        self.elev_g = 0.0
        self.elev_d = 0.0
        self.ballast_cible = P.BALLAST_NEUTRE_ML
        self.frame_annotee = np.zeros((P.CAM_HAUTEUR, P.CAM_LARGEUR, 3), dtype=np.uint8)

    def _configurer_raccourcis(self):
        self.scene.accept("m", self.basculer_mode_vision)
        self.scene.accept("f", self.basculer_fuite)
        self.scene.accept("r", self.reinitialiser)

    def basculer_mode_vision(self):
        self.ctrl.mode_vision = not self.ctrl.mode_vision
        etat_str = "VISION AUTONOME" if self.ctrl.mode_vision else "CONSIGNES FIXES"
        print(f"[SIMU] Mode de pilotage : {etat_str}")

    def basculer_fuite(self):
        self.humidite_sim = 85.0 if self.humidite_sim < 70.0 else 45.0
        print(f"[SIMU] Humidite interne simulee : {self.humidite_sim:.0f} %HR")

    def reinitialiser(self):
        self.sim = Modele6DDL()
        self.ctrl = ControleurRaie()
        self.accum_phys = 0.0
        self.accum_vision = 0.0
        self.n_pas = 0
        self.humidite_sim = 45.0
        self._init_commandes()
        print("[SIMU] Simulation, consignes et actionneurs reinitialises.")

    def lire_capteurs(self):
        return {
            "vx": self.sim.vel_body[0],
            "yaw": self.sim.att[2],
            "profondeur": self.sim.profondeur,
            "humidite": self.humidite_sim,
        }

    def _incruster_data(self, frame):
        mode_txt = "AUTO (VISION)" if self.ctrl.mode_vision else "MANUEL (FIXE)"
        cv2.putText(
            frame,
            f"{mode_txt} | Prof: {self.sim.profondeur:.2f}m (cible {self.ctrl.profondeur_cible:.2f}m)",
            (10, P.CAM_HAUTEUR - 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1,
        )
        cv2.putText(
            frame,
            f"Vit: {self.sim.vel_body[0]:.2f}m/s | Cap: {np.degrees(self.sim.att[2]):.0f} deg | Bal: {self.sim.ballast_ml:.0f}mL",
            (10, P.CAM_HAUTEUR - 12),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (200, 255, 200),
            1,
        )

    def _pas_simulation(self, task):
        dt = min(self.clock.getDt(), 0.05)

        # 1. Boucle physique (200 Hz) + controle PID (100 Hz)
        self.accum_phys += dt
        while self.accum_phys >= self.dt_phys:
            if self.n_pas % self.pas_par_ctrl == 0:
                (
                    self.cmd_g,
                    self.cmd_d,
                    self.elev_g,
                    self.elev_d,
                    self.ballast_cible,
                ) = self.ctrl.calculer_commandes(self.lire_capteurs(), self.dt_ctrl)

            self.sim.step(
                self.cmd_g, self.cmd_d, self.elev_g, self.elev_d, self.ballast_cible, self.dt_phys
            )
            self.accum_phys -= self.dt_phys
            self.n_pas += 1

        # 2. Rendu 3D Panda3D
        self.scene.maj(self.sim, dt)

        # 3. Boucle vision OpenCV (25Hz)
        self.accum_vision += dt
        if self.accum_vision >= self.dt_vision:
            frame_bgr = self.scene.capturer_frame_bgr()
            self.frame_annotee = self.ctrl.maj_consignes_vision(
                frame_bgr, self.lire_capteurs(), self.dt_vision
            )
            self._incruster_data(self.frame_annotee)
            cv2.imshow("Vision Embarquee - Le Petit Prince", self.frame_annotee)
            self.accum_vision -= self.dt_vision

        # 4. Gestion shortcut fenètre OpenCV
        if cv2.waitKey(1) & 0xFF in (ord("q"), 27):
            cv2.destroyAllWindows()
            self.scene.userExit()
            return Task.done

        return Task.cont

    def lancer(self):
        print("=== Simulateur 3D 6-DDL — Le Petit Prince ===")
        print("  Souris (clic gauche + glisser / molette) : camera orbitale / zoom")
        print("  [V] : basculer vue externe / vue nez embarquee")
        print("  [M] : basculer mode Vision Autonome / Consigne Fixe")
        print("  [F] : simuler une fuite d'eau (>70% HR)")
        print("  [R] : reinitialiser la simulation")
        print("  [Q] (fenetre OpenCV) : quitter")

        self.scene.taskMgr.add(self._pas_simulation, "boucle_simulation")
        self.scene.run()


if __name__ == "__main__":
    Simulation().lancer()