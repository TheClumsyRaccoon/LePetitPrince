import cv2
import numpy as np
import config as P


class Pid:
    """Regulateur PID avec anti saturation"""

    def __init__(self, kp, ki, kd, sortie_min, sortie_max):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.sortie_min = sortie_min
        self.sortie_max = sortie_max
        self.integrale = 0.0
        self.erreur_prec = 0.0

    def reset(self):
        self.integrale = 0.0
        self.erreur_prec = 0.0

    def calculer(self, consigne, mesure, dt, erreur_forcee=None):
        if dt <= 0.0:
            return 0.0

        erreur = (consigne - mesure) if erreur_forcee is None else erreur_forcee
        self.integrale += erreur * dt

        # Anti saturation par bornage du terme integral
        terme_i = self.ki * self.integrale
        if self.ki > 1e-6:
            if terme_i > self.sortie_max:
                terme_i = self.sortie_max
                self.integrale = self.sortie_max / self.ki
            elif terme_i < self.sortie_min:
                terme_i = self.sortie_min
                self.integrale = self.sortie_min / self.ki

        terme_d = (erreur - self.erreur_prec) / dt
        self.erreur_prec = erreur

        sortie = (self.kp * erreur) + terme_i + (self.kd * terme_d)
        return float(np.clip(sortie, self.sortie_min, self.sortie_max))


class DetecteurCibleVision:
    """Simulation du traitement OpenCV embarque sur la Raspberry Pi 5"""

    def __init__(self):
        # Plage HSV pour le poisson cible orange vif
        self.hsv_min = np.array([5, 130, 100], dtype=np.uint8)
        self.hsv_max = np.array([22, 255, 255], dtype=np.uint8)

    def analyser(self, frame_bgr):
        """
        Detecte la cible orange dans l'image BGR.
        Retourne (trouvee, err_x_norm, err_y_norm, aire_px, frame_annotee).
        err_x_norm dans [-1, +1] (+1 = cible a droite dans l'image)
        err_y_norm dans [-1, +1] (+1 = cible en bas dans l'image)
        """
        frame_out = frame_bgr.copy()
        h, w = frame_bgr.shape[:2]

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        masque = cv2.inRange(hsv, self.hsv_min, self.hsv_max)

        # Nettoyage morpho
        noyau = np.ones((5, 5), np.uint8)
        masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, noyau)

        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            cv2.putText(frame_out, "CIBLE : PERDUE (Recherche)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            return False, 0.0, 0.0, 0.0, frame_out

        c_max = max(contours, key=cv2.contourArea)
        aire = cv2.contourArea(c_max)
        if aire < 80.0:
            return False, 0.0, 0.0, 0.0, frame_out

        x, y, bw, bh = cv2.boundingRect(c_max)
        cx = x + bw // 2
        cy = y + bh // 2

        # Erreurs normalisees par rapport au centre de l'image
        err_x = (cx - (w / 2.0)) / (w / 2.0)
        err_y = (cy - (h / 2.0)) / (h / 2.0)

        # Annotation visuel
        cv2.rectangle(frame_out, (x, y), (x + bw, y + bh), (0, 255, 0), 2)
        cv2.circle(frame_out, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(frame_out, (w // 2, h // 2), (cx, cy), (255, 255, 0), 1)
        cv2.putText(frame_out, f"CIBLE OK | err=({err_x:+.2f}, {err_y:+.2f}) | aire={int(aire)}",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2)

        return True, float(err_x), float(err_y), float(aire), frame_out


class ControleurRaie:
    """Pilote automatique combinant guidage visuel et regulation 6-DDL."""

    def __init__(self):
        self.vision = DetecteurCibleVision()

        # Regulateurs bas niveau
        self.pid_vitesse = Pid(0.60, 0.15, 0.02, -1.0, 1.0)
        self.pid_cap = Pid(0.85, 0.08, 0.12, -0.55, 0.55)
        self.pid_profondeur = Pid(1.10, 0.12, 0.35, -1.0, 1.0)

        # Consignes
        self.mode_vision = True
        self.vitesse_cible = 0.55
        self.cap_cible = 0.0
        self.profondeur_cible = 1.8

    def maj_consignes_vision(self, frame_bgr, etat_capteurs, dt):
        """Met a jour cap_cible et profondeur_cible a partir de la camera"""
        trouvee, err_x, err_y, aire, frame_annotee = self.vision.analyser(frame_bgr)

        if not self.mode_vision:
            return frame_annotee

        yaw_mes = etat_capteurs["yaw"]
        prof_mes = etat_capteurs["profondeur"]

        if trouvee:
            # Si la cible est a droite dans l'image (err_x > 0), il faut tourner a droite (yaw diminue selon niotre convention)
            self.cap_cible = (yaw_mes - 0.65 * err_x + np.pi) % (2.0 * np.pi) - np.pi
            # Si la cible est en bas de l'image (err_y > 0), on descend plus profond
            self.profondeur_cible = float(np.clip(
                prof_mes + 0.8 * err_y, 0.3, P.PROFONDEUR_BASSIN_M - 0.4
            ))
            # Ralentir a l'approche si le poisson occupe une grande surface
            self.vitesse_cible = 0.25 if aire > 18000 else 0.60
        else:
            # Mode recherche : balayage lent en lacet a profondeur constante
            self.cap_cible = (yaw_mes + 0.35 * dt + np.pi) % (2.0 * np.pi) - np.pi
            self.vitesse_cible = 0.35

        return frame_annotee

    def calculer_commandes(self, etat_capteurs, dt):
        """Calcule les sorties actionneurs (100 Hz) : moteurs, elevons, ballast."""
        # Securite fuite (en supposant que le bmp280 soit remplacé'
        if etat_capteurs["humidite"] > 70.0:
            return 0.0, 0.0, 0.35, 0.35, 0.0

        # 1. Asservissement longitudinal (Vitesse)
        u_vit = self.pid_vitesse.calculer(self.vitesse_cible, etat_capteurs["vx"], dt)

        # 2. Asservissement directionnel (Cap / Yaw)
        err_cap = (self.cap_cible - etat_capteurs["yaw"] + np.pi) % (2.0 * np.pi) - np.pi
        u_cap = self.pid_cap.calculer(self.cap_cible, etat_capteurs["yaw"], dt, erreur_forcee=err_cap)

        # Mixage differentiel (ex: si err_cap > 0 : tourner a gauche -> moteur droit > moteur gauche)
        cmd_g = float(np.clip(u_vit + u_cap, -1.0, 1.0))
        cmd_d = float(np.clip(u_vit - u_cap, -1.0, 1.0))

        # 3. Asservissement vertical (Profondeur -> Ballast + Elevons)
        # err_prof > 0 signifie qu'on veut descendre plus profond
        u_prof = self.pid_profondeur.calculer(self.profondeur_cible, etat_capteurs["profondeur"], dt)

        # Ballast statique : > 50 mL pour plonger, < 50 mL pour remonter
        cible_ballast = float(np.clip(P.BALLAST_NEUTRE_ML + u_prof * 50.0, 0.0, 100.0))

        # Elevons dynamiques : < 0 pour piquer du nez (descendre), > 0 pour monter)
        cmd_elevon = float(np.clip(-u_prof * 0.35, -0.35, 0.35))

        return cmd_g, cmd_d, cmd_elevon, cmd_elevon, cible_ballast