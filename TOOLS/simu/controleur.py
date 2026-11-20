from pathlib import Path
import time
import cv2
import numpy as np
import config as P

try:
    from ultralytics import YOLO
    ULTRALYTICS_DISPO = True
except ImportError:
    ULTRALYTICS_DISPO = False


class Pid:
    """Regulateur PID avec anti-saturation (anti-windup)."""

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
    """Detection de cible (YOLOv8 OBSEA avec decimation temporelle, secours HSV)."""

    def __init__(self, chemin_poids="weights/yolov8x_21sp_5364img.pt"):
        self.hsv_min = np.array([5, 130, 100], dtype=np.uint8)
        self.hsv_max = np.array([22, 255, 255], dtype=np.uint8)

        self.conf_seuil = P.VISION_CONF_SEUIL
        self.taille_entree = P.VISION_TAILLE_ENTREE
        self.intervalle_yolo = max(1, int(P.VISION_INTERVALLE_YOLO))

        self.modele = None
        self.cible_precedente = None
        self.etat_prec = None
        self.derniere_boite = None
        self.compteur_frames = 0
        self.passes_manquees = 0
        self.vx_norm = 0.0
        self.taux_aire = 0.0

        p_weights = Path(__file__).resolve().parent / chemin_poids
        if not ULTRALYTICS_DISPO:
            print("[VISION] ultralytics absent -> repli HSV.")
        elif p_weights.exists():
            self.modele = YOLO(str(p_weights))
            print(f"[VISION] YOLOv8 charge : {p_weights.name} (1 frame / {self.intervalle_yolo})")
        else:
            print(f"[VISION] Poids introuvables ({p_weights}) -> repli HSV.")

    def _reset_suivi(self):
        self.cible_precedente = None
        self.etat_prec = None
        self.derniere_boite = None
        self.passes_manquees = 0
        self.vx_norm = 0.0
        self.taux_aire = 0.0

    @staticmethod
    def _erreurs_norm(cx, cy, w, h):
        err_x = (cx - w * 0.5) / (w * 0.5)
        err_y = (cy - h * 0.5) / (h * 0.5)
        return float(err_x), float(err_y)

    def _annoter_cible(self, frame, x, y, bw, bh, cx, cy, texte, couleur=(0, 255, 0)):
        h, w = frame.shape[:2]
        cv2.rectangle(frame, (x, y), (x + bw, y + bh), couleur, 2)
        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)
        cv2.line(frame, (w // 2, h // 2), (cx, cy), (255, 255, 0), 1)
        cv2.putText(frame, texte, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.50, couleur, 2)

    def analyser(self, frame_bgr):
        """Retourne (trouvee, err_x_norm, err_y_norm, ratio_aire, frame_annotee)."""
        if self.modele is None:
            return self._analyser_hsv(frame_bgr)

        lancer_yolo = (self.compteur_frames % self.intervalle_yolo == 0)
        self.compteur_frames += 1

        if lancer_yolo:
            return self._analyser_yolo(frame_bgr)
        return self._reutiliser_derniere_detection(frame_bgr)

    def _analyser_yolo(self, frame_bgr):
        frame_out = frame_bgr.copy()
        h, w = frame_bgr.shape[:2]
        aire_image = float(max(1, w * h))

        t0 = time.perf_counter()
        res = self.modele.predict(
            frame_bgr,
            conf=self.conf_seuil,
            imgsz=self.taille_entree,
            verbose=False,
        )[0]
        dt_ms = (time.perf_counter() - t0) * 1000.0

        if res.boxes is None or len(res.boxes) == 0:
            self.passes_manquees += 1
            if self.passes_manquees <= 2 and self.derniere_boite is not None:
                return self._reutiliser_derniere_detection(frame_bgr)
            self._reset_suivi()
            cv2.putText(frame_out, f"CIBLE : PERDUE ({dt_ms:.0f} ms)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
            return False, 0.0, 0.0, 0.0, frame_out

        self.passes_manquees = 0
        meilleure = None
        meilleur_score = -1.0

        for box in res.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())
            conf = float(box.conf[0])
            espece = self.modele.names.get(int(box.cls[0]), "fish")

            bw, bh = x2 - x1, y2 - y1
            cx, cy = x1 + bw // 2, y1 + bh // 2
            aire = float(max(1, bw * bh))

            cv2.rectangle(frame_out, (x1, y1), (x2, y2), (255, 180, 0), 1)
            cv2.putText(frame_out, f"{espece} {conf:.2f}", (x1, max(15, y1 - 4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 180, 0), 1)

            # Bonus de continuite spatiale pour rester verrouille sur la meme cible
            bonus = 1.0
            if self.cible_precedente is not None:
                dist = np.hypot(cx - self.cible_precedente[0], cy - self.cible_precedente[1])
                bonus = 1.0 + 1.5 * np.exp(-dist / 120.0)

            score = aire * conf * bonus
            if score > meilleur_score:
                meilleur_score = score
                meilleure = (x1, y1, bw, bh, cx, cy, aire, conf, espece)

        x, y, bw, bh, cx, cy, aire, conf, espece = meilleure
        self.cible_precedente = (cx, cy)
        self.derniere_boite = meilleure

        err_x, err_y = self._erreurs_norm(cx, cy, w, h)

        maintenant = time.perf_counter()
        self.vx_norm = 0.0
        self.taux_aire = 0.0
        if self.etat_prec is not None:
            cx_p, _, aire_p, t_p = self.etat_prec
            dt_obs = maintenant - t_p
            if dt_obs > 1e-3:
                self.vx_norm = ((cx - cx_p) / (w * 0.5)) / dt_obs
                self.taux_aire = ((aire - aire_p) / max(aire_p, 1.0)) / dt_obs

        self.etat_prec = (cx, cy, aire, maintenant)

        label = f"{espece} ({conf:.2f}) | {dt_ms:.0f}ms | dA={self.taux_aire:+.2f}/s | err=({err_x:+.2f}, {err_y:+.2f})"
        self._annoter_cible(frame_out, x, y, bw, bh, cx, cy, label, (0, 255, 0))
        return True, err_x, err_y, aire / aire_image, frame_out

    def _reutiliser_derniere_detection(self, frame_bgr):
        frame_out = frame_bgr.copy()
        h, w = frame_bgr.shape[:2]
        aire_image = float(max(1, w * h))

        if self.derniere_boite is None:
            cv2.putText(frame_out, "CIBLE : PERDUE (Cache)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
            return False, 0.0, 0.0, 0.0, frame_out

        x, y, bw, bh, cx, cy, aire, conf, espece = self.derniere_boite
        err_x, err_y = self._erreurs_norm(cx, cy, w, h)

        label = f"{espece} ({conf:.2f}) [CACHE] | dA={self.taux_aire:+.2f}/s | err=({err_x:+.2f}, {err_y:+.2f})"
        self._annoter_cible(frame_out, x, y, bw, bh, cx, cy, label, (0, 220, 180))
        return True, err_x, err_y, aire / aire_image, frame_out

    def _analyser_hsv(self, frame_bgr):
        frame_out = frame_bgr.copy()
        h, w = frame_bgr.shape[:2]
        aire_image = float(max(1, w * h))

        hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
        masque = cv2.inRange(hsv, self.hsv_min, self.hsv_max)
        masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            self._reset_suivi()
            cv2.putText(frame_out, "CIBLE : PERDUE (HSV)", (10, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 165, 255), 2)
            return False, 0.0, 0.0, 0.0, frame_out

        c_max = max(contours, key=cv2.contourArea)
        aire = float(cv2.contourArea(c_max))
        if aire < 80.0:
            self._reset_suivi()
            return False, 0.0, 0.0, 0.0, frame_out

        x, y, bw, bh = cv2.boundingRect(c_max)
        cx, cy = x + bw // 2, y + bh // 2
        err_x, err_y = self._erreurs_norm(cx, cy, w, h)

        label = f"CIBLE HSV | err=({err_x:+.2f}, {err_y:+.2f}) | aire={int(aire)}"
        self._annoter_cible(frame_out, x, y, bw, bh, cx, cy, label, (0, 255, 0))
        return True, err_x, err_y, aire / aire_image, frame_out


class ControleurRaie:
    """Pilote automatique combinant guidage visuel et regulation 6-DDL."""

    def __init__(self):
        self.vision = DetecteurCibleVision()

        # Parametres PID charges depuis config.py : (Kp, Ki, Kd, sortie_min, sortie_max)
        self.pid_vitesse = Pid(*P.PID_VITESSE)
        self.pid_cap = Pid(*P.PID_CAP)
        self.pid_profondeur = Pid(*P.PID_PROF)

        self.mode_vision = True
        self.vitesse_cible = 0.55
        self.cap_cible = 0.0
        self.profondeur_cible = 1.8

    def maj_consignes_vision(self, frame_bgr, etat_capteurs, dt):
        """Met a jour les consignes de cap, profondeur et vitesse depuis la camera."""
        trouvee, err_x, err_y, ratio_aire, frame_annotee = self.vision.analyser(frame_bgr)

        if not self.mode_vision:
            return frame_annotee

        yaw_mes = etat_capteurs["yaw"]
        prof_mes = etat_capteurs["profondeur"]

        if trouvee:
            err_x_pred = err_x + 0.20 * self.vision.vx_norm
            self.cap_cible = (yaw_mes - 0.65 * err_x_pred + np.pi) % (2.0 * np.pi) - np.pi
            self.profondeur_cible = float(np.clip(
                prof_mes + 0.8 * err_y, 0.3, P.PROFONDEUR_BASSIN_M - 0.4
            ))

            if ratio_aire > P.VISION_RATIO_AIRE_PROCHE or self.vision.taux_aire > 0.35:
                self.vitesse_cible = 0.20
            elif self.vision.taux_aire < -0.35:
                self.vitesse_cible = 0.75
            else:
                self.vitesse_cible = 0.55
        else:
            # Integration continue du cap cible pour balayer sur 360 deg
            self.cap_cible = (self.cap_cible + 0.35 * dt + np.pi) % (2.0 * np.pi) - np.pi
            self.vitesse_cible = 0.35

        return frame_annotee

    def calculer_commandes(self, etat_capteurs, dt):
        """Calcule les commandes actionneurs : moteurs, elevons, ballast."""
        if etat_capteurs["humidite"] > 70.0:
            return 0.0, 0.0, 0.35, 0.35, 0.0

        u_vit = self.pid_vitesse.calculer(self.vitesse_cible, etat_capteurs["vx"], dt)

        err_cap = (self.cap_cible - etat_capteurs["yaw"] + np.pi) % (2.0 * np.pi) - np.pi
        u_cap = self.pid_cap.calculer(self.cap_cible, etat_capteurs["yaw"], dt, erreur_forcee=err_cap)

        # Mixage differentiel : u_cap > 0 applique +u_cap a gauche et -u_cap a droite
        cmd_g = float(np.clip(u_vit + u_cap, -1.0, 1.0))
        cmd_d = float(np.clip(u_vit - u_cap, -1.0, 1.0))

        u_prof = self.pid_profondeur.calculer(self.profondeur_cible, etat_capteurs["profondeur"], dt)
        cible_ballast = float(np.clip(P.BALLAST_NEUTRE_ML + u_prof * 50.0, 0.0, 100.0))
        cmd_elevon = float(np.clip(-u_prof * 0.35, -0.35, 0.35))

        return cmd_g, cmd_d, cmd_elevon, cmd_elevon, cible_ballast