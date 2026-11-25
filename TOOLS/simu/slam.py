import cv2
import numpy as np
import config as P


class SlamVisuelInertiel:
    """Premiere version slam ORB + carte 2D."""

    def __init__(self, taille_carte_px=450, emprise_carte_m=60.0):
        self.taille_px = taille_carte_px
        self.emprise_m = emprise_carte_m
        self.echelle_px_m = taille_carte_px / emprise_carte_m

        self.orb = cv2.ORB_create(nfeatures=400, scaleFactor=1.2, nlevels=4)
        self.matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)

        self.kp_prec = None
        self.des_prec = None
        self.pos_est = np.array([0.0, 0.0], dtype=float)
        self.init_ok = False

        self.traj_est = []
        self.traj_reelle = []

        fov_rad = np.radians(P.CAM_FOV_DEG)
        self.focale_px = (640.0 * 0.5) / np.tan(fov_rad * 0.5)

    def reset(self, pos_reelle_xy=None):
        self.kp_prec = None
        self.des_prec = None
        self.pos_est = np.array(pos_reelle_xy[:2], dtype=float) if pos_reelle_xy is not None else np.zeros(2)
        self.traj_est.clear()
        self.traj_reelle.clear()

    def _monde_vers_px(self, x, y):
        u = int(np.clip(self.taille_px * 0.5 + x * self.echelle_px_m, 0, self.taille_px - 1))
        v = int(np.clip(self.taille_px * 0.5 - y * self.echelle_px_m, 0, self.taille_px - 1))
        return u, v

    def maj(self, frame_bgr, etat_capteurs, pos_reelle_xyz):
        h, w = frame_bgr.shape[:2]
        self.focale_px = (w * 0.5) / np.tan(np.radians(P.CAM_FOV_DEG) * 0.5)

        if not self.init_ok:
            self.pos_est = np.array(pos_reelle_xyz[:2], dtype=float)
            self.init_ok = True

        gris = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        kp, des = self.orb.detectAndCompute(gris, None)

        yaw = etat_capteurs["yaw"]
        prof = etat_capteurs["profondeur"]
        altitude_fond = float(np.clip(P.PROFONDEUR_BASSIN_M - prof, 0.6, 5.5))

        nb_pts = 0
        if self.des_prec is not None and des is not None and len(des) >= 10:
            matches = sorted(self.matcher.match(self.des_prec, des), key=lambda m: m.distance)[:60]
            if len(matches) >= 8:
                pts_anc = np.float32([self.kp_prec[m.queryIdx].pt for m in matches])
                pts_nouv = np.float32([kp[m.trainIdx].pt for m in matches])
                du, dv = np.median(pts_nouv - pts_anc, axis=0)
                nb_pts = len(matches)

                dx_robot = (dv / self.focale_px) * altitude_fond
                dy_robot = -(du / self.focale_px) * altitude_fond
                c, s = np.cos(yaw), np.sin(yaw)
                self.pos_est[0] += c * dx_robot - s * dy_robot
                self.pos_est[1] += s * dx_robot + c * dy_robot

        self.kp_prec = kp
        self.des_prec = des
        self.traj_est.append(self.pos_est.copy())
        self.traj_reelle.append(np.array(pos_reelle_xyz[:2], dtype=float))

        img = np.full((self.taille_px, self.taille_px, 3), (25, 18, 12), dtype=np.uint8)
        if len(self.traj_reelle) >= 2:
            cv2.polylines(img, [np.array([self._monde_vers_px(*p) for p in self.traj_reelle])], False, (0, 220, 80), 2)
        if len(self.traj_est) >= 2:
            cv2.polylines(img, [np.array([self._monde_vers_px(*p) for p in self.traj_est])], False, (255, 210, 0), 2)

        err = float(np.linalg.norm(self.pos_est - np.array(pos_reelle_xyz[:2])))
        cv2.putText(img, f"SLAM ORB | Pts: {nb_pts} | Derive: {err:.2f}m", (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (230, 230, 230), 1)
        return self.pos_est.copy(), img