import cv2
import numpy as np
import config as P


class SlamVisuelInertiel:
    """oodometrie visuelle ORB avec Lucas Kanade + proj au sol et carte 2D."""

    def __init__(self, taille_carte_px=420, emprise_carte_m=60.0):
        self.taille_px = taille_carte_px
        self.emprise_m = emprise_carte_m
        self.echelle_px_m = taille_carte_px / emprise_carte_m

        self.gris_prec = None
        self.pts_prec = None
        self.att_prec = None
        self.prof_prec = None

        self.pos_est = np.array([0.0, 0.0], dtype=float)
        self.init_ok = False

        self.traj_est = []
        self.traj_reelle = []
        self.carte_exp_reelle = np.zeros((taille_carte_px, taille_carte_px), dtype=np.uint8)
        self.carte_exp_slam = np.zeros((taille_carte_px, taille_carte_px), dtype=np.uint8)

    def reset(self, pos_reelle_xy=None):
        self.gris_prec = None
        self.pts_prec = None
        self.att_prec = None
        self.prof_prec = None
        if pos_reelle_xy is not None:
            self.pos_est = np.array(pos_reelle_xy[:2], dtype=float)
        else:
            self.pos_est = np.array([0.0, 0.0], dtype=float)
        self.traj_est.clear()
        self.traj_reelle.clear()
        self.carte_exp_reelle.fill(0)
        self.carte_exp_slam.fill(0)

    def _monde_vers_px(self, x, y):
        u = int(self.taille_px * 0.5 + x * self.echelle_px_m)
        v = int(self.taille_px * 0.5 - y * self.echelle_px_m)
        return (
            int(np.clip(u, 0, self.taille_px - 1)),
            int(np.clip(v, 0, self.taille_px - 1)),
        )

    @staticmethod
    def _projeter_pixels_au_sol(pts_uv, w, h, focale_px, roll, pitch, yaw, altitude_fond):
        """Projete des pixels (u, v) sur le plan du sol situe a 'altitude_fond' sous la camera."""
        if len(pts_uv) == 0:
            return np.empty((0, 2), dtype=float), np.zeros(0, dtype=bool)

        x_opt = (pts_uv[:, 0] - w * 0.5) / focale_px
        y_opt = (pts_uv[:, 1] - h * 0.5) / focale_px
        z_opt = np.ones_like(x_opt)

        rays_body = np.stack([z_opt, -x_opt, -y_opt], axis=1)

        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
        Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
        Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
        R = Rz @ Ry @ Rx

        rays_world = rays_body @ R.T

        valides = (rays_world[:, 2] < -0.05)
        proj_xy = np.zeros((len(pts_uv), 2), dtype=float)
        if np.any(valides):
            t = -altitude_fond / rays_world[valides, 2]
            proche = (t > 0.3) & (t < 20.0)
            idx_val = np.where(valides)[0]
            valides[idx_val[~proche]] = False
            proj_xy[valides, 0] = rays_world[valides, 0] * t[proche]
            proj_xy[valides, 1] = rays_world[valides, 1] * t[proche]

        return proj_xy, valides

    def _detecter_points_sol(self, gris, w, h):
        masque = np.zeros_like(gris)
        y_debut = int(h * 0.35)
        masque[y_debut:h - 10, 10:w - 10] = 255
        return cv2.goodFeaturesToTrack(
            gris, maxCorners=80, qualityLevel=0.012, minDistance=16, mask=masque
        )

    @staticmethod
    def _dessiner_reperes_hud(frame, pts_anc, pts_nouv):
        """Affiche des viseurs carrés bleus avec vecteur sur la video."""
        for (u1, v1), (u2, v2) in zip(pts_anc, pts_nouv):
            iu1, iv1 = int(u1), int(v1)
            iu2, iv2 = int(u2), int(v2)
            cv2.rectangle(frame, (iu2 - 4, iv2 - 4), (iu2 + 4, iv2 + 4), (255, 220, 0), 1)
            cv2.circle(frame, (iu2, iv2), 1, (255, 255, 255), -1)
            if np.hypot(iu2 - iu1, iv2 - iv1) >= 1.0:
                cv2.line(frame, (iu1, iv1), (iu2, iv2), (255, 180, 0), 1)

    def maj(self, frame_bgr, etat_capteurs, pos_reelle_xyz, frame_visu=None):
        """MAJ le SLAM et retourne (pos_est, carte_reelle, carte_slam, frame_out)."""
        h, w = frame_bgr.shape[:2]
        fov_rad = np.radians(P.CAM_FOV_DEG)
        focale_px = (w * 0.5) / np.tan(fov_rad * 0.5)

        if not self.init_ok:
            self.pos_est = np.array(pos_reelle_xyz[:2], dtype=float)
            self.init_ok = True

        gris = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2GRAY)
        frame_out = frame_visu if frame_visu is not None else frame_bgr.copy()

        roll = etat_capteurs.get("roll", 0.0)
        pitch = -etat_capteurs.get("pitch", 0.0)
        yaw = etat_capteurs["yaw"]
        prof = etat_capteurs["profondeur"]
        altitude_fond = float(np.clip(P.PROFONDEUR_BASSIN_M - prof, 0.6, 5.8))

        nb_pts = 0
        if (
            self.gris_prec is not None
            and self.pts_prec is not None
            and len(self.pts_prec) >= 6
            and self.att_prec is not None
        ):
            pts_nouv, statut, _ = cv2.calcOpticalFlowPyrLK(
                self.gris_prec, gris, self.pts_prec, None, winSize=(21, 21), maxLevel=3
            )
            ok = (statut.flatten() == 1)
            if np.sum(ok) >= 6:
                uv_anc = self.pts_prec[ok].reshape(-1, 2)
                uv_nouv = pts_nouv[ok].reshape(-1, 2)

                r_p, p_p, y_p = self.att_prec
                sol_anc, val_anc = self._projeter_pixels_au_sol(
                    uv_anc, w, h, focale_px, r_p, p_p, y_p, self.prof_prec
                )
                sol_nouv, val_nouv = self._projeter_pixels_au_sol(
                    uv_nouv, w, h, focale_px, roll, pitch, yaw, altitude_fond
                )

                val = val_anc & val_nouv
                if np.sum(val) >= 5:
                    dpos = sol_anc[val] - sol_nouv[val]
                    dx_monde = float(np.median(dpos[:, 0]))
                    dy_monde = float(np.median(dpos[:, 1]))

                    residus = np.hypot(dpos[:, 0] - dx_monde, dpos[:, 1] - dy_monde)
                    inliers = residus < 0.35
                    if np.sum(inliers) >= 4 and np.hypot(dx_monde, dy_monde) < 0.30:
                        self.pos_est[0] += dx_monde
                        self.pos_est[1] += dy_monde
                        nb_pts = int(np.sum(inliers))
                        self._dessiner_reperes_hud(
                            frame_out, uv_anc[val][inliers], uv_nouv[val][inliers]
                        )

                dans_cadre = (
                    (uv_nouv[:, 0] > 10) & (uv_nouv[:, 0] < w - 10)
                    & (uv_nouv[:, 1] > int(h * 0.35)) & (uv_nouv[:, 1] < h - 10)
                )
                pts_restants = uv_nouv[dans_cadre].reshape(-1, 1, 2)
                if len(pts_restants) < 45:
                    nouveaux = self._detecter_points_sol(gris, w, h)
                    if nouveaux is not None:
                        pts_restants = (
                            np.vstack([pts_restants, nouveaux]) if len(pts_restants) > 0 else nouveaux
                        )
                self.pts_prec = pts_restants[:80] if len(pts_restants) > 0 else None
            else:
                self.pts_prec = self._detecter_points_sol(gris, w, h)
        else:
            self.pts_prec = self._detecter_points_sol(gris, w, h)

        cv2.putText(
            frame_out,
            f"SLAM LK: {nb_pts} pts",
            (w - 155, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.50,
            (255, 220, 0),
            1,
        )

        self.gris_prec = gris
        self.att_prec = (roll, pitch, yaw)
        self.prof_prec = altitude_fond

        self.traj_est.append(self.pos_est.copy())
        self.traj_reelle.append(np.array(pos_reelle_xyz[:2], dtype=float))
        if len(self.traj_est) > 1500:
            self.traj_est.pop(0)
            self.traj_reelle.pop(0)

        rayon_cone = max(8, int(altitude_fond * 1.4 * self.echelle_px_m))
        px_reel = self._monde_vers_px(pos_reelle_xyz[0], pos_reelle_xyz[1])
        px_est = self._monde_vers_px(self.pos_est[0], self.pos_est[1])
        cv2.circle(self.carte_exp_reelle, px_reel, rayon_cone, 90, -1)
        cv2.circle(self.carte_exp_slam, px_est, rayon_cone, 90, -1)

        erreur_m = float(np.linalg.norm(self.pos_est - np.array(pos_reelle_xyz[:2])))
        carte_reelle = self._dessiner_une_carte(
            self.traj_reelle,
            pos_reelle_xyz[:2],
            yaw,
            self.carte_exp_reelle,
            couleur=(0, 220, 80),
            titre=f"POSITION REELLE | X={pos_reelle_xyz[0]:+.1f}m Y={pos_reelle_xyz[1]:+.1f}m",
            sous_titre="Verite terrain simulateur",
        )
        carte_slam = self._dessiner_une_carte(
            self.traj_est,
            self.pos_est,
            yaw,
            self.carte_exp_slam,
            couleur=(255, 210, 0),
            titre=f"POSITION SLAM | Pts={nb_pts} | Derive={erreur_m:.2f}m",
            sous_titre=f"Estimee X={self.pos_est[0]:+.1f}m Y={self.pos_est[1]:+.1f}m",
        )
        return self.pos_est.copy(), carte_reelle, carte_slam, frame_out

    def _dessiner_une_carte(self, trajectoire, pos_xy, yaw, masque_exp, couleur, titre, sous_titre):
        img = np.zeros((self.taille_px, self.taille_px, 3), dtype=np.uint8)
        img[:, :] = (25, 18, 12)
        img[masque_exp > 0] = (65, 55, 30)

        pas_px = int(10.0 * self.echelle_px_m)
        for k in range(0, self.taille_px, max(1, pas_px)):
            cv2.line(img, (k, 0), (k, self.taille_px), (40, 35, 30), 1)
            cv2.line(img, (0, k), (self.taille_px, k), (40, 35, 30), 1)

        if len(trajectoire) >= 2:
            pts = np.array([self._monde_vers_px(p[0], p[1]) for p in trajectoire], dtype=np.int32)
            cv2.polylines(img, [pts], False, couleur, 2)

        p_cur = self._monde_vers_px(pos_xy[0], pos_xy[1])
        cv2.circle(img, p_cur, 5, couleur, -1)

        fleche = (
            int(p_cur[0] + 18 * np.cos(yaw)),
            int(p_cur[1] - 18 * np.sin(yaw)),
        )
        cv2.arrowedLine(img, p_cur, fleche, couleur, 2, tipLength=0.35)

        cv2.putText(img, titre, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (235, 235, 235), 1)
        cv2.putText(img, sous_titre, (10, 46), cv2.FONT_HERSHEY_SIMPLEX, 0.44, couleur, 1)
        return img