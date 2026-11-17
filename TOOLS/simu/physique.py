import numpy as np
import config as P


class Modele6DDL:
    def __init__(self):
        self.masse = P.MASSE_KG
        self.masse_ajoutee = np.array(P.MASSE_AJOUTEE_KG, dtype=float)
        self.inertie = np.array(P.INERTIE_KG_M2, dtype=float)
        self.bras_levier_moteur = P.BRAS_LEVIER_MOTEUR_M

        self.poussee_max_avant = P.POUSSEE_MAX_AVANT_N
        self.poussee_max_arriere = P.POUSSEE_MAX_ARRIERE_N
        self.tau_moteur = P.TAU_MOTEUR_S
        self.ballast_neutre = P.BALLAST_NEUTRE_ML
        self.debit_ballast_max = P.BALLAST_DEBIT_MAX_MLS

        self.drag_lin = np.array(P.TRAINEE_LINEAIRE, dtype=float)
        self.drag_rot = np.array(P.TRAINEE_ROTATION, dtype=float)
        self.coef_portance = P.COEF_PORTANCE_AILES
        self.efficacite_elevon = P.EFFICACITE_ELEVON
        self.couple_rappel_xy = P.COUPLE_RAPPEL_PASSIF

        self.reset()

    def reset(self):
        # Convention repère monde : Z = 0 en surface, Z < 0 sous l'eau
        self.pos = np.array([0.0, 0.0, -1.5], dtype=float)
        # Convention repère robot : X avant, Y droite, Z haut
        self.vel_body = np.zeros(3, dtype=float)
        self.att = np.zeros(3, dtype=float)
        self.omega = np.zeros(3, dtype=float)

        self.pg_eff = 0.0
        self.pd_eff = 0.0
        self.elevon_g = 0.0
        self.elevon_d = 0.0
        self.ballast_ml = P.BALLAST_NEUTRE_ML
        self.courant_marin = np.zeros(3, dtype=float)

    def _matrice_rotation(self):
        roll, pitch, yaw = self.att
        # Convention : pitch > 0 = nez vers le haut (monte) et pitch < 0 = nez vers le bas (plonge)
        theta = -pitch
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(theta), np.sin(theta)
        cy, sy = np.cos(yaw), np.sin(yaw)
        return np.array([
            [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp,     cp * sr,                cp * cr]
        ])

    def _local_vers_monde(self, vec):
        return self._matrice_rotation() @ vec

    def _monde_vers_local(self, vec):
        return self._matrice_rotation().T @ vec

    def step(self, cmd_g, cmd_d, cmd_elev_g, cmd_elev_d, cible_ballast_ml, dt):
        # Inertie du 1er ordre des propulseurs et asymetrie marche arriere
        cmd_g = float(np.clip(cmd_g, -1.0, 1.0))
        cmd_d = float(np.clip(cmd_d, -1.0, 1.0))
        alpha = dt / (self.tau_moteur + dt)
        self.pg_eff += alpha * (cmd_g - self.pg_eff)
        self.pd_eff += alpha * (cmd_d - self.pd_eff)

        fg = self.pg_eff * (self.poussee_max_avant if self.pg_eff >= 0 else self.poussee_max_arriere)
        fd = self.pd_eff * (self.poussee_max_avant if self.pd_eff >= 0 else self.poussee_max_arriere)

        # Debattement elevons et debit limite de la seringue de ballast
        deb_max = P.ELEVON_DEBATTEMENT_MAX_RAD
        self.elevon_g = float(np.clip(cmd_elev_g, -deb_max, deb_max))
        self.elevon_d = float(np.clip(cmd_elev_d, -deb_max, deb_max))

        diff_bal = np.clip(cible_ballast_ml - self.ballast_ml,
                           -self.debit_ballast_max * dt, self.debit_ballast_max * dt)
        self.ballast_ml = float(np.clip(self.ballast_ml + diff_bal, 0.0, P.BALLAST_CAPACITE_ML))

        u, v, w = self.vel_body
        p, q, r = self.omega

        # Bilan des forces lineaires dans le repere robot
        f_prop = np.array([fg + fd, 0.0, 0.0])
        f_drag = -self.drag_lin * self.vel_body * np.abs(self.vel_body)

        alpha_inc = np.arctan2(-w, max(abs(u), 0.1)) if abs(u) > 0.05 else 0.0
        f_lift = np.array([0.0, 0.0, self.coef_portance * (u ** 2) * np.sin(2 * alpha_inc)])

        ecart_kg = (self.ballast_ml - self.ballast_neutre) * 1e-3
        f_buoy_world = np.array([0.0, 0.0, -ecart_kg * 9.81])
        f_buoy_body = self._monde_vers_local(f_buoy_world)

        m_tot = self.masse + self.masse_ajoutee
        f_coriolis = np.array([
            m_tot[1] * v * r - m_tot[2] * w * q,
            m_tot[2] * w * p - m_tot[0] * u * r,
            m_tot[0] * u * q - m_tot[1] * v * p
        ])

        acc_body = (f_prop + f_drag + f_lift + f_buoy_body + f_coriolis) / m_tot
        self.vel_body += acc_body * dt

        # Bilan des couples (elevons en v^2 + rappel pendulaire + lacet differentiel)
        elev_sym = 0.5 * (self.elevon_g + self.elevon_d)
        elev_diff = 0.5 * (self.elevon_g - self.elevon_d)
        v2 = u * abs(u)

        tau_roll = self.efficacite_elevon * v2 * elev_diff - self.couple_rappel_xy * np.sin(self.att[0])
        tau_pitch = self.efficacite_elevon * v2 * elev_sym - self.couple_rappel_xy * np.sin(self.att[1])
        tau_yaw = (fg - fd) * self.bras_levier_moteur

        tau_drag = -self.drag_rot * self.omega * np.abs(self.omega) - 0.35 * self.omega
        tau_tot = np.array([tau_roll, tau_pitch, tau_yaw]) + tau_drag

        self.omega += (tau_tot / self.inertie) * dt

        # Integration cinematique et limites verticales du bassin
        self.att += self.omega * dt
        self.att[0] = np.clip(self.att[0], -np.pi / 3, np.pi / 3)
        self.att[1] = np.clip(self.att[1], -np.pi / 3, np.pi / 3)
        self.att[2] = (self.att[2] + np.pi) % (2 * np.pi) - np.pi

        vel_world = self._local_vers_monde(self.vel_body) + self.courant_marin
        self.pos += vel_world * dt

        fond_z = -(P.PROFONDEUR_BASSIN_M - 0.2)
        if self.pos[2] > -0.15:
            self.pos[2] = -0.15
            if vel_world[2] > 0:
                self.vel_body[2] = min(0.0, self.vel_body[2])
        elif self.pos[2] < fond_z:
            self.pos[2] = fond_z

    @property
    def profondeur(self):
        return -self.pos[2]

    def lire_capteurs_bruites(self):
        roll, pitch, yaw = self.att
        return {
            "roll": roll + np.random.normal(0, P.BRUIT_ROLL_PITCH_RAD),
            "pitch": pitch + np.random.normal(0, P.BRUIT_ROLL_PITCH_RAD),
            "yaw": yaw + np.random.normal(0, P.BRUIT_YAW_RAD),
            "profondeur": self.profondeur + np.random.normal(0, P.BRUIT_PROFONDEUR_M),
            "vitesse_x": self.vel_body[0] + np.random.normal(0, P.BRUIT_VITESSE_MS),
            "omega": self.omega + np.random.normal(0, P.BRUIT_GYRO_RADS)
        }