from pathlib import Path
import numpy as np
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight, DirectionalLight, Filename, Fog, Geom, GeomNode,
    GeomTriangles, GeomVertexData, GeomVertexFormat, GeomVertexWriter,
    NodePath, PerspectiveLens, Texture, Vec3, Vec4, WindowProperties
)
import config as P


def creer_boite(nom, sx, sy, sz, couleur):
    fmt = GeomVertexFormat.getV3n3c4()
    vdata = GeomVertexData(nom, fmt, Geom.UHStatic)
    vw = GeomVertexWriter(vdata, "vertex")
    nw = GeomVertexWriter(vdata, "normal")
    cw = GeomVertexWriter(vdata, "color")

    hx, hy, hz = sx / 2.0, sy / 2.0, sz / 2.0
    coins = [
        (-hx, -hy, -hz), (hx, -hy, -hz), (hx, hy, -hz), (-hx, hy, -hz),
        (-hx, -hy,  hz), (hx, -hy,  hz), (hx, hy,  hz), (-hx, hy,  hz)
    ]
    faces = [
        ((0, 1, 2, 3), (0, 0, -1)),
        ((4, 7, 6, 5), (0, 0,  1)),
        ((0, 4, 5, 1), (0, -1, 0)),
        ((2, 6, 7, 3), (0,  1, 0)),
        ((0, 3, 7, 4), (-1, 0, 0)),
        ((1, 5, 6, 2), ( 1, 0, 0)),
    ]

    tris = GeomTriangles(Geom.UHStatic)
    idx = 0
    for quad, norm in faces:
        for q in quad:
            vw.addData3(*coins[q])
            nw.addData3(*norm)
            cw.addData4(*couleur)
        tris.addVertices(idx, idx + 1, idx + 2)
        tris.addVertices(idx, idx + 2, idx + 3)
        idx += 4

    geom = Geom(vdata)
    geom.addPrimitive(tris)
    node = GeomNode(nom)
    node.addGeom(geom)
    return NodePath(node)


class SceneSousMarine3D(ShowBase):
    def __init__(self):
        super().__init__()
        self.disableMouse()

        props = WindowProperties()
        props.setTitle("Le Petit Prince - Simulateur 3D")
        props.setSize(1280, 720)
        self.win.requestProperties(props)

        # Turbidité
        couleur_eau = Vec4(0.03, 0.22, 0.36, 1.0)
        self.setBackgroundColor(couleur_eau)
        brouillard = Fog("EauFog")
        brouillard.setColor(0.03, 0.22, 0.36)
        brouillard.setExpDensity(0.055)
        self.render.setFog(brouillard)

        self._configurer_lumiere()
        self._construire_scene()
        self._construire_raie()
        self._construire_cible()
        self._configurer_camera_embarquee()

        # Parametres camera TPS
        self.t_scene = 0.0
        self.vue_embarquee = False
        self.cam_dist = 3.2
        self.cam_azimut = -140.0
        self.cam_elevation = 22.0
        self.souris_drag = False
        self.derniere_souris = (0, 0)

        self.accept("mouse1", self._debut_drag)
        self.accept("mouse1-up", self._fin_drag)
        self.accept("wheel_up", lambda: self._zoom(-0.3))
        self.accept("wheel_down", lambda: self._zoom(0.3))
        self.accept("v", self.basculer_camera)

    def basculer_camera(self):
        self.vue_embarquee = not self.vue_embarquee

    def _configurer_lumiere(self):
        amb = AmbientLight("ambiante")
        amb.setColor(Vec4(0.38, 0.48, 0.58, 1.0))
        self.render.setLight(self.render.attachNewNode(amb))

        soleil = DirectionalLight("soleil")
        soleil.setColor(Vec4(0.85, 0.92, 1.0, 1.0))
        np_soleil = self.render.attachNewNode(soleil)
        np_soleil.setHpr(35, -65, 0)
        self.render.setLight(np_soleil)

    def _construire_scene(self):
        fond_z = -P.PROFONDEUR_BASSIN_M
        sol = creer_boite("fond", 40.0, 40.0, 0.2, (0.55, 0.52, 0.38, 1.0))
        sol.setPos(0, 0, fond_z - 0.1)
        sol.reparentTo(self.render)

        # Quadrillage au fond pour repere visuel
        for i in range(-18, 20, 2):
            gx = creer_boite(f"gx_{i}", 38.0, 0.05, 0.02, (0.35, 0.38, 0.30, 1.0))
            gx.setPos(0, i, fond_z + 0.01)
            gx.reparentTo(self.render)

            gy = creer_boite(f"gy_{i}", 0.05, 38.0, 0.02, (0.35, 0.38, 0.30, 1.0))
            gy.setPos(i, 0, fond_z + 0.01)
            gy.reparentTo(self.render)

        surface = creer_boite("surface", 40.0, 40.0, 0.02, (0.45, 0.78, 0.95, 0.35))
        surface.setTransparency(True)
        surface.reparentTo(self.render)

        # Rochers et bouee dans le bassin
        obstacles = [
            (4.5,  2.5, fond_z + 1.2, 1.4, 1.4, 2.4, (0.28, 0.32, 0.35, 1.0)),
            (7.0, -3.0, fond_z + 1.5, 1.8, 1.2, 3.0, (0.25, 0.30, 0.33, 1.0)),
            (-4.0, 4.0, fond_z + 1.0, 1.5, 1.5, 2.0, (0.30, 0.33, 0.36, 1.0)),
            (2.0, -4.5, -2.0,         0.35, 0.35, 0.35, (0.9, 0.2, 0.2, 1.0)),
        ]
        for idx, (ox, oy, oz, sx, sy, sz, col) in enumerate(obstacles):
            obs = creer_boite(f"obs_{idx}", sx, sy, sz, col)
            obs.setPos(ox, oy, oz)
            obs.reparentTo(self.render)

    def _construire_raie(self):
        """Assemble la raie Mobula alignee sur +Y (axe avant natif Panda3D)."""
        self.robot_np = self.render.attachNewNode("RaieMobula")

        corps = creer_boite("corps", 0.22, 0.55, 0.14, (0.12, 0.18, 0.28, 1.0))
        corps.reparentTo(self.robot_np)

        nez = creer_boite("nez", 0.14, 0.12, 0.09, (0.95, 0.85, 0.20, 1.0))
        nez.setPos(0.0, 0.30, 0.0)
        nez.reparentTo(self.robot_np)

        aile_g = creer_boite("aile_g", 0.32, 0.38, 0.025, (0.18, 0.28, 0.42, 1.0))
        aile_g.setPos(-0.25, -0.02, 0.0)
        aile_g.setH(18)
        aile_g.reparentTo(self.robot_np)

        aile_d = creer_boite("aile_d", 0.32, 0.38, 0.025, (0.18, 0.28, 0.42, 1.0))
        aile_d.setPos(0.25, -0.02, 0.0)
        aile_d.setH(-18)
        aile_d.reparentTo(self.robot_np)

        self.elevon_g_np = self.robot_np.attachNewNode("pivot_elev_g")
        self.elevon_g_np.setPos(-0.22, -0.24, 0.0)
        eg_mesh = creer_boite("eg", 0.20, 0.10, 0.018, (0.95, 0.45, 0.10, 1.0))
        eg_mesh.setPos(0.0, -0.04, 0.0)
        eg_mesh.reparentTo(self.elevon_g_np)

        self.elevon_d_np = self.robot_np.attachNewNode("pivot_elev_d")
        self.elevon_d_np.setPos(0.22, -0.24, 0.0)
        ed_mesh = creer_boite("ed", 0.20, 0.10, 0.018, (0.95, 0.45, 0.10, 1.0))
        ed_mesh.setPos(0.0, -0.04, 0.0)
        ed_mesh.reparentTo(self.elevon_d_np)

        prop_g = creer_boite("prop_g", 0.06, 0.14, 0.06, (0.1, 0.1, 0.1, 1.0))
        prop_g.setPos(-0.18, -0.28, -0.05)
        prop_g.reparentTo(self.robot_np)

        prop_d = creer_boite("prop_d", 0.06, 0.14, 0.06, (0.1, 0.1, 0.1, 1.0))
        prop_d.setPos(0.18, -0.28, -0.05)
        prop_d.reparentTo(self.robot_np)

    def _construire_cible(self):
        """Genere un nombre aleatoire de poissons a partir de tous les modeles dans assets/."""
        self.rng = np.random.default_rng()
        self.poissons = []

        dossier_assets = Path(__file__).resolve().parent / "assets"
        fichiers_glb = sorted(dossier_assets.glob("*.glb")) if dossier_assets.exists() else []

        nb_poissons = int(self.rng.integers(max(220, len(fichiers_glb)), max(280, len(fichiers_glb) + 5)))

        for idx in range(nb_poissons):
            racine = self.render.attachNewNode(f"Poisson_{idx}")

            if fichiers_glb:
                # Garantit que chaque modele .glb de assets/ est utilise au moins une fois
                fichier = fichiers_glb[idx] if idx < len(fichiers_glb) else self.rng.choice(fichiers_glb)
                modele = self.loader.loadModel(Filename.fromOsSpecific(str(fichier)))
                modele.reparentTo(racine)

                p_min, p_max = modele.getTightBounds()
                taille = p_max - p_min
                dim_max = max(taille.x, taille.y, taille.z, 1e-6)
                longueur_cible = float(self.rng.uniform(0.30, 0.55))
                echelle = longueur_cible / dim_max
                modele.setScale(echelle)
                centre = (p_min + p_max) * 0.5 * echelle
                modele.setPos(-centre)
            else:
                corps = creer_boite(f"p_corps_{idx}", 0.10, 0.34, 0.14, (1.0, 0.42, 0.05, 1.0))
                corps.reparentTo(racine)
                queue = creer_boite(f"p_queue_{idx}", 0.03, 0.12, 0.16, (1.0, 0.65, 0.10, 1.0))
                queue.setPos(0.0, -0.22, 0.0)
                queue.reparentTo(racine)

            pos = np.array([
                self.rng.uniform(-10.0, 12.0),
                self.rng.uniform(-10.0, 10.0),
                self.rng.uniform(-P.PROFONDEUR_BASSIN_M + 0.6, -0.6),
            ])
            cap = float(self.rng.uniform(-np.pi, np.pi))
            racine.setPos(*pos)
            racine.setHpr(np.degrees(cap) - 90.0, 0, 0)

            self.poissons.append({
                "np": racine,
                "pos": pos,
                "cap": cap,
                "vitesse": float(self.rng.uniform(0.35, 0.85)),
                "vitesse_cap": float(self.rng.uniform(-0.4, 0.4)),
                "vz": 0.0,
                "timer_virage": float(self.rng.uniform(1.0, 4.0)),
            })

        print(f"[3D] {len(self.poissons)} poissons charges depuis {len(fichiers_glb)} fichiers .glb")
        for i, p in enumerate(self.poissons):
            print(f"  - Poisson {i} en pos={np.round(p['pos'], 2)}")
        if self.poissons:
            self.cible_np = self.poissons[0]["np"]

    def _configurer_camera_embarquee(self):
        """Cree une camera virtuelle hors-ecran fixee sur le nez de la raie (+Y)."""
        self.cam_w = P.CAM_LARGEUR
        self.cam_h = P.CAM_HAUTEUR
        self.tex_embarquee = Texture()
        self.buf_embarque = self.win.makeTextureBuffer(
            "CamEmbarquee", self.cam_w, self.cam_h, self.tex_embarquee, to_ram=True
        )
        self.buf_embarque.setSort(-100)

        self.cam_robot = self.makeCamera(self.buf_embarque)
        lens = PerspectiveLens()
        lens.setFov(P.CAM_FOV_DEG)
        lens.setNearFar(0.08, 40.0)
        self.cam_robot.node().setLens(lens)

        # Nez de la raie est sur +Y 
        self.cam_robot.reparentTo(self.robot_np)
        self.cam_robot.setPos(0.0, 0.36, 0.0)
        self.cam_robot.setHpr(0, 0, 0)

    def capturer_frame_bgr(self):
        """Extrait l'image de la camera virtuelle au format OpenCV (H, W, 3) BGR."""
        if not self.tex_embarquee.hasRamImage():
            return np.zeros((self.cam_h, self.cam_w, 3), dtype=np.uint8)

        data = self.tex_embarquee.getRamImage()
        arr = np.frombuffer(data, dtype=np.uint8)
        if arr.size != self.cam_w * self.cam_h * 4:
            return np.zeros((self.cam_h, self.cam_w, 3), dtype=np.uint8)

        img = arr.reshape((self.cam_h, self.cam_w, 4))
        return np.flipud(img[:, :, :3]).copy()

    def maj(self, sim, dt):
        self.t_scene += dt

        # Position et orientation de la raie
        self.robot_np.setPos(*sim.pos)
        roll_deg, pitch_deg, yaw_deg = np.degrees(sim.att)
        self.robot_np.setHpr(yaw_deg - 90.0, -pitch_deg, roll_deg)

        # Braquage visuel des elevons
        self.elevon_g_np.setP(-np.degrees(sim.elevon_g))
        self.elevon_d_np.setP(-np.degrees(sim.elevon_d))

        # Deplacement aleatoire des poissons dans le bassin
        z_min = -P.PROFONDEUR_BASSIN_M + 0.5
        z_max = -0.5
        for p in self.poissons:
            p["timer_virage"] -= dt
            if p["timer_virage"] <= 0.0:
                p["vitesse_cap"] = float(self.rng.uniform(-0.7, 0.7))
                p["vitesse"] = float(self.rng.uniform(0.30, 0.90))
                p["vz"] = float(self.rng.uniform(-0.18, 0.18))
                p["timer_virage"] = float(self.rng.uniform(1.5, 4.5))

            # Demi-tour doux a l'approche des parois du bassin
            if abs(p["pos"][0]) > 15.0 or abs(p["pos"][1]) > 15.0:
                cap_centre = np.arctan2(-p["pos"][1], -p["pos"][0])
                err = (cap_centre - p["cap"] + np.pi) % (2.0 * np.pi) - np.pi
                p["vitesse_cap"] = float(np.clip(1.2 * err, -1.2, 1.2))

            p["cap"] = (p["cap"] + p["vitesse_cap"] * dt + np.pi) % (2.0 * np.pi) - np.pi
            p["pos"][0] += p["vitesse"] * np.cos(p["cap"]) * dt
            p["pos"][1] += p["vitesse"] * np.sin(p["cap"]) * dt
            p["pos"][2] = float(np.clip(p["pos"][2] + p["vz"] * dt, z_min, z_max))

            pitch_p = np.degrees(np.arctan2(p["vz"], max(p["vitesse"], 0.1)))
            p["np"].setPos(*p["pos"])
            p["np"].setHpr(np.degrees(p["cap"]) - 90.0, -pitch_p, 0.0)

        # Camera TPS (commandée à la souris qui suit la raie
        if self.vue_embarquee:
            self.camera.reparentTo(self.robot_np)
            self.camera.setPos(0.0, 0.36, 0.04)
            self.camera.setHpr(0, 0, 0)
        else:
            self.camera.reparentTo(self.render)
            if self.souris_drag and self.mouseWatcherNode.hasMouse():
                mx = self.mouseWatcherNode.getMouseX()
                my = self.mouseWatcherNode.getMouseY()
                dx = mx - self.derniere_souris[0]
                dy = my - self.derniere_souris[1]
                self.cam_azimut -= dx * 120.0
                self.cam_elevation = float(np.clip(self.cam_elevation - dy * 90.0, -40.0, 85.0))
                self.derniere_souris = (mx, my)

            az = np.radians(self.cam_azimut)
            el = np.radians(self.cam_elevation)
            offset = Vec3(
                self.cam_dist * np.cos(el) * np.cos(az),
                self.cam_dist * np.cos(el) * np.sin(az),
                self.cam_dist * np.sin(el)
            )
            cible_cam = Vec3(*sim.pos)
            self.camera.setPos(cible_cam + offset)
            self.camera.lookAt(cible_cam)

    def _debut_drag(self):
        if self.mouseWatcherNode.hasMouse():
            self.souris_drag = True
            self.derniere_souris = (
                self.mouseWatcherNode.getMouseX(),
                self.mouseWatcherNode.getMouseY()
            )

    def _fin_drag(self):
        self.souris_drag = False

    def _zoom(self, delta):
        self.cam_dist = float(np.clip(self.cam_dist + delta, 1.0, 12.0))