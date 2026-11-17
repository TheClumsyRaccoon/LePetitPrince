import numpy as np
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight, DirectionalLight, Fog, Geom, GeomNode,
    GeomTriangles, GeomVertexData, GeomVertexFormat, GeomVertexWriter,
    NodePath, Vec3, Vec4, WindowProperties
)
import config as P


def creer_boite(nom, sx, sy, sz, couleur):
    fmt = GeomVertexFormat.get_v3n3c4()
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
        props.setTitle("Le Petit Prince - Simulateur 3D (V3)")
        props.setSize(1280, 720)
        self.win.requestProperties(props)

        # Tubidité
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

        # Parametres camera TPS
        self.t_scene = 0.0
        self.cam_dist = 3.2
        self.cam_azimut = -140.0
        self.cam_elevation = 22.0
        self.souris_drag = False
        self.derniere_souris = (0, 0)

        self.accept("mouse1", self._debut_drag)
        self.accept("mouse1-up", self._fin_drag)
        self.accept("wheel_up", lambda: self._zoom(-0.3))
        self.accept("wheel_down", lambda: self._zoom(0.3))

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
        """Poisson cible mobile orange vif (aligne sur +Y)."""
        self.cible_np = self.render.attachNewNode("PoissonCible")
        corps = creer_boite("poisson_corps", 0.10, 0.34, 0.14, (1.0, 0.42, 0.05, 1.0))
        corps.reparentTo(self.cible_np)
        queue = creer_boite("poisson_queue", 0.03, 0.12, 0.16, (1.0, 0.65, 0.10, 1.0))
        queue.setPos(0.0, -0.22, 0.0)
        queue.reparentTo(self.cible_np)

    def maj(self, sim, dt):
        self.t_scene += dt

        # Position et orientation de la raie
        self.robot_np.setPos(*sim.pos)
        roll_deg, pitch_deg, yaw_deg = np.degrees(sim.att)
        self.robot_np.setHpr(yaw_deg - 90.0, -pitch_deg, roll_deg)

        # Braquage visuel des elevons
        self.elevon_g_np.setP(-np.degrees(sim.elevon_g))
        self.elevon_d_np.setP(-np.degrees(sim.elevon_d))

        # Trajectoire circulaire du poisson cible
        cx = 5.5 + 3.5 * np.cos(0.18 * self.t_scene)
        cy = 3.5 * np.sin(0.18 * self.t_scene)
        cz = -2.0 + 0.5 * np.sin(0.35 * self.t_scene)
        cap_poisson = np.degrees(np.arctan2(
            3.5 * 0.18 * np.cos(0.18 * self.t_scene),
            -3.5 * 0.18 * np.sin(0.18 * self.t_scene)
        ))
        self.cible_np.setPos(cx, cy, cz)
        self.cible_np.setHpr(cap_poisson - 90.0, 0, 0)

        # Camera TPS (commandée à la souris qui suit la raie
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