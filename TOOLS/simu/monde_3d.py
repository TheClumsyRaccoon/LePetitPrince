import numpy as np
from direct.showbase.ShowBase import ShowBase
from panda3d.core import (
    AmbientLight, DirectionalLight, Geom, GeomNode,
    GeomTriangles, GeomVertexData, GeomVertexFormat, GeomVertexWriter,
    NodePath, WindowProperties
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
        props.setTitle("Le Petit Prince - Simulateur 3D (V2)")
        props.setSize(1024, 640)
        self.win.requestProperties(props)

        self.setBackgroundColor(0.05, 0.25, 0.38)
        self._configurer_lumiere()
        self._construire_scene()
        self._construire_raie()

    def _configurer_lumiere(self):
        amb = AmbientLight("ambiante")
        amb.setColor((0.45, 0.55, 0.65, 1.0))
        self.render.setLight(self.render.attachNewNode(amb))

        soleil = DirectionalLight("soleil")
        soleil.setColor((0.9, 0.95, 1.0, 1.0))
        np_soleil = self.render.attachNewNode(soleil)
        np_soleil.setHpr(35, -65, 0)
        self.render.setLight(np_soleil)

    def _construire_scene(self):
        fond_z = -P.PROFONDEUR_BASSIN_M
        sol = creer_boite("fond", 40.0, 40.0, 0.2, (0.58, 0.54, 0.40, 1.0))
        sol.setPos(0, 0, fond_z - 0.1)
        sol.reparentTo(self.render)

        # Quadrillage au fond pour visualiser l'avance et la derive
        for i in range(-18, 20, 2):
            gx = creer_boite(f"gx_{i}", 38.0, 0.05, 0.02, (0.35, 0.38, 0.30, 1.0))
            gx.setPos(0, i, fond_z + 0.01)
            gx.reparentTo(self.render)

            gy = creer_boite(f"gy_{i}", 0.05, 38.0, 0.02, (0.35, 0.38, 0.30, 1.0))
            gy.setPos(i, 0, fond_z + 0.01)
            gy.reparentTo(self.render)

        surface = creer_boite("surface", 40.0, 40.0, 0.02, (0.35, 0.75, 0.92, 0.30))
        surface.setTransparency(True)
        surface.reparentTo(self.render)

        # Cible fixe de reference
        self.cible_np = creer_boite("cible", 0.35, 0.20, 0.20, (1.0, 0.38, 0.04, 1.0))
        self.cible_np.setPos(4.5, 0.0, -1.5)
        self.cible_np.reparentTo(self.render)

    def _construire_raie(self):
        """Assemble la raie alignée sur +"""
        self.robot_np = self.render.attachNewNode("RaieMobula")

        # Corps central 
        corps = creer_boite("corps", 0.22, 0.55, 0.14, (0.12, 0.18, 0.28, 1.0))
        corps.reparentTo(self.robot_np)

        # Nez et camera avant (+Y)
        nez = creer_boite("nez", 0.14, 0.12, 0.09, (0.95, 0.85, 0.20, 1.0))
        nez.setPos(0.0, 0.30, 0.0)
        nez.reparentTo(self.robot_np)

        # Ailes delta gauche (-X) et droite (+X)
        aile_g = creer_boite("aile_g", 0.32, 0.38, 0.025, (0.18, 0.28, 0.42, 1.0))
        aile_g.setPos(-0.25, -0.02, 0.0)
        aile_g.setH(18)
        aile_g.reparentTo(self.robot_np)

        aile_d = creer_boite("aile_d", 0.32, 0.38, 0.025, (0.18, 0.28, 0.42, 1.0))
        aile_d.setPos(0.25, -0.02, 0.0)
        aile_d.setH(-18)
        aile_d.reparentTo(self.robot_np)

        # Pivots des elevons arriere (-Y)
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

        # Nacelles propulseurs
        prop_g = creer_boite("prop_g", 0.06, 0.14, 0.06, (0.1, 0.1, 0.1, 1.0))
        prop_g.setPos(-0.2, -0.28, -0.05)
        prop_g.reparentTo(self.robot_np)

        prop_d = creer_boite("prop_d", 0.06, 0.14, 0.06, (0.1, 0.1, 0.1, 1.0))
        prop_d.setPos(0.2, -0.28, -0.05)
        prop_d.reparentTo(self.robot_np)

    def maj(self, sim, dt):
        self.robot_np.setPos(*sim.pos)
        roll_deg, pitch_deg, yaw_deg = np.degrees(sim.att)
        # H = yaw - 90 (0 deg physique = +X), P = -pitch (nez haut), R = roll
        self.robot_np.setHpr(yaw_deg - 90.0, -pitch_deg, roll_deg)

        # Braquage visuel des elevons
        self.elevon_g_np.setP(-np.degrees(sim.elevon_g))
        self.elevon_d_np.setP(-np.degrees(sim.elevon_d))

        # Camera de poursuite derriere le robot
        yaw = sim.att[2]
        cam_x = sim.pos[0] - 2.6 * np.cos(yaw)
        cam_y = sim.pos[1] - 2.6 * np.sin(yaw)
        cam_z = min(-0.3, sim.pos[2] + 0.8)
        self.camera.setPos(cam_x, cam_y, cam_z)
        self.camera.lookAt(self.robot_np)