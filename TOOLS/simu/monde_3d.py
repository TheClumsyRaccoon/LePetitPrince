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
        props.setTitle("Le Petit Prince - Prototype 3D (V1)")
        props.setSize(960, 600)
        self.win.requestProperties(props)

        self.setBackgroundColor(0.05, 0.25, 0.38)
        self._configurer_lumiere()
        self._construire_scene()

    def _configurer_lumiere(self):
        amb = AmbientLight("ambiante")
        amb.setColor((0.5, 0.6, 0.7, 1.0))
        self.render.setLight(self.render.attachNewNode(amb))

        soleil = DirectionalLight("soleil")
        soleil.setColor((0.9, 0.95, 1.0, 1.0))
        np_soleil = self.render.attachNewNode(soleil)
        np_soleil.setHpr(30, -70, 0)
        self.render.setLight(np_soleil)

    def _construire_scene(self):
        # Fond et surface du bassin
        sol = creer_boite("fond", 40.0, 40.0, 0.2, (0.6, 0.55, 0.4, 1.0))
        sol.setPos(0, 0, -P.PROFONDEUR_BASSIN_M - 0.1)
        sol.reparentTo(self.render)

        surface = creer_boite("surface", 40.0, 40.0, 0.02, (0.3, 0.7, 0.9, 0.25))
        surface.setTransparency(True)
        surface.reparentTo(self.render)

        # Cible fixe de reference
        self.cible_np = creer_boite("cible", 0.35, 0.2, 0.2, (1.0, 0.35, 0.0, 1.0))
        self.cible_np.setPos(4.0, 0.0, -1.5)
        self.cible_np.reparentTo(self.render)

        # Prototype robot (un corps + un nez rouge pour verifier l'orientation)
        self.robot_np = self.render.attachNewNode("raie_v1")
        corps = creer_boite("corps", 0.80, 0.60, 0.10, (0.15, 0.35, 0.55, 1.0))
        corps.reparentTo(self.robot_np)

        nez = creer_boite("nez", 0.12, 0.12, 0.12, (0.9, 0.2, 0.2, 1.0))
        nez.setPos(0.0, 0.32, 0.0)  # fix nez sur +Y
        nez.reparentTo(self.robot_np)

    def maj(self, sim, dt):
        self.robot_np.setPos(*sim.pos)
        roll_deg, pitch_deg, yaw_deg = np.degrees(sim.att)
        self.robot_np.setHpr(yaw_deg - 90.0, roll_deg, -pitch_deg)

        # Camera de poursuite derriere le robot
        yaw = sim.att[2]
        cam_x = sim.pos[0] - 2.5 * np.cos(yaw)
        cam_y = sim.pos[1] - 2.5 * np.sin(yaw)
        cam_z = min(-0.3, sim.pos[2] + 0.8)
        self.camera.setPos(cam_x, cam_y, cam_z)
        self.camera.lookAt(self.robot_np)
        