import config as P
from physique import Modele6DDL
from monde_3d import SceneSousMarine3D


def tester_v1():
    sim = Modele6DDL()
    scene = SceneSousMarine3D()

    def boucle_test(task):
        dt = min(globalClock.getDt(), 0.05)
        # Avance droite puis virage a droiet
        if task.time < 4.0:
            sim.step(0.4, 0.4, 0.0, 0.0, P.BALLAST_NEUTRE_ML, dt)
        else:
            sim.step(0.45, 0.25, -0.03, -0.03, P.BALLAST_NEUTRE_ML, dt)

        scene.maj(sim, dt)
        return task.cont

    scene.taskMgr.add(boucle_test, "test_v1")
    scene.run()


if __name__ == "__main__":
    tester_v1()