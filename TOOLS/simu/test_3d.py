import cv2
import numpy as np
from direct.task import Task
import config as P
from physique import Modele6DDL
from monde_3d import SceneSousMarine3D


def lancer_test_3d():
    sim = Modele6DDL()
    scene = SceneSousMarine3D()

    print("Souris : orbite / zoom | Touche 'V' : vue embarquée | 'Q' : quitter")

    def boucle_test(task):
        dt = min(globalClock.getDt(), 0.05)

        nb_pas = max(1, int(round(dt / P.DT_PHYSIQUE_S)))
        dt_sub = dt / nb_pas
        for _ in range(nb_pas):
            sim.step(0.35, 0.30, -0.02, -0.02, P.BALLAST_NEUTRE_ML, dt_sub)

        scene.maj(sim, dt)

        # Verifictaion du flux video embarque vers OpenCV
        frame_bgr = scene.capturer_frame_bgr()
        cv2.putText(
            frame_bgr,
            f"Prof: {sim.profondeur:.2f}m | Yaw: {np.degrees(sim.att[2]):.1f} deg",
            (10, 25),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
        )
        cv2.imshow("Camera Embarquee Virtuelle (OpenCV)", frame_bgr)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            cv2.destroyAllWindows()
            scene.userExit()
            return Task.done

        return Task.cont

    scene.taskMgr.add(boucle_test, "boucle_test_3d")
    scene.run()


if __name__ == "__main__":
    lancer_test_3d()