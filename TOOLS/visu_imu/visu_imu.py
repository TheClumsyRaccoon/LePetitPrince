#!/usr/bin/env python3
"""Visualiseur 3D de l'IMU dans le navigateur.

Lit les lignes envoyees par test_imu.ino sur le port serie et affiche un bloc 3D
qui s'oriente comme la carte, avec les axes X (rouge), Y (vert), Z (bleu) de la puce.

    pip install pyserial
    python visu_imu.py              # liste les ports serie
    python visu_imu.py COM5         # lit la carte (fermer le moniteur serie de l'IDE !)
    python visu_imu.py --demo       # donnees fictives, sans carte

Touche R dans le navigateur : remet le yaw a zero.
Le yaw vient de l'integration du gyro : il derive, c'est normal (pas de magnetometre ici).
"""
import argparse
import json
import math
import queue
import re
import sys
import threading
import time
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

N = r"([-+]?\d+(?:\.\d+)?)"
MOTIF = re.compile(r"acc\[g\]\s+{n}\s+{n}\s+{n}.*gyro\[dps\]\s+{n}\s+{n}\s+{n}".format(n=N))

clients = []
verrou = threading.Lock()


def diffuser(msg):
    with verrou:
        for q in clients:
            try:
                q.put_nowait(msg)
            except queue.Full:
                pass


def traiter_ligne(ligne):
    m = MOTIF.search(ligne)
    if not m:
        return False
    ax, ay, az, gx, gy, gz = (float(x) for x in m.groups())
    diffuser(json.dumps({"t": time.time(), "ax": ax, "ay": ay, "az": az,
                         "gx": gx, "gy": gy, "gz": gz}))
    return True


def boucle_serie(s):
    while True:
        try:
            ligne = s.readline().decode(errors="ignore")
        except Exception as e:
            print("Erreur serie :", e)
            return
        traiter_ligne(ligne)


def boucle_demo():
    t0 = time.time()
    while True:
        t = time.time() - t0
        roll = math.radians(30 * math.sin(t))
        pitch = math.radians(20 * math.sin(0.7 * t))
        ax = -math.sin(pitch)
        ay = math.sin(roll) * math.cos(pitch)
        az = math.cos(roll) * math.cos(pitch)
        gx = 30 * math.cos(t)
        gy = 14 * math.cos(0.7 * t)
        gz = 20.0
        diffuser(json.dumps({"t": time.time(), "ax": ax, "ay": ay, "az": az,
                             "gx": gx, "gy": gy, "gz": gz}))
        time.sleep(0.02)


PAGE = r"""<!DOCTYPE html>
<html><head><meta charset="utf-8"><title>Visu IMU</title>
<style>
body{margin:0;background:#111;color:#eee;font-family:monospace;overflow:hidden}
#info{position:absolute;top:10px;left:10px;line-height:1.6;font-size:14px;
      background:rgba(0,0,0,.5);padding:8px 12px;border-radius:6px}
canvas{display:block}
</style></head><body>
<div id="info">en attente de donnees...</div>
<script src="https://cdnjs.cloudflare.com/ajax/libs/three.js/r128/three.min.js"></script>
<script>
const scene = new THREE.Scene();
scene.background = new THREE.Color(0x111111);
const camera = new THREE.PerspectiveCamera(50, innerWidth / innerHeight, 0.1, 100);
camera.up.set(0, 0, 1);
camera.position.set(-4, -3, 2.5);
camera.lookAt(0, 0, 0);
const renderer = new THREE.WebGLRenderer({antialias: true});
renderer.setSize(innerWidth, innerHeight);
document.body.appendChild(renderer.domElement);

const sol = new THREE.GridHelper(8, 16, 0x444444, 0x2a2a2a);
sol.rotation.x = Math.PI / 2;
scene.add(sol);

const corps = new THREE.Group();
const boite = new THREE.Mesh(new THREE.BoxGeometry(2.0, 1.4, 0.4),
    new THREE.MeshBasicMaterial({color: 0x3a6ea5, transparent: true, opacity: 0.35}));
corps.add(boite);
corps.add(new THREE.LineSegments(new THREE.EdgesGeometry(boite.geometry),
    new THREE.LineBasicMaterial({color: 0x9fc5ee})));

function etiquette(texte, couleur, pos) {
  const c = document.createElement('canvas');
  c.width = c.height = 64;
  const g = c.getContext('2d');
  g.fillStyle = couleur; g.font = 'bold 48px monospace';
  g.textAlign = 'center'; g.textBaseline = 'middle';
  g.fillText(texte, 32, 34);
  const s = new THREE.Sprite(new THREE.SpriteMaterial({map: new THREE.CanvasTexture(c)}));
  s.position.copy(pos); s.scale.set(0.5, 0.5, 0.5);
  return s;
}
const axes = [
  ['X', '#ff4444', 0xff4444, new THREE.Vector3(1, 0, 0)],
  ['Y', '#44dd44', 0x44dd44, new THREE.Vector3(0, 1, 0)],
  ['Z', '#5599ff', 0x5599ff, new THREE.Vector3(0, 0, 1)],
];
for (const [nom, css, hex, dir] of axes) {
  corps.add(new THREE.ArrowHelper(dir, new THREE.Vector3(0, 0, 0), 1.7, hex, 0.3, 0.18));
  corps.add(etiquette(nom, css, dir.clone().multiplyScalar(2.05)));
}
scene.add(corps);

const MONDE_HAUT = new THREE.Vector3(0, 0, 1);
const D2R = Math.PI / 180;
let accLisse = null, yaw = 0, tPrec = null;
const info = document.getElementById('info');

addEventListener('keydown', e => { if (e.key === 'r' || e.key === 'R') yaw = 0; });
addEventListener('resize', () => {
  camera.aspect = innerWidth / innerHeight; camera.updateProjectionMatrix();
  renderer.setSize(innerWidth, innerHeight);
});

const flux = new EventSource('/events');
flux.onmessage = (e) => {
  const d = JSON.parse(e.data);
  const acc = new THREE.Vector3(d.ax, d.ay, d.az);
  const norme = acc.length();
  if (norme < 1e-6) return;
  if (!accLisse) accLisse = acc.clone(); else accLisse.lerp(acc, 0.25);
  const haut = accLisse.clone().normalize();       // "haut" exprime dans le repere de la puce

  const dt = (tPrec === null) ? 0 : Math.min(d.t - tPrec, 0.2);
  tPrec = d.t;
  const wz = (d.gx * haut.x + d.gy * haut.y + d.gz * haut.z) * D2R;   // rotation autour de la verticale
  yaw += wz * dt;

  const qInclin = new THREE.Quaternion().setFromUnitVectors(haut, MONDE_HAUT);
  const qYaw = new THREE.Quaternion().setFromAxisAngle(MONDE_HAUT, yaw);
  corps.quaternion.copy(qYaw.multiply(qInclin));

  const roll = Math.atan2(d.ay, d.az) / D2R;
  const pitch = Math.atan2(-d.ax, Math.hypot(d.ay, d.az)) / D2R;
  const bon = norme > 0.95 && norme < 1.05;
  info.innerHTML =
    'norme acc : <b style="color:' + (bon ? '#6f6' : '#f66') + '">' + norme.toFixed(2) + ' g</b> (attendu ~1.00 a l\'arret)<br>' +
    'acc  [g]   : ' + d.ax.toFixed(2) + '  ' + d.ay.toFixed(2) + '  ' + d.az.toFixed(2) + '<br>' +
    'gyro [dps] : ' + d.gx.toFixed(1) + '  ' + d.gy.toFixed(1) + '  ' + d.gz.toFixed(1) + '<br>' +
    'roll ' + roll.toFixed(0) + '   pitch ' + pitch.toFixed(0) + '   yaw (gyro) ' + (yaw / D2R).toFixed(0) + '<br>' +
    '<span style="color:#999">R = remettre le yaw a zero</span>';
};

(function boucle() { requestAnimationFrame(boucle); renderer.render(scene, camera); })();
</script></body></html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path == "/":
            corps = PAGE.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(corps)))
            self.end_headers()
            self.wfile.write(corps)
        elif self.path == "/events":
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-cache")
            self.end_headers()
            q = queue.Queue(maxsize=50)
            with verrou:
                clients.append(q)
            try:
                while True:
                    try:
                        msg = q.get(timeout=15)
                        self.wfile.write(("data: " + msg + "\n\n").encode())
                    except queue.Empty:
                        self.wfile.write(b": ping\n\n")
                    self.wfile.flush()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError):
                pass
            finally:
                with verrou:
                    if q in clients:
                        clients.remove(q)
        else:
            self.send_error(404)


def main():
    p = argparse.ArgumentParser(description="Visualiseur 3D de l'IMU")
    p.add_argument("port", nargs="?", help="port serie (ex : COM5 ou /dev/ttyACM0)")
    p.add_argument("--demo", action="store_true", help="donnees fictives")
    p.add_argument("--vitesse", type=int, default=115200)
    p.add_argument("--http", type=int, default=8765)
    a = p.parse_args()

    if a.demo:
        threading.Thread(target=boucle_demo, daemon=True).start()
    else:
        try:
            import serial
            import serial.tools.list_ports
        except ImportError:
            sys.exit("Installez pyserial :  pip install pyserial")
        if not a.port:
            print("Ports serie disponibles :")
            for x in serial.tools.list_ports.comports():
                print("  ", x.device, "-", x.description)
            sys.exit("Relancez avec le port, par exemple :  python visu_imu.py COM5")
        try:
            s = serial.Serial(a.port, a.vitesse, timeout=1)
        except serial.SerialException as e:
            sys.exit("Impossible d'ouvrir %s : %s\n(le moniteur serie de l'IDE est-il encore ouvert ?)" % (a.port, e))
        threading.Thread(target=boucle_serie, args=(s,), daemon=True).start()

    serveur = ThreadingHTTPServer(("127.0.0.1", a.http), Handler)
    serveur.daemon_threads = True
    url = "http://127.0.0.1:%d" % a.http
    print("Visualiseur sur", url, "(Ctrl+C pour arreter)")
    webbrowser.open(url)
    try:
        serveur.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
