import base64
import json
import re
import subprocess
import urllib.request
from pathlib import Path

BT = "`" * 3

# Exclusions
DOSSIERS_EXCLUS = {
    ".git", ".github", "__pycache__", ".vscode", ".idea",
    "node_modules", "TODO", "RAPPORTS_DE_SEANCES",
}
EXTENSIONS_EXCLUES = (".pyc", ".pyo")
FICHIERS_EXCLUS = {".DS_Store"}

# Descriptions, penser à update avec l'jaout de nouveaux tools.
DESCRIPTIONS_TOOLS = {
    "scan_i2c": "Détecte les adresses présentes sur le bus I²C",
    "test_lecture_bme280": "Test du capteur interne (température et pression, BMP280)",
    "test_lecture_MS5837": "Test du capteur de pression externe (profondeur)",
    "test_imu": "Test de l'IMU 6 axes ICM-45686 et du magnétomètre",
    "test_ina226": "Calibration et mesure tension/courant des 3 INA226",
    "visu_imu": "Visualisation 3D de l'orientation de l'IMU dans le navigateur",
    "simu_robot": "Simulateur 3D 6-DDL (Panda3D + OpenCV) pour tester les PID et la vision",
}


def get_repo_github():
    """Récupère (owner, repo) depuis le remote origin."""
    url = subprocess.run(
        ["git", "remote", "get-url", "origin"],
        capture_output=True, text=True, check=True,
    ).stdout.strip()
    m = re.search(r"github\.com[:/]([^/]+)/([^/.]+)(?:\.git)?$", url)
    if not m:
        raise RuntimeError(f"URL du remote non reconnue : {url}")
    return m.group(1), m.group(2)


def api_github(endpoint, token=None):
    headers = {"Accept": "application/vnd.github+json", "User-Agent": "gen-readme"}
    if token:
        headers["Authorization"] = f"Bearer {token}"
    req = urllib.request.Request(f"https://api.github.com{endpoint}", headers=headers)
    with urllib.request.urlopen(req, timeout=10) as resp:
        return json.loads(resp.read().decode("utf-8"))


def lire_fichier(owner, repo, chemin, branche, token=None):
    try:
        data = api_github(f"/repos/{owner}/{repo}/contents/{chemin}?ref={branche}", token)
        return base64.b64decode(data["content"]).decode("utf-8", errors="ignore")
    except Exception:
        return ""  


def lire_defines(contenu):
    """Récupère les #define de config.h sous forme de dict."""
    defines = {}
    for ligne in contenu.splitlines():
        m = re.match(r"^\s*#define\s+([A-Z0-9_]+)\s+([^\s/]+)", ligne)
        if m:
            defines[m.group(1)] = m.group(2)
    return defines


def arborescence(chemins, nom_racine, max_prof=3):
    arbre = {}
    for chemin in chemins:
        parties = chemin.split("/")
        if any(p in DOSSIERS_EXCLUS or p.startswith(".") for p in parties):
            continue
        if parties[-1] in FICHIERS_EXCLUS or parties[-1].endswith(EXTENSIONS_EXCLUES):
            continue
        if len(parties) > max_prof:
            continue
        noeud = arbre
        for p in parties:
            noeud = noeud.setdefault(p, {})

    def afficher(d, prefixe=""):
        lignes = []
        # dossiers d'abord, puis fichiers
        cles = sorted(d, key=lambda k: (not d[k], k.lower()))
        for i, k in enumerate(cles):
            dernier = i == len(cles) - 1
            lignes.append(f"{prefixe}{'└── ' if dernier else '├── '}{k}{'/' if d[k] else ''}")
            if d[k]:
                lignes += afficher(d[k], prefixe + ("    " if dernier else "│   "))
        return lignes

    return "\n".join([f"{nom_racine}/"] + afficher(arbre))


def generer_readme(owner=None, repo=None, branche="main", token=None):
    if not owner or not repo:
        owner, repo = get_repo_github()

    arbre_git = api_github(f"/repos/{owner}/{repo}/git/trees/{branche}?recursive=1", token)
    chemins = [i["path"] for i in arbre_git.get("tree", []) if i.get("type") == "blob"]

    # Adresses et constantes lues dans le fichier config
    config = lire_fichier(owner, repo, "FIRMWARE/LePetitPrince_ESP32/config.h", branche, token)
    d = lire_defines(config)

    addr_bmp = d.get("ADDR_BMP280", "0x77")
    addr_ms = d.get("ADDR_MS5837", "0x76")
    addr_ina_g = d.get("ADDR_INA_G", "0x40")
    addr_ina_d = d.get("ADDR_INA_D", "0x41")
    addr_ina_e = d.get("ADDR_INA_ELEC", "0x44")
    addr_imu = d.get("ADDR_ICM45686", "0x68")
    addr_mag = d.get("ADDR_QMC6309", "0x7C")
    baud_uart = d.get("VITESSE_UART", "921600")
    courant_max = d.get("COURANT_MAX_A", "15.0")

    # Liste des tools
    tools = sorted(
        {c.split("/")[1] for c in chemins
         if c.startswith("TOOLS/") and c.count("/") >= 2 and c.split("/")[1] not in DOSSIERS_EXCLUS},
        key=str.lower,
    )
    bloc_tools = "\n".join(
        f"- **`TOOLS/{t}/`** : {DESCRIPTIONS_TOOLS.get(t, 'Outil de test matériel')}"
        for t in tools
    ) or "- Aucun outil trouvé"

    arbre = arborescence(chemins, repo)

    contenu = f"""# Le Petit Prince

Robot sous-marin autonome d'environ 4 kg et 80 cm d'envergure, inspiré de la raie *Mobula*.

---

## 1. Présentation

Le robot utilise une propulsion différentielle, des surfaces portantes articulées et un ballast pour se déplacer en 6 degrés de liberté.

- **Bas niveau (ESP32-S3 / FreeRTOS)** : acquisition des capteurs à 200 Hz (cœur 1), filtre de Kalman pour l'attitude, PID à 100 Hz et sécurités matérielles (cœur 0).
- **Haut niveau (Raspberry Pi 5 + AI HAT)** : odométrie visuelle / SLAM, détection d'obstacles et de cibles (OpenCV / YOLOv8). Les consignes passent par une liaison UART binaire à `{baud_uart}` bauds.
- **Actionneurs** : 2 propulseurs brushless à hélice 4 pales (poussée max 27,5 N par moteur), 2 élevons arrière (servos) pour le tangage et le roulis, et un ballast à seringue de 100 mL entraîné par un moteur pas-à-pas.

---

## 2. Matériel et bus I²C

### Composants
- **Calculateurs** : ESP32-S3 Dev Module, Raspberry Pi 5 + caméra grand-angle.
- **Capteurs** : IMU 6 axes `ICM-45686`, magnétomètre `QMC6309`, capteur de pression externe `MS5837-30BA`, capteur interne `BMP280`, 3 moniteurs courant/tension `INA226` (shunts `2 mΩ`).
- **CAO** : fichiers Fusion 360 (`.f3z`) et maillage 3D dans `Hardware/`.

### Adresses I²C (`400 kHz`)

| Composant | Rôle | Adresse | Configuration |
| :--- | :--- | :---: | :--- |
| **MS5837** | Pression externe, profondeur | `{addr_ms}` | Adresse fixe |
| **BMP280** | Température et pression internes | `{addr_bmp}` | `SDO` sur `VCC` (évite le conflit sur `0x76`) |
| **INA226 #1** | Moteur gauche | `{addr_ina_g}` | `A0=GND`, `A1=GND` |
| **INA226 #2** | Moteur droit | `{addr_ina_d}` | `A0=VCC`, `A1=GND` |
| **INA226 #3** | Électronique | `{addr_ina_e}` | `A0=GND`, `A1=VCC` |
| **ICM-45686** | Accéléromètre et gyroscope | `{addr_imu}` | `AD0=GND` |
| **QMC6309** | Magnétomètre (cap) | `{addr_mag}` | Adresse fixe |

---

## 3. Installation

### Prérequis
- **Simulateur et outils PC** : Python 3.10+, `panda3d`, `numpy`, `opencv-python`, `matplotlib`.
- **Firmware** : Arduino IDE 2.x ou PlatformIO avec le support `esp32`, et les bibliothèques `Adafruit_BMP280`, `Adafruit_INA2xx`, `MS5837`.

### Récupération du projet

{BT}bash
git clone https://github.com/{owner}/{repo}.git
cd {repo}/TOOLS/simu_robot
pip install -r requirements.txt
{BT}

---

## 4. Utilisation

### Simulateur 6-DDL (`TOOLS/simu_robot/`)
Le simulateur reproduit la dynamique sous-marine (masse ajoutée, traînée quadratique, portance des ailes, couple des élevons en $v^2$) et fournit une caméra virtuelle en `640×480` pour OpenCV.

{BT}bash
cd TOOLS/simu_robot
python test_physique.py    # réponse 6-DDL en boucle ouverte
python test_3d.py          # rendu Panda3D et flux caméra
python test_controleur.py  # convergence des PID
{BT}

### Firmware ESP32-S3
1. Ouvrir `FIRMWARE/LePetitPrince_ESP32/` dans l'IDE Arduino.
2. Choisir la carte **ESP32S3 Dev Module** avec **USB CDC On Boot: Enabled**.
3. Vérifier le brochage et les seuils dans `config.h` avant de téléverser.

---

## 5. Structure du dépôt

{BT}text
{arbre}
{BT}

### Outils (`TOOLS/`)

{bloc_tools}

"""

    cible = Path("README.md")
    cible.write_text(contenu.strip() + "\n", encoding="utf-8")
    print(f"README généré : {cible.resolve()} ({owner}/{repo}, branche {branche})")


if __name__ == "__main__":
    generer_readme()