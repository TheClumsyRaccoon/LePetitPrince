# Le Petit Prince

Robot sous-marin autonome d'environ 4 kg et 80 cm d'envergure, inspiré de la raie *Mobula*.

---

## 1. Présentation

Le robot utilise une propulsion différentielle, des surfaces portantes articulées et un ballast pour se déplacer en 6 degrés de liberté.

- **Bas niveau (ESP32-S3 / FreeRTOS)** : acquisition des capteurs à 200 Hz (cœur 1), filtre de Kalman pour l'attitude, PID à 100 Hz et sécurités matérielles (cœur 0).
- **Haut niveau (Raspberry Pi 5 + AI HAT)** : odométrie visuelle / SLAM, détection d'obstacles et de cibles (OpenCV / YOLOv8). Les consignes passent par une liaison UART binaire à `921600` bauds.
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
| **MS5837** | Pression externe, profondeur | `0x76` | Adresse fixe |
| **BMP280** | Température et pression internes | `0x77` | `SDO` sur `VCC` (évite le conflit sur `0x76`) |
| **INA226 #1** | Moteur gauche | `0x40` | `A0=GND`, `A1=GND` |
| **INA226 #2** | Moteur droit | `0x41` | `A0=VCC`, `A1=GND` |
| **INA226 #3** | Électronique | `0x42` | `A0=GND`, `A1=VCC` |
| **ICM-45686** | Accéléromètre et gyroscope | `0x68` | `AD0=GND` |
| **QMC6309** | Magnétomètre (cap) | `0x7C` | Adresse fixe |

---

## 3. Installation

### Prérequis
- **Simulateur et outils PC** : Python 3.10+, `panda3d`, `numpy`, `opencv-python`, `matplotlib`.
- **Firmware** : Arduino IDE 2.x ou PlatformIO avec le support `esp32`, et les bibliothèques `Adafruit_BMP280`, `Adafruit_INA2xx`, `MS5837`.

### Récupération du projet

```bash
git clone https://github.com/TheClumsyRaccoon/LePetitPrince.git
cd LePetitPrince/TOOLS/simu_robot
pip install -r requirements.txt
```

---

## 4. Utilisation

### Simulateur 6-DDL (`TOOLS/simu_robot/`)
Le simulateur reproduit la dynamique sous-marine (masse ajoutée, traînée quadratique, portance des ailes, couple des élevons en $v^2$) et fournit une caméra virtuelle en `640×480` pour OpenCV.

```bash
cd TOOLS/simu_robot
python test_physique.py    # réponse 6-DDL en boucle ouverte
python test_3d.py          # rendu Panda3D et flux caméra
python test_controleur.py  # convergence des PID
```

### Firmware ESP32-S3
1. Ouvrir `FIRMWARE/LePetitPrince_ESP32/` dans l'IDE Arduino.
2. Choisir la carte **ESP32S3 Dev Module** avec **USB CDC On Boot: Enabled**.
3. Vérifier le brochage et les seuils dans `config.h` avant de téléverser.

---

## 5. Structure du dépôt

```text
LePetitPrince/
├── FIRMWARE/
│   └── LePetitPrince_ESP32/
│       ├── config.h
│       ├── motors.h
│       ├── sensors.cpp
│       └── sensors.h
├── Hardware/
│   ├── Bouchon de guidage.f3z
│   └── model-84c-manta-ray-mesh.zip
├── TOOLS/
│   ├── scan_i2c/
│   │   └── scan_i2c.ino
│   ├── simu/
│   │   ├── config.py
│   │   ├── controleur.py
│   │   ├── monde_3d.py
│   │   ├── physique.py
│   │   ├── test_3d.py
│   │   ├── test_controleur.py
│   │   └── test_physique.py
│   ├── test_imu/
│   │   └── test_imu.ino
│   ├── test_lecture_bme280/
│   │   └── test_lecture_bme280.ino
│   ├── test_lecture_MS5837/
│   │   └── test_lecture_MS5837.ino
│   └── visu_imu/
│       └── visu_imu.py
└── README.md
```

### Outils (`TOOLS/`)

- **`TOOLS/scan_i2c/`** : Détecte les adresses présentes sur le bus I²C
- **`TOOLS/simu/`** : Outil de test matériel
- **`TOOLS/test_imu/`** : Test de l'IMU 6 axes ICM-45686 et du magnétomètre
- **`TOOLS/test_lecture_bme280/`** : Test du capteur interne (température et pression, BMP280)
- **`TOOLS/test_lecture_MS5837/`** : Test du capteur de pression externe (profondeur)
- **`TOOLS/visu_imu/`** : Visualisation 3D de l'orientation de l'IMU dans le navigateur
