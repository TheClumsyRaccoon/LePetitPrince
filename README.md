# Le Petit Prince, Robot sous-marin biomimétique

Projet robotique 2025-2026, ARRIGONI Benoît & BERNARD Guillaume.

## Objectif
Démonstrateur de robot sous-marin autonome inspiré de la raie *Mobula mobular*,
capable d'observer la faune marine avec le moins de perturbations possibles.

## Architecture
- **ESP32-S3** : acquisition capteurs + commande bas niveau (moteurs, servos, ballast)
- **Raspberry Pi 5** : SLAM visuel-inertiel, YOLO, navigation (à venir)

## Modules
- `config.h`       pinout et constantes
- `sensors`        IMU 9 axes (ICM-45686 + QMC6309), 3× INA219, BME280, MS5837, Kalman
- `motors`         2 ESC brushless + PID (vitesse et cap)

## À venir
- `actuators`  servos élevons + ballast seringue
- `comms`      UART binaire avec la Raspberry Pi
