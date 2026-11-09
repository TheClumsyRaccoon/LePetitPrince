#pragma once

/*
 * ============================================================
 *  Ce fichier regroupe TOUTES les constantes du projet :
 *  broches, adresses I2C, fréquences, gains PID, seuils.
 *  Rien n'ets encore fixé, le choix des GPIO peut changer etc.
 * ============================================================
 */

/* ============================================================
 *  1. BUS I2C
 * ============================================================ */
#define PIN_SDA         8       /* Broche SDA */
#define PIN_SCL         9       /* Broche SCL  */
#define FREQ_I2C        400000   /* 400 kHz, mode fast I2C */


/* ============================================================
 *  2. LIAISON UART avec la Raspberry Pi 5 (via opto)
 * ============================================================ */
#define PIN_UART_TX     17       /* ESP32 TX */
#define PIN_UART_RX     16       /* ESP32 RX */
#define VITESSE_UART    921600   /*  baud rate élevé pour avoir de la marge par rapport à la taille d enotre trame*/


/* ============================================================
 *  3. MOTEURS BRUSHLESS
 *  Signal PWM 50 Hz, impulsion 1000-2000 µs
 * Standard des ESC, a modifier si besoin
 * ============================================================ */
#define PIN_MOTEUR_G    4       /* ESC moteur gauche */
#define PIN_MOTEUR_D    5       /* ESC moteur droit */
#define FREQ_ESC        50       /* Fréquence PWM */
#define RES_ESC         16       /* Résolution ESC */
#define ESC_MIN_US      1000     /* Impulsion poussée arrière max (µs) */
#define ESC_MAX_US      2000     /* Impulsion poussée avant max (µs) */

/* ============================================================
 *  4. SERVOS
 * ============================================================ */

#define PIN_SERVO_L     6
#define PIN_SERVO_R     7

/* ============================================================
 *  5. STEPPER DU BALLAST 
 * ============================================================ */

#define PIN_STEP        10
#define PIN_DIR         11
#define PIN_EN          12

/* ============================================================
 *  6. ADRESSES I2C DES CAPTEURS
 * ============================================================ */

/* --- IMU 9 axes : ICM-45686 (accel+gyro) + QMC6309 (magnéto) --- */
#define ADDR_ICM45686   0x68     /* Adresse par défaut (AD0 = GND) */
#define ADDR_QMC6309    0x7C     /* Magnétomètre QMC6309 */

/* --- 3× INA226 : mesure tension/courant des packs Li-ion --- */
#define ADDR_INA_G      0x40     /* Pack moteur gauche (5S2P) */
#define ADDR_INA_D      0x41     /* Pack moteur droit (5S2P) */
#define ADDR_INA_ELEC   0x42     /* Pack électronique (2S2P) */
#define SHUNT_INA_OHM   0.002f   /* résistance shunt INA226*/

/* --- BME280 : humidité interne du tube (détection de fuite) --- */
#define ADDR_BME280     0x77     /* Jumper SDO → VCC (sinon 0x76) */

/* --- MS5837-30BA : pression extérieure → profondeur --- */
#define ADDR_MS5837     0x76     /* Adresse fixe du capteur bleu */


/* ============================================================
 *  7. FRÉQUENCES DES TÂCHES FREERTOS
 * ------------------------------------------------------------
 *  Chaque tâche tourne à sa propre cadence
 * (cf Critère de Shannon), valeurs à modifier si besoin
 * ============================================================ */
#define FREQ_CAPTEURS   200      /* Hz — IMU (fusion Kalman) */
#define FREQ_CONTROLE   100      /* Hz — PID moteurs */
#define FREQ_COMMS      20       /* Hz — UART vers Raspberry Pi */


/* ============================================================
 *  8. GAINS PID
 * ------------------------------------------------------------
 *  KP : proportionnel 
 *  KI : intégral 
 *  KD : dérivé 
 *
 *  Valeurs à définir
 * ============================================================ */

/* --- Boucle vitesse d'avance (poussée commune) --- */
#define KP_VITESSE      0.6f
#define KI_VITESSE      0.15f
#define KD_VITESSE      0.02f

/* --- Boucle cap (différence de poussée G/D) --- */
#define KP_CAP          0.8f
#define KI_CAP          0.10f
#define KD_CAP          0.05f


/* ============================================================
 *  9. SEUILS DE SÉCURITÉ
 * ============================================================ */
#define HUMIDITE_MAX    70.0f    /* %HR */
#define COURANT_MAX_A   15.0f    /* A */


/* ============================================================
 *  10. VERSION
 * ============================================================ */
#define VERSION_FW      "0.1.0"
#define NOM_PROJET      "Le Petit Prince"