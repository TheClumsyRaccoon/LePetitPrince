#pragma once

/*
 * ============================================================
 *  Lecture des capteurs
 * ------------------------------------------------------------
 *  Regroupe : IMU 9 axes, 3× INA (courant/tension),
 *             BME280 (humidité interne), MS5837 (profondeur).
 *  Le filtre de Kalman pour fusion est intégré au .cpp.
 * ============================================================
 */

#include <stdint.h>


/* ---------- État de l'IMU (après fusion Kalman) ---------- */
typedef struct {
    float roll;      /* rad (filtré) */
    float pitch;     /* rad (filtré) */
    float yaw;         /* rad (filtré) */

    /* Valeurs brutes*/
    float gx, gy, gz;  /* rad/s gyroscope */
    float ax, ay, az;  /* m/s² accéléromètre */
} EtatImu;


/* ---------- État d'un pack de batteries ---------- */
typedef struct {
    float tension;     /* V */
    float courant;     /* A */
    float puissance;   /* W */
} EtatAlim;


/* ---------- État de l'environnement ---------- */
typedef struct {
    float temperature; /* °C air interne du tube */
    float humidite;    /* %HR air interne du tube (fuite) */
    float pression;    /* hPa pression atmosphérique interne */
    float profondeur;  /* m sous l'eau (via pression extérieure) */
} EtatEnv;


/* ---------- API ---------- */

/* Initialise le bus I2C et tous les capteurs.
   À appeler une seule fois dans setup() */
void sensors_init();

/* Lit l'IMU et applique le filtre de Kalman.
   Retourne false si la lecture a échoué */
bool sensors_lire_imu(EtatImu *e);

/* Lit les 3 INA (gauche, droite, électronique).
   Retourne false si au moins un capteur est absent */
bool sensors_lire_alim(EtatAlim *g, EtatAlim *d, EtatAlim *elec);

/* Lit le BME280 et le MS5837 */
bool sensors_lire_env(EtatEnv *e);