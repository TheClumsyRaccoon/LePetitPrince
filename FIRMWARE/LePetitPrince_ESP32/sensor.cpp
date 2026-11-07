#include "sensors.h"
#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

#include <Adafruit_INA2xx.h>
#include <Adafruit_BME280.h>
#include <MS5837.h>


/* ============================================================
 *  NOTE INA226
 * ------------------------------------------------------------
 *  Les 3 packs Li-ion sont mesures par des INA226 (plage 15 A).
 *  La resistance du shunt est definie dans config.h
 *  (SHUNT_INA_OHM), normalement 0.002 ohm, A verifier
 * ============================================================ */


/* ============================================================
 *  FILTRE DE KALMAN (1D, roll et pitch)
 * ------------------------------------------------------------
 *  Voir rapport section I.9.2.
 * ============================================================ */
typedef struct {
    float angle;      /* angle filtre en rad */
    float bias;       /* biais gyro estime en rad/s */
    float P[2][2];    /* matrice de covariance */
    float Q_angle;    /* bruit de processus sur l'angle */
    float Q_bias;     /* bruit de processus sur le biais */
    float R;          /* bruit de mesure (accelerometre) */
} Filtre;

static Filtre filtre_roll;
static Filtre filtre_pitch;

static void filtre_init(Filtre *f, float Qa, float Qb, float R) {
    f->Q_angle = Qa;
    f->Q_bias  = Qb;
    f->R       = R;
    f->angle   = 0.0f;
    f->bias    = 0.0f;
    f->P[0][0] = 0.0f; f->P[0][1] = 0.0f;
    f->P[1][0] = 0.0f; f->P[1][1] = 0.0f;
}

static float filtre_maj(Filtre *f, float angle_mes, float vitesse, float dt) {
    /* --- Prediction (gyroscope) --- */
    float vitesse_corrigee = vitesse - f->bias;
    f->angle += dt * vitesse_corrigee;

    f->P[0][0] += dt * (dt * f->P[1][1] - f->P[0][1] - f->P[1][0] + f->Q_angle);
    f->P[0][1] -= dt * f->P[1][1];
    f->P[1][0] -= dt * f->P[1][1];
    f->P[1][1] += f->Q_bias * dt;

    /* --- Correction (accelerometre) --- */
    float S  = f->P[0][0] + f->R;
    float K0 = f->P[0][0] / S;
    float K1 = f->P[1][0] / S;

    float ecart = angle_mes - f->angle;
    f->angle += K0 * ecart;
    f->bias  += K1 * ecart;

    float P00 = f->P[0][0], P01 = f->P[0][1];
    f->P[0][0] -= K0 * P00;
    f->P[0][1] -= K0 * P01;
    f->P[1][0] -= K1 * P00;
    f->P[1][1] -= K1 * P01;

    return f->angle;
}


/* ============================================================
 *  LECTURE BRUTE DE L'IMU (ICM-45686 + QMC6309)
 * ------------------------------------------------------------
 *  TODO : implémenter la lecture réelle
 * ============================================================ */
static bool imu_lire_brut(float *ax, float *ay, float *az,
                          float *gx, float *gy, float *gz,
                          float *mx, float *my, float *mz) {
    return false;   
}


/* ============================================================
 *  CAPTEURS D'ALIMENTATION (3x INA226)
 * ============================================================ */
static Adafruit_INA2xx ina_gauche;
static Adafruit_INA2xx ina_droite;
static Adafruit_INA2xx ina_elec;


/* ============================================================
 *  CAPTEURS D'ENVIRONNEMENT
 * ============================================================ */
static Adafruit_BME280 bme;
static MS5837          ms5837;


/* ============================================================
 *  INITIALISATION
 * ============================================================ */
void sensors_init() {
    /* --- Bus I2C --- */
    Wire.begin(PIN_SDA, PIN_SCL, FREQ_I2C);
    Serial.println("[SENS] Bus I2C initialise");

    /* --- Filtres de Kalman --- */
    filtre_init(&filtre_roll,  0.001f, 0.003f, 0.03f);
    filtre_init(&filtre_pitch, 0.001f, 0.003f, 0.03f);

    /* --- INA226 x3 --- */
    if (!ina_gauche.begin(ADDR_INA_G, &Wire))
        Serial.println("[SENS] INA226 gauche absent");
    if (!ina_droite.begin(ADDR_INA_D, &Wire))
        Serial.println("[SENS] INA226 droite absent");
    if (!ina_elec.begin(ADDR_INA_ELEC, &Wire))
        Serial.println("[SENS] INA226 electronique absent");

    /* Configuration : shunt + courant max attendu.
       Calibre automatiquement l'echelle interne du capteur. */
    ina_gauche.setShunt(SHUNT_INA_OHM, COURANT_MAX_A);
    ina_droite.setShunt(SHUNT_INA_OHM, COURANT_MAX_A);
    ina_elec.setShunt(SHUNT_INA_OHM, COURANT_MAX_A);

    /* --- BME280 (humidite interne) --- */
    if (!bme.begin(ADDR_BME280)) Serial.println("[SENS] BME280 absent");

    /* --- MS5837 (profondeur exterieure) --- */
    ms5837.setModel(MS5837::MS5837_30BA);
    if (!ms5837.init()) Serial.println("[SENS] MS5837 absent");
}


/* ============================================================
 *  LECTURE IMU + FUSION KALMAN
 * ============================================================ */
static uint32_t t_prec_imu_us = 0;
static float    yaw_integre   = 0.0f;

bool sensors_lire_imu(EtatImu *e) {
    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;

    if (!imu_lire_brut(&ax, &ay, &az, &gx, &gy, &gz, &mx, &my, &mz)) {
        return false;
    }

    /* --- dt entre deux lectures --- */
    uint32_t maintenant = micros();
    float dt = (t_prec_imu_us == 0)
             ? 0.005f
             : (maintenant - t_prec_imu_us) * 1e-6f;
    t_prec_imu_us = maintenant;
    if (dt > 0.05f) dt = 0.05f;    /* securite anti-blocage */

    /* --- Angles bruts accelerometre --- */
    float roll_acc  = atan2f(ay, az);
    float pitch_acc = atan2f(-ax, sqrtf(ay*ay + az*az));

    /* --- Gyro en rad/s (drivers souvent en degres/s) --- */
    const float D2R = 0.0174532925f;
    float gx_r = gx * D2R;
    float gy_r = gy * D2R;
    float gz_r = gz * D2R;

    /* --- Fusion Kalman : roll et pitch --- */
    e->roll  = filtre_maj(&filtre_roll,  roll_acc,  gx_r, dt);
    e->pitch = filtre_maj(&filtre_pitch, pitch_acc, gy_r, dt);

    /* --- Yaw : integration gyro + correction magnetometre --- */
    float yaw_mag = atan2f(-my, mx);
    yaw_integre += gz_r * dt;

    float ecart = yaw_mag - yaw_integre;
    while (ecart >  M_PI) ecart -= 2.0f * M_PI;
    while (ecart < -M_PI) ecart += 2.0f * M_PI;

    yaw_integre += 0.02f * ecart;   /* correction douce */
    e->yaw = yaw_integre;

    /* --- Valeurs brutes pour debug --- */
    e->gx = gx_r; e->gy = gy_r; e->gz = gz_r;
    e->ax = ax;   e->ay = ay;   e->az = az;

    return true;
}


/* ============================================================
 *  LECTURE ALIMENTATION
 * ============================================================ */
static void lire_un_ina(Adafruit_INA2xx &ina, EtatAlim *a) {
    a->tension   = ina.getBusVoltage_V();
    a->courant   = ina.getCurrent_mA() / 1000.0f;   /* mA vers A */
    a->puissance = a->tension * a->courant;
}

bool sensors_lire_alim(EtatAlim *g, EtatAlim *d, EtatAlim *elec) {
    lire_un_ina(ina_gauche, g);
    lire_un_ina(ina_droite, d);
    lire_un_ina(ina_elec,   elec);
    return true;
}


/* ============================================================
 *  LECTURE ENVIRONNEMENT
 * ============================================================ */
bool sensors_lire_env(EtatEnv *e) {
    e->temperature = bme.readTemperature();
    e->humidite    = bme.readHumidity();
    e->pression    = bme.readPressure() / 100.0f;   /* Pa vers hPa */

    ms5837.read();
    e->profondeur = ms5837.depth();

    return true;    
}