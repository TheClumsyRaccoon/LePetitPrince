#pragma once

/*
 * ============================================================
 *  Commande des moteurs brushless
 * ------------------------------------------------------------
 *  Deux ESC pilotes en PWM 50 Hz (1000-2000 us).
 *  Deux regulateurs PID : un pour la vitesse d'avance,
 *  un pour le cap (poussee differentielle gauche/droite).
 *  (voir rapport section II.4).
 * ============================================================
 */


/* ---------- PID ---------- */
typedef struct {
    float kp, ki, kd;       /* gains */
    float integrale;        /* cumul de l'erreur (terme I) */
    float erreur_prec;      /* erreur a l'iteration precedente (terme D) */
    float sortie_min;       /* borne basse de la sortie */
    float sortie_max;       /* borne haute de la sortie */
} Pid;


/* ---------- API ---------- */

/* Initialise les sorties PWM des deux ESC et les regulateurs PID.
   A appeler une seule fois dans setup(). */
void motors_init();

/* Applique directement une poussee sur chaque moteur.
   poussee_gauche et poussee_droite entre [-1 ; +1].
   -1 = arriere max, 0 = arret, +1 = avant max. */
void motors_set(float poussee_gauche, float poussee_droite);

/* Coupe les deux moteurs (poussee nulle). */
void motors_stop();

/* Boucle de controle complete :
   - vitesse_cible / cap_cible : consignes issues du raspberryy Pi
   - vitesse_mes / cap_mes     : mesures issues des capteurs
   - dt                        : periode de la boucle, en secondes */
void motors_update(float vitesse_cible, float cap_cible,
                   float vitesse_mes,   float cap_mes, float dt);