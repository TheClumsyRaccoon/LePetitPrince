"""NB convention des tuples : (X avant, Y droite, Z haut)"""

# Parametres physiques et geometrie du robot
MASSE_KG = 4.0
MASSE_AJOUTEE_KG = (0.8, 1.5, 6.0)  # X avant, Y droite, Z haut, TODO à confirmer par Guillaume (Fusion)
INERTIE_KG_M2 = (0.12, 0.18, 0.28)  # Ix, Iy, Iz, approximation TODO les simuler sous Fusion
BRAS_LEVIER_MOTEUR_M = 0.2         # Demi écartement entre propulseurs, TODO à confirmer par Guillaume (Fusion)

# Actionneurs (propulseurs, elevons, ballast)
POUSSEE_MAX_AVANT_N = 27.5          # CF datasheet moteur entre 18-20 V, TODO mesurer en bassin 
POUSSEE_MAX_ARRIERE_N = 19.0        # Asymetrie helice marche arriere (70% de la poussée max avant), TODO mesurer en bassin
TAU_MOTEUR_S = 0.15                 # Constante de temps moteur + helice, TODO mesurer en bassin

BALLAST_CAPACITE_ML = 100.0
BALLAST_NEUTRE_ML = 50.0            # Equilibre hydrostatique, TODO a vérifier en bassin 
BALLAST_DEBIT_MAX_MLS = 12.0        # TODO a vérifier 

ELEVON_DEBATTEMENT_MAX_RAD = 0.45   # 26 deg avant decrochage, cf literature

# Hydrodynamique
TRAINEE_LINEAIRE = (4.2, 14.0, 22.0)    #approximation, TODO a sim sous OPENFOAM
TRAINEE_ROTATION = (0.6, 0.7, 0.8)      #approximation, TODO a sim sous OPENFOAM
COEF_PORTANCE_AILES = 8.5               #approximation, TODO a récup sur Fusion
EFFICACITE_ELEVON = 2.4             # Couple proportionnel a v^2
COUPLE_RAPPEL_PASSIF = 1.6          # (N.m/rad), TODO a mesurer sur Fusion

# Bruits capteurs simulé (ecarts-types)
BRUIT_ROLL_PITCH_RAD = 0.008        # ICM-45686 + Kalman
BRUIT_YAW_RAD = 0.015               # QMC6309
BRUIT_PROFONDEUR_M = 0.01           # MS5837
BRUIT_VITESSE_MS = 0.02             #SLAM
BRUIT_GYRO_RADS = 0.01              # ICM-45686

# Regulateurs PID (Kp, Ki, Kd, sortie_min, sortie_max)
PID_VITESSE = (0.35, 0.08, 0.01, -0.80, 0.80)
PID_CAP     = (0.45, 0.02, 0.18, -0.35, 0.35)
PID_PROF    = (0.80, 0.10, 0.45, -1.00, 1.00)
PID_PITCH   = (0.90, 0.05, 0.25, -0.45, 0.45)
PID_ROLL    = (1.20, 0.02, 0.25, -0.35, 0.35)

# Vision et rechcerche
CAM_LARGEUR = 1920
CAM_HAUTEUR = 1080
CAM_FOV_DEG = 102

HSV_CIBLE_MIN = (5, 130, 100)
HSV_CIBLE_MAX = (22, 255, 255)
AIRE_CIBLE_MIN_PX = 150
AIRE_CIBLE_PROCHE_PX = 15000
AIRE_CIBLE_APPROCHE_PX = 7000

# Parametres guidage visuel (YOLOv8)
VISION_CONF_SEUIL       = 0.25
VISION_TAILLE_ENTREE    = 224
VISION_INTERVALLE_YOLO  = 4
VISION_RATIO_AIRE_PROCHE = 0.06  # ~6% de l'image (~18 000 px en 640x480)

# Simulation et bassin
DT_PHYSIQUE_S = 0.005               # Pas de calcul (200 Hz)
PROFONDEUR_BASSIN_M = 6.0
COURANT_MARIN_TEST = (0.0, 0.22, 0.0)