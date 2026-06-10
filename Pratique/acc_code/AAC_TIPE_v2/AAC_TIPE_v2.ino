/*
 * ============================================================
 *  RÉGULATEUR DE VITESSE ADAPTATIF (AAC) — TIPE CPGE PT
 *  Version 2 — sélection du correcteur par #define
 * ============================================================
 *
 *  ┌─ MATÉRIEL ──────────────────────────────────────────────┐
 *  │  Arduino Nano Every                                      │
 *  │  Shield Arduino ASX00061 (microSD via SPI)               │
 *  │  Pont en H L298N — channel B → moteur brushed RC390      │
 *  │  Capteur ultrason HC-SR04                                │
 *  │  Capteur effet Hall + 6 aimants (roue arrière gauche)    │
 *  └──────────────────────────────────────────────────────────┘
 *
 *  ┌─ SÉLECTION DU CORRECTEUR ───────────────────────────────┐
 *  │  Décommentez UNE SEULE ligne parmi les quatre :          │
 *  └──────────────────────────────────────────────────────────┘
 */

#define CORRECTEUR P
// #define CORRECTEUR PI
// #define CORRECTEUR PD
// #define CORRECTEUR PID

/*
 *  ┌─ RÉGLAGE DES GAINS ─────────────────────────────────────┐
 *  │  Kp : gain proportionnel (toujours actif)                │
 *  │  Ki : gain intégral     (actif si PI ou PID)             │
 *  │  Kd : gain dérivé       (actif si PD ou PID)             │
 *  │                                                          │
 *  │  Ces valeurs sont à ajuster expérimentalement.           │
 *  │  Point de départ conseillé :                             │
 *  │    P   → Kp = 50                                         │
 *  │    PI  → Kp = 40,  Ki = 8                                │
 *  │    PD  → Kp = 40,  Kd = 5                                │
 *  │    PID → Kp = 35,  Ki = 6,  Kd = 4                       │
 *  └──────────────────────────────────────────────────────────┘
 */
#define Kp_VAL  50.0f
#define Ki_VAL   8.0f
#define Kd_VAL   5.0f

/* ─── Broches ────────────────────────────────────────────── */
#define TRIG_PIN    7       // HC-SR04 : déclenchement
#define ECHO_PIN    6       // HC-SR04 : réception écho
#define HALL_PIN    2       // Capteur Hall (INT0 — interruption externe)
#define ENB_PIN     9       // L298N ENB  — PWM vitesse (DOIT être une broche PWM)
#define IN3_PIN     4       // L298N IN3  — sens rotation A
#define IN4_PIN     5       // L298N IN4  — sens rotation B
#define SD_CS_PIN   10      // Chip Select de la microSD (vérifier selon shield)

/* ─── Paramètres physiques ───────────────────────────────── */
// Roue : diamètre 83 mm → périmètre = π × 0.083 m
const float PERIMETRE_M   = 3.14159f * 0.083f;  // ≈ 0.2608 m
const uint8_t NB_AIMANTS  = 6;                   // aimants par tour de roue

/* ─── Consignes ──────────────────────────────────────────── */
const float DIST_CONSIGNE_M    = 0.30f;  // distance de sécurité cible : 30 cm
const float VITESSE_CIBLE_MS   = 0.40f;  // vitesse de croisière : 0,40 m/s

/* ─── Seuils de sécurité ─────────────────────────────────── */
const float DIST_URGENCE_M     = 0.10f;  // freinage d'urgence si < 10 cm
const float DIST_MAX_OBSTACLE_M= 2.00f;  // au-delà : pas d'obstacle détecté

/* ─── Anti-windup : saturation de l'intégrale ───────────── */
const float INTEGRALE_MAX = 100.0f;

/* ─── Périodes de la boucle ──────────────────────────────── */
const uint16_t CONTROL_PERIOD_MS = 50;   // régulation : toutes les 50 ms
const uint16_t LOG_PERIOD_MS     = 100;  // écriture CSV : toutes les 100 ms

/* ─── Saturation de la commande PWM ─────────────────────── */
// Le L298N accepte 0-255 sur ENB
// On borne la commande entre -255 (freinage max) et +255 (accél. max)

/* ===========================================================
 *   MACROS INTERNES — ne pas modifier
 * =========================================================== */
#define _MODE_STR_(x) #x
#define MODE_STR _MODE_STR_(CORRECTEUR)

// Drapeaux booléens pour activer/désactiver les termes I et D
#if defined(CORRECTEUR)
  #if   CORRECTEUR == PI
    #define USE_I 1
    #define USE_D 0
  #elif CORRECTEUR == PD
    #define USE_I 0
    #define USE_D 1
  #elif CORRECTEUR == PID
    #define USE_I 1
    #define USE_D 1
  #else   // P par défaut
    #define USE_I 0
    #define USE_D 0
  #endif
#endif

#include <SPI.h>
#include <SD.h>

/* ─── Variables globales ─────────────────────────────────── */
// Compteur d'impulsions Hall — modifié dans l'ISR, lu dans la boucle
volatile uint32_t hallPulseCount = 0;

// Valeur du compteur au dernier calcul de vitesse
uint32_t lastHallCount = 0;

// Vitesse mesurée (m/s)
float vitesse_ms = 0.0f;

// Variables du correcteur
float erreurPrecedente = 0.0f;   // erreur à l'instant t-1 (pour le terme D)
float integrale        = 0.0f;   // accumulateur de l'intégrale (terme I)

// Horodatages
uint32_t lastControlTime = 0;
uint32_t lastLogTime     = 0;

// Dernière commande appliquée (pour le log)
float derniereCommande = 0.0f;

// Objet fichier SD
File logFile;
bool sdOk = false;


/* ===========================================================
 *   ISR — INTERRUPTION HALL
 *
 *   Déclenchée à chaque front DESCENDANT sur D2.
 *   Un aimant passe devant le capteur → le champ change →
 *   la sortie du capteur passe LOW → front descendant.
 *   On incrémente simplement le compteur.
 *
 *   IMPORTANT : cette fonction est exécutée en dehors de la
 *   boucle principale, à n'importe quel moment. C'est pourquoi
 *   hallPulseCount est déclaré volatile : le compilateur ne
 *   doit pas le mettre en cache dans un registre.
 * =========================================================== */
void hallISR() {
  hallPulseCount++;
}


/* ===========================================================
 *   lireDistance()
 *   Retourne la distance mesurée par le HC-SR04, en mètres.
 *   Retourne -1.0 si hors plage ou pas d'écho.
 *
 *   Principe :
 *   1) On envoie une impulsion TRIG de 10 µs.
 *   2) Le HC-SR04 émet 8 salves ultrasoniques à 40 kHz.
 *   3) Quand l'écho revient, ECHO passe HIGH pendant une
 *      durée proportionnelle à la distance : d = v_son × t/2
 *   4) pulseIn() mesure cette durée en microsecondes.
 *   5) Conversion : d(m) = durée(µs) × 343e-6 / 2
 *                        = durée × 0.0001715
 * =========================================================== */
float lireDistance() {
  // Générer l'impulsion TRIG
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);          // s'assurer que TRIG est bien LOW avant
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);         // durée minimale de l'impulsion : 10 µs
  digitalWrite(TRIG_PIN, LOW);

  // Mesurer la durée de l'écho
  // Timeout = 30 000 µs → distance max ≈ 5,1 m (amplement suffisant)
  long duree_us = pulseIn(ECHO_PIN, HIGH, 30000UL);

  // Si pas d'écho reçu dans le délai : hors plage
  if (duree_us == 0) return -1.0f;

  // Conversion µs → mètres
  float dist_m = (float)duree_us * 0.0001715f;

  // Le HC-SR04 est fiable entre ~2 cm et ~4 m
  if (dist_m < 0.02f || dist_m > 4.0f) return -1.0f;

  return dist_m;
}


/* ===========================================================
 *   calculerVitesse()
 *   Calcule la vitesse en m/s depuis le compteur Hall.
 *   Appelée toutes les CONTROL_PERIOD_MS millisecondes.
 *
 *   Principe :
 *   - On lit le compteur d'impulsions Hall (en désactivant
 *     brièvement les interruptions pour éviter une lecture
 *     partielle pendant une incrémentation).
 *   - delta = nombre d'impulsions depuis le dernier appel.
 *   - Nombre de tours = delta / NB_AIMANTS
 *   - Distance parcourue = tours × périmètre_roue
 *   - Vitesse = distance / temps_écoulé
 *
 *   Paramètre dt_ms : durée depuis le dernier appel (ms)
 * =========================================================== */
float calculerVitesse(uint32_t dt_ms) {
  // Lecture atomique du compteur (interruptions suspendues le temps de copier)
  noInterrupts();
  uint32_t countActuel = hallPulseCount;
  interrupts();

  // Nombre d'impulsions depuis le dernier appel
  uint32_t delta = countActuel - lastHallCount;
  lastHallCount  = countActuel;   // mise à jour du repère

  // Sécurité : si dt = 0 on évite la division par zéro
  if (dt_ms == 0) return 0.0f;

  // Nombre de tours de roue correspondant aux delta impulsions
  float tours = (float)delta / (float)NB_AIMANTS;

  // Distance parcourue = tours × périmètre (m)
  float distance_m = tours * PERIMETRE_M;

  // Vitesse = distance / temps (conversion ms → s)
  float vitesse = distance_m / ((float)dt_ms / 1000.0f);

  return vitesse;
}


/* ===========================================================
 *   appliquerCommande()
 *   Traduit la commande flottante [-255 ; +255] en signaux
 *   pour le pont en H L298N.
 *
 *   cmd > 0  → avancer  : IN3=HIGH, IN4=LOW,  ENB=PWM
 *   cmd < 0  → freinage : IN3=LOW,  IN4=HIGH, ENB=PWM
 *              (le moteur est court-circuité → freinage actif)
 *   cmd = 0  → roue libre (ou freinage selon configuration)
 *
 *   Pourquoi le freinage actif fonctionne-t-il ?
 *   En court-circuitant les bornes du moteur (IN3=IN4),
 *   la FCEM (force contre-électromotrice) du moteur en
 *   rotation génère un courant de freinage qui s'oppose
 *   au mouvement — c'est le freinage rhéostatique.
 * =========================================================== */
void appliquerCommande(float cmd) {
  // Saturation entre -255 et +255
  int pwm = (int)constrain(cmd, -255.0f, 255.0f);

  if (pwm >= 0) {
    // Avancer
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, (uint8_t)pwm);
  } else {
    // Freinage actif (court-circuit bobines du moteur)
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, (uint8_t)(-pwm));
  }
}


/* ===========================================================
 *   calculerCorrecteur()
 *   Cœur du régulateur. Calcule la commande u(t) selon le
 *   mode sélectionné par #define CORRECTEUR.
 *
 *   Entrées :
 *     erreur  : e(t) = consigne − mesure  (en mètres)
 *     dt_s    : pas de temps              (en secondes)
 *
 *   Sortie :
 *     commande u(t) — valeur réelle, bornée ensuite par
 *     appliquerCommande() entre -255 et +255.
 *
 *   ┌─ RAPPEL MATHÉMATIQUE ──────────────────────────────────┐
 *   │                                                         │
 *   │  Correcteur P  : u(t) = Kp × e(t)                       │
 *   │                                                         │
 *   │  Correcteur PI : u(t) = Kp × e(t)                       │
 *   │                       + Ki × ∫e(τ)dτ                    │
 *   │                                                         │
 *   │  Correcteur PD : u(t) = Kp × e(t)                       │
 *   │                       + Kd × de(t)/dt                   │
 *   │                                                         │
 *   │  Correcteur PID: u(t) = Kp × e(t)                       │
 *   │                       + Ki × ∫e(τ)dτ                    │
 *   │                       + Kd × de(t)/dt                   │
 *   │                                                         │
 *   │  Discrétisation (Euler) :                               │
 *   │    Intégrale ≈ Σ e(k) × dt    (rectangle à gauche)      │
 *   │    Dérivée   ≈ (e(k) − e(k-1)) / dt                    │
 *   └─────────────────────────────────────────────────────────┘
 * =========================================================== */
float calculerCorrecteur(float erreur, float dt_s) {

  // ── Terme Proportionnel ──────────────────────────────────
  // u_P = Kp × e(t)
  // Si erreur > 0 : on est trop loin → on accélère
  // Si erreur < 0 : on est trop près → on freine
  float u_P = Kp_VAL * erreur;

  // ── Terme Intégral ────────────────────────────────────────
  // Approximation rectangulaire d'Euler :
  //   integrale += e(t) × dt
  // Représente l'aire sous la courbe de l'erreur.
  // Permet de corriger une erreur statique persistante.
  float u_I = 0.0f;
#if USE_I
  integrale += erreur * dt_s;

  // Anti-windup : on sature l'accumulateur pour éviter que
  // l'intégrale n'augmente indéfiniment quand la commande
  // est saturée (ex : obstacle lointain, moteur à fond).
  // Sans cela, une grande intégrale accumulée provoquerait
  // un dépassement important quand l'obstacle arrive.
  integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);

  u_I = Ki_VAL * integrale;
#endif

  // ── Terme Dérivé ──────────────────────────────────────────
  // Approximation différences finies en arrière :
  //   de/dt ≈ (e(k) - e(k-1)) / dt
  // Représente la vitesse de variation de l'erreur.
  // Anticipe la dynamique : si l'erreur diminue vite, on
  // réduit la commande avant d'atteindre la consigne
  // → réduit le dépassement.
  // Attention : amplifie le bruit de mesure. Avec seulement
  // 6 aimants, le signal de vitesse est quantifié en paliers
  // → le terme D est sensible à ce bruit.
  float u_D = 0.0f;
#if USE_D
  float derivee = 0.0f;
  if (dt_s > 0.0f) {
    derivee = (erreur - erreurPrecedente) / dt_s;
  }
  u_D = Kd_VAL * derivee;
#endif

  // Mémorisation de l'erreur courante pour le prochain appel
  erreurPrecedente = erreur;

  // ── Commande totale ───────────────────────────────────────
  return u_P + u_I + u_D;
}


/* ===========================================================
 *   initSD()
 *   Initialise la carte microSD et crée le fichier de log.
 *   Le nom du fichier inclut le type de correcteur :
 *     log_P_000.csv, log_PI_001.csv, etc.
 *   On incrémente le suffixe jusqu'à trouver un nom libre.
 * =========================================================== */
void initSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD : échec initialisation"));
    sdOk = false;
    return;
  }

  // Construction du nom de fichier : log_<MODE>_NNN.csv
  char filename[];
  uint8_t n = 0;
  do {
    // snprintf écrit dans filename le nom formaté
    snprintf(filename, sizeof(filename), "log_%s_Kp-%3.1f_Ki-%3.1f_Kd-%3.1f_%03d.csv", MODE_STR, Kp_VAL, Ki_VAL, Kd_VAL, n);
    n++;
  } while (SD.exists(filename) && n < 255);

  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.println(F("SD : impossible d'ouvrir le fichier"));
    sdOk = false;
    return;
  }

  // ── En-tête CSV ───────────────────────────────────────────
  // Format pensé pour pandas / matplotlib :
  //   - noms de colonnes sans espace ni accent
  //   - séparateur virgule
  //   - unités dans le nom de colonne entre crochets
  //   - temps en secondes (float) plutôt qu'en ms
  //     pour faciliter les tracés temporels
  logFile.println(F("t_s,correcteur,Kp,Ki,Kd,vitesse_ms,consigne_vitesse_ms,"
                    "distance_m,consigne_dist_m,"
                    "erreur_m,commande_pwm"));
  logFile.flush();

  sdOk = true;
  Serial.print(F("SD : fichier ouvert → "));
  Serial.println(filename);
}


/* ===========================================================
 *   ecrireLog()
 *   Écrit une ligne de données dans le fichier CSV.
 *   Appelée toutes les LOG_PERIOD_MS millisecondes.
 * =========================================================== */
void ecrireLog(float t_s, float dist_m, float erreur_m) {
  if (!sdOk) return;

  // t_s : temps en secondes (float, 3 décimales)
  logFile.print(t_s,          3);  logFile.print(',');
  logFile.print(MODE_STR);         logFile.print(',');
  logFile.print(Kp_VAL);         logFile.print(',');
  logFile.print(Ki_VAL);         logFile.print(',');
  logFile.print(Kd_VAL);         logFile.print(',');
  logFile.print(vitesse_ms,   4);  logFile.print(',');
  logFile.print(VITESSE_CIBLE_MS, 4); logFile.print(',');
  logFile.print(dist_m,       4);  logFile.print(',');
  logFile.print(DIST_CONSIGNE_M, 4); logFile.print(',');
  logFile.print(erreur_m,     4);  logFile.print(',');
  logFile.println(derniereCommande, 1);

  // flush() force l'écriture physique sur la SD.
  // C'est coûteux (~5 ms) mais évite la perte de données
  // en cas d'arrêt brutal (débranchement batterie).
  logFile.flush();
}


/* ===========================================================
 *   SETUP
 * =========================================================== */
void setup() {
  Serial.begin(115200);
  Serial.print(F("=== AAC TIPE — correcteur : "));
  Serial.print(MODE_STR);
  Serial.println(F(" ==="));

  // ── Ultrason ─────────────────────────────────────────────
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  // ── Hall ──────────────────────────────────────────────────
  // INPUT_PULLUP : la résistance de tirage interne évite les
  // lectures parasites si le capteur est en collecteur ouvert.
  // FALLING : on détecte le front descendant (aimant passant
  // devant → sortie passe HIGH→LOW).
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  // ── Pont en H ─────────────────────────────────────────────
  pinMode(ENB_PIN,  OUTPUT);
  pinMode(IN3_PIN,  OUTPUT);
  pinMode(IN4_PIN,  OUTPUT);
  // Moteur arrêté au démarrage
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  // ── SD ────────────────────────────────────────────────────
  initSD();

  // ── Initialisation des horodatages ────────────────────────
  lastControlTime = millis();
  lastLogTime     = millis();
}


/* ===========================================================
 *   LOOP
 *   Structure : on vérifie l'écoulement du temps et on
 *   exécute les tâches à leur propre période.
 *   On n'utilise pas delay() pour ne pas bloquer.
 * =========================================================== */
void loop() {
  uint32_t now = millis();

  // ── Tâche de régulation (toutes les 50 ms) ───────────────
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {

    // Calcul du pas de temps réel (peut légèrement dériver)
    uint32_t dt_ms = now - lastControlTime;
    lastControlTime = now;
    float dt_s = (float)dt_ms / 1000.0f;

    // ── 1. Mesures ─────────────────────────────────────────

    // Vitesse roue (m/s) depuis le compteur Hall
    vitesse_ms = calculerVitesse(dt_ms);

    // Distance obstacle (m) depuis le HC-SR04
    float dist_m = lireDistance();

    // ── 2. Logique de régulation ───────────────────────────

    float erreur       = 0.0f;
    float commande     = 0.0f;
    bool  urgence      = false;
    bool  sansObstacle = false;

    if (dist_m > 0.0f && dist_m < DIST_URGENCE_M) {
      // ── CAS A : obstacle très proche → freinage d'urgence ─
      // On vide l'intégrale pour éviter un rebond après freinage
      integrale        = 0.0f;
      erreurPrecedente = 0.0f;
      commande         = -255.0f;    // freinage maximum
      erreur           = dist_m - DIST_CONSIGNE_M;
      urgence          = true;

    } else if (dist_m < 0.0f || dist_m >= DIST_MAX_OBSTACLE_M) {
      // ── CAS B : pas d'obstacle détecté ────────────────────
      // On régule sur la VITESSE (pas sur la distance).
      // erreur_v = vitesse_cible - vitesse_mesurée
      // Si on va trop lentement → erreur > 0 → commande > 0 → on accélère
      // Si on va trop vite     → erreur < 0 → commande < 0 → on freine
      sansObstacle = true;
      erreur   = VITESSE_CIBLE_MS - vitesse_ms;
      commande = calculerCorrecteur(erreur, dt_s);

    } else {
      // ── CAS C : obstacle dans la zone de régulation ────────
      // On régule sur la DISTANCE.
      // erreur_d = distance_mesurée - distance_consigne
      // Si l'obstacle est loin  → erreur > 0 → on accélère
      // Si l'obstacle est proche→ erreur < 0 → on freine
      erreur   = dist_m - DIST_CONSIGNE_M;
      commande = calculerCorrecteur(erreur, dt_s);
    }

    // Mémoriser la commande pour le log
    derniereCommande = commande;

    // Appliquer la commande au moteur
    appliquerCommande(commande);

    // ── 3. Log CSV (toutes les 100 ms) ─────────────────────
    if (now - lastLogTime >= LOG_PERIOD_MS) {
      lastLogTime = now;
      float t_s   = (float)now / 1000.0f;
      // En mode sans obstacle, on logue la distance comme -1
      // pour qu'elle soit facilement filtrée en Python
      float dist_log = sansObstacle ? -1.0f : dist_m;
      ecrireLog(t_s, dist_log, erreur);
    }

    // ── 4. Debug port série ─────────────────────────────────
    Serial.print(F("["));  Serial.print(MODE_STR);  Serial.print(F("] "));
    Serial.print(F("v=")); Serial.print(vitesse_ms, 3); Serial.print(F(" m/s"));
    if (!sansObstacle) {
      Serial.print(F(" | d=")); Serial.print(dist_m, 3); Serial.print(F(" m"));
    } else {
      Serial.print(F(" | (pas d'obstacle)"));
    }
    if (urgence) Serial.print(F(" !! URGENCE !!"));
    Serial.print(F(" | cmd=")); Serial.println(commande, 1);
  }
}
