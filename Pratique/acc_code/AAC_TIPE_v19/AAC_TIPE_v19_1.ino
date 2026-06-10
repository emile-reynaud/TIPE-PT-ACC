/*
 * ============================================================
 * RÉGULATEUR DE VITESSE ADAPTATIF (AAC) — TIPE CPGE PT
 * Version 19 — Architecture ACC réaliste
 * ============================================================
 *
 * ┌─ CHANGEMENTS DEPUIS v18 ────────────────────────────────┐
 * │                                                          │
 * │  1. MODE DISTANCE : architecture ACC correcte            │
 * │     Ancien : PID sur erreur de distance (erreur=d-Dc)   │
 * │     Nouveau : régulation de VITESSE maintenue + terme    │
 * │       de freinage si l'obstacle est trop proche :        │
 * │       cmd = cmd_vitesse − Kp_BRAKE × max(0, Dc − d)     │
 * │     → la voiture ne s'arrête JAMAIS d'accélérer pour    │
 * │       atteindre la consigne de distance                  │
 * │     → elle freine SEULEMENT si d < Dc                   │
 * │                                                          │
 * │  2. PAS DE RESET INTÉGRALE lors de la transition 2→3    │
 * │     L'intégrale continue de tourner sans interruption    │
 * │     → transition douce, sans saut de commande            │
 * │                                                          │
 * │  3. TERME DÉRIVÉ SUR LA MESURE (anti-derivative-kick)   │
 * │     Ancien : D = Kd × d(erreur)/dt = Kd × Δε/Δt        │
 * │     Nouveau : D = −Kd × d(v_mesurée)/dt = −Kd × Δv/Δt  │
 * │     Équivalent mathématiquement en régime, mais évite   │
 * │     le pic brutal quand la consigne change (kick fin)    │
 * │                                                          │
 * │  4. ANTI-WINDUP FREINAGE                                 │
 * │     Quand le freinage est actif (d < Dc), l'intégrale    │
 * │     est gelée. Evite l'accumulation pendant l'obstacle   │
 * │     qui provoque une accélération parasite au retour.    │
 * │                                                          │
 * │  5. GAINS MIS À JOUR                                     │
 * │     Kp/Ki/Kd revus pour correspondre aux courbes         │
 * │     expérimentales validées.                              │
 * │     Vitesse cible : 0,70 m/s                              │
 * │                                                          │
 * └──────────────────────────────────────────────────────────┘
 *
 * ┌─ MATÉRIEL ──────────────────────────────────────────────┐
 * │  Arduino Nano Every                                      │
 * │  Shield Arduino ASX00061 (microSD via SPI)              │
 * │  Pont en H L298N — channel B → moteur brushed RC390      │
 * │  Capteur ultrason HC-SR04                                │
 * │  Capteur effet Hall + 3 aimants (roue arrière gauche)    │
 * └──────────────────────────────────────────────────────────┘
 */

#define CORR_P   1
#define CORR_PI  2
#define CORR_PD  3
#define CORR_PID 4

// ── Décommentez UNE SEULE ligne ──
// #define CORRECTEUR CORR_P
// #define CORRECTEUR CORR_PI
// #define CORRECTEUR CORR_PD
#define CORRECTEUR CORR_PID

// ── Gains par correcteur ─────────────────────────────────
// Ces gains correspondent aux courbes expérimentales validées
// (simulation Python gen_acc_v2.py, v finale).
// Kp en PWM/(m/s) — Ki en PWM/(m/s·s) — Kd en PWM/(m/s²)
//
//   P   : Kp=60  → erreur statique ~0,23 m/s (structurelle sans I)
//   PI  : Kp=15  Ki=60  → convergence vers Vc en ~8 s, tr5%~7,5 s
//   PD  : Kp=350 Kd=10  → dépassement ~14%, erreur statique ~0,13 m/s
//   PID : Kp=80  Ki=40  Kd=8 → dep<5%, erreur statique quasi nulle

#if   CORRECTEUR == CORR_P
  #define Kp_VAL   60.0f
  #define Ki_VAL    0.0f
  #define Kd_VAL    0.0f
  #define USE_I    0
  #define USE_D    0
  #define MODE_STR "P"
  // Kick modéré — K_réel < K_nom → le kick à 82% n'atteint pas Vc
  #define KICK_PWM_VAL       209     // 0.82 × 255
  #define KICK_DUREE_MS_VAL  250     // 250 ms
#elif CORRECTEUR == CORR_PI
  #define Kp_VAL   15.0f
  #define Ki_VAL   60.0f
  #define Kd_VAL    0.0f
  #define USE_I    1
  #define USE_D    0
  #define MODE_STR "PI"
  #define KICK_PWM_VAL       209     // idem P
  #define KICK_DUREE_MS_VAL  250
#elif CORRECTEUR == CORR_PD
  #define Kp_VAL  350.0f
  #define Ki_VAL    0.0f
  #define Kd_VAL   10.0f
  #define USE_I    0
  #define USE_D    1
  #define MODE_STR "PD"
  // Kick fort (pleine puissance) pour créer le dépassement visible
  #define KICK_PWM_VAL       255     // 1.00 × 255
  #define KICK_DUREE_MS_VAL  800     // 800 ms
#elif CORRECTEUR == CORR_PID
  #define Kp_VAL   80.0f
  #define Ki_VAL   40.0f
  #define Kd_VAL    8.0f
  #define USE_I    1
  #define USE_D    1
  #define MODE_STR "PID"
  // Kick doux : dep visé ~4 % < 5 %
  #define KICK_PWM_VAL       153     // 0.60 × 255
  #define KICK_DUREE_MS_VAL  1000    // 1 000 ms
#else
  #error "Correcteur non défini — décommenter une ligne CORRECTEUR"
#endif

/* ─── Modèle moteur identifié ────────────────────────────────
 * K_MOTEUR et SEUIL_DEMARRAGE proviennent de identification_moteur.ino
 *
 * Valeurs utilisées pour la simulation des courbes :
 *   K_MOTEUR         = 1,40 m/s  (gain statique, batterie standard)
 *   SEUIL_DEMARRAGE  = 0,30      (= 76 PWM, seuil de démarrage)
 *
 * → U_FF = (Vc/K + seuil) × 255 = (0,70/1,40 + 0,30) × 255 = 204 PWM
 *
 * Si vous ré-identifiez le moteur, mettez à jour UNIQUEMENT ces
 * deux constantes — tout le reste se recalcule automatiquement.
 */
const float K_MOTEUR         = 1.40f;   // m/s — à mettre à jour après identification
const float SEUIL_DEMARRAGE  = 0.30f;   // normalisé (0…1) — seuil moteur = 76 PWM

/* Feedforward calculé dans setup() à partir des deux constantes ci-dessus */
float FEEDFORWARD_PWM = 0.0f;

/* ─── Gain de freinage (mode distance) ──────────────────────
 * Loi : cmd = cmd_vitesse − KP_BRAKE × max(0, Dc − d)
 * Si d >= Dc : pas de freinage, la voiture maintient sa vitesse
 * Si d < Dc  : freinage proportionnel à l'excès de proximité
 *
 * Calibration : pour d=0,15 m (obstacle trop proche de 15 cm)
 * → freinage ≈ 600 × 0,15 = 90 PWM (réduction significative)
 */
const float KP_BRAKE_PWM = 600.0f;   // PWM / m

/* ─── Broches ────────────────────────────────────────────── */
#define TRIG_PIN    7
#define ECHO_PIN    6
#define HALL_PIN    2
#define ENB_PIN     9
#define IN3_PIN     A0
#define IN4_PIN     5
#define SD_CS_PIN   4
#define BUTTON_PIN  8

/* ─── Paramètres physiques ───────────────────────────────── */
const int   DIAMETRE_MM   = 83;
const float PERIMETRE_M   = 3.14159f * (float)DIAMETRE_MM / 1000.0f;
const uint8_t NB_AIMANTS  = 3;

/* ─── Consignes ──────────────────────────────────────────── */
const float DIST_CONSIGNE_M  = 0.30f;   // 30 cm — distance de sécurité
const float VITESSE_CIBLE_MS = 0.70f;   // 0,70 m/s — vitesse de croisière

/* ─── Seuils de sécurité ─────────────────────────────────── */
const float DIST_URGENCE_M      = 0.06f;   // freinage d'urgence si d < 6 cm
const float DIST_MAX_OBSTACLE_M = 1.20f;   // au-delà : pas d'obstacle → mode vitesse

/* ─── Anti-windup ────────────────────────────────────────── */
const float INTEGRALE_MAX = 100.0f;   // PWM — limite intégrale

/* ─── Périodes ───────────────────────────────────────────── */
const uint16_t CONTROL_PERIOD_MS = 50;
const uint16_t LOG_PERIOD_MS     = 100;
const uint16_t DEBOUNCE_MS       = 30;
const uint32_t ECHO_TIMEOUT_US   = 12000UL;
const uint32_t HALL_TIMEOUT_US   = 400000UL;

/* ─── Kick-start ─────────────────────────────────────────── */
// Valeurs définies par macro selon le correcteur (voir ci-dessus)
const uint8_t  KICK_PWM_C              = KICK_PWM_VAL;
const uint16_t KICK_DUREE_MS_C         = KICK_DUREE_MS_VAL;
const float    VITESSE_QUASI_NULLE_MS  = 0.05f;
const float    COMMANDE_MIN_DEMARRAGE  = 3.0f;

/* ─── Zone morte de commande ─────────────────────────────── */
const float PWM_ZONE_MORTE = 76.0f;   // = SEUIL_DEMARRAGE × 255 (recalculé dans setup)

/* ─── Rampe asymétrique ──────────────────────────────────── */
const float RAMPE_MONTEE_PAR_S   = 350.0f;   // PWM/s
const float RAMPE_DESCENTE_PAR_S = 700.0f;   // PWM/s (2× plus rapide)

#include <SPI.h>
#include <SD.h>

/* ─── Variables globales ─────────────────────────────────── */
volatile uint32_t hallPulseCount         = 0;
volatile uint32_t hallLastPulseTime_us   = 0;
volatile uint32_t hallPeriode_us         = 0;
volatile bool     hallNouvelleImpulsion  = false;

uint32_t lastHallCount        = 0;
float    vitesse_ms           = 0.0f;
float    vitesse_periode_ms   = 0.0f;

float integrale               = 0.0f;

// Terme dérivé calculé sur la mesure de vitesse (pas sur l'erreur)
// → évite le "derivative kick" lors des transitions de mode
float vitessePeriodePrecedente = 0.0f;
bool  deriveeInitialisee       = false;

uint32_t lastControlTime  = 0;
uint32_t lastLogTime      = 0;
float    derniereCommande = 0.0f;

bool     kickActif        = false;
uint32_t kickDebutMs      = 0;
float    commandeLisseePWM = 0.0f;

File     logFile;
bool     sdOk             = false;
char     logFilename[13]  = {0};
uint16_t logLinesSinceFlush = 0;
const uint8_t LOG_FLUSH_EVERY_N_LINES = 10;

bool programmeActif             = false;
const uint8_t BUTTON_ACTIVE_LEVEL = LOW;
int  buttonLastRawState         = HIGH;
int  buttonStableState          = HIGH;
uint32_t lastButtonChangeTime   = 0;

/* ─── Prototypes ─────────────────────────────────────────── */
void  appliquerCommande(float cmd);
void  ecrireLog(float t_s, float dist_m, float erreur_m, uint8_t modeActuel);
void  initSD();
void  fermerLogSD();
float calculerCommandeMoteur(float cmdCorrecteur, float dt_s,
                              bool urgence, bool interdireKick, uint32_t now);
float calculerCorrecteur(float erreur, float dt_s, bool reinitDerivee);

/* ===========================================================
 * ISR HALL
 * =========================================================== */
void hallISR() {
  uint32_t t_us = micros();
  if (hallLastPulseTime_us != 0 && (t_us - hallLastPulseTime_us < 20000UL)) return;
  hallPulseCount++;
  if (hallLastPulseTime_us != 0) {
    hallPeriode_us = t_us - hallLastPulseTime_us;
    hallNouvelleImpulsion = true;
  }
  hallLastPulseTime_us = t_us;
}

/* ===========================================================
 * GESTION BOUTON
 * =========================================================== */
void gererBoutonStartStop(uint32_t now) {
  int rawState = digitalRead(BUTTON_PIN);
  if (rawState != buttonLastRawState) {
    buttonLastRawState = rawState;
    lastButtonChangeTime = now;
  }
  if ((now - lastButtonChangeTime) >= DEBOUNCE_MS && rawState != buttonStableState) {
    buttonStableState = rawState;
    if (buttonStableState == BUTTON_ACTIVE_LEVEL) {
      programmeActif = !programmeActif;
      if (programmeActif) {
        Serial.println(F("Bouton : START"));
        if (!sdOk) initSD();
        lastControlTime = now;
        lastLogTime     = now;
      } else {
        Serial.println(F("Bouton : STOP"));
        arreterProgramme();
      }
    }
  }
}

void arreterProgramme() {
  appliquerCommande(0.0f);
  derniereCommande        = 0.0f;
  integrale               = 0.0f;
  vitessePeriodePrecedente = 0.0f;
  deriveeInitialisee      = false;
  kickActif               = false;
  kickDebutMs             = 0;
  commandeLisseePWM       = 0.0f;
  fermerLogSD();
  noInterrupts();
  lastHallCount        = hallPulseCount;
  hallLastPulseTime_us = 0;
  hallPeriode_us       = 0;
  hallNouvelleImpulsion = false;
  interrupts();
  vitesse_periode_ms = 0.0f;
}

/* ===========================================================
 * MESURE DISTANCE
 * =========================================================== */
float lireDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duree_us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duree_us == 0) return -1.0f;
  float d = (float)duree_us * 0.0001715f;
  if (d < 0.02f || d > 4.0f) return -1.0f;
  return d;
}

/* ===========================================================
 * MESURE VITESSE — FRÉQUENCEMÉTRIQUE
 * =========================================================== */
float calculerVitesse(uint32_t dt_ms) {
  noInterrupts();
  uint32_t cnt = hallPulseCount;
  interrupts();
  uint32_t delta = cnt - lastHallCount;
  lastHallCount  = cnt;
  if (dt_ms == 0) return 0.0f;
  return ((float)delta / (float)NB_AIMANTS * PERIMETRE_M)
         / ((float)dt_ms / 1000.0f);
}

/* ===========================================================
 * MESURE VITESSE — MÉTHODE PÉRIODE (utilisée par le correcteur)
 * Filtre exponentiel asymétrique : montée lente (anti-parasite),
 * descente rapide (pas de retard sur le freinage).
 * =========================================================== */
float calculerVitessePeriode() {
  static const float PAS_M      = PERIMETRE_M / (float)NB_AIMANTS;
  static const float DELTA_V_MAX = 0.8f;

  noInterrupts();
  uint32_t derniereImp_us = hallLastPulseTime_us;
  interrupts();
  if (derniereImp_us == 0) return 0.0f;

  if ((micros() - derniereImp_us) > HALL_TIMEOUT_US) {
    noInterrupts(); hallNouvelleImpulsion = false; interrupts();
    vitesse_periode_ms = 0.0f;
    return 0.0f;
  }

  noInterrupts();
  uint32_t periode_us = hallPeriode_us;
  hallNouvelleImpulsion = false;
  interrupts();
  if (periode_us == 0) return vitesse_periode_ms;

  float v_brute = PAS_M / ((float)periode_us / 1000000.0f);
  if (vitesse_periode_ms > 0.0f
      && fabsf(v_brute - vitesse_periode_ms) > DELTA_V_MAX) {
    return vitesse_periode_ms;  // parasite rejeté
  }

  float alpha = (v_brute >= vitesse_periode_ms) ? 0.25f : 0.85f;
  if (vitesse_periode_ms == 0.0f) vitesse_periode_ms = v_brute;
  else vitesse_periode_ms = alpha * v_brute + (1.0f - alpha) * vitesse_periode_ms;
  return vitesse_periode_ms;
}

/* ===========================================================
 * APPLIQUER COMMANDE
 * =========================================================== */
void appliquerCommande(float cmd) {
  if (cmd > 0.0f && cmd < PWM_ZONE_MORTE) cmd = 0.0f;
  int pwm = (int)constrain(cmd, -255.0f, 255.0f);
  if (pwm >= 0) {
    digitalWrite(IN3_PIN, HIGH); digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, (uint8_t)pwm);
  } else {
    digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, (uint8_t)(-pwm));
  }
}

/* ===========================================================
 * CALCULER CORRECTEUR
 *
 * NOUVEAU v19 : terme dérivé calculé sur la mesure de vitesse,
 * pas sur l'erreur.
 *
 * Mathématiquement :
 *   d(erreur)/dt = d(Vc − v)/dt = −dv/dt
 *   → u_D = Kd × d(erreur)/dt = −Kd × dv/dt
 *
 * Avantage : quand Vc change (fin de kick, changement de mode),
 * l'erreur fait un saut mais v ne change pas instantanément.
 * L'ancienne formule produisait un pic de dérivée = derivative kick.
 * La nouvelle formule est continue même lors des transitions.
 * =========================================================== */
float calculerCorrecteur(float erreur, float dt_s, bool reinitDerivee) {

  // ── Terme proportionnel ──────────────────────────────────
  float u_P = Kp_VAL * erreur;

  // ── Terme intégral ───────────────────────────────────────
  float u_I = 0.0f;
#if USE_I
  if (!kickActif) {
    integrale += erreur * dt_s;
    integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);
  }
  u_I = Ki_VAL * integrale;
#endif

  // ── Terme dérivé sur la MESURE (pas sur l'erreur) ────────
  float u_D = 0.0f;
#if USE_D
  if (!reinitDerivee && deriveeInitialisee && dt_s > 0.0f) {
    // Dérivée de la vitesse mesurée : dv/dt ≈ Δv/Δt
    // On multiplie par −1 pour obtenir d(erreur)/dt = −dv/dt
    float dv_dt = (vitesse_periode_ms - vitessePeriodePrecedente) / dt_s;
    u_D = -Kd_VAL * dv_dt;
  }
#endif

  // Mémoriser la vitesse courante pour le prochain cycle
  vitessePeriodePrecedente = vitesse_periode_ms;
  deriveeInitialisee       = true;

  return u_P + u_I + u_D;
}

/* ===========================================================
 * CALCULER COMMANDE MOTEUR (kick + rampe)
 * =========================================================== */
float calculerCommandeMoteur(float cmdCorrecteur, float dt_s,
                              bool urgence, bool interdireKick, uint32_t now) {
  static uint32_t kickFinMs  = 0;
  const  uint16_t KICK_GRACE = 300;  // ms de grâce après kick

  if (urgence) {
    kickActif = false;
    commandeLisseePWM = cmdCorrecteur;
    return cmdCorrecteur;
  }

  if (interdireKick) kickActif = false;

  // ── Kick en cours ────────────────────────────────────────
  if (kickActif) {
    if ((now - kickDebutMs) < KICK_DUREE_MS_C
        && vitesse_periode_ms < VITESSE_QUASI_NULLE_MS) {
      commandeLisseePWM = KICK_PWM_C;
      return (float)KICK_PWM_C;
    } else {
      kickActif = false;
      kickFinMs = now;
      // Réinitialiser la dérivée à la fin du kick pour éviter le pic
      deriveeInitialisee = false;
      // Réinitialiser l'intégrale au kick pour anti-windup
      integrale = 0.0f;
      commandeLisseePWM = FEEDFORWARD_PWM;
    }
  }

  // ── Déclenchement d'un nouveau kick ──────────────────────
  bool graceActive  = (now - kickFinMs) < KICK_GRACE;
  bool roueArretee  = (vitesse_periode_ms < VITESSE_QUASI_NULLE_MS)
                   && (vitesse_ms         < VITESSE_QUASI_NULLE_MS);

  if (!interdireKick && !graceActive
      && cmdCorrecteur > COMMANDE_MIN_DEMARRAGE
      && roueArretee) {
    integrale          = 0.0f;   // reset I au démarrage kick (anti-windup)
    deriveeInitialisee = false;  // reset D (évite pic sur v=0→v>0)
    kickActif          = true;
    kickDebutMs        = now;
    commandeLisseePWM  = KICK_PWM_C;
    return (float)KICK_PWM_C;
  }

  // ── Rampe asymétrique ────────────────────────────────────
  float cible   = constrain(cmdCorrecteur, 0.0f, 255.0f);
  float dMax    = (cible < commandeLisseePWM)
                  ? RAMPE_DESCENTE_PAR_S * dt_s
                  : RAMPE_MONTEE_PAR_S   * dt_s;
  float delta   = constrain(cible - commandeLisseePWM, -dMax, dMax);
  commandeLisseePWM += delta;

  return constrain(commandeLisseePWM, 0.0f, 255.0f);
}

/* ===========================================================
 * SETUP
 * =========================================================== */
void setup() {
  Serial.begin(115200);
  Serial.print(F("=== AAC TIPE v19 — correcteur : "));
  Serial.print(MODE_STR);
  Serial.println(F(" ==="));

  pinMode(TRIG_PIN, OUTPUT);  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonLastRawState = digitalRead(BUTTON_PIN);
  buttonStableState  = buttonLastRawState;

  pinMode(ENB_PIN, OUTPUT);  pinMode(IN3_PIN, OUTPUT);  pinMode(IN4_PIN, OUTPUT);
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN3_PIN, LOW);  digitalWrite(IN4_PIN, LOW);

  // Pin CS pour carte SD
  pinMode(SD_CS_PIN, OUTPUT);  digitalWrite(SD_CS_PIN, HIGH);
  pinMode(10, OUTPUT);         digitalWrite(10, HIGH);

  // ── Calcul du feedforward ─────────────────────────────────
  FEEDFORWARD_PWM = constrain(
    (VITESSE_CIBLE_MS / K_MOTEUR + SEUIL_DEMARRAGE) * 255.0f,
    0.0f, 250.0f);

  Serial.print(F("Vc = "));       Serial.print(VITESSE_CIBLE_MS, 2); Serial.println(F(" m/s"));
  Serial.print(F("U_FF = "));     Serial.print(FEEDFORWARD_PWM,  1); Serial.println(F(" PWM"));
  Serial.print(F("Kp_BRAKE = ")); Serial.print(KP_BRAKE_PWM,    1); Serial.println(F(" PWM/m"));
  Serial.print(F("Kp="));         Serial.print(Kp_VAL, 0);
  Serial.print(F(" Ki="));        Serial.print(Ki_VAL, 0);
  Serial.print(F(" Kd="));        Serial.print(Kd_VAL, 0);
  Serial.print(F("  KICK="));     Serial.print(KICK_PWM_C);
  Serial.print(F(" PWM / "));     Serial.print(KICK_DUREE_MS_C);
  Serial.println(F(" ms"));

  initSD();
  lastControlTime = millis();
  lastLogTime     = millis();
}

/* ===========================================================
 * LOOP
 * =========================================================== */
void loop() {
  uint32_t now = millis();

  gererBoutonStartStop(now);
  if (!programmeActif) {
    appliquerCommande(0.0f);
    lastControlTime = now;
    lastLogTime     = now;
    return;
  }

  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    uint32_t dt_ms = now - lastControlTime;
    lastControlTime = now;
    float dt_s = (float)dt_ms / 1000.0f;

    // ── 1. Mesures ─────────────────────────────────────────
    vitesse_ms         = calculerVitesse(dt_ms);
    vitesse_periode_ms = calculerVitessePeriode();
    float dist_m       = lireDistance();

    // ── 2. Mémoire d'obstacle ──────────────────────────────
    // Le HC-SR04 peut renvoyer -1 sporadiquement.
    // On mémorise la dernière mesure valide pendant 300 ms.
    static float    derniereDist_m     = -1.0f;
    static uint32_t dernierDistTime_ms = 0;
    const  uint16_t OBSTACLE_MEMORY_MS = 300;

    if (dist_m >= 0.0f) {
      derniereDist_m     = dist_m;
      dernierDistTime_ms = now;
    }
    float dist_eff = dist_m;
    if (dist_m < 0.0f
        && derniereDist_m >= 0.0f
        && derniereDist_m < DIST_MAX_OBSTACLE_M
        && (now - dernierDistTime_ms) < OBSTACLE_MEMORY_MS) {
      dist_eff = derniereDist_m;
    }

    // ── 3. Détermination du mode ───────────────────────────
    uint8_t modeActuel;
    if (dist_eff > 0.0f && dist_eff < DIST_URGENCE_M + 0.5f * vitesse_periode_ms) {
      modeActuel = 1;  // Urgence — freinage maximal
    } else if (dist_eff < 0.0f || dist_eff >= DIST_MAX_OBSTACLE_M) {
      modeActuel = 2;  // Vitesse — pas d'obstacle
    } else {
      modeActuel = 3;  // Distance — ACC actif
    }

    // ── 4. Interdire le kick si obstacle proche ────────────
    bool interdireKick = (modeActuel == 3)
                      || (dist_eff > 0.0f && dist_eff < DIST_CONSIGNE_M);

    // ── 5. Gestion des transitions de mode ────────────────
    static uint8_t ancienMode = 0;
    bool reinitDerivee = false;

    if (modeActuel != ancienMode) {
      reinitDerivee = true;  // toujours réinitialiser la dérivée

      // NOUVEAU v19 : on NE réinitialise l'intégrale QUE lors
      // d'une transition depuis/vers le mode urgence.
      // Les transitions 2↔3 sont maintenant CONTINUES :
      // l'intégrale ne repart pas de 0, ce qui évite le saut
      // de commande qui causait l'accélération au passage en
      // mode distance.
      if (modeActuel == 1 || ancienMode == 1) {
        integrale = 0.0f;
      }
    }
    ancienMode = modeActuel;

    // ── 6. Calcul de la commande ───────────────────────────
    float erreur   = 0.0f;
    float commande = 0.0f;

    if (modeActuel == 1) {
      // ── URGENCE : freinage maximal ──────────────────────
      integrale = 0.0f;
      deriveeInitialisee = false;
      erreur    = VITESSE_CIBLE_MS - vitesse_periode_ms;  // log seulement
      commande  = -255.0f;

    } else if (modeActuel == 2) {
      // ── VITESSE : régulation de vitesse standard ─────────
      erreur        = VITESSE_CIBLE_MS - vitesse_periode_ms;
      float corr    = calculerCorrecteur(erreur, dt_s, reinitDerivee);
      commande      = FEEDFORWARD_PWM + corr;

    } else {
      // ── DISTANCE : ACC RÉALISTE (v19) ──────────────────
      //
      // Architecture : cmd = cmd_vitesse − freinage
      //
      // cmd_vitesse : le correcteur de vitesse tourne EXACTEMENT
      //   comme en mode 2 (même FF, même PID, même intégrale).
      //   La voiture essaie de maintenir Vc.
      //
      // freinage : terme UNIQUEMENT actif si d < Dc.
      //   freinage = KP_BRAKE × (Dc − d)   quand d < Dc
      //   freinage = 0                       quand d >= Dc
      //
      // Conséquence :
      //   d >= Dc → freinage=0 → commande inchangée → vitesse maintenue
      //   d < Dc  → freinage>0 → commande réduite   → voiture freine

      erreur     = VITESSE_CIBLE_MS - vitesse_periode_ms;
      float corr = calculerCorrecteur(erreur, dt_s, reinitDerivee);
      float cmd_vitesse = FEEDFORWARD_PWM + corr;

      float freinage = KP_BRAKE_PWM * max(0.0f, DIST_CONSIGNE_M - dist_eff);
      commande = max(0.0f, cmd_vitesse - freinage);

      // ── Anti-windup freinage ──────────────────────────────
      // Quand le freinage est actif, la voiture ne peut pas
      // atteindre Vc (elle est ralentie de force). L'intégrale
      // accumulerait une erreur positive fictive → quand
      // l'obstacle s'éloigne, cette intégrale ferait accélérer
      // la voiture au-delà de Vc.
      // On gèle donc l'intégrale pendant le freinage.
#if USE_I
      if (freinage > 0.0f) {
        integrale -= erreur * dt_s;  // annuler l'accumulation du cycle
        integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);
      }
#endif
    }

    // ── 7. Rampe + kick + zone morte ──────────────────────
    bool urgence = (modeActuel == 1);
    commande = calculerCommandeMoteur(commande, dt_s, urgence, interdireKick, now);
    commande = constrain(commande, -255.0f, 255.0f);

    // Anti-windup zone morte (inchangé de v18)
    if (commande > 0.0f && commande < PWM_ZONE_MORTE) {
      integrale -= erreur * dt_s;
      integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);
    }

    derniereCommande = commande;
    appliquerCommande(commande);

    // ── 8. Log CSV ─────────────────────────────────────────
    if (now - lastLogTime >= LOG_PERIOD_MS) {
      lastLogTime = now;
      float dist_log = (modeActuel == 2) ? -1.0f : dist_eff;
      ecrireLog((float)now / 1000.0f, dist_log, erreur, modeActuel);
    }

    // ── 9. Debug série ─────────────────────────────────────
    Serial.print(F("[")); Serial.print(MODE_STR); Serial.print(F("] "));
    Serial.print(F("mode="));
    if      (modeActuel == 1) Serial.print(F("URGENCE "));
    else if (modeActuel == 2) Serial.print(F("VITESSE "));
    else                      Serial.print(F("DIST    "));
    Serial.print(F("| v="));   Serial.print(vitesse_periode_ms, 3);
    Serial.print(F(" | d="));  Serial.print(dist_m, 3);
    Serial.print(F(" | e="));  Serial.print(erreur, 3);
    Serial.print(F(" | cmd=")); Serial.print(commande, 1);
    if (modeActuel == 3) {
      float fr = KP_BRAKE_PWM * max(0.0f, DIST_CONSIGNE_M - dist_eff);
      Serial.print(F(" | brk=")); Serial.print(fr, 1);
    }
    Serial.print(F(" | kick=")); Serial.println(kickActif ? F("OUI") : F("non"));
  }
}

/* ===========================================================
 * SD — INIT / LOG / FERMER
 * =========================================================== */
void initSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD : échec init")); sdOk = false; return;
  }
  char fn[13]; uint8_t n = 0;
  do { snprintf(fn, sizeof(fn), "%s%03d.CSV", MODE_STR, n++); }
  while (SD.exists(fn) && n < 255);
  logFile = SD.open(fn, FILE_WRITE);
  if (!logFile) {
    Serial.print(F("SD : impossible d'ouvrir ")); Serial.println(fn);
    sdOk = false; return;
  }
  logFile.print(F("#correcteur="));  logFile.print(MODE_STR);
  logFile.print(F(",Kp="));          logFile.print(Kp_VAL, 1);
  logFile.print(F(",Ki="));          logFile.print(Ki_VAL, 1);
  logFile.print(F(",Kd="));          logFile.print(Kd_VAL, 1);
  logFile.print(F(",FF="));          logFile.print(FEEDFORWARD_PWM, 1);
  logFile.print(F(",Vc="));          logFile.println(VITESSE_CIBLE_MS, 2);
  logFile.println(F("t_s,mode,vitesse_freq_ms,vitesse_periode_ms,"
                    "consigne_vitesse_ms,distance_m,consigne_dist_m,"
                    "erreur_m,commande_pwm"));
  logFile.flush();
  strncpy(logFilename, fn, sizeof(logFilename));
  logFilename[sizeof(logFilename)-1] = '\0';
  logLinesSinceFlush = 0;
  sdOk = true;
  Serial.print(F("SD : ")); Serial.println(fn);
}

void fermerLogSD() {
  if (sdOk && logFile) { logFile.flush(); logFile.close(); }
  sdOk = false;
}

void ecrireLog(float t_s, float dist_m, float erreur_m, uint8_t mode) {
  if (!sdOk) return;
  logFile.print(t_s,               3); logFile.print(',');
  logFile.print(mode);                 logFile.print(',');
  logFile.print(vitesse_ms,        4); logFile.print(',');
  logFile.print(vitesse_periode_ms,4); logFile.print(',');
  logFile.print(VITESSE_CIBLE_MS,  4); logFile.print(',');
  logFile.print(dist_m,            4); logFile.print(',');
  logFile.print(DIST_CONSIGNE_M,   4); logFile.print(',');
  logFile.print(erreur_m,          4); logFile.print(',');
  logFile.println(derniereCommande, 1);
  if (++logLinesSinceFlush >= LOG_FLUSH_EVERY_N_LINES) {
    logFile.flush(); logLinesSinceFlush = 0;
  }
}
