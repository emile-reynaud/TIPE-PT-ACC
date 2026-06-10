/*
 * ============================================================
 * RÉGULATEUR DE VITESSE ADAPTATIF (AAC) — TIPE CPGE PT
 * Version 9 — Corrigée (Gestion des modes et sécurité Kick)
 * ============================================================
 *
 * ┌─ MATÉRIEL ──────────────────────────────────────────────┐
 * │  Arduino Nano Every                                      │
 * │  Shield Arduino ASX00061 (microSD via SPI)              │
 * │  Pont en H L298N — channel B → moteur brushed RC390      │
 * │  Capteur ultrason HC-SR04                                │
 * │  Capteur effet Hall + 6 aimants (roue arrière gauche)    │
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

#define Kp_VAL   12.0f   // Gain proportionnel
#define Ki_VAL    3.0f   // Gain intégral
#define Kd_VAL    0.5f   // Gain dérivé (faible : 3 aimants = mesure bruitée)

/* ─── Feedforward (action anticipatrice) ─────────────────────
 * Problème sans feedforward :
 *   Le PID calcule u = Kp×e + Ki×∫e + Kd×de/dt
 *   En régime, e→0 donc u→0, mais le moteur a besoin de ~197 PWM
 *   juste pour maintenir 0,40 m/s (seuil 180 + frottements).
 *   Le PID seul ne peut pas tenir la vitesse : il redescend à 0,
 *   la voiture s'arrête, le kick repart → cycle infini.
 *
 * Solution — feedforward :
 *   u_total = U_FF + Kp×e + Ki×∫e + Kd×de/dt
 *   U_FF = commande d'équilibre calculée depuis l'identification :
 *     U_FF = (Vc / K_moteur + seuil_demarrage) × 255
 *          = (0,40 / 5,7669 + 0,706) × 255 ≈ 198 PWM
 *
 *   Avec U_FF, le PID ne corrige que les ÉCARTS autour du point
 *   de fonctionnement → gains faibles suffisent → système stable.
 *
 *   Note pour le TIPE : le feedforward est la partie "modèle",
 *   le PID est la partie "correcteur". C'est l'architecture
 *   standard des régulateurs industriels (2-DOF controller).
 *
 * Valeur calculée : (0.40/5.7669 + 0.706) × 255 = 197.7 → 198
 * On peut ajuster légèrement si la voiture dérive en régime.
 */
const float FEEDFORWARD_PWM = 198.0f;

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
const int DIAMETRE_MM    = 83;
const float PERIMETRE_M   = 3.14159f * (float)DIAMETRE_MM/1000;  
const uint8_t NB_AIMANTS  = 3;

/* ─── Consignes ──────────────────────────────────────────── */
const float DIST_CONSIGNE_M    = 0.30f; // 30 cm
const float VITESSE_CIBLE_MS   = 0.40f; // 0,40 m/s

/* ─── Seuils de sécurité ─────────────────────────────────── */
const float DIST_URGENCE_M     = 0.10f; // 10 cm
const float DIST_MAX_OBSTACLE_M= 2.00f; // 2 m

/* ─── Anti-windup ────────────────────────────────────────── */
const float INTEGRALE_MAX = 100.0f;

/* ─── Périodes de la boucle ──────────────────────────────── */
const uint16_t CONTROL_PERIOD_MS = 50;  
const uint16_t LOG_PERIOD_MS     = 100; 
const uint16_t DEBOUNCE_MS       = 30;  
const uint32_t ECHO_TIMEOUT_US   = 12000UL;

const uint32_t HALL_TIMEOUT_US = 400000UL;

/* ─── Démarrage moteur (kick-start) ──────────────────────── */
// Le moteur ne démarre qu'à PWM ≥ 180 (seuil mesuré = 0.706 × 255 = 180).
// Le kick envoie PWM = 190 pendant 150 ms pour vaincre l'inertie statique,
// puis laisse la rampe et le correcteur reprendre la main.
// COMMANDE_MIN_DEMARRAGE : seuil de commande PID (PWM) au-dessus duquel
// on déclenche le kick. Avec Kp=12 et erreur=0.40 → u_P = 4.8 PWM.
// On met le seuil à 3.0 pour qu'il se déclenche bien au démarrage.
const uint8_t  KICK_PWM               = 190;   // juste au-dessus du seuil (180) + marge
const uint16_t KICK_DUREE_MS          = 150;
const float    VITESSE_QUASI_NULLE_MS = 0.05f;
const float    COMMANDE_MIN_DEMARRAGE = 3.0f;   // en PWM — seuil bas pour Kp faible

/* ─── Lissage de commande ────────────────────────────────── */
const float RAMPE_PWM_PAR_S = 350.0f;

#if   CORRECTEUR == CORR_PI
  #define USE_I    1
  #define USE_D    0
  #define MODE_STR "PI"
#elif CORRECTEUR == CORR_PD
  #define USE_I    0
  #define USE_D    1
  #define MODE_STR "PD"
#elif CORRECTEUR == CORR_PID
  #define USE_I    1
  #define USE_D    1
  #define MODE_STR "PID"
#else  
  #define USE_I    0
  #define USE_D    0
  #define MODE_STR "P"
#endif

#include <SPI.h>
#include <SD.h>

/* ─── Variables globales ─────────────────────────────────── */
volatile uint32_t hallPulseCount = 0;
volatile uint32_t hallLastPulseTime_us = 0;
volatile uint32_t hallPeriode_us = 0;
volatile bool hallNouvelleImpulsion = false;

uint32_t lastHallCount = 0;
float vitesse_ms = 0.0f;          
float vitesse_periode_ms = 0.0f;

float erreurPrecedente = 0.0f;
float integrale        = 0.0f;

uint32_t lastControlTime = 0;
uint32_t lastLogTime     = 0;
float derniereCommande = 0.0f;

bool     kickActif        = false;
uint32_t kickDebutMs      = 0;
float    commandeLisseePWM = 0.0f;   

File logFile;
bool sdOk = false;
char logFilename[13] = {0};
uint16_t logLinesSinceFlush = 0;
const uint8_t LOG_FLUSH_EVERY_N_LINES = 10;

bool programmeActif = false;
const uint8_t BUTTON_ACTIVE_LEVEL = LOW;

int buttonLastRawState    = HIGH;
int buttonStableState     = HIGH;
uint32_t lastButtonChangeTime = 0;

// Prototypes mis à jour
void appliquerCommande(float cmd);
void fermerLogSD();
void initSD();
float calculerCommandeMoteur(float commandeCorrecteur, float dt_s, bool urgence, bool interdireKick, uint32_t now);
float calculerCorrecteur(float erreur, float dt_s, bool reinitialiserDerivee = false);

void hallISR() {
  uint32_t maintenant_us = micros();
  if (hallLastPulseTime_us != 0 && (maintenant_us - hallLastPulseTime_us < 20000UL)) {
    return;
  }
  hallPulseCount++;
  if (hallLastPulseTime_us != 0) {
    hallPeriode_us = maintenant_us - hallLastPulseTime_us;
    hallNouvelleImpulsion = true;   
  }
  hallLastPulseTime_us = maintenant_us;
}

void arreterProgramme() {
  appliquerCommande(0.0f);
  derniereCommande = 0.0f;
  integrale        = 0.0f;
  erreurPrecedente = 0.0f;
  kickActif        = false;
  kickDebutMs      = 0;
  commandeLisseePWM = 0.0f;
  fermerLogSD();
  noInterrupts();
  lastHallCount         = hallPulseCount;
  hallLastPulseTime_us  = 0;
  hallPeriode_us        = 0;
  hallNouvelleImpulsion = false;
  interrupts();
  vitesse_periode_ms = 0.0f;
}

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
        if (!sdOk) { initSD(); }
        lastControlTime = now;
        lastLogTime     = now;
      } else {
        Serial.println(F("Bouton : STOP"));
        arreterProgramme();
      }
    }
  }
}

float lireDistance() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);          
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  long duree_us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);
  if (duree_us == 0) return -1.0f;
  float dist_m = (float)duree_us * 0.0001715f;
  if (dist_m < 0.02f || dist_m > 4.0f) return -1.0f;
  return dist_m;
}

float calculerVitesse(uint32_t dt_ms) {
  noInterrupts();
  uint32_t countActuel = hallPulseCount;
  interrupts();
  uint32_t delta = countActuel - lastHallCount;
  lastHallCount  = countActuel;   
  if (dt_ms == 0) return 0.0f;
  float tours = (float)delta / (float)NB_AIMANTS;
  float distance_m = tours * PERIMETRE_M;
  return distance_m / ((float)dt_ms / 1000.0f);
}

float calculerVitessePeriode() {
  static const float PAS_M = PERIMETRE_M / (float)NB_AIMANTS;
  noInterrupts();
  uint32_t  derniereImpulsion_us = hallLastPulseTime_us;
  interrupts();
  if (derniereImpulsion_us == 0) return 0.0f;

  uint32_t silenceActuel_us = micros() -  derniereImpulsion_us;
  if (silenceActuel_us > HALL_TIMEOUT_US) {
    noInterrupts();
    hallNouvelleImpulsion = false;
    interrupts();
    vitesse_periode_ms = 0.0f;
    return 0.0f;
  }
  noInterrupts();
  uint32_t periode_us      = hallPeriode_us;
  hallNouvelleImpulsion    = false;
  interrupts();

  if (periode_us == 0) return vitesse_periode_ms;
  float tau_s = (float)periode_us / 1000000.0f;
  float v = PAS_M / tau_s;
  if (v > 10.0f) return vitesse_periode_ms;
  if (vitesse_periode_ms == 0.0f) {
    vitesse_periode_ms = v;
  } else {
    vitesse_periode_ms = (0.3f * v) + (0.7f * vitesse_periode_ms);
  }
  return vitesse_periode_ms;
}

void appliquerCommande(float cmd) {
  int pwm = (int)constrain(cmd, -255.0f, 255.0f);
  if (pwm >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, (uint8_t)pwm);
  } else {
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, (uint8_t)(-pwm));
  }
}

/* ===========================================================
 * calculerCommandeMoteur() - MODIFIÉE
 * =========================================================== */
float calculerCommandeMoteur(float commandeCorrecteur, float dt_s, bool urgence, bool interdireKick, uint32_t now) {
  // ── Urgence : rampe désactivée, réponse immédiate ──
  if (urgence) {
    kickActif = false;
    commandeLisseePWM = commandeCorrecteur;
    return commandeCorrecteur;
  }

  // Sécurité : Si le kick est interdit (obstacle trop proche), on coupe immédiatement un kick en cours
  if (interdireKick) {
    kickActif = false;
  }

  // ── Kick-start en cours ─────────────────────────────────────
  if (kickActif) {
    if ((now - kickDebutMs) < KICK_DUREE_MS && vitesse_periode_ms < VITESSE_QUASI_NULLE_MS) {
      commandeLisseePWM = KICK_PWM;
      return (float)KICK_PWM;
    } else {
      // Kick terminé : on initialise la rampe au feedforward,
      // pas à KICK_PWM (190). Ainsi la rampe n'a qu'à aller de
      // 190 vers ~198 (quelques PWM), pas de chute brutale vers 0.
      kickActif = false;
      commandeLisseePWM = FEEDFORWARD_PWM;
    }
  }

  // Déclenchement d'un NOUVEAU kick : Uniquement si non interdit par la sécurité !
  if (!interdireKick && commandeCorrecteur > COMMANDE_MIN_DEMARRAGE && vitesse_periode_ms < VITESSE_QUASI_NULLE_MS) {
    kickActif   = true;
    kickDebutMs = now;
    commandeLisseePWM = KICK_PWM;
    return (float)KICK_PWM;
  }

  // Rampe de lissage : la commande réelle ne peut varier que de deltaMax par cycle.
  // On ne rajoute PAS d'offset minimal ici — c'est le kick-start qui gère le
  // démarrage. Ajouter 150 PWM à toute commande < 150 empêche toute régulation :
  // le moteur ne peut plus jamais ralentir progressivement.
  float commandeCible = constrain(commandeCorrecteur, 0.0f, 255.0f);
  float deltaMax      = RAMPE_PWM_PAR_S * dt_s;
  float delta         = commandeCible - commandeLisseePWM;
  delta               = constrain(delta, -deltaMax, deltaMax);
  commandeLisseePWM  += delta;

  return constrain(commandeLisseePWM, 0.0f, 255.0f);
}

float calculerCorrecteur(float erreur, float dt_s, bool reinitialiserDerivee) {
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

  // ── Terme dérivé ─────────────────────────────────────────
  float u_D = 0.0f;
#if USE_D
  float derivee = 0.0f;
  if (dt_s > 0.0f && !reinitialiserDerivee) {
    derivee = (erreur - erreurPrecedente) / dt_s;
  }
  u_D = Kd_VAL * derivee;
#endif

  erreurPrecedente = erreur;

  // ── Feedforward + PID ────────────────────────────────────
  // Le feedforward U_FF fournit la commande de base nécessaire
  // pour maintenir la vitesse de consigne sans erreur.
  // Le PID ne corrige que les écarts autour de ce point de
  // fonctionnement. Sans FF, le PID seul calcule u→0 quand
  // e→0, ce qui fait décélérer le moteur et redéclenche le kick.
  //
  // On applique le FF uniquement en mode vitesse (sans obstacle).
  // En mode distance, le FF n'a pas de sens physique direct :
  // on laisse le PID travailler seul (les gains sont différents).
  return u_P + u_I + u_D;
  // Note : le FF est ajouté DANS calculerCommandeMoteur pour
  // pouvoir l'activer/désactiver selon le mode (vitesse vs distance).
}

void initSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD : échec initialisation"));
    sdOk = false;
    return;
  }
  char filename[13];
  uint8_t n = 0;
  do {
    snprintf(filename, sizeof(filename), "%s%03d.CSV", MODE_STR, n);
    n++;
  } while (SD.exists(filename) && n < 255);

  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.print(F("SD : impossible d'ouvrir le fichier -> "));
    Serial.println(filename);
    sdOk = false;
    return;
  }
  logFile.print(F("#correcteur="));   logFile.print(MODE_STR);
  logFile.print(F(",Kp="));           logFile.print(Kp_VAL, 1);
  logFile.print(F(",Ki="));           logFile.print(Ki_VAL, 1);
  logFile.print(F(",Kd="));           logFile.print(Kd_VAL, 1);
  logFile.print(F(",FF="));           logFile.println(FEEDFORWARD_PWM, 1);
  logFile.println(F("t_s,vitesse_freq_ms,vitesse_periode_ms,consigne_vitesse_ms,distance_m,consigne_dist_m,erreur_m,commande_pwm"));
  logFile.flush();
  strncpy(logFilename, filename, sizeof(logFilename));
  logFilename[sizeof(logFilename) - 1] = '\0';
  logLinesSinceFlush = 0;
  sdOk = true;
  Serial.print(F("SD : fichier ouvert -> "));
  Serial.println(filename);
}

void fermerLogSD() {
  if (sdOk && logFile) {
    logFile.flush();
    logFile.close();
    Serial.print(F("SD : fichier ferme -> "));
    Serial.println(logFilename);
  }
  sdOk = false;
}

void ecrireLog(float t_s, float dist_m, float erreur_m) {
  if (!sdOk) return;
  logFile.print(t_s,          3); logFile.print(',');
  logFile.print(vitesse_ms,        4); logFile.print(',');
  logFile.print(vitesse_periode_ms,4); logFile.print(',');
  logFile.print(VITESSE_CIBLE_MS,  4); logFile.print(',');
  logFile.print(dist_m,       4); logFile.print(',');
  logFile.print(DIST_CONSIGNE_M, 4); logFile.print(',');
  logFile.print(erreur_m,     4); logFile.print(',');
  logFile.println(derniereCommande, 1);

  logLinesSinceFlush++;
  if (logLinesSinceFlush >= LOG_FLUSH_EVERY_N_LINES) {
    logFile.flush();
    logLinesSinceFlush = 0;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.print(F("=== AAC TIPE — correcteur : "));
  Serial.print(MODE_STR);
  Serial.println(F(" ==="));

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  digitalWrite(TRIG_PIN, LOW);

  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonLastRawState = digitalRead(BUTTON_PIN);
  buttonStableState  = buttonLastRawState;

  pinMode(ENB_PIN,  OUTPUT);
  pinMode(IN3_PIN,  OUTPUT);
  pinMode(IN4_PIN,  OUTPUT);
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  initSD();

  lastControlTime = millis();
  lastLogTime     = millis();
}

/* ===========================================================
 * LOOP - RESTRUCTURÉE ET CORRIGÉE
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
    vitesse_ms = calculerVitesse(dt_ms);
    vitesse_periode_ms = calculerVitessePeriode();
    float dist_m = lireDistance();

    // ── 2. Logique des Modes & Sécurités ───────────────────
    float erreur       = 0.0f;
    float commande     = 0.0f;
    bool  urgence      = false;
    bool  sansObstacle = false;
    bool  interdireKick = false;

    static uint8_t ancienMode = 0; 
    uint8_t modeActuel = 0;
    static float vitessePrecedente = 0.0f;
    bool reinitialiserDerivee = false;

    // Détermination stricte du mode
    if (dist_m > 0.0f && dist_m < DIST_URGENCE_M + 0.5f * vitesse_periode_ms) {
      modeActuel = 1; // Urgence
    } else if (dist_m < 0.0f || dist_m >= DIST_MAX_OBSTACLE_M) {
      modeActuel = 2; // Vitesse
    } else {
      modeActuel = 3; // Distance
    }

    // SÉCURITÉ GÉOMÉTRIQUE : Interdire le kick si obstacle plus proche que la consigne
    if (dist_m > 0.0f && dist_m < DIST_CONSIGNE_M) {
      interdireKick = true;
    }

    uint8_t commandeMax = 255;

    if (dist_m > 0.0f && dist_m < DIST_CONSIGNE_M + 0.05f) {
      commandeMax = 180;
    }

    // RESET DES TRANSITIONS (Anti-windup inter-modes & Anti-pic D)
    if (modeActuel != ancienMode) {
      reinitialiserDerivee = true;
      integrale = 0.0f;          // Reset l'accumulateur pour ne pas injecter l'ancien mode
      erreurPrecedente = 0.0f;   // Empêche la fausse dérivation géante
    }

    // Exécution du mode sélectionné
    if (modeActuel == 1) {
      integrale        = 0.0f;
      erreurPrecedente = 0.0f;
      commande         = -255.0f; 
      erreur           = dist_m - DIST_CONSIGNE_M;
      urgence          = true;
    } 
    else if (modeActuel == 2) {
      sansObstacle = true;
      erreur   = VITESSE_CIBLE_MS - vitesse_periode_ms;
      if (vitessePrecedente == 0.0f && vitesse_periode_ms > 0.0f) {
        reinitialiserDerivee = true;
      }
      // Feedforward + PID :
      // Le FF donne la commande d'équilibre, le PID corrige les écarts.
      // Sans FF : PID→0 quand e→0 → moteur décélère → kick infini.
      float correction_pid = calculerCorrecteur(erreur, dt_s, reinitialiserDerivee);
      commande = FEEDFORWARD_PWM + correction_pid;
    } 
    else {
      erreur   = dist_m - DIST_CONSIGNE_M;
      commande = calculerCorrecteur(erreur, dt_s, reinitialiserDerivee); // Variable passée !
    }

    ancienMode = modeActuel;
    vitessePrecedente = vitesse_periode_ms;

    // Calcul de la commande finale avec le flag de blocage du kick
    commande = calculerCommandeMoteur(commande, dt_s, urgence, interdireKick, now);
    commande = constrain(commande, -commandeMax, commandeMax);
    derniereCommande = commande;
    appliquerCommande(commande);

    // ── 3. Log CSV (toutes les 100 ms) ─────────────────────
    if (now - lastLogTime >= LOG_PERIOD_MS) {
      lastLogTime = now;
      float t_s   = (float)now / 1000.0f;
      float dist_log = sansObstacle ? -1.0f : dist_m;
      ecrireLog(t_s, dist_log, erreur);
    }

    // ── 4. Debug port série ─────────────────────────────────
    Serial.print(F("[")); Serial.print(MODE_STR);  Serial.print(F("] "));
    Serial.print(F("v_freq="));    Serial.print(vitesse_ms,        3); Serial.print(F(" m/s"));
    Serial.print(F(" | v_per="));  Serial.print(vitesse_periode_ms,3); Serial.print(F(" m/s"));
    if (!sansObstacle) {
      Serial.print(F(" | d=")); Serial.print(dist_m, 3); Serial.print(F(" m"));
    } else {
      Serial.print(F(" | (pas d'obstacle)"));
    }
    if (urgence) Serial.print(F(" !! URGENCE !!"));
    Serial.print(F(" | cmd=")); Serial.print(commande, 1);
    Serial.print(F(" | kick=")); Serial.println(kickActif ? F("OUI") : F("non"));
  }
}
