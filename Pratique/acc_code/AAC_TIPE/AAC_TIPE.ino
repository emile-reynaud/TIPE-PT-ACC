/*
 * ============================================================
 *  RÉGULATEUR DE VITESSE ADAPTATIF (AAC) — TIPE CPGE PT
 * ============================================================
 *  Matériel :
 *   - Arduino Nano Every
 *   - Shield Arduino ASX00061 (SPI + microSD intégré)
 *   - Pont en H L298N (channel B → moteur RC390)
 *   - Capteur ultrason HC-SR04
 *   - Capteur à effet Hall + 6 aimants (roue arrière gauche)
 *   - Carte microSD (via shield, SPI)
 *
 *  Modes de régulation sélectionnables :
 *   P  → correcteur proportionnel seul
 *   PI → proportionnel + intégral
 *   PD → proportionnel + dérivé
 *   PID→ complet
 *
 *  Sélection du mode : broche A0 (potentiomètre ou ponts résistifs)
 *   0–255   → P
 *   256–511 → PI
 *   512–767 → PD
 *   768–1023→ PID
 *
 *  Le log sur microSD enregistre toutes les 100 ms :
 *   timestamp_ms, mode, vitesse_cm_s, distance_cm, commande_PWM
 * ============================================================
 */

#include <SPI.h>
#include <SD.h>

// ── Broches ────────────────────────────────────────────────
// Ultrason HC-SR04
#define TRIG_PIN    7
#define ECHO_PIN    6

// Capteur Hall (interruption)
#define HALL_PIN    2   // INT0 sur Nano Every

// Pont en H L298N — channel B
#define ENB_PIN     9   // PWM vitesse moteur  (pin PWM !)
#define IN3_PIN     4   // sens de rotation A
#define IN4_PIN     5   // sens de rotation B

// Sélection mode régulateur
#define MODE_PIN    A0

// SD chip select (shield ASX00061 utilise la broche 4 ou 10 selon config)
// Sur le MKR SD shield porté sur Nano Every via adaptateur : CS = 4
// Vérifiez votre shield — modifiez si nécessaire.
#define SD_CS_PIN   10

// ── Paramètres physiques ───────────────────────────────────
const uint8_t  NB_AIMANTS        = 6;     // aimants dans la jante
const float    PERIMETRE_CM      = 34.6f; // périmètre roue (diam ≈ 11 cm)

// ── Consignes ──────────────────────────────────────────────
const float DIST_CONSIGNE_CM    = 30.0f;  // distance de sécurité cible (cm)
const float VITESSE_CIBLE_CM_S  = 80.0f;  // vitesse de croisière (cm/s)

// ── Paramètres PID (à ajuster expérimentalement) ───────────
// Correcteur sur l'ERREUR DE DISTANCE (cm)
// Commande = PWM envoyé au moteur (0-255)
float Kp  = 2.5f;   // gain proportionnel
float Ki  = 0.8f;   // gain intégral
float Kd  = 1.2f;   // gain dérivé

// ── Variables globales ─────────────────────────────────────
volatile uint32_t hallPulseCount = 0;  // compteur d'impulsions Hall (ISR)
uint32_t lastHallCount  = 0;
uint32_t lastSpeedTime  = 0;
float    vitesse_cm_s   = 0.0f;        // vitesse mesurée

float    erreurPrecedente = 0.0f;
float    integrale        = 0.0f;

uint32_t lastControlTime  = 0;
uint32_t lastLogTime      = 0;

const uint16_t CONTROL_PERIOD_MS = 50;   // période de régulation (ms)
const uint16_t LOG_PERIOD_MS     = 100;  // période de log SD (ms)

// Saturation commande PWM
const uint8_t  PWM_MIN = 0;
const uint8_t  PWM_MAX = 255;

// Seuil distance minimale (freinage d'urgence)
const float DIST_URGENCE_CM = 10.0f;

File logFile;
bool sdOk = false;

// ─────────────────────────────────────────────────────────────
// ISR : interruption Hall
// ─────────────────────────────────────────────────────────────
void IRAM_ATTR hallISR() {
  hallPulseCount++;
}

// ─────────────────────────────────────────────────────────────
// Lecture distance ultrason (cm)
// Retourne -1 si hors plage
// ─────────────────────────────────────────────────────────────
float lireDistance() {
  // Impulsion TRIG 10 µs
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Mesure durée écho (timeout 30 ms → ~5 m maxi)
  long duree = pulseIn(ECHO_PIN, HIGH, 30000UL);
  if (duree == 0) return -1.0f;
  return duree * 0.01715f;  // cm = duree_µs × vitesse_son/2
}

// ─────────────────────────────────────────────────────────────
// Calcul vitesse (cm/s) depuis compteur Hall
// Appelé toutes les CONTROL_PERIOD_MS
// ─────────────────────────────────────────────────────────────
float calculerVitesse(uint32_t dt_ms) {
  noInterrupts();
  uint32_t count = hallPulseCount;
  interrupts();

  uint32_t delta = count - lastHallCount;
  lastHallCount  = count;

  // tours = delta / NB_AIMANTS
  // vitesse = tours × perimetre / temps
  if (dt_ms == 0) return 0.0f;
  float tours = (float)delta / (float)NB_AIMANTS;
  return tours * PERIMETRE_CM / ((float)dt_ms / 1000.0f);
}

// ─────────────────────────────────────────────────────────────
// Lecture mode régulateur via A0
// ─────────────────────────────────────────────────────────────
enum ModeRegulateur { MODE_P, MODE_PI, MODE_PD, MODE_PID };

ModeRegulateur lireMode() {
  int val = analogRead(MODE_PIN);
  if      (val < 256)  return MODE_P;
  else if (val < 512)  return MODE_PI;
  else if (val < 768)  return MODE_PD;
  else                 return MODE_PID;
}

const char* nomMode(ModeRegulateur m) {
  switch (m) {
    case MODE_P:   return "P";
    case MODE_PI:  return "PI";
    case MODE_PD:  return "PD";
    default:       return "PID";
  }
}

// ─────────────────────────────────────────────────────────────
// Calcul commande PID (ou sous-ensemble)
// erreur = distance_mesurée - distance_consigne
// commande positive → accélérer, négative → ralentir
// ─────────────────────────────────────────────────────────────
float calculerCommande(float erreur, float dt_s, ModeRegulateur mode) {
  float P_term = Kp * erreur;
  float I_term = 0.0f;
  float D_term = 0.0f;

  if (mode == MODE_PI || mode == MODE_PID) {
    integrale += erreur * dt_s;
    // Anti-windup par saturation de l'intégrale
    integrale = constrain(integrale, -50.0f, 50.0f);
    I_term = Ki * integrale;
  }

  if (mode == MODE_PD || mode == MODE_PID) {
    float derivee = (dt_s > 0.0f) ? (erreur - erreurPrecedente) / dt_s : 0.0f;
    D_term = Kd * derivee;
  }

  erreurPrecedente = erreur;
  return P_term + I_term + D_term;
}

// ─────────────────────────────────────────────────────────────
// Application commande moteur
// cmd > 0 → avancer, cmd < 0 → reculer/freiner
// ─────────────────────────────────────────────────────────────
void appliquerCommande(float cmd) {
  int pwm = (int)constrain(cmd, -255.0f, 255.0f);

  if (pwm >= 0) {
    digitalWrite(IN3_PIN, HIGH);
    digitalWrite(IN4_PIN, LOW);
    analogWrite(ENB_PIN, (uint8_t)pwm);
  } else {
    // Freinage moteur (court-circuit bobines)
    digitalWrite(IN3_PIN, LOW);
    digitalWrite(IN4_PIN, HIGH);
    analogWrite(ENB_PIN, (uint8_t)(-pwm));
  }
}

// ─────────────────────────────────────────────────────────────
// Initialisation SD et création fichier de log
// ─────────────────────────────────────────────────────────────
void initSD() {
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("SD: échec initialisation"));
    sdOk = false;
    return;
  }

  // Nom de fichier unique basé sur un compteur
  char filename[16];
  uint8_t n = 0;
  do {
    snprintf(filename, sizeof(filename), "log%03d.csv", n++);
  } while (SD.exists(filename) && n < 255);

  logFile = SD.open(filename, FILE_WRITE);
  if (!logFile) {
    Serial.println(F("SD: impossible d'ouvrir le fichier"));
    sdOk = false;
    return;
  }

  // En-tête CSV
  logFile.println(F("t_ms,mode,vitesse_cm_s,distance_cm,commande_PWM,erreur_cm"));
  logFile.flush();
  sdOk = true;
  Serial.print(F("SD: log → "));
  Serial.println(filename);
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);
  Serial.println(F("=== AAC TIPE — démarrage ==="));

  // Ultrason
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Hall
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  // Pont en H
  pinMode(ENB_PIN, OUTPUT);
  pinMode(IN3_PIN, OUTPUT);
  pinMode(IN4_PIN, OUTPUT);
  analogWrite(ENB_PIN, 0);

  // Mode
  pinMode(MODE_PIN, INPUT);

  // SD
  initSD();

  lastControlTime = millis();
  lastLogTime     = millis();
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────
void loop() {
  uint32_t now = millis();

  // ── Régulation (50 ms) ──────────────────────────────────
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    uint32_t dt_ms = now - lastControlTime;
    lastControlTime = now;
    float dt_s = dt_ms / 1000.0f;

    // Mesures
    vitesse_cm_s      = calculerVitesse(dt_ms);
    float distance_cm = lireDistance();

    ModeRegulateur mode = lireMode();

    float commande_pwm = 0.0f;

    if (distance_cm < 0.0f) {
      // Capteur ultrason hors plage → maintenir vitesse cible simple
      float erreur_v = VITESSE_CIBLE_CM_S - vitesse_cm_s;
      commande_pwm   = Kp * erreur_v;

    } else if (distance_cm < DIST_URGENCE_CM) {
      // FREINAGE D'URGENCE
      integrale    = 0.0f;
      commande_pwm = -255.0f;

    } else {
      // Régulation normale sur erreur de distance
      float erreur = distance_cm - DIST_CONSIGNE_CM;
      // erreur > 0 → trop loin → accélérer
      // erreur < 0 → trop près → freiner
      commande_pwm = calculerCommande(erreur, dt_s, mode);
    }

    appliquerCommande(commande_pwm);

    // ── Log SD (100 ms) ────────────────────────────────────
    if (sdOk && (now - lastLogTime >= LOG_PERIOD_MS)) {
      lastLogTime = now;
      float erreur_log = (distance_cm > 0) ? distance_cm - DIST_CONSIGNE_CM : 0.0f;
      logFile.print(now);             logFile.print(',');
      logFile.print(nomMode(mode));   logFile.print(',');
      logFile.print(vitesse_cm_s, 2); logFile.print(',');
      logFile.print(distance_cm, 1);  logFile.print(',');
      logFile.print(commande_pwm, 0); logFile.print(',');
      logFile.println(erreur_log, 1);
      logFile.flush();  // garantir l'écriture physique
    }

    // ── Debug série ───────────────────────────────────────
    Serial.print(nomMode(mode));
    Serial.print(F(" | v="));    Serial.print(vitesse_cm_s, 1);
    Serial.print(F(" cm/s | d="));Serial.print(distance_cm, 1);
    Serial.print(F(" cm | cmd=")); Serial.println(commande_pwm, 0);
  }
}
