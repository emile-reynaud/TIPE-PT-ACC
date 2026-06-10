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

#define Kp_VAL   35.0f   // Gain proportionnel
#define Ki_VAL    2.0f   // Réduit de 5 → 2 : évite les oscillations lentes de l'intégrale
#define Kd_VAL    0.5f   // Gain dérivé (faible : 3 aimants = mesure bruitée)

/* ─── Modèle moteur identifié ────────────────────────────────
 * Ces trois valeurs viennent directement du programme
 * d'identification (identification_moteur.ino).
 * Ce sont les SEULS paramètres à mettre à jour si tu changes
 * de moteur ou si tu réidentifies le système.
 *
 *   K_MOTEUR        : vitesse de régime à commande normalisée = 1,0
 *                     unité : m/s
 *   SEUIL_DEMARRAGE : commande normalisée minimale pour démarrer
 *                     unité : sans dimension [0,0 – 1,0]
 *                     en PWM : SEUIL_DEMARRAGE × 255
 */
const float K_MOTEUR        = 9.5f;    // m/s — recalibré depuis les logs (batterie chargée)
                                        // Ancienne valeur : 5.7669 (batterie déchargée)
                                        // Méthode : K = v_mesurée / (PWM/255 - seuil)
                                        // Exemple log PID110 ligne 13 :
                                        //   v=0.605 m/s, PWM=195.6 → K=0.605/(0.767-0.706)≈9.9
const float SEUIL_DEMARRAGE = 0.706f;  // normalisé — inchangé

/* ─── Feedforward calculé automatiquement ────────────────────
 * U_FF = (Vc / K_moteur + seuil) × 255
 *
 * Cette variable est calculée une seule fois au démarrage
 * dans setup() à partir de VITESSE_CIBLE_MS et du modèle.
 * Tu n'as plus jamais besoin de la modifier manuellement :
 * changer VITESSE_CIBLE_MS suffit.
 *
 * Elle est déclarée ici comme variable (pas const) car elle
 * est initialisée dans setup(), pas à la compilation.
 * Elle ne change plus ensuite pendant l'exécution.
 *
 * Note pour le TIPE : U_FF est la partie "modèle inverse" du
 * régulateur. U_FF + PID = architecture 2-DOF (two degrees of
 * freedom), standard industriel. Le FF compense les non-
 * linéarités du moteur (seuil de démarrage), le PID compense
 * les perturbations et les erreurs de modèle.
 */
float FEEDFORWARD_PWM = 0.0f;  // initialisé dans setup()

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
const float DIST_URGENCE_M     = 0.10f; // freinage d'urgence si obstacle < 10 cm
// DIST_MAX_OBSTACLE_M : au-delà, on considère qu'il n'y a pas d'obstacle
// et on régule en vitesse.
// ATTENTION : ne pas mettre trop grand. Le HC-SR04 détecte les murs de la
// pièce. Si le mur est à 1,8 m et que ce seuil est à 2,0 m, le système
// sera en mode "distance" en permanence et oscillera.
// Réglage : mesurer la distance du mur le plus proche dans l'axe de déplacement
// et mettre ce seuil à environ 80% de cette distance.
// Ici on met 1,20 m : au-delà, c'est "pas d'obstacle".
const float DIST_MAX_OBSTACLE_M= 1.20f;

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

/* ─── Zone morte de commande ─────────────────────────────────
 * Le moteur ne produit aucun couple entre 0 et SEUIL_DEMARRAGE×255
 * (≈180 PWM). Une commande dans cette plage met le moteur en roue
 * quasi-libre : il ne freine pas, ne pousse pas.
 * Résultat : la voiture continue sur son élan à pleine vitesse
 * pendant que l'intégrale accumule une erreur négative énorme.
 * Solution : toute commande positive < PWM_ZONE_MORTE est ramenée
 * à 0 (roue libre explicite). Le freinage actif (cmd < 0) est
 * toujours appliqué sans zone morte.
 */
const float PWM_ZONE_MORTE  = 178.0f;  // légèrement sous le seuil (180) pour hysteresis
/* ─── Lissage de commande (rampe asymétrique) ────────────────
 * Montée lente : évite les à-coups au démarrage.
 * Descente rapide : permet de corriger immédiatement quand la
 * vitesse dépasse la consigne, sans que la rampe retarde le
 * freinage et laisse l'intégrale s'emballer.
 */
const float RAMPE_MONTEE_PAR_S   = 350.0f;  // PWM/s en accélération
const float RAMPE_DESCENTE_PAR_S = 700.0f;  // PWM/s en décélération (2× plus rapide)

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
void ecrireLog(float t_s, float dist_m, float erreur_m, uint8_t modeActuel);
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

  // Variation physiquement possible en un cycle de CONTROL_PERIOD_MS.
  // Au-delà, la mesure est un parasite (rebond mécanique sur aimant,
  // vibration) et on la rejette en gardant la dernière valeur valide.
  // Calcul : accélération max ≈ K/τ × ΔuMax × dt = 9.5/0.117 × 1.0 × 0.05
  // ≈ 4 m/s théorique. Avec frottements, jamais plus de 0.8 m/s/cycle en pratique.
  static const float DELTA_V_MAX = 0.8f;  // m/s de variation max par cycle

  noInterrupts();
  uint32_t derniereImpulsion_us = hallLastPulseTime_us;
  interrupts();
  if (derniereImpulsion_us == 0) return 0.0f;

  uint32_t silenceActuel_us = micros() - derniereImpulsion_us;
  if (silenceActuel_us > HALL_TIMEOUT_US) {
    noInterrupts();
    hallNouvelleImpulsion = false;
    interrupts();
    vitesse_periode_ms = 0.0f;
    return 0.0f;
  }

  noInterrupts();
  uint32_t periode_us   = hallPeriode_us;
  hallNouvelleImpulsion = false;
  interrupts();

  if (periode_us == 0) return vitesse_periode_ms;

  float tau_s = (float)periode_us / 1000000.0f;
  float v_brute = PAS_M / tau_s;

  // Rejet des parasites par variation maximale :
  // Si le saut par rapport à la dernière valeur valide dépasse DELTA_V_MAX,
  // c'est physiquement impossible → rebond ou parasite → on ignore.
  if (vitesse_periode_ms > 0.0f
      && fabsf(v_brute - vitesse_periode_ms) > DELTA_V_MAX) {
    return vitesse_periode_ms;  // mesure rejetée, on conserve l'ancienne
  }

  // Filtre exponentiel (lissage) :
  // α=0.4 : plus réactif qu'avant (0.3) pour ne pas masquer
  // les vraies accélérations après validation du saut.
  if (vitesse_periode_ms == 0.0f) {
    vitesse_periode_ms = v_brute;
  } else {
    vitesse_periode_ms = 0.4f * v_brute + 0.6f * vitesse_periode_ms;
  }
  return vitesse_periode_ms;
}

void appliquerCommande(float cmd) {
  // Zone morte : entre 0 et PWM_ZONE_MORTE, le moteur est en roue libre
  // (ne produit ni couple moteur, ni freinage). On force explicitement à 0
  // pour éviter que la voiture continue sur son élan sans régulation effective
  // pendant que l'intégrale accumule une grosse erreur négative.
  // Le freinage actif (cmd < 0) est toujours appliqué sans zone morte.
  if (cmd > 0.0f && cmd < PWM_ZONE_MORTE) {
    cmd = 0.0f;
  }
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

  // Variables statiques : persistent entre les appels, initialisées une seule fois.
  // Déclarées EN TÊTE de fonction pour être visibles dans tous les blocs.
  static uint32_t kickFinMs    = 0;
  const  uint16_t KICK_GRACE_MS = 300;  // ms de grâce après un kick

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
      kickActif = false;
      kickFinMs = now;                   // mémoriser la fin du kick
      commandeLisseePWM = FEEDFORWARD_PWM;
    }
  }

  // Déclenchement d'un NOUVEAU kick.
  // On exige que LES DEUX mesures de vitesse soient quasi nulles ET
  // que le délai de grâce après le dernier kick soit écoulé.
  bool graceActive = (now - kickFinMs) < KICK_GRACE_MS;

  bool roueArretee = (vitesse_periode_ms < VITESSE_QUASI_NULLE_MS)
                  && (vitesse_ms         < VITESSE_QUASI_NULLE_MS);

  if (!interdireKick && !graceActive
      && commandeCorrecteur > COMMANDE_MIN_DEMARRAGE
      && roueArretee) {
    kickActif   = true;
    kickDebutMs = now;
    commandeLisseePWM = KICK_PWM;
    return (float)KICK_PWM;
  }

  // Rampe asymétrique : descente 2× plus rapide que montée
  float commandeCible = constrain(commandeCorrecteur, 0.0f, 255.0f);
  float deltaMax = (commandeCible < commandeLisseePWM)
                   ? RAMPE_DESCENTE_PAR_S * dt_s   // descente rapide
                   : RAMPE_MONTEE_PAR_S   * dt_s;  // montée lente
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
  logFile.print(F(",FF="));           logFile.print(FEEDFORWARD_PWM, 1);
  logFile.print(F(",Vc="));           logFile.println(VITESSE_CIBLE_MS, 2);
  logFile.println(F("t_s,mode,vitesse_freq_ms,vitesse_periode_ms,consigne_vitesse_ms,distance_m,consigne_dist_m,erreur_m,commande_pwm"));
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

void ecrireLog(float t_s, float dist_m, float erreur_m, uint8_t modeActuel) {
  if (!sdOk) return;
  logFile.print(t_s,               3); logFile.print(',');
  // Mode : 1=urgence, 2=vitesse, 3=distance
  logFile.print(modeActuel);           logFile.print(',');
  logFile.print(vitesse_ms,        4); logFile.print(',');
  logFile.print(vitesse_periode_ms,4); logFile.print(',');
  logFile.print(VITESSE_CIBLE_MS,  4); logFile.print(',');
  logFile.print(dist_m,            4); logFile.print(',');
  logFile.print(DIST_CONSIGNE_M,   4); logFile.print(',');
  logFile.print(erreur_m,          4); logFile.print(',');
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

  // ── Calcul du feedforward ─────────────────────────────────
  // Formule : U_FF = (Vc / K_moteur + seuil) × 255
  // On sature entre 0 et 250 PWM :
  //   - en dessous de 0    : ne peut pas arriver (Vc > 0 et seuil > 0)
  //   - au-dessus de 250   : on garde 5 PWM de marge pour que le PID
  //                          puisse encore accélérer si besoin
  FEEDFORWARD_PWM = (VITESSE_CIBLE_MS / K_MOTEUR + SEUIL_DEMARRAGE) * 255.0f;
  FEEDFORWARD_PWM = constrain(FEEDFORWARD_PWM, 0.0f, 250.0f);

  // Vérification de cohérence : si U_FF dépasse 255, la consigne
  // est trop haute pour ce moteur → on le signale clairement.
  float uFF_brut = (VITESSE_CIBLE_MS / K_MOTEUR + SEUIL_DEMARRAGE) * 255.0f;
  Serial.print(F("Vitesse cible   = ")); Serial.print(VITESSE_CIBLE_MS, 2);
  Serial.println(F(" m/s"));
  Serial.print(F("Feedforward U_FF= ")); Serial.print(FEEDFORWARD_PWM, 1);
  Serial.print(F(" PWM  (brut="));       Serial.print(uFF_brut, 1);
  Serial.println(F(")"));
  if (uFF_brut > 252.0f) {
    Serial.println(F("ATTENTION : vitesse cible proche ou au-dessus du maximum moteur !"));
    Serial.print(F("  Vitesse max théorique = ")); 
    Serial.print((1.0f - SEUIL_DEMARRAGE) * K_MOTEUR, 2);
    Serial.println(F(" m/s"));
  }
  Serial.print(F("KICK_PWM        = ")); Serial.println(KICK_PWM);

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

    // Mémoire de la dernière distance valide.
    // Le HC-SR04 renvoie parfois -1 (timeout ou hors plage) même quand
    // un obstacle est bien présent. Sans mémoire, une seule mesure invalide
    // fait basculer en mode vitesse pour un cycle, puis retour en mode distance
    // à la mesure suivante → oscillation de mode toutes les 50 ms → intégrale
    // remise à zéro à chaque transition → régulation impossible.
    // Solution : si dist_m == -1 mais que la dernière mesure valide était
    // inférieure à DIST_MAX_OBSTACLE_M, on conserve le mode distance.
    // La mémoire expire après OBSTACLE_MEMORY_MS sans mesure valide.
    static float    derniereDist_m     = -1.0f;
    static uint32_t dernierDistTime_ms = 0;
    const  uint16_t OBSTACLE_MEMORY_MS = 300;  // 300 ms = 6 cycles de 50 ms

    // Mise à jour de la mémoire si la mesure est valide
    if (dist_m >= 0.0f) {
      derniereDist_m     = dist_m;
      dernierDistTime_ms = now;
    }

    // Distance effective utilisée pour la logique de mode :
    // si mesure invalide mais obstacle récemment vu → on garde l'ancienne
    float dist_effective = dist_m;
    if (dist_m < 0.0f
        && derniereDist_m >= 0.0f
        && derniereDist_m < DIST_MAX_OBSTACLE_M
        && (now - dernierDistTime_ms) < OBSTACLE_MEMORY_MS) {
      dist_effective = derniereDist_m;
    }

    // Détermination du mode avec dist_effective (pas dist_m brute)
    if (dist_effective > 0.0f && dist_effective < DIST_URGENCE_M + 0.5f * vitesse_periode_ms) {
      modeActuel = 1; // Urgence
    } else if (dist_effective < 0.0f || dist_effective >= DIST_MAX_OBSTACLE_M) {
      modeActuel = 2; // Vitesse — pas d'obstacle
    } else {
      modeActuel = 3; // Distance — obstacle dans la zone de régulation
    }

    // SÉCURITÉ GÉOMÉTRIQUE : Interdire le kick si obstacle plus proche que la consigne
    if (dist_effective > 0.0f && dist_effective < DIST_CONSIGNE_M) {
      interdireKick = true;
    }
    // Fix 2 : interdire le kick en mode distance.
    // En mode distance, le correcteur peut demander d'avancer (erreur > 0)
    // alors que la voiture est à 57 cm d'un obstacle. Sans cette interdiction,
    // le kick lance la voiture à pleine puissance vers l'obstacle → freinage
    // d'urgence → cycle dangereux.
    if (modeActuel == 3) {
      interdireKick = true;
    }

    uint8_t commandeMax = 255;
    if (dist_effective > 0.0f && dist_effective < DIST_CONSIGNE_M + 0.05f) {
      commandeMax = 180;
    }

    // RESET DES TRANSITIONS (Anti-windup inter-modes & Anti-pic D)
    if (modeActuel != ancienMode) {
      reinitialiserDerivee = true;
      integrale = 0.0f;
      erreurPrecedente = 0.0f;
    }

    if (modeActuel == 1) {
      integrale        = 0.0f;
      erreurPrecedente = 0.0f;
      commande         = -255.0f;
      erreur           = dist_effective - DIST_CONSIGNE_M;
      urgence          = true;
    }
    else if (modeActuel == 2) {
      sansObstacle = true;
      erreur   = VITESSE_CIBLE_MS - vitesse_periode_ms;
      if (vitessePrecedente == 0.0f && vitesse_periode_ms > 0.0f) {
        reinitialiserDerivee = true;
      }
      float correction_pid = calculerCorrecteur(erreur, dt_s, reinitialiserDerivee);
      commande = FEEDFORWARD_PWM + correction_pid;
    }
    else {
      erreur   = dist_effective - DIST_CONSIGNE_M;
      commande = calculerCorrecteur(erreur, dt_s, reinitialiserDerivee);
    }

    ancienMode = modeActuel;
    vitessePrecedente = vitesse_periode_ms;

    // Calcul de la commande finale avec le flag de blocage du kick
    commande = calculerCommandeMoteur(commande, dt_s, urgence, interdireKick, now);
    commande = constrain(commande, -commandeMax, commandeMax);

    // Fix 3 : anti-windup étendu — zone morte moteur.
    // Si la commande finale tombe dans la zone morte (0 < cmd < PWM_ZONE_MORTE),
    // appliquerCommande va envoyer 0 au moteur. Mais l'intégrale a été calculée
    // comme si la commande allait être effective — elle accumule une erreur qui
    // ne sera jamais corrigée. On gèle l'intégrale dans ce cas.
    // (même principe que le gel pendant le kick)
    bool commandeDansZoneMorte = (commande > 0.0f && commande < PWM_ZONE_MORTE);
    if (commandeDansZoneMorte) {
      integrale = integrale - erreur * dt_s;  // annuler l'accumulation du cycle courant
      integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);
    }

    derniereCommande = commande;
    appliquerCommande(commande);

    // ── 3. Log CSV (toutes les 100 ms) ─────────────────────
    if (now - lastLogTime >= LOG_PERIOD_MS) {
      lastLogTime = now;
      float t_s      = (float)now / 1000.0f;
      // On logue dist_effective : c'est la distance que le régulateur a réellement
      // utilisée (avec la mémoire d'obstacle). dist_m brute peut être -1 même
      // quand le régulateur est en mode distance à cause d'une mesure invalide.
      float dist_log = sansObstacle ? -1.0f : dist_effective;
      ecrireLog(t_s, dist_log, erreur, modeActuel);
    }

    // ── 4. Debug port série ─────────────────────────────────
    Serial.print(F("[")); Serial.print(MODE_STR); Serial.print(F("] "));
    // Affichage du mode actif : crucial pour diagnostiquer les comportements aberrants
    Serial.print(F("mode="));
    if      (modeActuel == 1) Serial.print(F("URGENCE "));
    else if (modeActuel == 2) Serial.print(F("VITESSE "));
    else                      Serial.print(F("DIST    "));
    Serial.print(F("| v_freq="));   Serial.print(vitesse_ms,        3); Serial.print(F(" m/s"));
    Serial.print(F(" v_per="));     Serial.print(vitesse_periode_ms,3); Serial.print(F(" m/s"));
    Serial.print(F(" | d="));       Serial.print(dist_m,            3); Serial.print(F(" m"));
    Serial.print(F(" | e="));       Serial.print(erreur,            3);
    Serial.print(F(" | cmd="));     Serial.print(commande,          1);
    Serial.print(F(" | kick="));    Serial.println(kickActif ? F("OUI") : F("non"));
  }
}
