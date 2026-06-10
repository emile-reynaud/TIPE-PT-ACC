/*
 * ============================================================
 *  RÉGULATEUR DE VITESSE ADAPTATIF (AAC) — TIPE CPGE PT
 *  Version 5 — sélection du correcteur par #define
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
 *  │  Décommentez UNE SEULE ligne parmi les quatre.           │
 *  │  Les tokens CORR_P/CORR_PI etc. évitent la collision     │
 *  │  avec la macro PI d'Arduino (définie comme float).       │
 *  └──────────────────────────────────────────────────────────┘
 */

// Valeurs entières pour éviter la collision avec la macro PI d'Arduino
// (définie dans Common.h comme 3.141592 — incompatible avec #if PI == ...)
#define CORR_P   1
#define CORR_PI  2
#define CORR_PD  3
#define CORR_PID 4

// ── Décommentez UNE SEULE ligne ──
#define CORRECTEUR CORR_P
// #define CORRECTEUR CORR_PI
// #define CORRECTEUR CORR_PD
// #define CORRECTEUR CORR_PID

/*
 *  ┌─ RÉGLAGE DES GAINS ─────────────────────────────────────┐
 *  │  Kp : gain proportionnel (toujours actif)                │
 *  │  Ki : gain intégral     (actif si CORR_PI ou CORR_PID)   │
 *  │  Kd : gain dérivé       (actif si CORR_PD ou CORR_PID)   │
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
#define BUTTON_PIN  8       // Bouton START/STOP (broche libre)

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
const uint16_t DEBOUNCE_MS       = 30;   // anti-rebond du bouton

/* ─── Saturation de la commande PWM ─────────────────────── */
// Le L298N accepte 0-255 sur ENB
// On borne la commande entre -255 (freinage max) et +255 (accél. max)

/* ─── Aide progressive au démarrage ────────────────────────
 * Problème : un moteur + L298N ne démarre souvent pas avec un
 * PWM faible. Au lieu d'imposer brutalement un minimum fixe
 * (ex : 120), on détecte que la vitesse reste quasi nulle alors
 * que le correcteur demande d'avancer, puis on ajoute petit à
 * petit du PWM. Dès que la roue tourne, cette aide redescend.
 */
const float VITESSE_QUASI_NULLE_MS = 0.03f;  // sous 3 cm/s, on considère que la voiture est bloquée
const float COMMANDE_MIN_DEMARRAGE = 5.0f;   // on n'aide que si le correcteur demande vraiment d'avancer
const float AIDE_PWM_MAX           = 140.0f; // limite supérieure de l'aide au décollage
const float AIDE_PWM_MONTEE_S      = 80.0f;  // montée de l'aide : +80 PWM par seconde si vitesse = 0
const float AIDE_PWM_DESCENTE_S    = 180.0f; // descente plus rapide quand la roue tourne

/* ─── Lissage de commande ──────────────────────────────────
 * Évite les alternances brutales 0/120/0. La commande réellement
 * envoyée au moteur ne peut varier que d'une quantité limitée
 * à chaque période de régulation.
 */
const float RAMPE_PWM_PAR_S        = 350.0f; // variation max du PWM par seconde

/* ===========================================================
 *   MACROS INTERNES — ne pas modifier
 *
 *   On utilise des comparaisons entières (#if CORRECTEUR == CORR_PI)
 *   pour éviter toute collision avec la macro PI d'Arduino (float).
 *   Le nom lisible (pour CSV et Serial) est défini séparément
 *   comme chaîne de caractères littérale.
 * =========================================================== */

// Drapeaux booléens : activent ou désactivent les termes I et D
// à la compilation (le code mort est supprimé du binaire)
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
#else  // CORR_P par défaut
  #define USE_I    0
  #define USE_D    0
  #define MODE_STR "P"
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

// États de l'aide au démarrage et du lissage
float aideDemarragePWM = 0.0f;
float commandeLisseePWM = 0.0f;

// Objet fichier SD
File logFile;
bool sdOk = false;

// ─── Bouton START/STOP ─────────────────────────────────────
// Sécurité : le programme démarre arrêté. Un appui lance, l'appui suivant arrête.
bool programmeActif = false;

// Avec un bouton câblé entre D8 et GND, INPUT_PULLUP donne : relâché=HIGH, appuyé=LOW.
const uint8_t BUTTON_ACTIVE_LEVEL = LOW;

// Variables d'anti-rebond
int buttonLastRawState    = HIGH;
int buttonStableState     = HIGH;
uint32_t lastButtonChangeTime = 0;



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


// Prototype utile si le fichier est compilé hors préprocesseur Arduino.
void appliquerCommande(float cmd);
float calculerCommandeMoteur(float commandeCorrecteur, float dt_s, bool urgence);

/* ===========================================================
 *   arreterProgramme()
 *   Met le moteur à l'arrêt et remet à zéro les états du
 *   correcteur pour repartir proprement au prochain START.
 * =========================================================== */
void arreterProgramme() {
  appliquerCommande(0.0f);
  derniereCommande = 0.0f;

  integrale        = 0.0f;
  erreurPrecedente = 0.0f;
  aideDemarragePWM = 0.0f;
  commandeLisseePWM = 0.0f;

  // On remet aussi les compteurs de vitesse à jour pour éviter
  // un saut artificiel au redémarrage.
  noInterrupts();
  lastHallCount = hallPulseCount;
  interrupts();
}


/* ===========================================================
 *   gererBoutonStartStop()
 *   Détecte un appui stable sur le bouton et bascule l'état :
 *   STOP -> START, puis START -> STOP.
 * =========================================================== */
void gererBoutonStartStop(uint32_t now) {
  int rawState = digitalRead(BUTTON_PIN);

  // Détection d'un changement brut : on relance la temporisation anti-rebond.
  if (rawState != buttonLastRawState) {
    buttonLastRawState = rawState;
    lastButtonChangeTime = now;
  }

  // Si l'état est stable depuis DEBOUNCE_MS, on le valide.
  if ((now - lastButtonChangeTime) >= DEBOUNCE_MS && rawState != buttonStableState) {
    buttonStableState = rawState;

    // On ne bascule qu'au moment de l'appui, pas au relâchement.
    if (buttonStableState == BUTTON_ACTIVE_LEVEL) {
      programmeActif = !programmeActif;

      if (programmeActif) {
        Serial.println(F("Bouton : START"));
        // Horodatages remis à maintenant pour éviter un grand dt au démarrage.
        lastControlTime = now;
        lastLogTime     = now;
      } else {
        Serial.println(F("Bouton : STOP"));
        arreterProgramme();
      }
    }
  }
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
 *   calculerCommandeMoteur()
 *   Combine trois choses :
 *   1) la commande du correcteur P/PI/PD/PID ;
 *   2) une aide progressive si la voiture ne décolle pas ;
 *   3) une rampe de lissage pour éviter les à-coups.
 *
 *   L'aide n'est PAS une commande constante : elle augmente
 *   seulement si la vitesse reste quasi nulle alors que le
 *   correcteur demande d'avancer. Dès que la roue tourne, elle
 *   redescend automatiquement.
 * =========================================================== */
float calculerCommandeMoteur(float commandeCorrecteur, float dt_s, bool urgence) {
  // En urgence, on ne lisse pas : priorité au freinage immédiat.
  if (urgence || commandeCorrecteur < 0.0f) {
    aideDemarragePWM = 0.0f;
    commandeLisseePWM = commandeCorrecteur;
    return commandeCorrecteur;
  }

  // Si le correcteur veut avancer mais que la roue ne tourne pas,
  // on augmente progressivement l'aide au décollage.
  if (commandeCorrecteur > COMMANDE_MIN_DEMARRAGE && vitesse_ms < VITESSE_QUASI_NULLE_MS) {
    aideDemarragePWM += AIDE_PWM_MONTEE_S * dt_s;
    aideDemarragePWM = constrain(aideDemarragePWM, 0.0f, AIDE_PWM_MAX);
  } else {
    // Dès que ça roule, l'aide disparaît progressivement pour
    // laisser le correcteur reprendre la main.
    aideDemarragePWM -= AIDE_PWM_DESCENTE_S * dt_s;
    aideDemarragePWM = constrain(aideDemarragePWM, 0.0f, AIDE_PWM_MAX);
  }

  // Commande cible = correcteur + aide éventuelle.
  float commandeCible = commandeCorrecteur + aideDemarragePWM;
  commandeCible = constrain(commandeCible, 0.0f, 255.0f);

  // Rampe : on limite la variation d'un cycle à l'autre.
  float deltaMax = RAMPE_PWM_PAR_S * dt_s;
  float delta = commandeCible - commandeLisseePWM;
  delta = constrain(delta, -deltaMax, deltaMax);
  commandeLisseePWM += delta;

  return constrain(commandeLisseePWM, 0.0f, 255.0f);
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
 *   │  Correcteur P  : u(t) = Kp × e(t)                      │
 *   │                                                         │
 *   │  Correcteur PI : u(t) = Kp × e(t)                      │
 *   │                       + Ki × ∫e(τ)dτ                   │
 *   │                                                         │
 *   │  Correcteur PD : u(t) = Kp × e(t)                      │
 *   │                       + Kd × de(t)/dt                  │
 *   │                                                         │
 *   │  Correcteur PID: u(t) = Kp × e(t)                      │
 *   │                       + Ki × ∫e(τ)dτ                   │
 *   │                       + Kd × de(t)/dt                  │
 *   │                                                         │
 *   │  Discrétisation (Euler) :                               │
 *   │    Intégrale ≈ Σ e(k) × dt    (rectangle à gauche)     │
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
  char filename[20];
  uint8_t n = 0;
  do {
    // snprintf écrit dans filename le nom formaté
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

  // ── En-tête CSV ───────────────────────────────────────────
  // Format pensé pour pandas / matplotlib :
  //   - noms de colonnes sans espace ni accent
  //   - séparateur virgule
  //   - unités dans le nom de colonne entre crochets
  //   - temps en secondes (float) plutôt qu'en ms
  //     pour faciliter les tracés temporels
  logFile.println(F("t_s,correcteur,vitesse_ms,consigne_vitesse_ms,"
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

  // ── Bouton START/STOP ─────────────────────────────────────
  // Pour un bouton simple : connecter une borne à D8 et l'autre à GND.
  // Pour un module 3 broches : GND → GND, OUT/SIG → D8, VCC → 5V.
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  buttonLastRawState = digitalRead(BUTTON_PIN);
  buttonStableState  = buttonLastRawState;

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

  // ── Bouton START/STOP ────────────────────────────────────
  gererBoutonStartStop(now);

  // Tant que le programme n'est pas lancé, on garde le moteur arrêté.
  if (!programmeActif) {
    appliquerCommande(0.0f);
    lastControlTime = now;
    lastLogTime     = now;
    return;
  }

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

    // Transformation finale de la commande :
    // correcteur + aide progressive au démarrage + lissage.
    commande = calculerCommandeMoteur(commande, dt_s, urgence);

    // Mémoriser la commande réellement envoyée au moteur pour le log
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
    Serial.print(F(" | cmd=")); Serial.print(commande, 1);
    Serial.print(F(" | aide=")); Serial.println(aideDemarragePWM, 1);
  }
}
