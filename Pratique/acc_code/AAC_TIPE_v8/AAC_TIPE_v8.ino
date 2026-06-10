/*
 * ============================================================
 *  RÉGULATEUR DE VITESSE ADAPTATIF (AAC) — TIPE CPGE PT
 *  Version 7 — double mesure de vitesse : fréquencemétrique + période
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
#define IN3_PIN     A0      // L298N IN3  — sens rotation A (déplacé : D4 est réservé à la SD sur ASX00061)
#define IN4_PIN     5       // L298N IN4  — sens rotation B
#define SD_CS_PIN   4       // ASX00061 : D4 est le Chip Select microSD par défaut
#define BUTTON_PIN  8       // Bouton START/STOP (broche libre)

/* ─── Paramètres physiques ───────────────────────────────── */
// Roue : diamètre 83 mm → périmètre = π × 0.083 m
const int DIAMETRE_MM    = 83;
const float PERIMETRE_M   = 3.14159f * (float)DIAMETRE_MM/1000;  // ≈ 0.2608 m
const uint8_t NB_AIMANTS  = 3;                   // aimants par tour de roue

/* ─── Consignes ──────────────────────────────────────────── */
const float DIST_CONSIGNE_M    = 0.30f;  // distance de sécurité cible : 30 cm
const float VITESSE_CIBLE_MS   = 0.40f;  // vitesse de croisière : 0,40 m/s = 1,44 km/h

/* ─── Seuils de sécurité ─────────────────────────────────── */
const float DIST_URGENCE_M     = 0.10f;  // freinage d'urgence si < 10 cm
const float DIST_MAX_OBSTACLE_M= 2.00f;  // au-delà : pas d'obstacle détecté

/* ─── Anti-windup : saturation de l'intégrale ───────────── */
const float INTEGRALE_MAX = 100.0f;

/* ─── Périodes de la boucle ──────────────────────────────── */
const uint16_t CONTROL_PERIOD_MS = 50;   // régulation : toutes les 50 ms
const uint16_t LOG_PERIOD_MS     = 100;  // écriture CSV : toutes les 100 ms
const uint16_t DEBOUNCE_MS       = 30;   // anti-rebond du bouton
const uint32_t ECHO_TIMEOUT_US   = 12000UL; // timeout HC-SR04 ≈ 2 m, évite les pauses longues

/* ─── Méthode période : timeout vitesse nulle ───────────────
 * Si aucune impulsion n'arrive pendant plus de HALL_TIMEOUT_US
 * microsecondes, on considère que la roue est à l'arrêt et on
 * force vitesse_periode_ms = 0.
 * À 0,05 m/s avec p = 0,0869 m : τ = p/v ≈ 1,74 s → 1 740 000 µs.
 * On prend une marge : 2 000 000 µs (2 secondes).
 */
const uint32_t HALL_TIMEOUT_US = 2000000UL;



/* ─── Démarrage moteur (kick-start) ────────────────────────
 * Problème : un moteur brushed + L298N ne démarre pas avec un
 * PWM faible à cause des frottements statiques. Une montée
 * progressive de 80 PWM/s met ~1,5 s à atteindre un PWM utile,
 * pendant lequel le moteur chauffe sans tourner.
 *
 * Solution : dès que le correcteur demande d'avancer et que la
 * roue est à l'arrêt, on envoie une impulsion courte à PWM élevé
 * (KICK_PWM) pendant KICK_DUREE_MS, puis on redonne la main au
 * correcteur. Si la roue ne tourne toujours pas, on réessaie.
 */
const uint8_t  KICK_PWM        = 200;   // PWM de l'impulsion de démarrage (0-255)
const uint16_t KICK_DUREE_MS   = 80;    // durée de l'impulsion en ms
const float    VITESSE_QUASI_NULLE_MS = 0.03f;  // seuil "roue à l'arrêt" en m/s
const float    COMMANDE_MIN_DEMARRAGE = 5.0f;   // commande mini pour déclencher le kick

/* ─── Lissage de commande ──────────────────────────────────
 * Une fois démarré, la commande ne peut varier que de
 * RAMPE_PWM_PAR_S × dt par cycle — évite les à-coups.
 */
const float RAMPE_PWM_PAR_S = 350.0f;

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

// ── Variables pour la méthode période ────────────────────────
// Toutes volatile car écrites dans l'ISR et lues dans la boucle.

// Horodatage (µs) du dernier front Hall détecté.
// micros() sur le Nano Every a une résolution de ~1 µs.
volatile uint32_t hallLastPulseTime_us = 0;

// Durée (µs) entre les deux derniers fronts consécutifs.
// Mise à jour à chaque impulsion dans l'ISR.
// Vaut 0 tant qu'aucune impulsion n'a encore eu lieu.
volatile uint32_t hallPeriode_us = 0;

// Drapeau : une nouvelle période est disponible depuis le dernier appel
// à calculerVitessePeriode(). Permet d'éviter d'utiliser deux fois
// la même mesure si la boucle tourne plus vite que les impulsions.
volatile bool hallNouvelleImpulsion = false;

// Valeur du compteur au dernier calcul de vitesse
uint32_t lastHallCount = 0;

// Vitesse mesurée (m/s)
float vitesse_ms = 0.0f;          // méthode fréquencemétrique (comptage sur fenêtre)
float vitesse_periode_ms = 0.0f;  // méthode période (temps entre 2 impulsions)

// Variables du correcteur
float erreurPrecedente = 0.0f;   // erreur à l'instant t-1 (pour le terme D)
float integrale        = 0.0f;   // accumulateur de l'intégrale (terme I)

// Horodatages
uint32_t lastControlTime = 0;
uint32_t lastLogTime     = 0;

// Dernière commande appliquée (pour le log)
float derniereCommande = 0.0f;

// États du démarrage et du lissage
bool     kickActif        = false;   // true pendant la durée de l'impulsion de kick
uint32_t kickDebutMs      = 0;       // horodatage du début du kick
float    commandeLisseePWM = 0.0f;   // commande lissée (rampe)

// Objet fichier SD
File logFile;
bool sdOk = false;
char logFilename[13] = {0};
uint16_t logLinesSinceFlush = 0;
const uint8_t LOG_FLUSH_EVERY_N_LINES = 10;

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
 *
 *   DEUX choses sont faites ici :
 *   1) On incrémente hallPulseCount (méthode fréquencemétrique).
 *   2) On mesure la durée depuis le front précédent (méthode période).
 *      micros() est accessible dans une ISR sur le Nano Every.
 *      On stocke aussi un drapeau pour signaler qu'une nouvelle
 *      mesure est prête.
 *
 *   On fait le minimum de travail dans l'ISR (quelques µs)
 *   pour ne pas bloquer la boucle principale trop longtemps.
 * =========================================================== */
void hallISR() {
  // ── Méthode fréquencemétrique ────────────────────────────
  hallPulseCount++;

  // ── Méthode période ──────────────────────────────────────
  uint32_t maintenant_us = micros();

  // Si ce n'est pas la toute première impulsion (hallLastPulseTime_us != 0),
  // on peut calculer une période valide.
  if (hallLastPulseTime_us != 0) {
    // Durée depuis l'impulsion précédente.
    // La soustraction reste correcte même si micros() a débordé
    // (arithmétique non signée sur 32 bits → résultat juste si
    // la période est < 2^32 µs ≈ 71 minutes, largement suffisant).
    hallPeriode_us = maintenant_us - hallLastPulseTime_us;
    hallNouvelleImpulsion = true;   // nouvelle mesure disponible
  }

  hallLastPulseTime_us = maintenant_us;  // mémoriser pour le prochain front
}


// Prototype utile si le fichier est compilé hors préprocesseur Arduino.
void appliquerCommande(float cmd);
void fermerLogSD();
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
  kickActif        = false;
  kickDebutMs      = 0;
  commandeLisseePWM = 0.0f;

  // On ferme proprement le fichier : indispensable avant de retirer la carte SD.
  fermerLogSD();

  // On remet aussi les compteurs de vitesse à jour pour éviter
  // un saut artificiel au redémarrage.
  noInterrupts();
  lastHallCount         = hallPulseCount;
  hallLastPulseTime_us  = 0;   // remet le timeout période à zéro
  hallPeriode_us        = 0;
  hallNouvelleImpulsion = false;
  interrupts();
  vitesse_periode_ms = 0.0f;
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
        if (!sdOk) {
          initSD();
        }
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
  // Timeout court : environ 2 m. Cela évite que la boucle se bloque trop longtemps
  // quand aucun écho n’est reçu.
  long duree_us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

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
 *   calculerVitessePeriode()
 *   Calcule la vitesse en m/s par la MÉTHODE PÉRIODE.
 *
 *   Principe :
 *   Au lieu de compter des impulsions dans une fenêtre de temps
 *   fixe, on mesure le temps τ (µs) entre deux impulsions
 *   consécutives. La vitesse en découle directement :
 *
 *       v = p / τ    avec p = périmètre / NB_AIMANTS (m)
 *
 *   Avantage vs méthode fréquencemétrique :
 *   L'incertitude Δv = v² × Δτ / p diminue quand v diminue,
 *   là où la méthode comptage donnait Δv = p/T = constante.
 *   Les deux méthodes sont donc complémentaires.
 *
 *   Cas particulier — vitesse nulle ou très faible :
 *   Si aucune impulsion n'arrive depuis plus de HALL_TIMEOUT_US,
 *   on retourne 0. Sans ce timeout, la dernière période mesurée
 *   resterait en mémoire indéfiniment, et on lirait une vitesse
 *   non nulle même roue à l'arrêt.
 *
 *   Cette fonction ne bloque jamais : elle lit des variables
 *   déjà calculées par l'ISR.
 * =========================================================== */
float calculerVitessePeriode() {
  // Distance entre deux aimants consécutifs (arc de roue, en mètres).
  // Constante calculée une seule fois ici grâce au mot-clé static :
  // la valeur est initialisée au premier appel et conservée ensuite.
  static const float PAS_M = PERIMETRE_M / (float)NB_AIMANTS;  // ≈ 0,0869 m

  // ── Vérification du timeout ──────────────────────────────
  // On lit micros() et hallLastPulseTime_us de façon atomique
  // pour éviter une lecture partielle (32 bits non atomique sur AVR).
  noInterrupts();
  uint32_t derniereImpulsion_us = hallLastPulseTime_us;
  interrupts();

  // Si jamais aucune impulsion reçue depuis le démarrage :
  // hallLastPulseTime_us == 0 → vitesse inconnue → on retourne 0.
  if (derniereImpulsion_us == 0) return 0.0f;

  // Temps écoulé depuis la dernière impulsion.
  // Si ce délai dépasse le timeout, la roue est à l'arrêt.
  uint32_t silenceActuel_us = micros() - derniereImpulsion_us;
  if (silenceActuel_us > HALL_TIMEOUT_US) {
    // On remet aussi le drapeau à zéro pour repartir proprement.
    noInterrupts();
    hallNouvelleImpulsion = false;
    interrupts();
    return 0.0f;
  }

  // ── Lecture de la dernière période mesurée ───────────────
  // Copie atomique des deux variables volatile en une seule
  // section critique (on les lit ensemble pour qu'elles soient
  // cohérentes : période et drapeau vont par paire).
  noInterrupts();
  uint32_t periode_us      = hallPeriode_us;
  bool     nouvelleImpuls  = hallNouvelleImpulsion;
  hallNouvelleImpulsion    = false;  // on consomme la mesure
  interrupts();

  // Si la période est nulle (pas encore deux impulsions reçues) : vitesse inconnue.
  if (periode_us == 0) return 0.0f;

  // ── Calcul de la vitesse ─────────────────────────────────
  // τ en secondes
  float tau_s = (float)periode_us / 1000000.0f;

  // v = p / τ
  float v = PAS_M / tau_s;

  // Garde-fou : si le résultat est physiquement absurde
  // (ex : rebond parasite sur le capteur Hall → période très courte → v énorme),
  // on rejette la mesure et on conserve la dernière valeur connue.
  // 10 m/s est largement au-dessus de toute vitesse réelle de la maquette.
  if (v > 10.0f) return vitesse_periode_ms;

  return v;
}
/*   Traduit la commande flottante [-255 ; +255] en signaux
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
 *   Combine deux choses :
 *   1) un kick-start si la roue est à l'arrêt ;
 *   2) une rampe de lissage pour éviter les à-coups une fois lancé.
 *
 *   Fonctionnement du kick-start :
 *   Si le correcteur demande d'avancer (commande > COMMANDE_MIN_DEMARRAGE)
 *   et que la roue ne tourne pas encore (vitesse < VITESSE_QUASI_NULLE),
 *   on envoie KICK_PWM pendant KICK_DUREE_MS, puis on rend la main
 *   au correcteur. Si la roue ne tourne toujours pas au prochain
 *   cycle, un nouveau kick est déclenché.
 *
 *   Paramètre now : temps courant en ms (millis()), nécessaire
 *   pour mesurer la durée du kick sans utiliser delay().
 * =========================================================== */
float calculerCommandeMoteur(float commandeCorrecteur, float dt_s, bool urgence, uint32_t now) {

  // ── Urgence ou freinage : rampe désactivée, réponse immédiate ──
  if (urgence || commandeCorrecteur < 0.0f) {
    kickActif = false;
    commandeLisseePWM = commandeCorrecteur;
    return commandeCorrecteur;
  }

  // ── Kick-start ──────────────────────────────────────────────
  if (kickActif) {
    // On est en cours de kick : on maintient le PWM élevé
    // jusqu'à la fin de la durée, ou jusqu'à ce que la roue tourne.
    if ((now - kickDebutMs) < KICK_DUREE_MS && vitesse_periode_ms < VITESSE_QUASI_NULLE_MS) {
      // Kick en cours : on bypasse la rampe et on envoie directement KICK_PWM.
      commandeLisseePWM = KICK_PWM;
      return (float)KICK_PWM;
    } else {
      // Kick terminé (durée écoulée ou roue qui tourne) : on reprend le correcteur.
      kickActif = false;
      // On initialise la rampe à KICK_PWM pour éviter un saut brutal vers le bas.
      commandeLisseePWM = (float)KICK_PWM;
    }
  }

  // Déclenchement d'un nouveau kick si nécessaire :
  // le correcteur veut avancer mais la roue est encore à l'arrêt.
  if (commandeCorrecteur > COMMANDE_MIN_DEMARRAGE && vitesse_periode_ms < VITESSE_QUASI_NULLE_MS) {
    kickActif   = true;
    kickDebutMs = now;
    commandeLisseePWM = KICK_PWM;
    return (float)KICK_PWM;
  }

  // ── Rampe de lissage (régime normal) ────────────────────────
  // La commande réelle ne peut varier que de deltaMax par cycle.
  // Cela évite les transitions brusques qui font patiner la roue.
  float commandeCible = constrain(commandeCorrecteur, 0.0f, 255.0f);
  float deltaMax      = RAMPE_PWM_PAR_S * dt_s;
  float delta         = commandeCible - commandeLisseePWM;
  delta               = constrain(delta, -deltaMax, deltaMax);
  commandeLisseePWM  += delta;

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

  // Construction du nom de fichier — format 8.3 FAT32 strict :
  // 8 caractères max avant le point, 3 après.
  // Le buffer doit contenir au maximum 12 caractères + '\0' = 13 octets.
  // On utilise le schéma : P000.CSV, PI001.CSV, PD002.CSV, PID003.CSV
  // "PID" = 3 caractères + "000" = 3 caractères + ".CSV" = 4 → 10 caractères + '\0' : OK.
  //
  // ATTENTION : ne JAMAIS dépasser 12 caractères avant '\0'.
  // L'ancien schéma "log_P_Kp-50.0_Ki-8.0_Kd-5.0_000.CSV" faisait 37 caractères
  // → snprintf tronquait silencieusement → SD.open() recevait une chaîne corrompue
  // → "impossible d'ouvrir le fichier".
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

  // ── En-tête CSV ───────────────────────────────────────────
  // Les gains sont écrits en commentaire sur la première ligne
  // (préfixe #) pour être ignorés par pandas mais rester lisibles.
  // Cela remplace l'ancien encodage dans le nom de fichier
  // qui dépassait la limite 8.3 de FAT32.
  logFile.print(F("#correcteur="));  logFile.print(MODE_STR);
  logFile.print(F(",Kp="));           logFile.print(Kp_VAL, 1);
  logFile.print(F(",Ki="));           logFile.print(Ki_VAL, 1);
  logFile.print(F(",Kd="));           logFile.println(Kd_VAL, 1);
  logFile.println(F("t_s,vitesse_freq_ms,vitesse_periode_ms,consigne_vitesse_ms,"
                    "distance_m,consigne_dist_m,"
                    "erreur_m,commande_pwm"));
  logFile.flush();

  strncpy(logFilename, filename, sizeof(logFilename));
  logFilename[sizeof(logFilename) - 1] = '\0';
  logLinesSinceFlush = 0;

  sdOk = true;
  Serial.print(F("SD : fichier ouvert -> "));
  Serial.println(filename);
}


/* ===========================================================
 *   fermerLogSD()
 *   Force l’écriture finale et ferme le fichier. À appeler
 *   au STOP avant de retirer la carte SD.
 * =========================================================== */
void fermerLogSD() {
  if (sdOk && logFile) {
    logFile.flush();
    logFile.close();
    Serial.print(F("SD : fichier ferme -> "));
    Serial.println(logFilename);
  }
  sdOk = false;
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
  // Méthode fréquencemétrique : comptage d'impulsions sur la fenêtre de 50 ms
  logFile.print(vitesse_ms,        4);  logFile.print(',');
  // Méthode période : temps entre les 2 dernières impulsions Hall
  logFile.print(vitesse_periode_ms,4);  logFile.print(',');
  logFile.print(VITESSE_CIBLE_MS,  4);  logFile.print(',');
  logFile.print(dist_m,       4);  logFile.print(',');
  logFile.print(DIST_CONSIGNE_M, 4); logFile.print(',');
  logFile.print(erreur_m,     4);  logFile.print(',');
  logFile.println(derniereCommande, 1);

  // On ne flush pas à chaque ligne : certaines SD bloquent longtemps.
  // Toutes les 10 lignes, on force l'écriture. Le STOP ferme aussi le fichier.
  logLinesSinceFlush++;
  if (logLinesSinceFlush >= LOG_FLUSH_EVERY_N_LINES) {
    logFile.flush();
    logLinesSinceFlush = 0;
  }
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
  // Sur les cartes Nano/AVR, garder D10 en OUTPUT évite des soucis SPI master,
  // même si le CS effectif du shield ASX00061 est D4.
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
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

    // Vitesse roue (m/s) depuis le compteur Hall — méthode fréquencemétrique
    vitesse_ms = calculerVitesse(dt_ms);

    // Vitesse roue (m/s) — méthode période (temps entre 2 impulsions)
    // Cette valeur est calculée à chaque cycle mais reflète la dernière
    // impulsion reçue, indépendamment de la période de 50 ms.
    vitesse_periode_ms = calculerVitessePeriode();

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
      erreur   = VITESSE_CIBLE_MS - vitesse_periode_ms;
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
    Serial.print(F("v_freq="));    Serial.print(vitesse_ms,        3); Serial.print(F(" m/s"));
    Serial.print(F(" | v_per="));  Serial.print(vitesse_periode_ms,3); Serial.print(F(" m/s"));
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
