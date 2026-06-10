/*
 * RÉGULATEUR DE VITESSE ADAPTATIF ET SA BOUCLE DE RÉTROACTION PID
 * TIPE Par Emile Reynaud
 */

// Définition de 4 constantes numériques au lieu d'avoir des str
#define CORR_P 1 // correcteur proportionnel seul
#define CORR_PI 2 // correcteur proportionnel + intégral
#define CORR_PD 3 // correcteur proportionnel + dérivé
#define CORR_PID 4 // correcteur proportionnel + intégral + dérivé

// Choix du correcteur en décommentant une seule ligne
// #define CORRECTEUR CORR_P
// #define CORRECTEUR CORR_PI
// #define CORRECTEUR CORR_PD
#define CORRECTEUR CORR_PID

// Définition des bonnes constantes en fonction du correcteur
#if CORRECTEUR == CORR_P           
  #define Kp_VAL 60.0f           
  #define Ki_VAL 0.0f           
  #define Kd_VAL 0.0f           
  #define USE_I 0 // activation ou non de l'intégrale
  #define USE_D 0 // idem pour la dérivée
  #define MODE_STR "P" // label pour logs
  #define KICK_PWM_VAL 209 // PWM du kick
  #define KICK_DUREE_MS_VAL 250 // durée du kick

#elif CORRECTEUR == CORR_PI        
  #define Kp_VAL 15.0f           
  #define Ki_VAL 60.0f           
  #define Kd_VAL 0.0f           
  #define USE_I 1               
  #define USE_D 0               
  #define MODE_STR "PI"            
  #define KICK_PWM_VAL 209   
  #define KICK_DUREE_MS_VAL 250   

#elif CORRECTEUR == CORR_PD        
  #define Kp_VAL 350.0f           
  #define Ki_VAL 0.0f           
  #define Kd_VAL 10.0f           
  #define USE_I 0               
  #define USE_D 1               
  #define MODE_STR "PD"            
  #define KICK_PWM_VAL 255   
  #define KICK_DUREE_MS_VAL 800   

#elif CORRECTEUR == CORR_PID       
  #define Kp_VAL 80.0f           
  #define Ki_VAL 40.0f           
  #define Kd_VAL 8.0f           
  #define USE_I 1               
  #define USE_D 1               
  #define MODE_STR "PID"           
  #define KICK_PWM_VAL 153   
  #define KICK_DUREE_MS_VAL 1000  

#else
  // Si pas de correcteur valide, renvoie une erreur
  #error "Correcteur non défini — décommenter une ligne CORRECTEUR ci-dessus"
#endif

// Identification moteur
const float K_MOTEUR = 1.40f; // gain statique du moteur
const float SEUIL_DEMARRAGE = 0.30f; // commande minimale normalisée pour démarrer 

// Déclaration de la variable du feedforward
// Calcul de cette dernière dans setup()
float FEEDFORWARD_PWM = 0.0f;

// Gain de freinage pour le mode distance
const float KP_BRAKE_PWM = 600.0f; // en PWM/m

// Pins Arduino
#define TRIG_PIN 7 // pin TRIG du capteur à ultrason
#define ECHO_PIN 6 // pin ECHO du capteur à ultrason
#define HALL_PIN 2 // pin SIG du capteur à effet Hall
#define ENB_PIN 9 // pin ENB du pont en H
#define IN3_PIN A0 // pin IN3 du pont en H
#define IN4_PIN 5 // pin IN4 du pont en H
#define SD_CS_PIN 4 // pin SD_CS de la carte microSD
#define BUTTON_PIN 8 // bouton démarrage programme

// Roue
const int DIAMETRE_MM = 83; // diamètre de la roue (mm)
const float PERIMETRE_M = 3.14159f * (float)DIAMETRE_MM / 1000.0f; // Périmètre de la roue (m)
const uint8_t NB_AIMANTS = 3; // nombre d'aimants

// Constantes de consigne
const float DIST_CONSIGNE_M = 0.30f; // distance de consigne (m)
const float VITESSE_CIBLE_MS = 0.70f; // vitesse de consigne (m/s)

// Constantes de sécurité
const float DIST_URGENCE_M = 0.06f; // distance freinage d'urgence
const float DIST_MAX_OBSTACLE_M = 1.20f; // distance max à laquelle un objet est détecté

// Limite de l'intégrale
const float INTEGRALE_MAX = 100.0f; // valeur maximale de l'intégral (en PWM)

// Constantes temporelles
const uint16_t CONTROL_PERIOD_MS = 50; // période de la boucle de contrôle
const uint16_t LOG_PERIOD_MS = 100; // période d'écriture sur la carte SD
const uint16_t DEBOUNCE_MS = 30; // durée de l'anti-rebond du bouton
const uint32_t ECHO_TIMEOUT_US = 12000UL; // timeout capteur à ultrason
const uint32_t HALL_TIMEOUT_US = 400000UL; // timeout capteur à effet Hall

// Constantes pour le kick start
const uint8_t  KICK_PWM_C = KICK_PWM_VAL; // puissance du kick 
const uint16_t KICK_DUREE_MS_C = KICK_DUREE_MS_VAL; // durée du kick (ms)
const float VITESSE_QUASI_NULLE_MS = 0.05f; // en dessous de cette vitesse, on considère la roue arrêtée
const float COMMANDE_MIN_DEMARRAGE = 3.0f; // commande minimale pour déclencher un kick (PWM)

// Zone morte
const float PWM_ZONE_MORTE = 76.0f;

// Rampe de commande (lissage asymétrique)
const float RAMPE_MONTEE_PAR_S = 350.0f; // PWM/s accélération
const float RAMPE_DESCENTE_PAR_S = 700.0f; // PWM/s décélération

// Bibliothèques
#include <SPI.h> // protocole SPI nécessaire pour la carte SD
#include <SD.h> // permet de lire/écrire sur la carte SD

// Variables volatiles (pour éviter que le programme les mettent en cache car elle sont constamment modifiées)
volatile uint32_t hallPulseCount = 0; // nombre d'impulsions Hall reçues
volatile uint32_t hallLastPulseTime_us = 0; // timestamp (microseconde) de la dernière impulsion Hall
volatile uint32_t hallPeriode_us = 0; // durée (microseconde) entre les deux dernières impulsions
volatile bool hallNouvelleImpulsion = false; // nouvelle impulsion ou non

// Variables de mesures
uint32_t lastHallCount = 0; // valeur de hallPulseCount au dernier cycle (méthode fréquencemétrique)
float vitesse_ms = 0.0f; // vitesse mesurée par méthode fréquencemétrique (m/s)
float vitesse_periode_ms = 0.0f; // vitesse mesurée par méthode période (m/s)

// Variables correcteur
float integrale = 0.0f; // accumulateur de l'intégrale (terme I du PI et PID)

// Dérivée calculée sur la mesure de vitesse
float vitessePeriodePrecedente = 0.0f; // vitesse_periode_ms du cycle précédent
bool deriveeInitialisee = false; // faux au démarrage, le premier cycle n'a pas de "précédent"

// Variables temporelles
uint32_t lastControlTime = 0; // timestamp du dernier contrôle (ms)
uint32_t lastLogTime = 0; // timestamp du dernier log SD (ms)
float derniereCommande = 0.0f; // PWM envoyée au moteur au dernier cycle

// Variables kick start
bool kickActif = false; // kick actif ou non
uint32_t kickDebutMs = 0; // timestamp (ms) du début du kick en cours
float commandeLisseePWM = 0.0f; // commande lissée par la rampe

// Variables carte SD
File logFile; // objet fichier ouvert sur la carte SD
bool sdOk = false; // carte SD initialisée et fichier ouvert ou non
char logFilename[13] = {0}; // nom du fichier CSV
uint16_t logLinesSinceFlush = 0; // compteur de lignes depuis le dernier flush() forcé
const uint8_t LOG_FLUSH_EVERY_N_LINES = 10; // on force un flush tous les 10 lignes pour ne pas perdre trop de données en cas de coupure

// Variables bouton
bool programmeActif = false; // régulateur actif ou non
const uint8_t BUTTON_ACTIVE_LEVEL = LOW; // état du pin quand le bouton est préssé
int buttonLastRawState = HIGH; // état lu au cycle précédent
int buttonStableState = HIGH; // état stable après anti-rebond
uint32_t lastButtonChangeTime = 0; // timestamp (ms) du dernier changement d'état brut

// Déclarations anticipées des fonctions
void  appliquerCommande(float cmd);
void  ecrireLog(float t_s, float dist_m, float erreur_m, uint8_t modeActuel);
void  initSD();
void  fermerLogSD();
float calculerCommandeMoteur(float cmdCorrecteur, float dt_s, bool urgence, bool interdireKick, uint32_t now);
float calculerCorrecteur(float erreur, float dt_s, bool reinitDerivee);

// Appelée à chaque passage d'aimant devant le capteur à effet Hall
void hallISR() {
  uint32_t t_us = micros(); // timestamp en microseconde au passage de l'aimant

  // Anti-rebond
  if (hallLastPulseTime_us != 0 && (t_us - hallLastPulseTime_us < 20000UL)) return;

  hallPulseCount++; // incrémenter le compteur total d'impulsions (méthode fréquencemétrique)

  if (hallLastPulseTime_us != 0) { // si pas la toute première impulsion
    hallPeriode_us = t_us - hallLastPulseTime_us; // durée entre les deux derniers aimants
    hallNouvelleImpulsion = true; // signalement nouvelle mesure dispo
  }

  hallLastPulseTime_us = t_us; // mémoriser l'heure de cette impulsion pour le prochain calcul de période
}

// logique du bouton
void gererBoutonStartStop(uint32_t now) {
  int rawState = digitalRead(BUTTON_PIN); // lire l'état du bouton (HIGH ou LOW)

  if (rawState != buttonLastRawState) { // état différent d'avant ?
    buttonLastRawState   = rawState; // mémoriser le nouvel état 
    lastButtonChangeTime = now; // démarrer le chrono de l'anti-rebond
  }

  // si l'état reste le même pendant 30 ms, on valide l'état
  if ((now - lastButtonChangeTime) >= DEBOUNCE_MS && rawState != buttonStableState) {
    buttonStableState = rawState; // valider le nouvel état stable

    if (buttonStableState == BUTTON_ACTIVE_LEVEL) { // bouton pressé ?
      programmeActif = !programmeActif; // changer l'état du programme

      if (programmeActif) { // on vient de démarrer ?
        Serial.println(F("Bouton : START"));
        if (!sdOk) initSD(); // ouvrir un nouveau fichier CSV si la SD n'est pas encore prête
        lastControlTime = now; // réinitialiser le timer de contrôle pour éviter un grand dt au premier cycle
        lastLogTime     = now; // réinitialiser le timer de log
      } else { // on vient de s'arrêter ?
        Serial.println(F("Bouton : STOP"));
        arreterProgramme(); // appelle arreterProgramme()
      }
    }
  }
}

/* ===========================================================
 * arreterProgramme() — ARRÊT PROPRE
 * Remet toutes les variables d'état à zéro et ferme le fichier.
 * =========================================================== */
void arreterProgramme() {
  appliquerCommande(0.0f);            // envoyer PWM = 0 → moteur à l'arrêt immédiat
  derniereCommande         = 0.0f;   // effacer la dernière commande mémorisée
  integrale                = 0.0f;   // remettre l'accumulateur intégral à zéro
  vitessePeriodePrecedente = 0.0f;   // effacer la vitesse précédente (terme D)
  deriveeInitialisee       = false;  // indiquer que le terme D doit être réinitialisé au prochain démarrage
  kickActif                = false;  // annuler tout kick en cours
  kickDebutMs              = 0;      // effacer le timestamp du kick
  commandeLisseePWM        = 0.0f;   // remettre la rampe à zéro
  fermerLogSD();                     // flush + fermeture propre du fichier CSV sur la SD

  noInterrupts();                          // désactiver les interruptions pour lire les variables volatiles sans conflit
  lastHallCount        = hallPulseCount;   // remettre le compteur fréquencemétrique à niveau
  hallLastPulseTime_us = 0;                // effacer le timestamp Hall (force v=0 au prochain calculerVitessePeriode)
  hallPeriode_us       = 0;               // effacer la période Hall
  hallNouvelleImpulsion = false;           // effacer le drapeau de nouvelle impulsion
  interrupts();                            // réactiver les interruptions

  vitesse_periode_ms = 0.0f;  // remettre la vitesse lissée à zéro
}

/* ===========================================================
 * lireDistance() — MESURE PAR ULTRASON HC-SR04
 * Envoie une impulsion et mesure le temps d'écho → distance.
 * Retourne la distance en mètres, ou -1.0f si mesure invalide.
 * =========================================================== */
float lireDistance() {
  digitalWrite(TRIG_PIN, LOW);            // s'assurer que TRIG est bas avant de démarrer
  delayMicroseconds(2);                   // attendre 2 µs pour stabiliser le signal
  digitalWrite(TRIG_PIN, HIGH);           // envoyer l'impulsion ultrason (début)
  delayMicroseconds(10);                  // maintenir l'impulsion 10 µs (durée minimale spec HC-SR04)
  digitalWrite(TRIG_PIN, LOW);            // arrêter l'impulsion → le capteur émet la rafale ultrason

  // Mesurer la durée de l'écho : pulseIn attend un front montant puis un front descendant sur ECHO_PIN.
  // ECHO_TIMEOUT_US = 12 000 µs → distance max ~2 m. Si timeout → retour 0.
  long duree_us = pulseIn(ECHO_PIN, HIGH, ECHO_TIMEOUT_US);

  if (duree_us == 0) return -1.0f;   // pas d'écho reçu (timeout ou obstacle hors portée)

  // Conversion durée → distance : d = (t × vitesse_son) / 2
  // vitesse du son ≈ 343 m/s → 0.000343 m/µs ; ÷2 car aller-retour → × 0.0001715 m/µs
  float d = (float)duree_us * 0.0001715f;

  if (d < 0.02f || d > 4.0f) return -1.0f;  // valeur hors plage physique : rejetée

  return d;   // distance valide en mètres
}

/* ===========================================================
 * calculerVitesse() — MÉTHODE FRÉQUENCEMÉTRIQUE
 * Compte les impulsions Hall sur l'intervalle dt_ms.
 * Plus précis à haute vitesse, inutilisable à basse vitesse.
 * Utilisé UNIQUEMENT pour le log (pas pour le PID).
 * =========================================================== */
float calculerVitesse(uint32_t dt_ms) {
  noInterrupts();                    // bloquer les interruptions pour lire hallPulseCount sans conflit
  uint32_t cnt = hallPulseCount;    // copier le compteur atomiquement
  interrupts();                      // rétablir les interruptions

  uint32_t delta = cnt - lastHallCount;  // nombre d'impulsions depuis le dernier appel
  lastHallCount  = cnt;                   // mémoriser la valeur actuelle pour le prochain cycle

  if (dt_ms == 0) return 0.0f;   // protection contre division par zéro

  // v = (nombre_de_tours × périmètre) / durée
  // nombre_de_tours = delta / NB_AIMANTS  (3 aimants = 3 impulsions par tour)
  return ((float)delta / (float)NB_AIMANTS * PERIMETRE_M)
         / ((float)dt_ms / 1000.0f);
}

/* ===========================================================
 * calculerVitessePeriode() — MÉTHODE PÉRIODE (utilisée par le PID)
 * Calcule la vitesse à partir de la durée entre deux aimants
 * consécutifs. Très précis à basse vitesse (×155 meilleur).
 * Filtre exponentiel asymétrique pour lisser sans retarder.
 * =========================================================== */
float calculerVitessePeriode() {
  // PAS_M : distance parcourue entre deux impulsions Hall consécutives
  // = périmètre / nb_aimants. static : calculé une seule fois.
  static const float PAS_M      = PERIMETRE_M / (float)NB_AIMANTS;

  // Variation maximale physiquement possible en un cycle de 50 ms.
  // Au-delà, c'est un parasite (rebond mécanique) → on rejette.
  static const float DELTA_V_MAX = 0.8f;  // m/s par cycle de 50 ms

  noInterrupts();
  uint32_t derniereImp_us = hallLastPulseTime_us;  // lire le timestamp de la dernière impulsion
  interrupts();

  if (derniereImp_us == 0) return 0.0f;   // pas encore d'impulsion reçue depuis le démarrage

  // Si aucune impulsion depuis plus de HALL_TIMEOUT_US (400 ms) → roue à l'arrêt
  if ((micros() - derniereImp_us) > HALL_TIMEOUT_US) {
    noInterrupts(); hallNouvelleImpulsion = false; interrupts();  // effacer le drapeau
    vitesse_periode_ms = 0.0f;  // forcer la vitesse à zéro
    return 0.0f;
  }

  noInterrupts();
  uint32_t periode_us = hallPeriode_us;  // copier la période mesurée par l'ISR
  hallNouvelleImpulsion = false;         // effacer le drapeau de nouvelle mesure
  interrupts();

  if (periode_us == 0) return vitesse_periode_ms;  // pas de nouvelle période → garder l'ancienne valeur

  // v = distance / temps = PAS_M / (periode_us / 1 000 000)
  float v_brute = PAS_M / ((float)periode_us / 1000000.0f);

  // Rejet des parasites : si le saut de vitesse est physiquement impossible → ignorer
  if (vitesse_periode_ms > 0.0f
      && fabsf(v_brute - vitesse_periode_ms) > DELTA_V_MAX) {
    return vitesse_periode_ms;  // mesure rejetée, on conserve la valeur précédente
  }

  // Filtre exponentiel asymétrique :
  //   montée  → alpha = 0.25 (filtre fort : atténue les fausses vitesses élevées)
  //   descente → alpha = 0.85 (filtre faible : suit vite la décélération réelle)
  // v_filtré = alpha × v_brute + (1-alpha) × v_filtré_précédent
  float alpha = (v_brute >= vitesse_periode_ms) ? 0.25f : 0.85f;

  if (vitesse_periode_ms == 0.0f) {        // premier calcul : initialiser directement sans filtrage
    vitesse_periode_ms = v_brute;
  } else {
    vitesse_periode_ms = alpha * v_brute + (1.0f - alpha) * vitesse_periode_ms;  // filtre IIR
  }

  return vitesse_periode_ms;  // retourner la vitesse filtrée en m/s
}

/* ===========================================================
 * appliquerCommande() — ENVOI PWM AU PONT EN H L298N
 * Commande positive → moteur en avant.
 * Commande négative → moteur en arrière (freinage actif).
 * Zone morte [0, PWM_ZONE_MORTE[ → roue libre (commande forcée à 0).
 * =========================================================== */
void appliquerCommande(float cmd) {
  // Zone morte : entre 0 et 76 PWM le moteur ne produit aucun couple.
  // On force à 0 pour éviter que l'intégrale accumule une erreur fictive.
  if (cmd > 0.0f && cmd < PWM_ZONE_MORTE) cmd = 0.0f;

  int pwm = (int)constrain(cmd, -255.0f, 255.0f);  // limiter entre -255 et +255 et convertir en entier

  if (pwm >= 0) {                              // commande positive → sens avant
    digitalWrite(IN3_PIN, HIGH);               // IN3 = HIGH : sélectionne le sens avant
    digitalWrite(IN4_PIN, LOW);                // IN4 = LOW  : complément de IN3
    analogWrite(ENB_PIN, (uint8_t)pwm);        // ENB : fixer la puissance par PWM (0…255)
  } else {                                     // commande négative → freinage actif / sens arrière
    digitalWrite(IN3_PIN, LOW);                // IN3 = LOW  : sens inverse
    digitalWrite(IN4_PIN, HIGH);               // IN4 = HIGH : complément de IN3
    analogWrite(ENB_PIN, (uint8_t)(-pwm));     // valeur absolue pour PWM (toujours positif 0…255)
  }
}

/* ===========================================================
 * calculerCorrecteur() — CALCUL PID
 *
 * Retourne la correction u_PID en PWM à ajouter au feedforward.
 * u_total = FEEDFORWARD_PWM + calculerCorrecteur(erreur, dt_s, ...)
 *
 * NOUVEAUTÉ v19 : dérivée calculée sur la mesure de vitesse (pas sur l'erreur).
 *   d(erreur)/dt = d(Vc − v)/dt = −dv/dt
 *   u_D = Kd × d(erreur)/dt = −Kd × dv/dt
 * Cela évite le pic brutal ("derivative kick") quand la consigne Vc
 * change soudainement (fin de kick, changement de mode).
 * =========================================================== */
float calculerCorrecteur(float erreur, float dt_s, bool reinitDerivee) {

  // ── Terme P ──────────────────────────────────────────────
  float u_P = Kp_VAL * erreur;  // proportionnel : réponse immédiate proportionnelle à l'écart courant

  // ── Terme I ──────────────────────────────────────────────
  float u_I = 0.0f;    // initialiser à 0 ; sera calculé seulement si USE_I = 1
#if USE_I              // bloc compilé seulement si le correcteur a une intégrale (PI ou PID)
  if (!kickActif) {                              // NE PAS intégrer pendant le kick (anti-windup kick)
    integrale += erreur * dt_s;                  // intégrale discrète : I += ε × Δt (approximation rectangulaire)
    integrale = constrain(integrale,             // saturation anti-windup : limiter l'intégrale à ±INTEGRALE_MAX
                          -INTEGRALE_MAX,
                          INTEGRALE_MAX);
  }
  u_I = Ki_VAL * integrale;  // terme intégral en PWM = Ki × ∫ε dt
#endif

  // ── Terme D sur la mesure (pas sur l'erreur) ─────────────
  float u_D = 0.0f;    // initialiser à 0 ; sera calculé seulement si USE_D = 1
#if USE_D              // bloc compilé seulement si le correcteur a une dérivée (PD ou PID)
  if (!reinitDerivee && deriveeInitialisee && dt_s > 0.0f) {
    // dv/dt ≈ (v_actuel − v_précédent) / Δt   (dérivée numérique 1er ordre)
    float dv_dt = (vitesse_periode_ms - vitessePeriodePrecedente) / dt_s;
    // u_D = −Kd × dv/dt   car d(erreur)/dt = −dv/dt (Vc constant en régime)
    u_D = -Kd_VAL * dv_dt;
  }
  // reinitDerivee = vrai lors d'une transition de mode → on saute ce cycle
  // deriveeInitialisee = faux au démarrage ou après un kick → pas de "précédent" disponible
#endif

  vitessePeriodePrecedente = vitesse_periode_ms;  // mémoriser la vitesse pour le prochain calcul D
  deriveeInitialisee       = true;                // indiquer que le "précédent" est maintenant valide

  return u_P + u_I + u_D;  // retourner la somme des trois termes PID (en PWM)
}

/* ===========================================================
 * calculerCommandeMoteur() — KICK-START + RAMPE ASYMÉTRIQUE
 *
 * Prend la commande brute du correcteur et applique :
 *   1) la logique de kick-start (impulsion initiale pour démarrer)
 *   2) la rampe asymétrique (lissage de la commande)
 * =========================================================== */
float calculerCommandeMoteur(float cmdCorrecteur, float dt_s,
                              bool urgence, bool interdireKick, uint32_t now) {

  static uint32_t kickFinMs  = 0;   // timestamp de la fin du dernier kick (persistant entre appels)
  const  uint16_t KICK_GRACE = 300; // délai de grâce après un kick : 300 ms sans nouveau kick

  // ── Mode urgence : bypasser tout, commande directe ────────
  if (urgence) {
    kickActif = false;                  // couper tout kick en cours
    commandeLisseePWM = cmdCorrecteur;  // mettre la rampe à jour (pour cohérence au retour)
    return cmdCorrecteur;               // retourner la commande d'urgence sans rampe ni kick
  }

  if (interdireKick) kickActif = false;  // si obstacle proche → couper le kick immédiatement

  // ── Kick en cours : continuer tant que conditions remplies ─
  if (kickActif) {
    if ((now - kickDebutMs) < KICK_DUREE_MS_C          // kick encore dans sa durée maximale
        && vitesse_periode_ms < VITESSE_QUASI_NULLE_MS) { // ET roue encore quasi arrêtée
      commandeLisseePWM = KICK_PWM_C;      // maintenir la rampe au niveau du kick
      return (float)KICK_PWM_C;            // envoyer la commande kick directement (sans rampe)
    } else {                               // fin du kick (durée écoulée OU voiture a démarré)
      kickActif = false;                   // désactiver le drapeau kick
      kickFinMs = now;                     // mémoriser l'heure de fin pour le délai de grâce
      deriveeInitialisee = false;          // réinitialiser D : évite le pic Δv au premier cycle post-kick
      integrale = 0.0f;                    // réinitialiser I : évite le windup accumulé pendant le kick
      commandeLisseePWM = FEEDFORWARD_PWM; // repartir la rampe depuis le feedforward (transition douce)
    }
  }

  // ── Déclenchement d'un nouveau kick ──────────────────────
  bool graceActive = (now - kickFinMs) < KICK_GRACE; // vrai si on est dans les 300 ms post-kick
  bool roueArretee = (vitesse_periode_ms < VITESSE_QUASI_NULLE_MS)  // mesure période < 0.05 m/s
                  && (vitesse_ms         < VITESSE_QUASI_NULLE_MS); // ET mesure fréq < 0.05 m/s

  if (!interdireKick          // kick non interdit (pas d'obstacle proche)
      && !graceActive         // délai de grâce écoulé (évite les kicks en rafale)
      && cmdCorrecteur > COMMANDE_MIN_DEMARRAGE  // le correcteur demande bien d'avancer
      && roueArretee) {       // les deux mesures confirment que la roue est arrêtée
    integrale          = 0.0f;   // reset I au lancement kick (anti-windup)
    deriveeInitialisee = false;  // reset D (v=0 → pas de "précédent" fiable)
    kickActif          = true;   // activer le kick
    kickDebutMs        = now;    // mémoriser l'heure de début
    commandeLisseePWM  = KICK_PWM_C;   // initialiser la rampe au niveau kick
    return (float)KICK_PWM_C;   // retourner la commande kick immédiatement
  }

  // ── Rampe asymétrique ────────────────────────────────────
  float cible = constrain(cmdCorrecteur, 0.0f, 255.0f);  // limiter la cible à [0, 255] PWM

  // Calcul du déplacement maximal autorisé ce cycle selon la direction
  float dMax = (cible < commandeLisseePWM)          // si on descend (frein ou décélération)
               ? RAMPE_DESCENTE_PAR_S * dt_s         //   descente rapide : 700 PWM/s × Δt
               : RAMPE_MONTEE_PAR_S   * dt_s;        //   montée lente   : 350 PWM/s × Δt

  float delta = constrain(cible - commandeLisseePWM, -dMax, dMax);  // delta limité à ±dMax
  commandeLisseePWM += delta;   // avancer la rampe d'un pas vers la cible

  return constrain(commandeLisseePWM, 0.0f, 255.0f);  // retourner la commande lissée, bornée [0,255]
}

/* ===========================================================
 * setup() — INITIALISATION (exécuté une seule fois au démarrage)
 * =========================================================== */
void setup() {
  Serial.begin(115200);  // démarrer le port série à 115 200 baud pour les messages de debug

  Serial.print(F("=== AAC TIPE v19 — correcteur : "));   // afficher la version et le correcteur actif
  Serial.print(MODE_STR);   // afficher "P", "PI", "PD" ou "PID" selon la sélection
  Serial.println(F(" ==="));

  // ── Capteur ultrason HC-SR04 ───────────────────────────────
  pinMode(TRIG_PIN, OUTPUT);  // TRIG est une sortie : on génère l'impulsion
  pinMode(ECHO_PIN, INPUT);   // ECHO est une entrée : on mesure le retour
  digitalWrite(TRIG_PIN, LOW);  // s'assurer que TRIG est bas au démarrage

  // ── Capteur Hall ───────────────────────────────────────────
  pinMode(HALL_PIN, INPUT_PULLUP);  // activer la résistance de pull-up interne (signal actif LOW)
  // Attacher l'ISR hallISR() à l'interruption INT0 (broche 2) sur front descendant (FALLING)
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  // ── Bouton start/stop ──────────────────────────────────────
  pinMode(BUTTON_PIN, INPUT_PULLUP);              // pull-up interne : bouton appuyé = LOW
  buttonLastRawState = digitalRead(BUTTON_PIN);   // lire l'état initial pour éviter un faux démarrage
  buttonStableState  = buttonLastRawState;         // initialiser l'état stable = état initial

  // ── Pont en H L298N ────────────────────────────────────────
  pinMode(ENB_PIN,  OUTPUT);   // Enable B : sortie PWM pour la vitesse
  pinMode(IN3_PIN,  OUTPUT);   // IN3 : sortie numérique pour le sens
  pinMode(IN4_PIN,  OUTPUT);   // IN4 : sortie numérique pour le sens (complément de IN3)
  analogWrite(ENB_PIN, 0);     // démarrer avec PWM = 0 → moteur arrêté
  digitalWrite(IN3_PIN, LOW);  // IN3 bas
  digitalWrite(IN4_PIN, LOW);  // IN4 bas → pas de courant dans le moteur

  // ── Carte SD via SPI ──────────────────────────────────────
  // Le shield SD partage le bus SPI avec d'éventuels autres périphériques.
  // On force le CS de la SD à HIGH (désélectionné) et le pin 10 aussi
  // (nécessaire sur certaines cartes pour que la SD fonctionne correctement).
  pinMode(SD_CS_PIN, OUTPUT);    // CS de la SD en sortie
  digitalWrite(SD_CS_PIN, HIGH); // SD désélectionnée au démarrage
  pinMode(10, OUTPUT);           // pin SPI maître nécessaire même si non utilisé comme CS
  digitalWrite(10, HIGH);        // mettre HIGH pour ne pas interférer avec le bus SPI

  // ── Calcul du feedforward ─────────────────────────────────
  // Formule : U_FF = (Vc / K_moteur + seuil_démarrage) × 255
  // Cette valeur est la commande qui maintient exactement Vc en régime permanent
  // si le modèle moteur (K, seuil) est correct.
  // On la sature à 250 pour garder 5 PWM de marge pour le terme P.
  FEEDFORWARD_PWM = constrain(
    (VITESSE_CIBLE_MS / K_MOTEUR + SEUIL_DEMARRAGE) * 255.0f,  // calcul brut
    0.0f, 250.0f);  // saturation à 250 PWM maxi

  // Afficher un résumé des paramètres sur le port série pour vérification avant démarrage
  Serial.print(F("Vc = "));       Serial.print(VITESSE_CIBLE_MS, 2); Serial.println(F(" m/s"));
  Serial.print(F("U_FF = "));     Serial.print(FEEDFORWARD_PWM,  1); Serial.println(F(" PWM"));
  Serial.print(F("Kp_BRAKE = ")); Serial.print(KP_BRAKE_PWM,    1); Serial.println(F(" PWM/m"));
  Serial.print(F("Kp="));         Serial.print(Kp_VAL, 0);
  Serial.print(F(" Ki="));        Serial.print(Ki_VAL, 0);
  Serial.print(F(" Kd="));        Serial.print(Kd_VAL, 0);
  Serial.print(F("  KICK="));     Serial.print(KICK_PWM_C);
  Serial.print(F(" PWM / "));     Serial.print(KICK_DUREE_MS_C);
  Serial.println(F(" ms"));

  initSD();                         // tenter d'ouvrir un nouveau fichier CSV sur la carte SD
  lastControlTime = millis();       // initialiser le timer de contrôle à l'instant présent
  lastLogTime     = millis();       // initialiser le timer de log à l'instant présent
}

/* ===========================================================
 * loop() — BOUCLE PRINCIPALE (répétée indéfiniment)
 * S'exécute aussi vite que possible. La régulation et le log
 * sont cadencés par des timers (toutes les 50 ms et 100 ms).
 * =========================================================== */
void loop() {
  uint32_t now = millis();   // lire l'horloge système en millisecondes (débordement tous les 49 jours)

  gererBoutonStartStop(now);   // vérifier l'état du bouton et décider start/stop

  if (!programmeActif) {        // si le programme est à l'arrêt :
    appliquerCommande(0.0f);    //   forcer le moteur à 0 en permanence (sécurité)
    lastControlTime = now;      //   maintenir le timer à jour pour éviter un grand dt au démarrage
    lastLogTime     = now;      //   idem pour le timer de log
    return;                     //   sortir immédiatement de loop() sans rien calculer
  }

  // ── Cadençage : exécuter le bloc de contrôle toutes les 50 ms ──
  if (now - lastControlTime >= CONTROL_PERIOD_MS) {
    uint32_t dt_ms = now - lastControlTime;  // temps réel écoulé depuis le dernier cycle (peut différer de 50 ms)
    lastControlTime = now;                    // mettre à jour le timestamp de référence
    float dt_s = (float)dt_ms / 1000.0f;    // convertir en secondes pour les calculs PID

    // ── ÉTAPE 1 : MESURES ────────────────────────────────────
    vitesse_ms         = calculerVitesse(dt_ms);         // vitesse fréquencemétrique (pour log seulement)
    vitesse_periode_ms = calculerVitessePeriode();        // vitesse période filtrée (utilisée par le PID)
    float dist_m       = lireDistance();                  // distance ultrason en mètres (-1 si invalide)

    // ── ÉTAPE 2 : MÉMOIRE D'OBSTACLE ─────────────────────────
    // Le HC-SR04 renvoie parfois -1 (timeout) même avec un obstacle présent.
    // Sans mémoire, un seul -1 ferait basculer en mode vitesse pour 50 ms,
    // remettant l'intégrale à 0, puis retour en mode distance → instabilité.
    // Solution : si une mesure invalide arrive dans les 300 ms suivant une
    // mesure valide, on réutilise la dernière valeur valide.
    static float    derniereDist_m     = -1.0f;   // dernière distance valide mémorisée
    static uint32_t dernierDistTime_ms = 0;        // timestamp de la dernière mesure valide
    const  uint16_t OBSTACLE_MEMORY_MS = 300;      // durée de validité de la mémoire d'obstacle

    if (dist_m >= 0.0f) {              // si la mesure est valide
      derniereDist_m     = dist_m;     //   mettre à jour la mémoire
      dernierDistTime_ms = now;         //   mémoriser l'heure
    }

    float dist_eff = dist_m;   // commencer avec la mesure brute
    if (dist_m < 0.0f                                    // si la mesure est invalide
        && derniereDist_m >= 0.0f                        // et qu'on avait une valeur mémorisée
        && derniereDist_m < DIST_MAX_OBSTACLE_M          // et que cette valeur signalait un obstacle
        && (now - dernierDistTime_ms) < OBSTACLE_MEMORY_MS) { // et que la mémoire n'a pas expiré
      dist_eff = derniereDist_m;   // utiliser la valeur mémorisée à la place de -1
    }

    // ── ÉTAPE 3 : DÉTERMINATION DU MODE ──────────────────────
    // Trois modes exclusifs, par ordre de priorité décroissante :
    //   Mode 1 — Urgence  : obstacle très proche → freinage maximal immédiat
    //   Mode 2 — Vitesse  : pas d'obstacle → maintenir Vc
    //   Mode 3 — Distance : obstacle en zone ACC → maintenir vitesse + freiner si trop proche
    uint8_t modeActuel;

    if (dist_eff > 0.0f && dist_eff < DIST_URGENCE_M + 0.5f * vitesse_periode_ms) {
      // Urgence : obstacle détecté ET sa distance est inférieure à 6 cm + marge cinétique
      // La marge 0.5×v permet d'anticiper l'urgence si la voiture roule vite
      modeActuel = 1;
    } else if (dist_eff < 0.0f || dist_eff >= DIST_MAX_OBSTACLE_M) {
      // Vitesse : pas d'obstacle détecté (dist=-1) OU obstacle trop loin (>1.20 m)
      modeActuel = 2;
    } else {
      // Distance : obstacle présent dans la zone [0.06 m, 1.20 m] → ACC actif
      modeActuel = 3;
    }

    // ── ÉTAPE 4 : INTERDIRE LE KICK SI OBSTACLE PROCHE ──────
    // Le kick envoie une impulsion de forte puissance.
    // Si un obstacle est proche, cette impulsion pourrait lancer
    // la voiture vers lui → collision → interdit.
    bool interdireKick = (modeActuel == 3)                             // en mode distance : toujours interdit
                      || (dist_eff > 0.0f && dist_eff < DIST_CONSIGNE_M); // obstacle dans la zone de sécurité

    // ── ÉTAPE 5 : GESTION DES TRANSITIONS DE MODE ────────────
    static uint8_t ancienMode = 0;   // mode du cycle précédent (persistant entre les appels)
    bool reinitDerivee = false;       // drapeau : réinitialiser le terme D ce cycle ?

    if (modeActuel != ancienMode) {   // si le mode vient de changer
      reinitDerivee = true;           // toujours réinitialiser la dérivée lors d'une transition
                                      // (évite un saut sur le terme D à cause du changement d'erreur)

      // Réinitialiser l'intégrale SEULEMENT en cas de transition depuis/vers l'urgence.
      // Les transitions 2↔3 sont intentionnellement continues (v19) :
      // garder l'intégrale évite le saut de commande qui causait l'accélération.
      if (modeActuel == 1 || ancienMode == 1) {
        integrale = 0.0f;   // reset I lors d'une transition urgence ↔ autre mode
      }
    }
    ancienMode = modeActuel;   // mémoriser le mode pour la comparaison du prochain cycle

    // ── ÉTAPE 6 : CALCUL DE LA COMMANDE ──────────────────────
    float erreur   = 0.0f;    // erreur de vitesse ε = Vc − v_mesurée (initialisée à 0)
    float commande = 0.0f;    // commande PWM à envoyer (initialisée à 0)

    if (modeActuel == 1) {
      // ── MODE 1 : URGENCE ────────────────────────────────────
      integrale          = 0.0f;          // vider l'intégrale → pas d'effet résiduel sur le freinage
      deriveeInitialisee = false;          // invalider le terme D (v peut chuter brutalement)
      erreur    = VITESSE_CIBLE_MS - vitesse_periode_ms;  // calculer l'erreur pour le log uniquement
      commande  = -255.0f;                // freinage maximal : -255 PWM → L298N en sens inverse

    } else if (modeActuel == 2) {
      // ── MODE 2 : VITESSE (régulation de croisière) ──────────
      // Erreur de vitesse : positive si on est trop lent, négative si trop rapide
      erreur    = VITESSE_CIBLE_MS - vitesse_periode_ms;
      // Calculer la correction PID
      float corr = calculerCorrecteur(erreur, dt_s, reinitDerivee);
      // Commande = feedforward (point de fonctionnement) + correction PID (écarts)
      commande  = FEEDFORWARD_PWM + corr;

    } else {
      // ── MODE 3 : DISTANCE (ACC réaliste, architecture v19) ──
      //
      // Principe : cmd = cmd_vitesse − freinage
      //
      // cmd_vitesse : le régulateur de vitesse tourne IDENTIQUEMENT au mode 2.
      //   Il essaie de maintenir Vc. L'intégrale continue de tourner (pas de reset).
      //   → la voiture maintient sa vitesse lors du passage en mode distance.
      //
      // freinage : terme de sécurité, UNIQUEMENT actif si d < Dc.
      //   freinage = KP_BRAKE × max(0, Dc − d)
      //   → d >= Dc : freinage = 0 → vitesse inchangée
      //   → d < Dc  : freinage > 0 → commande réduite → voiture freine

      erreur       = VITESSE_CIBLE_MS - vitesse_periode_ms;  // même erreur de vitesse que mode 2
      float corr   = calculerCorrecteur(erreur, dt_s, reinitDerivee);  // même PID que mode 2
      float cmd_vitesse = FEEDFORWARD_PWM + corr;  // commande qu'on aurait en mode vitesse

      // Terme de freinage : proportionnel à l'excès de proximité par rapport à Dc
      float freinage = KP_BRAKE_PWM * max(0.0f, DIST_CONSIGNE_M - dist_eff);

      commande = max(0.0f, cmd_vitesse - freinage);  // soustraire le freinage, jamais négatif ici

      // ── Anti-windup freinage ──────────────────────────────
      // Pendant le freinage, la voiture est ralentie de force sous Vc.
      // L'intégrale voit une erreur positive (v < Vc) et continue d'accumuler.
      // Sans correction, quand le freinage s'arrête, cette intégrale accumulée
      // ferait accélérer la voiture au-delà de Vc.
      // Solution : annuler l'accumulation du cycle courant quand le freinage est actif.
#if USE_I       // ce bloc n'est compilé que pour les correcteurs avec intégrale (PI et PID)
      if (freinage > 0.0f) {
        integrale -= erreur * dt_s;  // soustraire ce qu'on vient d'ajouter dans calculerCorrecteur()
        integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);  // re-saturer par sécurité
      }
#endif
    }

    // ── ÉTAPE 7 : RAMPE + KICK + ZONE MORTE ──────────────────
    bool urgence = (modeActuel == 1);  // drapeau urgence pour calculerCommandeMoteur()

    // Appliquer le kick-start et la rampe asymétrique sur la commande calculée
    commande = calculerCommandeMoteur(commande, dt_s, urgence, interdireKick, now);

    // Saturation finale à [-255, +255]
    commande = constrain(commande, -255.0f, 255.0f);

    // Anti-windup zone morte : si la commande atterrit dans [0, 76 PWM]
    // → le moteur est en roue libre mais le PID croit qu'il agit.
    // On annule l'accumulation de l'intégrale pour ce cycle.
    if (commande > 0.0f && commande < PWM_ZONE_MORTE) {
      integrale -= erreur * dt_s;  // annuler l'intégrale accumulée inutilement
      integrale = constrain(integrale, -INTEGRALE_MAX, INTEGRALE_MAX);  // re-saturer
    }

    derniereCommande = commande;    // mémoriser pour le log
    appliquerCommande(commande);    // envoyer au pont en H → moteur

    // ── ÉTAPE 8 : LOG CSV (toutes les 100 ms) ────────────────
    if (now - lastLogTime >= LOG_PERIOD_MS) {
      lastLogTime = now;  // mettre à jour le timer de log

      // En mode vitesse, loger dist = -1 (pas d'obstacle, distance non pertinente)
      float dist_log = (modeActuel == 2) ? -1.0f : dist_eff;

      // Écrire une ligne CSV sur la carte SD
      ecrireLog((float)now / 1000.0f, dist_log, erreur, modeActuel);
    }

    // ── ÉTAPE 9 : DEBUG PORT SÉRIE ────────────────────────────
    // Afficher l'état du système sur le moniteur série à chaque cycle de contrôle (50 ms)
    Serial.print(F("[")); Serial.print(MODE_STR); Serial.print(F("] "));  // [P], [PI], [PD] ou [PID]

    Serial.print(F("mode="));
    if      (modeActuel == 1) Serial.print(F("URGENCE "));  // mode urgence : freinage
    else if (modeActuel == 2) Serial.print(F("VITESSE "));  // mode vitesse : croisière
    else                      Serial.print(F("DIST    "));  // mode distance : ACC actif

    Serial.print(F("| v="));    Serial.print(vitesse_periode_ms, 3);  // vitesse mesurée (méthode période)
    Serial.print(F(" | d="));   Serial.print(dist_m, 3);               // distance brute du capteur ultrason
    Serial.print(F(" | e="));   Serial.print(erreur, 3);               // erreur de vitesse ε = Vc − v
    Serial.print(F(" | cmd=")); Serial.print(commande, 1);             // commande envoyée au moteur (PWM)

    if (modeActuel == 3) {    // en mode distance : afficher aussi la valeur du freinage
      float fr = KP_BRAKE_PWM * max(0.0f, DIST_CONSIGNE_M - dist_eff);  // recalculer le freinage pour l'affichage
      Serial.print(F(" | brk=")); Serial.print(fr, 1);   // valeur du freinage en PWM
    }

    Serial.print(F(" | kick=")); Serial.println(kickActif ? F("OUI") : F("non"));  // état du kick
  }
}

/* ===========================================================
 * initSD() — INITIALISATION DE LA CARTE MICROSD
 * Ouvre un nouveau fichier CSV avec un nom incrémental.
 * =========================================================== */
void initSD() {
  if (!SD.begin(SD_CS_PIN)) {  // tenter d'initialiser la SD sur la broche CS définie
    Serial.println(F("SD : échec init"));  // afficher l'erreur sur le port série
    sdOk = false;   // marquer la SD comme non disponible
    return;         // abandonner l'initialisation
  }

  char fn[13]; uint8_t n = 0;  // nom de fichier (max 8.3 = 12 chars + '\0') et numéro incrémental
  do {
    snprintf(fn, sizeof(fn), "%s%03d.CSV", MODE_STR, n++);  // générer "PID000.CSV", "PID001.CSV", etc.
  } while (SD.exists(fn) && n < 255);  // incrémenter jusqu'à trouver un nom non utilisé

  logFile = SD.open(fn, FILE_WRITE);  // ouvrir le fichier en écriture (crée le fichier si inexistant)
  if (!logFile) {   // si l'ouverture a échoué
    Serial.print(F("SD : impossible d'ouvrir ")); Serial.println(fn);  // afficher l'erreur
    sdOk = false;   // marquer la SD comme non disponible
    return;
  }

  // Écrire le commentaire d'en-tête avec les paramètres de l'essai (ligne commençant par #)
  logFile.print(F("#correcteur="));  logFile.print(MODE_STR);           // nom du correcteur
  logFile.print(F(",Kp="));          logFile.print(Kp_VAL, 1);          // gain proportionnel
  logFile.print(F(",Ki="));          logFile.print(Ki_VAL, 1);          // gain intégral
  logFile.print(F(",Kd="));          logFile.print(Kd_VAL, 1);          // gain dérivé
  logFile.print(F(",FF="));          logFile.print(FEEDFORWARD_PWM, 1); // feedforward calculé
  logFile.print(F(",Vc="));          logFile.println(VITESSE_CIBLE_MS, 2); // vitesse cible

  // Écrire la ligne d'en-tête CSV avec le nom des colonnes
  logFile.println(F("t_s,mode,vitesse_freq_ms,vitesse_periode_ms,"
                    "consigne_vitesse_ms,distance_m,consigne_dist_m,"
                    "erreur_m,commande_pwm"));

  logFile.flush();   // forcer l'écriture sur la carte SD (évite la perte des en-têtes en cas de coupure)

  strncpy(logFilename, fn, sizeof(logFilename));  // copier le nom du fichier dans logFilename
  logFilename[sizeof(logFilename)-1] = '\0';       // s'assurer que la chaîne est bien terminée par '\0'
  logLinesSinceFlush = 0;  // remettre le compteur de lignes depuis le dernier flush à zéro
  sdOk = true;             // marquer la SD comme opérationnelle

  Serial.print(F("SD : ")); Serial.println(fn);  // confirmer l'ouverture sur le port série
}

/* ===========================================================
 * fermerLogSD() — FERMETURE PROPRE DU FICHIER SD
 * =========================================================== */
void fermerLogSD() {
  if (sdOk && logFile) {   // si la SD est disponible et le fichier ouvert
    logFile.flush();        // écrire toutes les données en mémoire tampon sur la carte
    logFile.close();        // fermer proprement le fichier (finalise l'en-tête FAT)
  }
  sdOk = false;   // marquer la SD comme indisponible jusqu'à la prochaine initSD()
}

/* ===========================================================
 * ecrireLog() — ÉCRITURE D'UNE LIGNE CSV SUR LA CARTE SD
 * =========================================================== */
void ecrireLog(float t_s, float dist_m, float erreur_m, uint8_t mode) {
  if (!sdOk) return;  // ne rien faire si la SD n'est pas disponible

  logFile.print(t_s,               3); logFile.print(',');  // temps en secondes (3 décimales)
  logFile.print(mode);                 logFile.print(',');  // mode : 1=urgence, 2=vitesse, 3=distance
  logFile.print(vitesse_ms,        4); logFile.print(',');  // vitesse fréquencemétrique (m/s)
  logFile.print(vitesse_periode_ms,4); logFile.print(',');  // vitesse période filtrée (m/s) — utilisée par PID
  logFile.print(VITESSE_CIBLE_MS,  4); logFile.print(',');  // consigne de vitesse (m/s) — constante
  logFile.print(dist_m,            4); logFile.print(',');  // distance obstacle (-1 si mode vitesse)
  logFile.print(DIST_CONSIGNE_M,   4); logFile.print(',');  // consigne de distance (m) — constante
  logFile.print(erreur_m,          4); logFile.print(',');  // erreur de vitesse ε = Vc − v (m/s)
  logFile.println(derniereCommande, 1);                      // commande envoyée au moteur (PWM, newline)

  // Flush groupé : écrire physiquement sur la SD tous les LOG_FLUSH_EVERY_N_LINES cycles
  // (flush à chaque ligne serait trop lent et userait la carte SD inutilement)
  if (++logLinesSinceFlush >= LOG_FLUSH_EVERY_N_LINES) {
    logFile.flush();           // forcer l'écriture du buffer SD sur la carte
    logLinesSinceFlush = 0;    // remettre le compteur à zéro
  }
}
