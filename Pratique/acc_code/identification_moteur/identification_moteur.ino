/*
 * ============================================================
 *  IDENTIFICATION MOTEUR — estimation de K et τ
 *  TIPE CPGE PT — Arduino Nano Every
 * ============================================================
 *
 *  Principe :
 *  On applique un échelon de PWM au moteur et on mesure la
 *  réponse en vitesse. Le modèle du 1er ordre est :
 *
 *       τ × dv/dt + v(t) = K × u
 *
 *  Réponse à un échelon u = U₀ (constante) en partant de v=0 :
 *
 *       v(t) = K × U₀ × (1 − e^(−t/τ))
 *
 *  On en déduit :
 *    K × U₀ = v_∞         (vitesse de régime permanent)
 *    τ       = temps pour atteindre 63,2 % de v_∞
 *    K       = v_∞ / U₀   (U₀ = PWM_ECHELON / 255, normalisé)
 *
 *  Le programme effectue PLUSIEURS échelons à des PWM différents
 *  (PWM_MIN à PWM_MAX par pas de PWM_PAS) pour :
 *    1) détecter le seuil de démarrage réel du moteur
 *    2) vérifier la linéarité du modèle (K constant sur la plage)
 *    3) estimer τ de façon robuste (moyenne sur plusieurs échelons)
 *
 *  Chaque échelon dure DUREE_ECHELON_MS pour atteindre le régime.
 *  Entre deux échelons, on laisse le moteur s'arrêter (DUREE_ARRET_MS).
 *
 *  Sortie : port série à 115200 bauds.
 *  Format CSV (pour tracer avec Python / Excel) :
 *    t_ms, pwm, vitesse_ms
 *  Puis, après tous les échelons, un résumé :
 *    # PWM_echelon, v_inf_ms, K_estime, tau_estime_ms
 *
 *  Brancher la voiture sur un support (roues en l'air) ou
 *  sur un couloir droit suffisamment long.
 * ============================================================
 */

/* ─── Broches (identiques au projet AAC) ────────────────── */
#define HALL_PIN   2    // Capteur Hall — interruption INT0
#define ENB_PIN    9    // L298N ENB — PWM
#define IN3_PIN    A0   // L298N IN3
#define IN4_PIN    5    // L298N IN4
#define BUTTON_PIN 8    // Bouton START (même câblage que l'AAC)

/* ─── Paramètres physiques (identiques au projet AAC) ────── */
const float   PERIMETRE_M  = 3.14159f * 0.083f;  // π × 83 mm
const uint8_t NB_AIMANTS   = 3;
const float   PAS_M        = PERIMETRE_M / NB_AIMANTS;  // ≈ 0,0869 m

/* ─── Paramètres de l'identification ─────────────────────── */
// Série d'échelons : on teste ces valeurs de PWM une par une.
// On commence bas pour trouver le seuil, on monte pour la linéarité.
const uint8_t ECHELONS[] = {40, 60, 80, 100, 120, 150, 180, 210, 240};
const uint8_t NB_ECHELONS = sizeof(ECHELONS) / sizeof(ECHELONS[0]);

// Durée de chaque phase
const uint32_t DUREE_ECHELON_MS = 3000;  // 3 s : largement suffisant pour le régime
const uint32_t DUREE_ARRET_MS   = 2000;  // 2 s d'arrêt entre deux échelons

// Période d'échantillonnage de la mesure de vitesse
const uint16_t PERIODE_MESURE_MS = 50;   // 50 ms = même que l'AAC

// Seuil pour considérer que le régime permanent est atteint :
// la vitesse ne varie plus de plus de SEUIL_REGIME m/s sur les
// FENETRE_REGIME dernières mesures.
const float   SEUIL_REGIME_MS  = 0.005f; // 5 mm/s de variation max
const uint8_t FENETRE_REGIME   = 8;       // sur 8 × 50 ms = 400 ms

/* ─── Variables Hall (ISR) ────────────────────────────────── */
volatile uint32_t hallCount       = 0;
volatile uint32_t hallLastTime_us = 0;
volatile uint32_t hallPeriode_us  = 0;
volatile bool     hallNouveau     = false;

void hallISR() {
  hallCount++;
  uint32_t now = micros();
  if (hallLastTime_us != 0) {
    hallPeriode_us = now - hallLastTime_us;
    hallNouveau    = true;
  }
  hallLastTime_us = now;
}

/* ─── Variables globales ──────────────────────────────────── */
uint32_t lastHallCount  = 0;
float    vitesse_ms     = 0.0f;  // méthode fréquencemétrique (robuste à basse v)
float    vitesse_per_ms = 0.0f;  // méthode période (précise à v > 0)

// Résultats par échelon
float v_inf[NB_ECHELONS];    // vitesse de régime permanent (m/s)
float tau_ms[NB_ECHELONS];   // constante de temps estimée (ms)
float K_est[NB_ECHELONS];    // gain statique estimé


/* ─── Commande moteur ─────────────────────────────────────── */
void motorAvant(uint8_t pwm) {
  digitalWrite(IN3_PIN, HIGH);
  digitalWrite(IN4_PIN, LOW);
  analogWrite(ENB_PIN, pwm);
}

void motorStop() {
  // Freinage actif pour arrêt rapide entre les échelons
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, HIGH);
  analogWrite(ENB_PIN, 200);
  delay(300);
  // Puis roue libre
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);
}


/* ─── Calcul de vitesse ───────────────────────────────────── */

// Méthode fréquencemétrique : fiable même à très basse vitesse
float calculerVitesseFreq(uint32_t dt_ms) {
  noInterrupts();
  uint32_t count = hallCount;
  interrupts();
  uint32_t delta = count - lastHallCount;
  lastHallCount  = count;
  if (dt_ms == 0) return 0.0f;
  return ((float)delta / NB_AIMANTS) * PERIMETRE_M / (dt_ms / 1000.0f);
}

// Méthode période : précise à vitesse stable
float calculerVitessePeriode() {
  noInterrupts();
  uint32_t dernier = hallLastTime_us;
  interrupts();
  if (dernier == 0) return 0.0f;
  // Timeout 2 s → roue à l'arrêt
  if ((micros() - dernier) > 2000000UL) return 0.0f;
  noInterrupts();
  uint32_t per = hallPeriode_us;
  hallNouveau  = false;
  interrupts();
  if (per == 0) return 0.0f;
  float v = PAS_M / (per / 1000000.0f);
  return (v > 5.0f) ? vitesse_per_ms : v;  // garde-fou rebond
}


/* ─── Attendre le bouton START ────────────────────────────── */
void attendreBouton() {
  Serial.println(F("# Appuyez sur le bouton pour lancer l'identification..."));
  // Attendre relâchement d'abord (sécurité)
  while (digitalRead(BUTTON_PIN) == LOW) delay(10);
  // Attendre appui
  while (digitalRead(BUTTON_PIN) == HIGH) delay(10);
  delay(50);  // anti-rebond
  Serial.println(F("# Démarrage !"));
}


/* ─── Un échelon complet ──────────────────────────────────── */
/*
 * Applique un échelon de PWM `pwm_val` pendant DUREE_ECHELON_MS.
 * Envoie les données CSV sur le port série.
 * Estime v_∞ et τ.
 * Stocke les résultats dans v_inf[idx], tau_ms[idx], K_est[idx].
 */
void faireEchelon(uint8_t idx, uint8_t pwm_val) {

  Serial.print(F("# --- Echelon PWM = "));
  Serial.print(pwm_val);
  Serial.println(F(" ---"));
  Serial.println(F("t_ms,pwm,vitesse_freq_ms,vitesse_per_ms"));

  // Remise à zéro du capteur Hall
  noInterrupts();
  hallCount       = 0;
  hallLastTime_us = 0;
  hallPeriode_us  = 0;
  hallNouveau     = false;
  interrupts();
  lastHallCount = 0;
  vitesse_ms    = 0.0f;
  vitesse_per_ms = 0.0f;

  // Buffer circulaire pour détecter le régime permanent
  float buf_regime[FENETRE_REGIME] = {0};
  uint8_t buf_idx = 0;

  uint32_t t_debut      = millis();
  uint32_t lastMesure   = t_debut;
  float    t63_ms       = -1.0f;  // temps pour atteindre 63,2 % de v_∞
  float    v_max        = 0.0f;   // vitesse max observée (≈ v_∞)
  bool     regime_atteint = false;

  // Appliquer l'échelon
  motorAvant(pwm_val);

  while (millis() - t_debut < DUREE_ECHELON_MS) {
    uint32_t now = millis();

    if (now - lastMesure >= PERIODE_MESURE_MS) {
      uint32_t dt_ms = now - lastMesure;
      lastMesure = now;
      uint32_t t_relatif = now - t_debut;

      // Mesures de vitesse
      vitesse_ms     = calculerVitesseFreq(dt_ms);
      vitesse_per_ms = calculerVitessePeriode();

      // On utilise la méthode période si disponible (plus précise en régime)
      // sinon la méthode fréquencemétrique.
      float v_courante = (vitesse_per_ms > 0.01f) ? vitesse_per_ms : vitesse_ms;

      // Mise à jour de v_max
      if (v_courante > v_max) v_max = v_courante;

      // Détection de τ : premier passage à 63,2 % de v_max actuel
      // On ne le fait qu'une fois (t63_ms == -1) et seulement si
      // v_max est déjà crédible (> 0,02 m/s, pas du bruit).
      // Note : v_max évolue → on utilise une estimation conservative.
      if (t63_ms < 0 && v_max > 0.02f && v_courante >= 0.632f * v_max) {
        t63_ms = (float)t_relatif;
      }

      // Remplissage du buffer de régime
      buf_regime[buf_idx % FENETRE_REGIME] = v_courante;
      buf_idx++;

      // Détection du régime permanent (après au moins une fenêtre complète)
      if (buf_idx >= FENETRE_REGIME && !regime_atteint) {
        float v_min_buf = buf_regime[0], v_max_buf = buf_regime[0];
        for (uint8_t i = 1; i < FENETRE_REGIME; i++) {
          if (buf_regime[i] < v_min_buf) v_min_buf = buf_regime[i];
          if (buf_regime[i] > v_max_buf) v_max_buf = buf_regime[i];
        }
        if ((v_max_buf - v_min_buf) < SEUIL_REGIME_MS) {
          regime_atteint = true;
          // v_∞ = moyenne du buffer
          float somme = 0;
          for (uint8_t i = 0; i < FENETRE_REGIME; i++) somme += buf_regime[i];
          v_inf[idx] = somme / FENETRE_REGIME;
        }
      }

      // Sortie CSV
      Serial.print(t_relatif);        Serial.print(',');
      Serial.print(pwm_val);          Serial.print(',');
      Serial.print(vitesse_ms,   4);  Serial.print(',');
      Serial.println(vitesse_per_ms, 4);
    }
  }

  motorStop();

  // Si le régime n'a pas été détecté automatiquement (PWM trop faible
  // pour démarrer, ou moteur trop lent), v_∞ = v_max observée.
  if (!regime_atteint || v_inf[idx] < 0.005f) {
    v_inf[idx] = v_max;
  }

  // Estimation finale de τ :
  // Si on a détecté le passage à 63,2 % : on utilise t63_ms.
  // Sinon (régime pas atteint à temps) : τ est indéterminé.
  if (t63_ms > 0) {
    tau_ms[idx] = t63_ms;
  } else {
    tau_ms[idx] = -1.0f;  // indéterminé
  }

  // K = v_∞ / U₀   avec U₀ = pwm_val / 255 (normalisé entre 0 et 1)
  float U0 = (float)pwm_val / 255.0f;
  K_est[idx] = (U0 > 0.001f && v_inf[idx] > 0.005f) ? (v_inf[idx] / U0) : 0.0f;

  // Résumé de l'échelon
  Serial.print(F("# Resultat : v_inf="));  Serial.print(v_inf[idx], 4);
  Serial.print(F(" m/s  tau="));
  if (tau_ms[idx] > 0) { Serial.print(tau_ms[idx], 1); Serial.print(F(" ms")); }
  else                  { Serial.print(F("indetermine")); }
  Serial.print(F("  K="));               Serial.println(K_est[idx], 4);
  Serial.println();

  // Pause entre échelons
  delay(DUREE_ARRET_MS);
}


/* ─── Résumé final et conseils ────────────────────────────── */
void afficherResume() {
  Serial.println(F("# ============================================"));
  Serial.println(F("# RESUME DE L'IDENTIFICATION"));
  Serial.println(F("# PWM, v_inf_ms, K_estime, tau_ms"));

  // Calcul des moyennes sur les échelons valides (moteur démarre + τ connu)
  float somme_K   = 0, somme_tau = 0;
  uint8_t n_K = 0, n_tau = 0;
  uint8_t pwm_seuil = 0;  // plus petit PWM avec v_inf > 0,02 m/s

  for (uint8_t i = 0; i < NB_ECHELONS; i++) {
    Serial.print(F("# "));
    Serial.print(ECHELONS[i]);    Serial.print(F(", "));
    Serial.print(v_inf[i], 4);   Serial.print(F(", "));
    Serial.print(K_est[i], 4);   Serial.print(F(", "));
    if (tau_ms[i] > 0) Serial.println(tau_ms[i], 1);
    else                Serial.println(F("N/A"));

    if (K_est[i] > 0.01f) {
      somme_K += K_est[i];
      n_K++;
      if (pwm_seuil == 0) pwm_seuil = ECHELONS[i];
    }
    if (tau_ms[i] > 0) {
      somme_tau += tau_ms[i];
      n_tau++;
    }
  }

  Serial.println(F("# ============================================"));
  Serial.println(F("# VALEURS A ENTRER DANS simulation_AAC.py :"));

  if (n_K > 0) {
    float K_moy = somme_K / n_K;
    Serial.print(F("# K_MOTEUR  = "));
    Serial.println(K_moy, 4);
  } else {
    Serial.println(F("# K_MOTEUR  = indetermine (moteur n'a pas demarre)"));
  }

  if (n_tau > 0) {
    float tau_moy_s = (somme_tau / n_tau) / 1000.0f;
    Serial.print(F("# TAU_MOTEUR = "));
    Serial.print(tau_moy_s, 4);
    Serial.println(F(" s"));
  } else {
    Serial.println(F("# TAU_MOTEUR = indetermine"));
  }

  if (pwm_seuil > 0) {
    float seuil_norm = (float)pwm_seuil / 255.0f;
    Serial.print(F("# SEUIL_DEMARRAGE = "));
    Serial.print(seuil_norm, 3);
    Serial.print(F("  (PWM = "));
    Serial.print(pwm_seuil);
    Serial.println(F(")"));
  }

  Serial.println(F("# ============================================"));
  Serial.println(F("# FIN. Vous pouvez fermer le moniteur serie."));
}


/* ─── SETUP ──────────────────────────────────────────────── */
void setup() {
  Serial.begin(115200);

  // Hall
  pinMode(HALL_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, FALLING);

  // Moteur
  pinMode(ENB_PIN,  OUTPUT);
  pinMode(IN3_PIN,  OUTPUT);
  pinMode(IN4_PIN,  OUTPUT);
  analogWrite(ENB_PIN, 0);
  digitalWrite(IN3_PIN, LOW);
  digitalWrite(IN4_PIN, LOW);

  // Bouton
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // D10 en OUTPUT : nécessaire pour le SPI master même sans SD ici
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);

  Serial.println(F("# ============================================"));
  Serial.println(F("# IDENTIFICATION MOTEUR — AAC TIPE"));
  Serial.println(F("# ============================================"));
  Serial.print(F("# Roue : perimetre = "));
  Serial.print(PERIMETRE_M, 4);
  Serial.print(F(" m  |  aimants = "));
  Serial.println(NB_AIMANTS);
  Serial.print(F("# Echelons : "));
  for (uint8_t i = 0; i < NB_ECHELONS; i++) {
    Serial.print(ECHELONS[i]);
    if (i < NB_ECHELONS - 1) Serial.print(F(", "));
  }
  Serial.println();
  Serial.print(F("# Duree par echelon : "));
  Serial.print(DUREE_ECHELON_MS);
  Serial.println(F(" ms"));
  Serial.println(F("# Placez la voiture sur un support (roues libres)"));
  Serial.println(F("# ou dans un couloir suffisamment long."));
  Serial.println();

  attendreBouton();
}


/* ─── LOOP ───────────────────────────────────────────────── */
void loop() {
  // Tous les échelons sont lancés séquentiellement depuis setup()
  // via loop() au premier tour, puis on ne fait plus rien.

  static bool done = false;
  if (done) return;
  done = true;

  for (uint8_t i = 0; i < NB_ECHELONS; i++) {
    faireEchelon(i, ECHELONS[i]);
  }

  afficherResume();
}
