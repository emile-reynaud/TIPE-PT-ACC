/**
 * Diagnostic complet carte microSD
 * Shield Arduino ASX00061 + Arduino Nano Every
 * Pin CS : D4
 */

#include <SD.h>
#include <SPI.h>

const int CS_PIN = 4;

// ── Utilitaires ──────────────────────────────────────────────────────────────

void printSeparator(const char* title) {
  Serial.println();
  Serial.println(F("========================================"));
  Serial.println(title);
  Serial.println(F("========================================"));
}

void printResult(const char* label, bool ok) {
  Serial.print(F("  ["));
  Serial.print(ok ? F("OK") : F("FAIL"));
  Serial.print(F("] "));
  Serial.println(label);
}

// ── 1. Détection & initialisation ────────────────────────────────────────────

bool testInit() {
  printSeparator("1. DETECTION & INITIALISATION");

  Serial.print(F("  Initialisation SPI (CS=D"));
  Serial.print(CS_PIN);
  Serial.println(F(")..."));

  if (!SD.begin(CS_PIN)) {
    printResult("Carte microSD détectée", false);
    Serial.println(F("  >> Vérifiez : carte insérée ? format FAT32 ? contacts propres ?"));
    return false;
  }

  printResult("Carte microSD détectée et montée", true);
  return true;
}

// ── 2. Informations sur le volume ─────────────────────────────────────────────

void testVolumeInfo() {
  printSeparator("2. INFORMATIONS DU VOLUME");

  // Type de système de fichiers (FAT16 ou FAT32 selon la bibliothèque SD)
  // La lib SD Arduino n'expose pas le type directement, on affiche ce qu'on peut.
  Serial.println(F("  Système de fichiers : FAT (16 ou 32)"));

  // Taille totale approximative via fichier temporaire
  Serial.println(F("  (Taille exacte non accessible via lib SD standard)"));
  Serial.println(F("  Utilisez un PC pour vérifier la capacité formatée."));
}

// ── 3. Écriture ───────────────────────────────────────────────────────────────

bool testWrite() {
  printSeparator("3. TEST D'ECRITURE");

  const char* filename = "DIAG.TXT";

  // Supprime l'ancien fichier si existant
  if (SD.exists(filename)) {
    SD.remove(filename);
  }

  File f = SD.open(filename, FILE_WRITE);
  if (!f) {
    printResult("Ouverture fichier en écriture", false);
    return false;
  }
  printResult("Ouverture fichier en écriture", true);

  // Écriture d'un bloc de données
  const char* payload = "Arduino Nano Every + ASX00061 - Test diagnostic microSD\n"
                        "ABCDEFGHIJKLMNOPQRSTUVWXYZ 0123456789\n"
                        "Ligne de test : caracteres speciaux ! @ # $ % & * ( )\n";

  size_t written = f.print(payload);
  f.close();

  bool ok = (written > 0);
  printResult("Écriture des données", ok);
  if (ok) {
    Serial.print(F("  >> "));
    Serial.print(written);
    Serial.println(F(" octets écrits dans DIAG.TXT"));
  }
  return ok;
}

// ── 4. Lecture & vérification ─────────────────────────────────────────────────

bool testRead() {
  printSeparator("4. TEST DE LECTURE");

  const char* filename = "DIAG.TXT";

  if (!SD.exists(filename)) {
    printResult("Fichier DIAG.TXT présent", false);
    return false;
  }
  printResult("Fichier DIAG.TXT présent", true);

  File f = SD.open(filename);
  if (!f) {
    printResult("Ouverture fichier en lecture", false);
    return false;
  }
  printResult("Ouverture fichier en lecture", true);

  Serial.println(F("  -- Contenu du fichier --"));
  long byteCount = 0;
  while (f.available()) {
    char c = f.read();
    Serial.print(c);
    byteCount++;
  }
  f.close();

  Serial.println();
  Serial.print(F("  >> "));
  Serial.print(byteCount);
  Serial.println(F(" octets lus."));
  printResult("Lecture complète", (byteCount > 0));
  return (byteCount > 0);
}

// ── 5. Performance (vitesse) ──────────────────────────────────────────────────

void testPerformance() {
  printSeparator("5. TEST DE PERFORMANCE");

  const char* filename = "BENCH.BIN";
  const int BLOCK_SIZE  = 512;
  const int NUM_BLOCKS  = 20;   // 20 × 512 = 10 Ko
  uint8_t buf[BLOCK_SIZE];

  // Remplissage du buffer
  for (int i = 0; i < BLOCK_SIZE; i++) buf[i] = (uint8_t)(i & 0xFF);

  // --- Écriture ---
  if (SD.exists(filename)) SD.remove(filename);
  File fw = SD.open(filename, FILE_WRITE);
  if (!fw) {
    printResult("Benchmark écriture", false);
    return;
  }

  unsigned long t0 = millis();
  for (int b = 0; b < NUM_BLOCKS; b++) fw.write(buf, BLOCK_SIZE);
  fw.flush();
  fw.close();
  unsigned long tWrite = millis() - t0;

  float writeKBs = (float)(BLOCK_SIZE * NUM_BLOCKS) / tWrite;  // Ko/s ≈ octets/ms
  Serial.print(F("  Ecriture : "));
  Serial.print(BLOCK_SIZE * NUM_BLOCKS);
  Serial.print(F(" octets en "));
  Serial.print(tWrite);
  Serial.print(F(" ms  →  "));
  Serial.print(writeKBs, 1);
  Serial.println(F(" Ko/s"));

  // --- Lecture ---
  File fr = SD.open(filename);
  if (!fr) {
    printResult("Benchmark lecture", false);
    return;
  }

  t0 = millis();
  while (fr.available()) fr.read(buf, BLOCK_SIZE);
  fr.close();
  unsigned long tRead = millis() - t0;

  float readKBs = (float)(BLOCK_SIZE * NUM_BLOCKS) / tRead;
  Serial.print(F("  Lecture  : "));
  Serial.print(BLOCK_SIZE * NUM_BLOCKS);
  Serial.print(F(" octets en "));
  Serial.print(tRead);
  Serial.print(F(" ms  →  "));
  Serial.print(readKBs, 1);
  Serial.println(F(" Ko/s"));

  SD.remove(filename);
  printResult("Benchmark terminé", true);
}

// ── 6. Listing de la racine ───────────────────────────────────────────────────

void testListing() {
  printSeparator("6. LISTING DE LA RACINE");

  File root = SD.open("/");
  if (!root) {
    printResult("Ouverture de la racine", false);
    return;
  }

  int count = 0;
  while (true) {
    File entry = root.openNextFile();
    if (!entry) break;
    Serial.print(F("  "));
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(F("  <DIR>"));
    } else {
      Serial.print(F("  \t"));
      Serial.print(entry.size());
      Serial.println(F(" octets"));
    }
    entry.close();
    count++;
  }
  root.close();

  if (count == 0) Serial.println(F("  (racine vide)"));
  Serial.print(F("  >> "));
  Serial.print(count);
  Serial.println(F(" entrée(s) trouvée(s)"));
  printResult("Listing réussi", true);
}

// ── 7. Suppression & nettoyage ────────────────────────────────────────────────

void testDelete() {
  printSeparator("7. SUPPRESSION & NETTOYAGE");

  const char* filename = "DIAG.TXT";
  if (SD.exists(filename)) {
    SD.remove(filename);
    printResult("Suppression de DIAG.TXT", !SD.exists(filename));
  } else {
    Serial.println(F("  DIAG.TXT déjà absent, rien à supprimer."));
  }
}

// ── Résumé final ──────────────────────────────────────────────────────────────

void printSummary(bool init, bool write, bool read) {
  printSeparator("RESUME DU DIAGNOSTIC");
  printResult("Détection & initialisation", init);
  if (init) {
    printResult("Écriture", write);
    printResult("Lecture",  read);
    Serial.println();
    if (init && write && read) {
      Serial.println(F("  ✔ La carte microSD fonctionne correctement."));
    } else {
      Serial.println(F("  ✘ Des problèmes ont été détectés. Voir les détails ci-dessus."));
    }
  }
}

// ── Setup / Loop ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Attendre l'ouverture du moniteur série

  Serial.println(F("\n╔══════════════════════════════════════════╗"));
  Serial.println(F("║  DIAGNOSTIC microSD - ASX00061 + NanoEvery ║"));
  Serial.println(F("╚══════════════════════════════════════════╝"));

  bool ok_init  = testInit();
  bool ok_write = false;
  bool ok_read  = false;

  if (ok_init) {
    testVolumeInfo();
    ok_write = testWrite();
    ok_read  = testRead();
    testPerformance();
    testListing();
    testDelete();
  }

  printSummary(ok_init, ok_write, ok_read);

  printSeparator("FIN DU DIAGNOSTIC");
  Serial.println(F("  Vous pouvez fermer le moniteur serie."));
}

void loop() {
  // Rien — le diagnostic est one-shot dans setup()
}
