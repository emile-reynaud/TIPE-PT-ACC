/*
  Adaptive Cruise Control (Proportional Control Only)
  Platform:
   - Arduino Nano Every + Nano Connector Carrier
   - RC390 brushed motor
   - L298N H-Bridge (Channel B)
   - HC-SR04 ultrasonic distance sensor
   - Hall effect speed sensor
   - Onboard microSD card slot (SPI)

  Pin mapping (per your setup):
   - HC-SR04 TRIG : D9
   - HC-SR04 ECHO : D10
   - Hall sensor S: D3 (interrupt capable)
   - L298N IN3    : D7
   - L298N IN4    : D8
   - L298N ENB    : D6 (PWM)
   - microSD CS   : D4 on carrier (handled by SD library internally)

  Wheel:
   - Diameter: 52 mm
   - Magnets: 13 per revolution

  Control:
   - Proportional speed control only (P controller)
   - Adaptive cruise using distance threshold

  Logging:
   - Logs ALL relevant data to CSV on microSD
   - Logging rate: 10 Hz
*/

#include <SPI.h>
#include <SD.h>

// ---------------- PIN DEFINITIONS ----------------
#define TRIG_PIN 9
#define ECHO_PIN 10
#define HALL_PIN 3
#define IN3 7
#define IN4 8
#define ENB 6
#define SD_CS_PIN 4   // Carrier routes this internally

// ---------------- VEHICLE PARAMETERS ----------------
const int MAGNETS_PER_REV = 13;
const float WHEEL_DIAMETER = 0.085;           // meters
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;

// ---------------- CONTROL PARAMETERS ----------------
const float TARGET_SPEED = 0.5;               // m/s
const int SAFE_DISTANCE_CM = 30;               // cm
const float KP = 120.0;                        // proportional gain

const int PWM_MIN = 0;
const int PWM_START = 100;                      // minimum PWM to overcome motor dead zone
const int PWM_MAX = 255;

// ---------------- TIMING ----------------
const unsigned long LOOP_INTERVAL_MS = 100;   // 10 Hz
unsigned long lastLoopTime = 0;

// ---------------- SPEED MEASUREMENT ----------------
volatile unsigned long lastPulseMicros = 0;
volatile unsigned long pulsePeriodMicros = 0;

float wheelSpeed = 0.0;                        // m/s

// ---------------- LOGGING ----------------
File logFile;

// ---------------- FUNCTION DECLARATIONS ----------------
long readDistanceCM();
float computeSpeed();
void driveForward(int pwm);
void stopMotor();
void hallISR();
void closeLogFile();

// ====================================================
void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);

  Serial.begin(115200);

    // -------- SD CARD INIT --------
  if (!SD.begin()) {
    Serial.println("SD card init FAILED");
  } else {
    Serial.println("SD card ready");

        // Create unique log filename with KP value
    // Example: ACC_KP120_000.CSV
    char filename[20];
    int kpInt = (int)KP;  // integer part for filename

    for (int i = 0; i < 1000; i++) {
      sprintf(filename, "ACC_KP%03d_%03d.CSV", kpInt, i);
      if (!SD.exists(filename)) {
        logFile = SD.open(filename, FILE_WRITE);
        if (logFile) {
          Serial.print("Logging to: ");
          Serial.println(filename);
          logFile.println("time_ms,distance_cm,speed_mps,target_speed,error,pwm");
          logFile.flush();
        }
        break;
      }
    }
  }

  atexit(closeLogFile);
}

// ====================================================
void loop() {
  if (millis() - lastLoopTime >= LOOP_INTERVAL_MS) {
    lastLoopTime = millis();

    // -------- SENSORS --------
    long distance = readDistanceCM();
    wheelSpeed = computeSpeed();

        // -------- CONTROL --------
    float error = TARGET_SPEED - wheelSpeed;
    int pwmCmd = 0;

    if (distance > SAFE_DISTANCE_CM) {
      int rawPWM = (int)(KP * error);

      if (rawPWM > 0) {
        pwmCmd = constrain(rawPWM + PWM_START, PWM_START, PWM_MAX);
        driveForward(pwmCmd);
      } else {
        stopMotor();
        pwmCmd = 0;
      }
    } else {
      stopMotor();
      pwmCmd = 0;
    }


    // -------- LOGGING --------
    if (logFile) {
      logFile.print(millis()); logFile.print(",");
      logFile.print(distance); logFile.print(",");
      logFile.print(wheelSpeed, 4); logFile.print(",");
      logFile.print(TARGET_SPEED, 3); logFile.print(",");
      logFile.print(error, 4); logFile.print(",");
      logFile.println(pwmCmd);
      logFile.flush();
    }

    // -------- DEBUG --------
    Serial.print("D:"); Serial.print(distance);
    Serial.print("cm | V:"); Serial.print(wheelSpeed, 2);
    Serial.print(" m/s | PWM:"); Serial.println(pwmCmd);
  }
}

// ====================================================
// -------------------- FUNCTIONS ---------------------

void hallISR() {
  unsigned long now = micros();
  pulsePeriodMicros = now - lastPulseMicros;
  lastPulseMicros = now;
}

float computeSpeed() {
  noInterrupts();
  unsigned long period = pulsePeriodMicros;
  interrupts();

  if (period == 0) return 0.0;

  float pulsesPerSecond = 1000000.0 / period;
  float revPerSecond = pulsesPerSecond / MAGNETS_PER_REV;
  return revPerSecond * WHEEL_CIRCUMFERENCE;
}

long readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  if (duration == 0) return 400; // timeout safety

  return (long)(duration * 0.034 / 2);
}

void driveForward(int pwm) {
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, pwm);
}

void stopMotor() {
  analogWrite(ENB, 0);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, HIGH);
}

void closeLogFile() {
  if (logFile) {
    logFile.flush();
    logFile.close();
  }
}
