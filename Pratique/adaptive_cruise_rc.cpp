/*
  Simple Adaptive Cruise Control for RC Car with Data Logging (10 Hz)
  Hardware:
   - Arduino Nano Every
   - RC390 brushed motor via L298N H-bridge driver
   - HC-SR04 Ultrasonic sensor (front distance)
   - Hall effect sensor on wheel (speed feedback)
   - SD card module (SPI interface)

  Pin assignments (change as needed):
   - Ultrasonic: TRIG = 8, ECHO = 9
   - Hall sensor input = 2 (interrupt)
   - Motor PWM = 5, Motor DIR = 4
   - SD card CS pin = 10
*/

#include <SPI.h>
#include <SD.h>

#define TRIG_PIN 8
#define ECHO_PIN 9
#define HALL_PIN 2
#define MOTOR_PWM 5
#define MOTOR_DIR 4
#define SD_CS_PIN 10

// Parameters
const float WHEEL_CIRCUMFERENCE = 0.20;   // meters (adjust for your wheel)
const int PULSES_PER_REV = 1;             // Hall sensor pulses per revolution
const float TARGET_SPEED = 0.5;           // m/s desired cruising speed
const int SAFE_DISTANCE = 30;             // cm minimum following distance
const int MAX_PWM = 255;
const int MIN_PWM = 0;

// Variables
volatile unsigned long lastPulseTime = 0;
volatile unsigned long pulseInterval = 0;

float currentSpeed = 0.0;

File logFile;

// Timing for 10 Hz loop
unsigned long lastLoop = 0;
const unsigned long LOOP_INTERVAL = 100; // ms (10 Hz)

void closeLogFile() {
  if (logFile) {
    logFile.flush();
    logFile.close();
    Serial.println("Log file closed.");
  }
}

void setup() {
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(HALL_PIN, INPUT_PULLUP);
  pinMode(MOTOR_PWM, OUTPUT);
  pinMode(MOTOR_DIR, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallISR, RISING);

  Serial.begin(9600);

  // Initialize SD card
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD card init failed!");
  } else {
    Serial.println("SD card ready.");
    logFile = SD.open("data.csv", FILE_WRITE);
    if (logFile) {
      logFile.println("Time(s),Distance(cm),Speed(m/s),PWM");
      logFile.flush();
    }
  }

  // Register close function on exit/reset
  atexit(closeLogFile);
}

void loop() {
  if (millis() - lastLoop >= LOOP_INTERVAL) {
    lastLoop = millis();

    // Get distance from ultrasonic
    long distance = getDistanceCM();

    // Compute current speed
    currentSpeed = calcSpeed();

    // Adaptive Cruise Control logic
    int pwmOut = 0;
    if (distance > SAFE_DISTANCE) {
      float error = TARGET_SPEED - currentSpeed;
      pwmOut = constrain((int)(150 + error * 100), MIN_PWM, MAX_PWM);
    } else {
      pwmOut = 0;
    }

    // Drive motor
    analogWrite(MOTOR_PWM, pwmOut);
    digitalWrite(MOTOR_DIR, HIGH);

    // Debug output
    Serial.print("Distance: "); Serial.print(distance); Serial.print(" cm");
    Serial.print(" | Speed: "); Serial.print(currentSpeed); Serial.print(" m/s");
    Serial.print(" | PWM: "); Serial.println(pwmOut);

    // Log to SD card (keep file open for efficiency)
    if (logFile) {
      float nowSec = millis() / 1000.0;
      logFile.print(nowSec, 3); logFile.print(",");
      logFile.print(distance); logFile.print(",");
      logFile.print(currentSpeed); logFile.print(",");
      logFile.println(pwmOut);
      logFile.flush(); // flush buffer to ensure data is written
    }
  }
}

// Interrupt Service Routine for hall sensor
void hallISR() {
  unsigned long now = micros();
  pulseInterval = now - lastPulseTime;
  lastPulseTime = now;
}

// Calculate speed in m/s
float calcSpeed() {
  if (pulseInterval == 0) return 0.0;
  float revPerSec = 1000000.0 / (pulseInterval * PULSES_PER_REV);
  return revPerSec * WHEEL_CIRCUMFERENCE;
}

// Measure distance with HC-SR04
long getDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duration = pulseIn(ECHO_PIN, HIGH, 30000);
  long distance = duration * 0.034 / 2;
  return distance;
}
