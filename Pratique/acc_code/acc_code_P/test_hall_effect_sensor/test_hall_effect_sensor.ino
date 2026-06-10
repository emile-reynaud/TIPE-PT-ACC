void setup() {
  pinMode(3, INPUT_PULLUP);
  Serial.begin(115200);
}

void loop() {
  Serial.println(digitalRead(3));
  delay(100);
}
