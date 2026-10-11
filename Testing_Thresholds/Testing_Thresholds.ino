void setup() {
  Serial.begin(250000);

  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);
}

void loop() {
  analogWrite(6, 80);
  Serial.print("Sensor A5: ");
  Serial.println(analogRead(A5));
  delay(10);
}
