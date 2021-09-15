/*
  io-port-dump.ino

  Dump IO port status to serial console. Just for quick debugging of
  board input status.
*/
void setup() {
  Serial.begin(9600);
  for (int i = 2; i <= 17; i++) {
    pinMode(i, INPUT_PULLUP);
  }
}

void loop() {
  for (int i = 2; i <= 17; i++) {
    int read = digitalRead(i);
    Serial.print(i);
    Serial.print(": ");    
    Serial.print(read);
    Serial.print(" - ");    
  }
  Serial.println("");
  delay(200);
}
