void setup() {
  Serial.begin(9600);
    while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  pinMode(13, OUTPUT);
  establishContact();
}

void loop() {
  if (Serial.available() > 0) {
    int inByte = Serial.read();
    if (inByte == 'H') {
      digitalWrite(13, HIGH);
    } else {
      digitalWrite(13, LOW);
    }
  }
}

void establishContact() {
  while (Serial.available() <= 0) {
    Serial.print('A');   // send a capital A
    delay(300);
  }
}
