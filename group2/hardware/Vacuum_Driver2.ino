int vacuum = 13;
int vacuum_state = 0;
boolean command = LOW;

void setup() {
  // Connect to PC
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for port to connect
  }
  Serial.println("Connected");
}

void loop() {
  delay(100);
  while (Serial.available() > 0) {
    vacuum_state = int(Serial.read()) - 48;
    if (vacuum_state == 1) {
      digitalWrite(53, HIGH);
      Serial.println(vacuum_state);
    }
    else{
      digitalWrite(53, LOW);
    }
  }
}
