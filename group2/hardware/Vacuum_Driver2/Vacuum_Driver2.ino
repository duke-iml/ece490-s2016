int vacuum = 13;
int vacuum_state = 0;
boolean command = LOW;

void setup() {
  // Connect to PC
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for port to connect
  }
  
  pinMode(vacuum, OUTPUT);
  //Serial.println("Vacuum Driver Arduino");
  delay(1000); 
  Serial.println("c");

}


void loop() { 
  while (Serial.available() > 0) {
    
    // 49 is ascii number for "1"
    // send "1" (49) --> vacuum state = 49-48 = 1 --> HIGH
    vacuum_state = int(Serial.read()) - 48;
    if (vacuum_state == 1) {
      digitalWrite(vacuum, HIGH);
    }
    else{
      digitalWrite(vacuum, LOW);
    }
    Serial.println(vacuum_state);    
  }
}
