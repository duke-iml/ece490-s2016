int vacuum_left = 10;
int vacuum_right = 5;
int vacuum_state = 0;
boolean command = LOW;

void setup() {
  // Connect to PC
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for port to connect
  }
  
  pinMode(vacuum_left, OUTPUT);
  pinMode(vacuum_right, OUTPUT);
  //Serial.println("Vacuum Driver Arduino");
  delay(100); 
  Serial.println("c");

}


void loop() { 
  while (Serial.available() > 0) {
    
    // 49 is ascii number for "1"
    // send "1" (49) --> vacuum state = 49-48 = 1 --> HIGH
    
    // 0: left vacuum off
    // 1: left vacuum on
    // 2: right vacuum off
    // 3: right vacuum on
    
    vacuum_state = int(Serial.read()) - 48;
    if (vacuum_state == 0) {
      digitalWrite(vacuum_left, LOW);
    }
    else if (vacuum_state == 1) {
      digitalWrite(vacuum_left, HIGH);
    }
    else if (vacuum_state == 2) {
      digitalWrite(vacuum_right, LOW);
    }
    else if (vacuum_state == 3) {
      digitalWrite(vacuum_right, HIGH);
    }    
    Serial.println(vacuum_state);    
  }
}
