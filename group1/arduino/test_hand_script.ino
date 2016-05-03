const int RELAY_1_A = 2;
const int RELAY_1_B = 3;
const int ANALOG_POTENTIOMETER = 5;
const double POT_UPPER_BOUND = 1.0;
const double POT_LOWER_BOUND = 0.0;
bool extending = true;
int incomingByte = 0;

void setup() {
   pinMode(RELAY_1_A, OUTPUT);
   pinMode(RELAY_1_B, OUTPUT);
   //pinMode(ANALOG_POTENTIOMETER, INPUT); // Maybe?
   Serial.begin(9600);
}

void loop() {

//This is where your program logic goes
//You can call the functions to control the
//actuator here, as well as reading sensors, etc..

/*for(;;;)
{
  double reading = analogRead(ANALOG_POTENTIOMETER);
  if (extending)
  {
    if (reading < POT_UPPER_BOUND)
    {
      extendActuator(0);
    }
    else
    {
      retractActuator(0);
      extending = false;
    }
  }
  else
  {
    if(reading > POT_LOWER_BOUND)
    {
      retractActuator(0);
    }
    else
    {
      extendActuator(0);
      extending = true;
    }
  }
}*/

//openHand();
//closeHand();



//stopActuator(0);

//exit(0);

if (Serial.available() > 0)
{
  // Read the incoming byte
  incomingByte = Serial.read();

  // o will be open, c will be close
  // Say what you got
  Serial.print("I receive ");
  Serial.println(incomingByte, DEC);
  if (incomingByte == 99)
  {
    // That's a c, close
    closeHand();
  }

  else if(incomingByte == 111)
  {
    // That's an o, open
    openHand();
  }
}


}

void openHand()
{
  for (int i = 0; i < 30000; i++)
  {
    extendActuator(0);
  }
}

void closeHand()
{
  for (int i = 0; i < 30000; i++)
  {
    retractActuator(0);
  }
}
void extendActuator(int actuator) {
  //Set one relay one and the other off
  //this will move extend the actuator
  digitalWrite(RELAY_1_A, HIGH);
  digitalWrite(RELAY_1_B, LOW);
}

void retractActuator(int actuator) { 
  //Set one relay off and the other on 
  //this will move retract the actuator 
  digitalWrite(RELAY_1_A, LOW);
  digitalWrite(RELAY_1_B, HIGH); 
}

void stopActuator(int actuator) {
 //Set both relays off
 //this will stop the actuator in a braking
 digitalWrite(RELAY_1_A, LOW);
 digitalWrite(RELAY_1_B, LOW); 
 } 
