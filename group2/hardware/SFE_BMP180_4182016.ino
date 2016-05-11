
#include <SFE_BMP180.h>
#include <Wire.h>

int samples = 0;
int pressureThreshold = 850;
double pressureArray[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int avgPressure = 1000;

SFE_BMP180 pressure;

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    
    ; // wait for port to connect
  }

//  // Initialize the sensor (it is important to get calibration values stored on the device).
  if (pressure.begin()) {
    Serial.println("BMP180 init success");
  }
  else {
    // Oops, something went wrong, this is usually a connection problem,
    // see the comments at the top of this sketch for the proper connections.
    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
}

void loop() {
  char status;
  double T,P,p0,a;
  T = 1;
  
  status = pressure.startPressure(3);
  if (status != 0) {
    // Wait for the measurement to complete:
    delay(status);
      
    //try to read pressure
    if (pressure.getPressure(P,T) != 0) {
        
      pressureArray[0] = pressureArray[1];
      pressureArray[1] = pressureArray[2];
      pressureArray[2] = pressureArray[3];
      pressureArray[3] = pressureArray[4];
      pressureArray[4] = pressureArray[5];
      pressureArray[5] = pressureArray[6];
      pressureArray[6] = pressureArray[7];
      pressureArray[7] = pressureArray[8];
      pressureArray[8] = pressureArray[9];
      pressureArray[9] = P;

      //Compute Average
      double totalPressure = 0;
      double countPressure;

      for (int i = 0 ; i < 10 ; i++) {
        if (pressureArray[i] != 0) {
          countPressure = countPressure + 1;
          totalPressure = totalPressure + pressureArray[i];
        }
      }
      avgPressure = totalPressure/countPressure;
    }   
  }    
  // every 100 ms, send the status to python
  int tiempo = millis();
  if (tiempo % 10 == 0) {
    //Check if pressure is above or below threshold
    if (avgPressure < pressureThreshold) {
      Serial.println('<'+String(1)+'>');
      //Serial.println('<'+String(avgPressure)+'>');
    }
    else {
    Serial.println('<'+String(0)+'>');
    //Serial.println('<'+String(avgPressure)+'>');
    }
  }
}

