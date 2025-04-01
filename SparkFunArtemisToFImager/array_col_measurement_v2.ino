

#include <Wire.h>
#include "mbed_fault_handler.h"
#include <SparkFun_VL53L5CX_Library.h>  //http://librarymanager/All#SparkFun_VL53L5CX

#define DATATHRESH 502
#define OUTPUTTHRESH 900
#define MEASTHRESH 6

SparkFun_VL53L5CX myImager;
VL53L5CX_ResultsData measurementData;  // Result data class structure, 1356 byes of RAM

int imageResolution = 0;               // Used to pretty print output
int imageWidth = 0;                    // Used to pretty print output

long measurements = 0;                 // Used to calculate actual output rate
long measurementStartTime = 0;         // Used to calculate actual output rate

uint16_t averages[8];
uint8_t numMeass[8];
bool mins[8];

void setup() {
        
  Serial.begin(115200);
  delay(1000);
  Serial.println("SparkFun VL53L5CX Imager Example");
  
  Wire.begin();            // This resets I2C bus to 100kHz
  Wire.setClock(1000000);  //Sensor has max I2C freq of 1MHz
  
  Serial.println("I2C initialized");
  
  Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
  while (!myImager.begin()) {
    Serial.println(F("Sensor not found - check your wiring. Trying again..."));
    delay(1000);
  }
  
  myImager.setResolution(8 * 8);               // Enable all 64 pads
  imageResolution = myImager.getResolution();  // Query sensor for current resolution - either 4x4 or 8x8
  imageWidth = sqrt(imageResolution);          // Calculate printing width
  
  // Using 4x4, min frequency is 1Hz and max is 60Hz
  // Using 8x8, min frequency is 1Hz and max is 15Hz
  myImager.setRangingFrequency(15);

  Serial.println("Initialized, starting to range");
  myImager.startRanging();
  myImager.setSharpenerPercent(20);
  
  measurementStartTime = millis();
}
void loop() {
  // Poll sensor for new data
  if (myImager.isDataReady() == true) {
    if (myImager.getRangingData(&measurementData)) { // Read distance data into array
      // The ST library returns the data transposed from zone mapping shown in datasheet
      // Pretty-print data with increasing y, decreasing x to reflect reality measurementData.distance_mm[x + y] < DATATHRESH
      for (int x = imageWidth - 1; x >= 0; x--) {
        averages[x] = 0;
        numMeass[x] = 0;
        for (int y = 0; y <= imageWidth * (imageWidth - 1); y += imageWidth) {
          if ((measurementData.nb_target_detected[x + y] > 0) && x > 0 && x < 7 && measurementData.distance_mm[x + y] < DATATHRESH && measurementData.target_status[x + y] == 5) {
            averages[x] += measurementData.distance_mm[x + y];
            numMeass[x] ++;
            // Serial.print(measurementData.distance_mm[x + y]);
            // Serial.print("{");
            // Serial.print(measurementData.target_status[x + y]);
            // Serial.print("}");
          // } else {
          //   // Serial.print(2000);
          // }
          // Serial.print(measurementData.ambient_per_spad[x + y]);
          // Serial.print(",");
          }
        }
        // Serial.println();
        if(numMeass[x] >= MEASTHRESH){
          averages[x] /= numMeass[x];
        } else {
          averages[x] = 2000;
        }
        Serial.println(averages[x]);
      }
      Serial.println();

      int min = OUTPUTTHRESH;
      int minCol = 9;
      for (int i = imageWidth - 1; i >= 0; i--) {
        mins[i] = false;
        if (min >= averages[i] && averages[i] != 0 ){
          min = averages[i];
          minCol = i;
        }
      }
      analogWrite(13, min < OUTPUTTHRESH ? 10 * (minCol + 1) : 200);
      // Serial.print(minCol);
      // Serial.print(", ");
      // Serial.println(min < OUTPUTTHRESH ? 10 * (minCol + 1) : 200);
    }

    // Uncomment to display actual measurement rate 0.421 random number i need to remember for jake.
    // measurements++;
    // float measurementTime = (millis() - measurementStartTime) / 1000.0;
    // Serial.print("rate: ");
    // Serial.print(measurements / measurementTime, 3);
    // Serial.println("Hz");
  }
  delay(5);
}