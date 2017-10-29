#include "HX711.h"

HX711 scale;

void setup() {
  Serial.begin(38400);
  Serial.println();
  Serial.println("HX711 Demo");
  Serial.println();
  Serial.println("Initializing the scale");
  Serial.println();
  // parameter "gain" is omitted; the default value 128 is used by the library
  // HX711.DOUT	  - pin #A1
  // HX711.PD_SCK	- pin #A0
  scale.begin(A1, A0);

  // Prior to any measurement the regression line for the calibration have to be determined
  // by several measurements. The function has the generic form m = SCALE_M * measure + SCALE_B.
  // For the cheap 10 kg measurement cell the equation reads m [gram] = -0.0045 * x - 1662.8
  scale.set_scale(-0.004543, -1664.235837); // sets SCALE_M and SCALE_B according measurements

  scale.set_tare(20);                       // initialize tare OFFSETs, therefore reset the scale to 0
  Serial.print  ("tare OFFSET_RAW: \t\t");
  Serial.println(scale.get_offset_raw());   // print tare OFFSET_RAW
  Serial.print  ("tare OFFSET: \t\t");
  Serial.println(scale.get_offset());       // print tare OFFSET

  Serial.println("RAW readings from ADC:");
  Serial.print  ("value: \t\t");
  Serial.print  (scale.read());			         // print a single raw reading from the ADC
  Serial.print  ("\t\t|\t\t average: \t\t");
  Serial.println(scale.read_average(20));  	 // print the average of 20 readings from the ADC
  Serial.println();
  Serial.println("BRUTTO and TARE weight:");
  Serial.print  ("brutto weight: \t\t");
  Serial.println(scale.get_weight(5),1);		   // print the weight average of 5 readings from the ADC
  Serial.print  ("tare weight: \t\t");
  Serial.println(scale.get_tare(5),1);	     // print the tare average of 5 readings from the ADC
  Serial.println();
  Serial.println();
  Serial.println("Readings:");
}

void loop() {
  //Serial.print("single RAW reading reading:\t\t");
  //Serial.print(scale.read());
  Serial.print("\t\t|\t\t single reading:\t\t");
  Serial.print(scale.get_tare(), 1);
  Serial.print("\t\t|\t\t average:\t\t");
  Serial.println(scale.get_tare(10), 1);

  scale.power_down();			        // put the ADC in sleep mode
  delay(5000);
  scale.power_up();
}
