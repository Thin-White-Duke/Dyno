#include "HX711.h"
#include "Config.h"
#include "VescUart.h"
#include "datatypes.h"
#include "TimerThree.h"
#include "TimerFive.h"
#include <LiquidCrystal.h>

// initialize the LCD library by associating any needed LCD interface pins6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// HX711 load cell
HX711 scale;
float weight_avg;    // measured tare weight
float braketorque;   // brake torque of the dyno

// dyno switches and controls
volatile boolean startstop_mode = LOW;
float controlValue = 0.0;

// VESC UART communication
struct bldcMeasure measuredVal;
bool ioread = LOW;

// essential dyno values
float rpm_el, rpm_mech;
float U_mot, U_bat;
float P_batt, P_mot_el, P_mot_mech  ;
float eta_vesc, eta_mot;

void setup() {

  lcd.begin(16,2);

  lcd.setCursor(0,0);
  lcd.print("Dude Technology ");
  lcd.setCursor(0,1);
  lcd.print("Calibrating ... ");

  scale.begin(HX711_DOUT, HX711_PD_SCK);           // "gain" omitted, default value 128 is used by the library
  scale.set_scale(SCALE_M_CALIB, SCALE_B_CALIB);   // sets SCALE_M and SCALE_B according calibration
  scale.set_tare(20);                              // initialize tare OFFSETs, therefore reset the scale to 0
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("finished        ");
  delay(1000);
  lcd.setCursor(0,1);
  lcd.print("tare: ");
  lcd.print(scale.get_offset());
  lcd.print("g");
  delay(3000);

  // initialize UART port
  SERIALIO.begin(115200);
  SetSerialPort(&SERIALIO);

  // dyno switches and controls
  pinMode(STARTSTOP_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(STARTSTOP_PIN), startstop_toggle, FALLING);

  // interrupt timer for pwm signal for KSQ
  Timer3.initialize(10000);          // initialize timer3 to 100Hz/10ms or higher
  Timer3.pwm(PWM_PIN, 0);            // setup pwm on PWM_PIN, 0% duty cycle (1023 is 100%)

  // interrupt timer for output routine
  Timer5.initialize(2000000);       // initialize timer and set a 2 second period
  Timer5.attachInterrupt(lcd_out);  // attaches lcd_out() as a timer overflow interrupt

  DEBUGSERIAL.begin(115200);
  SetDebugSerialPort(&DEBUGSERIAL);
#ifdef DEBUG
  DEBUGSERIAL.println();
  DEBUGSERIAL.println();
  DEBUGSERIAL.println();
  DEBUGSERIAL.println("Initializing DYNO");
  DEBUGSERIAL.println();

  DEBUGSERIAL.print  ("tare raw offset: \t\t");
  DEBUGSERIAL.println(scale.get_offset_raw());     // print tare OFFSET_RAW
  DEBUGSERIAL.print  ("tare offset: \t\t\t\t");
  DEBUGSERIAL.print  (scale.get_offset(),1);       // print tare OFFSET
  DEBUGSERIAL.println(" gram");
  DEBUGSERIAL.println();
  DEBUGSERIAL.println("raw readings from ADC:");
  DEBUGSERIAL.print  ("single: \t\t\t");
  DEBUGSERIAL.println(scale.read());			         // print a single raw reading from the ADC
  DEBUGSERIAL.print  ("average: \t\t");
  DEBUGSERIAL.println(scale.read_average(10));  	 // print the average of 20 readings from the ADC
  DEBUGSERIAL.println();
  DEBUGSERIAL.println("BRUTTO and TARE weight:");
  DEBUGSERIAL.print  ("brutto weight: \t\t");
  DEBUGSERIAL.print  (scale.get_weight(10),1);		 // print the weight average of 5 readings from the ADC
  DEBUGSERIAL.println(" gram");
  DEBUGSERIAL.print  ("tare weight: \t\t");
  DEBUGSERIAL.print  (scale.get_tare(10),1);	     // print the tare average of 5 readings from the ADC
  DEBUGSERIAL.println(" gram");
  DEBUGSERIAL.println();
  DEBUGSERIAL.println("Init done");
  DEBUGSERIAL.println();
  DEBUGSERIAL.println();
#endif

  DEBUGSERIAL.println();
  DEBUGSERIAL.println("rpm\t\terpm\t\tTorque\t\tU_batt\t\tI_batt\t\tP_batt\t\tU_mot\t\tI_mot\t\tP_mot_mech\t\teta_mot");

}

void loop() {

  weight_avg  = scale.get_tare(10);                    // read weight in gram
  braketorque = 9.81*weight_avg/1000. * LEN_LEV/10.;   // moment in Ncm
  controlValue = mapfloat(analogRead(CONTROL_PIN), 0, 1023, 0, 1);
  ioread = VescUartGetValue(measuredVal);              // read current values from VESC
  if (!ioread) {
    DEBUGSERIAL.println("Failed to get data from VESC !!!");
    DEBUGSERIAL.println();
  }

  rpm_el     = measuredVal.rpm;
  rpm_mech   = rpm_el / POLE_PAIR;
  U_bat      = measuredVal.inpVoltage;
  P_batt     = measuredVal.inpVoltage * measuredVal.avgInputCurrent;
  //U_mot      = measuredVal.dutyNow * measuredVal.inpVoltage; // not reliable
  U_mot      = P_batt / measuredVal.avgMotorCurrent;
  P_mot_el   = U_mot * measuredVal.avgMotorCurrent;
  P_mot_mech = 2 * PI * (braketorque + M_NULL)/100 * rpm_mech/60 ;
  eta_vesc   = P_mot_el / P_batt;
  eta_mot    = P_mot_mech / P_mot_el;

  if (startstop_mode) {   // critical control actions should be put in here
    //VescUartSetRPM(2000.0);
    //VescUartSetCurrent(1.0);
    VescUartSetDuty(0.5);
    // max weight, torque / current and min rpm (400) should not be exceeded
    // current must only be set when the motor rotates
    Timer3.setPwmDuty(PWM_PIN, map(analogRead(CONTROL_PIN), 0, 1023, 0, (int)CURR_TORQUE_MAX));
  }

#ifdef DEBUG
  //DEBUGSERIAL.print("single RAW reading reading:\t\t");
  //DEBUGSERIAL.print(scale.read());
  //DEBUGSERIAL.print("\t\t|\t\t single reading:\t\t");
  DEBUGSERIAL.print  ("weight:\t\t\t\t\t\t");
  DEBUGSERIAL.print  (scale.get_tare(), 1);
  DEBUGSERIAL.println(" gram");
  DEBUGSERIAL.print  ("weight average:\t\t");
  DEBUGSERIAL.print  (weight_avg, 1);
  DEBUGSERIAL.println(" gram");
  DEBUGSERIAL.print  ("braking torque:\t\t");
  DEBUGSERIAL.print  (braketorque,4);
  DEBUGSERIAL.println(" Ncm");
  DEBUGSERIAL.println();
  if (ioread) {
    SerialPrint(measuredVal);
    DEBUGSERIAL.println();
    DEBUGSERIAL.print  ("rpm_mech:\t\t");
    DEBUGSERIAL.print  (rpm_mech, 0);
    DEBUGSERIAL.println(" rpm");
    DEBUGSERIAL.print  ("U_bat:\t\t\t\t");
    DEBUGSERIAL.print  (U_bat, 1);
    DEBUGSERIAL.println(" V");
    DEBUGSERIAL.print  ("U_mot:\t\t\t\t");
    DEBUGSERIAL.print  (U_mot, 1);
    DEBUGSERIAL.println(" V");
    DEBUGSERIAL.print  ("P_batt:\t\t\t\t");
    DEBUGSERIAL.print  (P_batt, 1);
    DEBUGSERIAL.println(" Watt");
    DEBUGSERIAL.print  ("P_mot_el:\t\t\t");
    DEBUGSERIAL.print  (P_mot_el, 1);
    DEBUGSERIAL.println(" Watt");
    DEBUGSERIAL.print  ("P_mot_mech:\t");
    DEBUGSERIAL.print  (P_mot_mech, 1);
    DEBUGSERIAL.println(" Watt");
    DEBUGSERIAL.print  ("eta_vesc:\t\t\t");
    DEBUGSERIAL.print  (eta_vesc*100, 1);
    DEBUGSERIAL.println(" %");
    DEBUGSERIAL.print  ("eta_mot:\t\t\t");
    DEBUGSERIAL.print  (eta_mot*100, 1);
    DEBUGSERIAL.println(" %");
    DEBUGSERIAL.println();
  }
#endif
}

void startstop_toggle() {
  if (startstop_mode) {
    startstop_mode = LOW;
    VescUartSetRPM(0.0);
    VescUartSetCurrent(0.0);
    VescUartSetDuty(0.0) ;
    delay(200);
  }
  else if (!startstop_mode)  startstop_mode = HIGH;
}

float mapfloat(long x, long in_min, long in_max, long out_min, long out_max) {
  return (float)(constrain(x, in_min, in_max) - in_min) * (out_max - out_min) / (float)(in_max - in_min) + out_min;
}

void lcd_out() {
  lcd.clear();
  lcd.setCursor(0,0);
  //lcd.print(measuredVal.rpm,0);
  lcd.print(rpm_mech,0);
  lcd.print("rpm");

  lcd.setCursor(8,0);
  //lcd.print(measuredVal.avgMotorCurrent,1);
  lcd.print(measuredVal.avgInputCurrent,1);
  lcd.print("A");

  if (startstop_mode) {
    lcd.setCursor(13,0);
    lcd.print("ON");
  }
  else if (!startstop_mode) {
    lcd.setCursor(13,0);
    lcd.print("OFF");
  }

  lcd.setCursor(0,1);
  lcd.print(braketorque, 1);
  lcd.print("Ncm");

  //lcd.setCursor(11,1);
  //lcd.print(controlValue);
  //lcd.print("%");

  lcd.setCursor(11,1);
  lcd.print(eta_mot*100,1);
  lcd.print("%");

  // data for further evaluation
  //DEBUGSERIAL.println();
  //DEBUGSERIAL.println("duty\t\trpm\t\terpm\t\tTorque\t\tU_batt\t\tI_batt\t\tP_batt\t\tU_mot\t\tI_mot\t\tP_mot_mech\t\teta_mot");
  DEBUGSERIAL.print  (measuredVal.dutyNow, 1);
  DEBUGSERIAL.print  ("\t\t");
  DEBUGSERIAL.print  (rpm_mech, 0);
  DEBUGSERIAL.print  ("\t");
  DEBUGSERIAL.print  (rpm_el, 0);
  DEBUGSERIAL.print  ("\t\t");
  DEBUGSERIAL.print  (braketorque, 1);
  DEBUGSERIAL.print  ("\t\t\t");
  DEBUGSERIAL.print  (U_bat, 1);
  DEBUGSERIAL.print  ("\t\t\t");
  DEBUGSERIAL.print  (measuredVal.avgInputCurrent, 1);
  DEBUGSERIAL.print  ("\t\t\t");
  DEBUGSERIAL.print  (P_batt, 1);
  DEBUGSERIAL.print  ("\t\t\t");
  DEBUGSERIAL.print  (U_mot, 1);
  DEBUGSERIAL.print  ("\t\t\t");
  DEBUGSERIAL.print  (measuredVal.avgMotorCurrent, 1);
  DEBUGSERIAL.print  ("\t\t\t");
  DEBUGSERIAL.print  (P_mot_mech, 1);
  DEBUGSERIAL.print  ("\t\t\t\t\t");
  DEBUGSERIAL.print  (eta_mot, 2);
}
