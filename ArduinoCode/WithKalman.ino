#include <Wire.h>
#include <Adafruit_ADS1X15.h>
#include <MedianFilterLib2.h>

#include <Kalman.h>
#include "IRSensors.h"
#include "LinearAct.h"
#include <HX711_ADC.h>

#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#endif
//~ pins for each system ~

//~Load cells 
const int HX711_dout_1 = 2; //mcu > HX711 no 1 dout pin
const int HX711_sck_1 = 3; //mcu > HX711 no 1 sck pin
const int HX711_dout_2 = 4; //mcu > HX711 no 2 dout pin
const int HX711_sck_2 = 5; //mcu > HX711 no 2 sck pin
const int HX711_dout_3 = 6; //mcu > HX711 no 1 dout pin
const int HX711_sck_3 = 7; //mcu > HX711 no 1 sck pin
const int HX711_dout_4 = 8; //mcu > HX711 no 2 dout pin
const int HX711_sck_4 = 9; //mcu > HX711 no 2 sck pin

HX711_ADC LoadCell_1(HX711_dout_1, HX711_sck_1); //HX711 1
HX711_ADC LoadCell_2(HX711_dout_2, HX711_sck_2); //HX711 2
HX711_ADC LoadCell_3(HX711_dout_3, HX711_sck_3); //HX711 3
HX711_ADC LoadCell_4(HX711_dout_4, HX711_sck_4); //HX711 4

unsigned long t = 0;

float Load1;
float Load2;
float Load3;
float Load4;

//~ ~~~~~pins IR Sensors and Linear actuators~~~~~~~~~~


const int model = 20150;

const int pin[] = {A0,A1,A2,A3};
float dist[4];


//Calibration values for pots
const int potIn[] = {993,1023};
const int potOut[] = {111,139};         

//
int16_t rawResults = 0;
//float medianResult = 0;
int16_t rawResults2 = 0;
//float medianResult2 = 0;

 /* ADS1115  @ +/- .256V gain (16-bit results) */


float calibrator = .21/.29;

Adafruit_ADS1115 ads; 



int ticker = 1;
Kalman myFilter0(.008,.5,.01,0);
Kalman myFilter1(.008,.5,.01,0);
void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200); delay(3);


//~~~~~~~~~~~~~~~~~~~~~~~~~~ Load Cell portion~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 6


  float calV1 = -42.14; // Calibration value for each load cell must be set individually
  float calV2 = -7.56;
  float calV3 = -31.80;
  float calV4 = 696;

  LoadCell_1.begin();
  LoadCell_2.begin();
  LoadCell_3.begin();
  LoadCell_4.begin();

  unsigned long stabilizingtime = 2000; 
  /* preciscion right after power-up can be improved 
  by adding a few seconds of stabilizing time*/
  boolean _tare = true;                
  byte loadcell_1_rdy = 0;
  byte loadcell_2_rdy = 0;
  byte loadcell_3_rdy = 0;
  byte loadcell_4_rdy = 0;

   //run startup, stabilization and tare, both modules simultaniously
  while ((loadcell_1_rdy + loadcell_2_rdy + loadcell_3_rdy + loadcell_4_rdy) < 4){
    if (!loadcell_1_rdy) loadcell_1_rdy = LoadCell_1.startMultiple(stabilizingtime, _tare);
    if (!loadcell_2_rdy) loadcell_2_rdy = LoadCell_2.startMultiple(stabilizingtime, _tare);
    if (!loadcell_3_rdy) loadcell_3_rdy = LoadCell_3.startMultiple(stabilizingtime, _tare);
    if (!loadcell_4_rdy) loadcell_4_rdy = LoadCell_4.startMultiple(stabilizingtime, _tare);

  } 
  LoadCell_1.setCalFactor(calV1); // user set calibration value (float)
  LoadCell_2.setCalFactor(calV2); // user set calibration value (float)
  LoadCell_3.setCalFactor(calV3); // user set calibration value (float)
  LoadCell_4.setCalFactor(calV4); // user set calibration value (float)


//~~~~~~~~~~~~~~~~~~~~~~~~~~IR Sensor Portion~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~4



//~~~~~~~~~~~~~~~~~~~~~~~~~Linear Actuators~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



//~~~~~~~~~~~~~~~~~~~~~~~~~~curent sensor portion~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~2


ads.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
 
ads.begin();

if (!ads.begin()) {
    Serial.println("Failed to initialize ADS.");
    while (1);
  }
}

void loop() {


  //~~~~~~~~~~~~~~~~~~~~~~~~~~ Load Cell portion~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~9
 
static boolean newDataReady = 0;
const int serialPrintInterval = 0; //increase value to slow down serial print activity

  // check for new data/start next conversion:
  if (LoadCell_1.update()) newDataReady = true;
  LoadCell_2.update();
  LoadCell_3.update();
  LoadCell_4.update();

 //get smoothed value from data set
  if ((newDataReady)) {
    if (millis() > t + serialPrintInterval) {
       Load1 = LoadCell_1.getData();
       Load2 = LoadCell_2.getData();
       Load3 = LoadCell_3.getData();
       Load4 = LoadCell_4.getData();
      
      newDataReady = 0;
      t = millis();
    }
  }


//~~~~~~~~~~~~~~~~~~~~~~~~~~IR Sensor Portion~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ 0

int ws = 5;

for (int i = 0; i < 2; i++){
dist[i] = IRSensors(pin[i], ws, model);

}


//~~~~~~~~~~~~~~~~~~~~~~~~~~~~Pot Positioning~~~~~~~~~~~~~~~~~~

for (int i = 2; i < 4; i++){
dist[i] = LinearAct(pin[i],potOut[i-2],potIn[i-2]);
  
}

//~~~~~~~~~~~~~~~~~~~~~~~~~~~Current Sensor Portion~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~



  float multiplier = .0078125F;

rawResults = ads.readADC_Differential_0_1();
//round(20*sin(ticker/20.0));//
float I0 = rawResults * multiplier*calibrator;
 I0 = myFilter0.getFilteredValue(I0);//rawResults2 * multiplier*calibrator;
rawResults2 = ads.readADC_Differential_2_3();
float I1 = rawResults2 * multiplier*calibrator;
 I1 = myFilter1.getFilteredValue(I1);//rawResults2 * multiplier*calibrator;
ticker = ticker +1;

//~~~~~~~~~~~~~~~~~~~~~~~~~~~Serial Print ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~




Serial.print (Load1);
Serial.print(",");
Serial.print (Load2);
Serial.print(",");
Serial.print (Load3);
Serial.print(",");
Serial.print (Load4);
Serial.print(",");


Serial.print (dist[0]);
Serial.print(",");
Serial.print (dist[1]);
Serial.print(",");
Serial.print (dist[2]);+
Serial.print(",");
Serial.print (dist[3]);
Serial.print(",");

Serial.print (I0);
Serial.print(",");
Serial.print (I1);

Serial.println(";");



}
