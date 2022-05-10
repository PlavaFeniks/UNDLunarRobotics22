
#include <SharpIR.h>
#include <Math.h>


/* Model :
  GP2Y0A02YK0F --> 20150
  GP2Y0A21YK0F --> 1080
  GP2Y0A710K0F --> 100500
  GP2YA41SK0F --> 430
*/

//Function IRsensor
// pin is analog in pin, ws is the widow size for median filter
// Use larger window for smoother reading but will slow down

float IRSensors(int pins, int wss, int models){

SharpIR sensor = SharpIR(pins, models);
float distance_cm;
float median;

for (int i = 0; i < wss+1; i++){
    MedianFilter2<float> medianFilter(wss);

    // Get a distance measurement and store it as distance_cm:
    distance_cm = sensor.distance();

   median = medianFilter.AddValue(distance_cm);
}


return median;
//return distance_cm;
}
