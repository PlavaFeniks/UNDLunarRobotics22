
/*

header for linear actuators using a linear potentiometer positioning system

potOut is the bit reading from an arduino analog read pin when the actuator is fully exteneded
potIn  is the same thing but for fully Un-extended (or something) ,


 */

float LinearAct( int pin, int potOut, int potIn){

float inputBit = analogRead(pin);

float actLength = 1.97f; // Travel of linear actuator


float pos = (inputBit-potIn)*actLength/(potOut-potIn); //returns position in inches

//Serial.print("Realding ");
//Serial.println(inputBit);
return pos;
}
