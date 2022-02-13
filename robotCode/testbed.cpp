/*
This executable is designed to facillitate the development and testing of various subsystems for when
    a. The robot is unavailable, or being used for other testing
    b. The robot is not the ideal mode of testing due to time required and cable management issues
    c. Because you may not have access to the robot due to location constraints

*/

#include "movement.h"
#include "infra/readSerial.h"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
//#include <wiringPi.h>


#define LSwitch 0 // Switch repersenting digger is fully lowered
#define HSwitch 7// Switch representing digger is fully raised

using namespace std;

TalonPair* buckets;
TalonPair* screwdriver;
readSerial ampSerial((char*)"/dev/ttyACM0");



void setup(){
    buckets = new TalonPair(5);
    screwdriver = new TalonPair(6);

    string interface;
	//Might need to add this into main loop
	interface = "can0";
	int temp; 
	if (temp = (ctre::phoenix::platform::can::SetCANInterface(interface.c_str())) == -1){
		perror("");
		std::_Exit(0);
	}
    return;
}

int main(int argc, char* argv[]){
    setup();
    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    string amps;


    char x;  // X button input
    float I;  // Current input
    float Ie; // Current epected
    while(true) {
        cout<<ampSerial.getSerial() <<endl;


    }
}