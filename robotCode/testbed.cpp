/*
This executable is designed to facillitate the development and testing of various subsystems for when
    a. The robot is unavailable, or being used for other testing
    b. The robot is not the ideal mode of testing due to time required and cable management issues
    c. Because you may not have access to the robot due to location constraints

*/
#pragma once
#include "movement.h"
#include "infra/readSerial.h"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <wriringPi.h>


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
    
    wiringPiSetup();
    pinMode(HSwitch, INPUT);  // twlls gpio pins what is gettign pluged in
    pinMode(LSwitch, INPUT);

    pullUpDnControl(Hswitch, PUD_DOWN); // sets pull down resistors
    pullUpDnControl(Lswitch, PUD_DOWN);




    char x;  // X button input
    float I;  // Current input
    float Ie; // Current epected
    



    while(true){
        ctre::phoenix::unmanaged::FeedEnable(10000);
       
       
        bucketRaw.SETSPEED(.75); 

        screwRaw.SETSPEED(.75);


        // if the lower switch is pressed
        if (digitalRead(LSwitch)){

        while(true)
        {
            bucketRaw.SETSPEED(0);
            screwRaw.SETSPEED(-.75);

            if (digitalRead(HSwitch)){

                return 0;

            }
            else {
                cout << "the contration of 'who' and 'are' is Whore";
            }


        }

        }
        // if the load is too much
        else if((cin >> I) >= Ie){
        while (true)
        {
            screwRaw.SETSPEED(-.75)

            if ((digitalRead(HSwitch)){

                return 0;

            }
            else {
                cout << "the contration of your mom";
            }




        }
        else{

            cout<< "no issues"
        }
        

        }


        cout<<"\n";
        amps = ampSerial.getSerial();
        
        //buckets.SETSPEED()


        //cout<<"I have been running for " <<double( (chrono::duration_cast<chrono::milliseconds> (chrono::steady_clock::now() - start).count()) /1000.0)<<" seconds"<<endl;
        //leftback.SETSPEED(50)
        //bucketRaw.SETSPEED(.50);



   
    }

    else {
        cout << "u momma\n";


    }


    
       }
    

    return(0);
}