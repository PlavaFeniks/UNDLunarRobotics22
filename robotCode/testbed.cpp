/*
This executable is designed to facillitate the development and testing of various subsystems for when
    a. The robot is unavailable, or being used for other testing
    b. The robot is not the ideal mode of testing due to time required and cable management issues
    c. Because you may not have access to the robot due to location constraints

*/

#include "movement.h"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>

using namespace std;

TalonPair* buckets;
TalonPair* screwdriver;

TalonPair bucketRaw(1);
TalonPair screwRaw(6);
TalonPair leftback(1,VELOCITY);

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
    
    char x;
    if (( cin >> x) == 'x'){
    while(true){
        ctre::phoenix::unmanaged::FeedEnable(10000);
       
        //bucketRaw.SETSPEED(.50);
        cout<<"\n";
        
        cout<<"I have been running for " <<double( (chrono::duration_cast<chrono::milliseconds> (chrono::steady_clock::now() - start).count()) /1000.0)<<" seconds"<<endl;
        leftback.SETSPEED(50)
   
   
    }

    else {
        cout << "u momma\n";


    }


    
       }
    

    return(0);
}