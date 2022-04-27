
#include <iostream>
// for delay function.
#include <chrono>
#include <map>
#include <string>
#include <thread>

// for signal handling
#include <signal.h>
#include "infra/readSerial.h"
#include <JetsonGPIO.h>


#define LSwitch 18 // Switch repersenting digger is fully lowered
#define HSwitch 22// Switch representing digger is fully raised

using namespace GPIO;
using namespace std;

void LimitSwitchTest(){


	GPIO::cleanup();
    //signal(SIGINT, signalHandler);
    GPIO::setmode(GPIO::BOARD);

    GPIO::setup(LSwitch, GPIO::IN);
    GPIO::setup(HSwitch, GPIO::IN);


    int switchL;
    int switchH;
    
    
    
while(true){
	
	
	switchL = GPIO::input(LSwitch);
	switchH = GPIO::input(HSwitch);

    if (switchL == 0){

    cout<<"Lowered"<<endl;
    //return;
    }
    else if (switchH == 0){
   
    cout<<"Raised"<<endl;
    std::_Exit(1);
    //return;
    }
    else {
        cout<<"neither Pressed" << endl;
    }

}
}


void MiningTime1(readSerial* ampSerial){
	GPIO::cleanup();
    //signal(SIGINT, signalHandler);
    GPIO::setmode(GPIO::BOARD);

    GPIO::setup(LSwitch, GPIO::IN);
    GPIO::setup(HSwitch, GPIO::IN);


    int switchL;
    int switchH;
    float *arr = (float*) malloc(sizeof(float) * 10);
    
    //setup();
    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    string amps;
    
   // wiringPiSetup();


    char  x;  // X button input
    float IBuckets;  // Current input
	float IScrew;  // Current input
	
    float  IBuckEX  = 9.5; // Current epected
	float  IScrewEX  = 7; // Current epected
    
    int BucketSpeed = 350; // quad speed = .1/60*4096*RPM/gearbox. Efficiency speed is 12150 rpm btw
//    int BucketSpeed = 0;
    //int buckScrewRatio = 5; //number of revolution of bucket shaft to revolutions of screw shaft
    //int ScrewSpeed = -800; //relation between bucket speed and screw speed
      float ScrewSpeed = -.3;
    //float ScrewSpeed = .5;
    int maxLoad = 30000;

    cout << "Mining Setup" << endl;

    //~~~~~~~~~~~~~Initialize Motors~~~~~~~~~~~~~~~~~~~

    float *limits = (float*)malloc(sizeof(float)*2);
    float *PID_valsBuck = (float*)malloc(sizeof(float)*4);
    float *PID_valsScrew = (float*)malloc(sizeof(float)*4);

    limits[0] = 0;
    limits[1]= 1;


    PID_valsBuck[PID_P] = .83;          // should probably tune one for buckets and one for screw
    PID_valsBuck[PID_I] = .000;
    PID_valsBuck[PID_D] = .8;
    PID_valsBuck[PID_F] = .5;
    
    PID_valsScrew[PID_P] = .22;          // should probably tune one for buckets and one for screw
    PID_valsScrew[PID_I] = .000;
    PID_valsScrew[PID_D] = .5;
    PID_valsScrew[PID_F] = .04;

    int loopCount = 0;

    TalonPair* buckets = new TalonPair(6,VELOCITY,limits, PID_valsBuck);
   
//    TalonPair* screwdriver= new TalonPair(5,VELOCITY,limits, PID_valsScrew);
TalonPair* screwdriver = new TalonPair(5);

	cout << "Mining Setup Done\n";
   
    
	arr = new float(10); 

std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() < 5)
{
screwdriver->SETSPEED(ScrewSpeed);
buckets->SETSPEED(BucketSpeed);
end = std::chrono::steady_clock::now();
}		
	while(true)
	{
		switchL = GPIO::input(LSwitch);
		switchH = GPIO::input(HSwitch);

		buckets->SETSPEED(BucketSpeed); // set speed for the two motors
		screwdriver->SETSPEED(ScrewSpeed); 
cout << "Regular IScrew: " << IScrew << endl;
cout << "Regular IBuckets: " << IBuckets << endl;
	
	

//		chrono::steady_clock::time_point start = chrono::steady_clock::now();
//		chrono::steady_clock::time_point end = chrono::steady_clock::now();


		arr = new float(10);            
cout << "amp serial entered\n";
		arr = ampSerial->getSerialVals(10);
cout << "amp serial exited\n";
		IBuckets = arr[9]; 	//
		IScrew = arr[8];	//
	
		float Regolith = (arr[0]+arr[1]+arr[2]+arr[3])/4.0f;

		end = std::chrono::steady_clock::now();
		// if the lower switch is pressed
		if (switchL == 0 or std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() > 120){
cout << "Finishing due to limit switch" << endl;
			while(true)
			{
				switchL = GPIO::input(LSwitch);
				switchH = GPIO::input(HSwitch);

				screwdriver->SETSPEED(-1.5*ScrewSpeed);
				buckets->SETSPEED(BucketSpeed);
				if (switchH == 0)
				{
					screwdriver->SETSPEED(0);
					buckets->SETSPEED(0);
					return;//mining complete
				}
			}
		}	
		
		else if(IScrew >= IScrewEX)
		{
cout << "IScred: " << IScrew << endl;
				switchL = GPIO::input(LSwitch);
				switchH = GPIO::input(HSwitch);
				arr = ampSerial->getSerialVals(10);
				
				IScrew = arr[8];
				
				screwdriver->SETSPEED(-1*ScrewSpeed);
				buckets->SETSPEED(BucketSpeed);
				if (switchH == 0)
				{
					screwdriver->SETSPEED(0);
					buckets->SETSPEED(0);
			   
					cout << "Finishing due to Excessive current, make adjustment";
					return;
				}
		}
		
		
		else if(IBuckets >= IBuckEX)
		{
while(IBuckets >= IBuckEX) {
cout << "IBuckets: " << IBuckets << endl;
				switchL = GPIO::input(LSwitch);
				switchH = GPIO::input(HSwitch);
				
				arr = ampSerial->getSerialVals(10);
				IBuckets = arr[9];
			
				
				screwdriver->SETSPEED(-2*ScrewSpeed);
				buckets->SETSPEED(BucketSpeed);
				if (switchH == 0)
				{
					screwdriver->SETSPEED(0);
					buckets->SETSPEED(0);
			   
					cout << "Finishing due to Excessive current, make adjustment";
					return;
				}
}
		}
		
		
		
		
		else if(Regolith >= maxLoad)
		{
			while (true){
				switchL = GPIO::input(LSwitch);
				switchH = GPIO::input(HSwitch);
				screwdriver->SETSPEED(-ScrewSpeed);
				buckets->SETSPEED(BucketSpeed);
				if (switchH == 0)
				{
					screwdriver->SETSPEED(0);
					buckets->SETSPEED(0);
					return;
				}
				else
				{
					cout << "Finish due to Hopper load";
				}
			}
		}
	}
		
	
}

/////////////////////
//dead function
// this function averages the reading of the load cells
/*
float hopperLoad(readSerial* ampSerial){
    
    
    float *arr;
    arr = ampSerial->getSerialVals(10);
    float Regolith = (arr[0]+arr[1]+arr[2]+arr[3])/4.0f;

    return Regolith;
}
*/
























