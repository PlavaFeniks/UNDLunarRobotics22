
//If the CANAble doesn't switch to active (blue -> green) run `./run.sh` 
//from the ~/Documents/gitrepos/UNDLunarRobotics

// what to run
// ~/Documents/gitrepos/UNDLunarRobotics22/robotCode$ sudo ./bin/autonomous mining


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


#define hallEffect 15 // Switch repersenting digger is fully lowered

#define LSwitch 18 // Switch repersenting digger is fully lowered
#define HSwitch 22 // Switch representing digger is fully raised

#define homePos .5

using namespace GPIO;
using namespace std;

void actuatorPos(readSerial* ampSerial,float setPOS){


	TalonPair * actuatorR;
	TalonPair * actuatorL;

	actuatorR = new TalonPair(8);
    actuatorL = new TalonPair(9);

    float *arr;
    arr = new float(10);            
    arr = ampSerial->getSerialVals(10);
	arr = ampSerial->getSerialVals(10);
    float rightPos = arr[6]; 	//
    float leftPos = arr[7];	//

    float speedPercent = .9;

    cout<<"Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;

    while(true){
		arr = ampSerial->getSerialVals(10);
		rightPos = arr[6]; 	//
		leftPos = arr[7];	//
        if ( rightPos>setPOS){
			while(true)
			{
				cout << "first\n";

				arr = ampSerial->getSerialVals(10);
				rightPos = arr[6]; 	
				leftPos = arr[7];
				actuatorR->SETSPEED(speedPercent); // set speed for the two motors
				actuatorL->SETSPEED(speedPercent); // set speed for the two motors
				cout<<"END Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;

				if(rightPos<(setPOS+.1f) or leftPos<(setPOS+.1f)  ){
					actuatorR->SETSPEED(0); // set speed for the two motors
					actuatorL->SETSPEED(0);
					arr = ampSerial->getSerialVals(10);
					rightPos = arr[6]; 	//
					leftPos = arr[7];
					
					cout<<"END Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;
					return;

				}
			}
        }
        else if ( rightPos<setPOS){
			while(true)
				{
				cout << "second\n";

				arr = ampSerial->getSerialVals(10);
				rightPos = arr[6]; 	//
				leftPos = arr[7];
				actuatorR->SETSPEED(-speedPercent); // set speed for the two motors
				actuatorL->SETSPEED(-speedPercent); // set speed for the two motors
				cout<<"Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;
				if(rightPos>(setPOS-.1f) or leftPos>(setPOS-.1f)){
					actuatorR->SETSPEED(0); // set speed for the two motors
					actuatorL->SETSPEED(0);
					arr = ampSerial->getSerialVals(10);
					rightPos = arr[6]; 	//
					leftPos = arr[7];
					
					cout<<"END Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;
					return;
				}
			}			
        }
    }
}
void actuatorCalibration(readSerial* ampSerial){


	TalonPair * actuatorR;
	TalonPair * actuatorL;

	actuatorR = new TalonPair(8);
    actuatorL = new TalonPair(9);

    float *arr;
    arr = new float(10);            
    arr = ampSerial->getSerialVals(10);
	arr = ampSerial->getSerialVals(10);
    float rightPos = arr[6]; 	//
    float leftPos = arr[7];	//

    float speedPercent = .8;

    cout<<"Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;
    
    std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
    

		while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() < 10)
		{
			arr = ampSerial->getSerialVals(10);

			end = std::chrono::steady_clock::now();
			arr = ampSerial->getSerialVals(10);
			rightPos = arr[6]; 	//
			leftPos = arr[7];
			actuatorR->SETSPEED(speedPercent); // set speed for the two motors
			actuatorL->SETSPEED(speedPercent); // set speed for the two motors

		}
	cout<<"Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;
	cout<<"press enter"<<endl;
    cin.get();
   
    std::chrono::steady_clock::time_point begin2 = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end2 = std::chrono::steady_clock::now();
	
	
	
		while(std::chrono::duration_cast<std::chrono::seconds>(end2-begin2).count() < 10)
		{
			arr = ampSerial->getSerialVals(10);

			end2 = std::chrono::steady_clock::now();
			arr = ampSerial->getSerialVals(10);
			rightPos = arr[6]; 	//
			leftPos = arr[7];
			actuatorR->SETSPEED(-speedPercent); // set speed for the two motors
			actuatorL->SETSPEED(-speedPercent); // set speed for the two motors

		}
		 return;
		 
	cout<<"Right POS "<< arr[6]<< " Left POS " << arr[7] << endl;

	
}

void LimitSwitchTest(){


	GPIO::cleanup();
    //signal(SIGINT, signalHandler);
    GPIO::setmode(GPIO::BOARD);

    GPIO::setup(LSwitch, GPIO::IN);
	GPIO::setup(HSwitch, GPIO::IN);
	GPIO::setup(hallEffect, GPIO::IN);

	int switchL;
	int switchH;

    
    
	while(true)
	{
	
		switchL = GPIO::input(LSwitch);
		switchH = GPIO::input(HSwitch);
		hallBoi = GPIO::input(hallEffect);

		if (switchL == 1)
		{

			cout<<"Lowered"<<endl;
			//return;
		}
		else if (switchH == 1)
		{		   
			cout<<"Raised"<<endl;
			//std::_Exit(1);
			//return;
		}
		else if (hallBoi == 0 )
		{
				cout<<"Halleffect"<<endl;
			
		}
		else {
			cout<<"neither Pressed" << endl;
		}
	}
}

void preMining(readSerial* ampSerial, TalonPair* buckets, TalonPair* screwdriver){

    //signal(SIGINT, signalHandler);
    GPIO::setmode(GPIO::BOARD);


    GPIO::setup(LSwitch, GPIO::IN);
	GPIO::setup(hallEffect, GPIO::IN);
    GPIO::setup(HSwitch, GPIO::IN);

	int hallBoi;

	actuatorPos(ampSerial,1.00);
			cout<<"act"<<endl;

    //int BucketSpeed = 350; // quad speed = .1/60*4096*RPM/gearbox. Efficiency speed is 12150 rpm btw
	int BucketSpeed = 0; // quad speed = .1/60*4096*RPM/gearbox. Efficiency speed is 12150 rpm btw

	float ScrewSpeed = 0;

	//float ScrewSpeed = -.3;


	// initialize timer stuff

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();


		while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() > 10){

			end = std::chrono::steady_clock::now();
			buckets->SETSPEED(BucketSpeed); // set speed for the two motors
			screwdriver->SETSPEED(ScrewSpeed); 
			cout<<"mining1"<<endl;
		}

	actuatorPos(ampSerial,.70);
	begin = std::chrono::steady_clock::now();
	end = std::chrono::steady_clock::now();

		while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() > 10){

			end = std::chrono::steady_clock::now();
			buckets->SETSPEED(BucketSpeed); // set speed for the two motors
			screwdriver->SETSPEED(ScrewSpeed); 
			cout<<"mining2"<<endl;
		}

	actuatorPos(ampSerial,.35);
	begin = std::chrono::steady_clock::now();
	end = std::chrono::steady_clock::now();

		while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() > 10){

			end = std::chrono::steady_clock::now();
			buckets->SETSPEED(BucketSpeed); // set speed for the two motors
			screwdriver->SETSPEED(ScrewSpeed); 
			cout<<"mining3"<<endl;
		}

		return;

}


void undeployBoi(readSerial* ampSerial,TalonPair* buckets, TalonPair* screwdriver){

GPIO::setup(hallEffect, GPIO::IN);

int switchL;
int switchH;
int hallBoi;


hallBoi = GPIO::input(hallEffect); //lower limit switch
switchL = GPIO::input(LSwitch); //lower limit switch
switchH = GPIO::input(HSwitch); //upper limit switch


int BucketSpeed = 350;
float ScrewSpeed = -.3;

	if(switchH !=1){

		screwdriver->SETSPEED(-ScrewSpeed);
		switchH = GPIO::input(HSwitch); //upper limit switch

		if (switchL == 1){
			cout<<"direction is set wrong/wtf"<<endl;
			return;
		}

	}

	while ( hallBoi != 0){
		hallBoi =GPIO::input(hallEffect); //lower limit switch

		buckets->SETSPEED(BucketSpeed);
	}

actuatorPos(ampSerial,homePos);

return;

}



void MiningTime1(readSerial* ampSerial, TalonPair* buckets, TalonPair* screwdriver){
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

   

    int loopCount = 0;


	cout << "Mining Setup Done\n";
   
    
	arr = new float(10); 

	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point end = std::chrono::steady_clock::now();
	
	//run belts and screw for x amount of seconds
	/*
	while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() < 5)
	{
		screwdriver->SETSPEED(ScrewSpeed);
		buckets->SETSPEED(BucketSpeed);
		end = std::chrono::steady_clock::now();
	}		
	*/

	//main logic loop
		while(true)
		{
			switchL = GPIO::input(LSwitch); //lower limit switch
			switchH = GPIO::input(HSwitch); //upper limit switch

			buckets->SETSPEED(BucketSpeed); // set speed for the two motors
			screwdriver->SETSPEED(ScrewSpeed); 
			cout << "Regular IScrew: " << IScrew << endl;
			cout << "Regular IBuckets: " << IBuckets << endl;
		
		

			//chrono::steady_clock::time_point start = chrono::steady_clock::now();
			//chrono::steady_clock::time_point end = chrono::steady_clock::now();


			arr = new float(10);            
			arr = ampSerial->getSerialVals(10);
			IBuckets = arr[9]; 	//
			IScrew = arr[8];	//
		
			float Regolith = (arr[0]+arr[1]+arr[2]+arr[3])/4.0f;

			end = std::chrono::steady_clock::now();
			
			// if the lower switch is pressed
			if (switchL == 1 or std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() > 120){
				cout << "Finishing due to limit switch" << endl;
				while(true) //lift buckets/screws up
				{
					switchL = GPIO::input(LSwitch);
					switchH = GPIO::input(HSwitch);

					screwdriver->SETSPEED(-1.5*ScrewSpeed);
					buckets->SETSPEED(BucketSpeed);
					if (switchH == 1)
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
				if (switchH == 1)
				{
					screwdriver->SETSPEED(0);
					buckets->SETSPEED(0);
				   
					cout << "Finishing due to Excessive current, make adjustment";
					return;
				}
			}
			
			else if(IBuckets >= IBuckEX)
			{
				while(IBuckets >= IBuckEX)
				{
					cout << "IBuckets: " << IBuckets << endl;
					switchL = GPIO::input(LSwitch);
					switchH = GPIO::input(HSwitch);
					
					arr = ampSerial->getSerialVals(10);
					IBuckets = arr[9];
				
					
					screwdriver->SETSPEED(-2*ScrewSpeed);
					buckets->SETSPEED(BucketSpeed);
					if (switchH == 1)
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
				cout << "Finishing due to Hopper load";
				while (true){
					switchL = GPIO::input(LSwitch);
					switchH = GPIO::input(HSwitch);
					screwdriver->SETSPEED(-ScrewSpeed);
					buckets->SETSPEED(BucketSpeed);
					
					if (switchH == 1)
					{
						screwdriver->SETSPEED(0);
						buckets->SETSPEED(0);
						return;
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












