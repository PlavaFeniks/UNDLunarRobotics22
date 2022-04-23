
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

/*
    wiringPiSetup();
    pinMode(HSwitch, INPUT);  // twlls gpio pins what is gettign pluged in
    pinMode(LSwitch, INPUT);

    pullUpDnControl(HSwitch, PUD_DOWN); // sets pull down resistors
    pullUpDnControl(LSwitch, PUD_DOWN);
    */
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

/*
void MiningTime1(readSerial* ampSerial){
    float *arr = (float*) malloc(sizeof(float) * 10);
    
    //setup();
    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    string amps;
    
   // wiringPiSetup();


    char  x;  // X button input
    float IBuckets;  // Current input
    float  Ie  = 1100; // Current epected
    
    int BucketSpeed = 300; // quad speed = .1/60*4096*RPM/gearbox. Efficiency speed is 12150 rpm btw
    //int buckScrewRatio = 5; //number of revolution of bucket shaft to revolutions of screw shaft
    int ScrewSpeed = -1000; //relation between bucket speed and screw speed
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
   
    TalonPair* screwdriver= new TalonPair(5,VELOCITY,limits, PID_valsScrew);


    
    //TalonPair * screwdriver = new TalonPair(5);
    //screwdriver->INVERT();
    //TalonPair * buckets = new TalonPair(6);
     //buckets->INVERT();
    //if(digitalRead(HSwitch)==0){
    if(true){
        while(true){
        
            buckets->SETSPEED(BucketSpeed); // set speed for the two motors
            screwdriver->SETSPEED(ScrewSpeed); 

            arr = ampSerial->getSerialVals(10);

            chrono::steady_clock::time_point start = chrono::steady_clock::now();
            chrono::steady_clock::time_point end = chrono::steady_clock::now();
        
            if (loopCount % 5000 == 0){
            arr = ampSerial->getSerialVals(10);
            cout<<"checked"<< endl;
            break;
            }
            loopCount++;
        

            IBuckets = arr[8];
            
            float Regolith = (arr[0]+arr[1]+arr[2]+arr[3])/4.0f;



            if (digitalRead(!HSwitch)){
                while(true){
                        screwdriver->SETSPEED(0);
                        buckets->SETSPEED(0);
                        cout<<"unexpected Halt" <<endl;
                        return; 
                        

                    }
            }
            // if the lower switch is pressed
            else if (digitalRead(!LSwitch)){

                while(true)
                {
                    
                    screwdriver->SETSPEED(-1.5*ScrewSpeed);
                    buckets->SETSPEED(BucketSpeed);
                    
                    if (digitalRead(!HSwitch)){
                        screwdriver->SETSPEED(0);
                        buckets->SETSPEED(0);
                        //return 0; mining complete
                        
                        return;
                        
                    }
                    else {
                        //cout << "Finishing due to limit switch";
                    }

                }

            }
            // if the load is too much
            //else if((cin >> I) >= Ie){
            else if(IBuckets >= Ie){
                while (true){
                    

                    arr = ampSerial->getSerialVals(10);
                    IBuckets = arr[8];


                    screwdriver->SETSPEED(-ScrewSpeed);

                    if (digitalRead(!HSwitch)){
                        screwdriver->SETSPEED(0);
                        buckets->SETSPEED(0);
                        
                        cout << "Finishing due to Excessive current, make adjustment";

                        return;

                    }
                    else {
                        //cout << "Load is larger then Expected";
                    }
                }
                
            }
            
            else if(Regolith >= maxLoad){
                while (true){

                    screwdriver->SETSPEED(-ScrewSpeed);
                    buckets->SETSPEED(BucketSpeed);
                    
                    if (!digitalRead(!HSwitch)){
                        screwdriver->SETSPEED(0);
                        buckets->SETSPEED(0);
                        return;

                    }
                    else {
                        cout << "Finish due to Hopper load";
                    }
                }
                
            }
            
        }
    }
    return;
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
* 
























* 
*/
