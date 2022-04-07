/*
This executable is designed to facillitate the development and testing of various subsystems for when
    a. The robot is unavailable, or being used for other testing
    b. The robot is not the ideal mode of testing due to time required and cable management issues
    c. Because you may not have access to the robot due to location constraints

*/


#include <chrono>

#include "infra/serialThread.h"

#include <string>
#include <iostream>
#include <chrono>
#include <thread>
#include <fstream>
#include <wiringPi.h>
#include <unistd.h>
#include "deposition.h"

#define LSwitch 0 // Switch repersenting digger is fully Raised
#define HSwitch 7// Switch representing digger is fully lowered

using namespace std;

TalonPair* buckets;
TalonPair* screwdriver;
readSerial ampSerial((char*)"/dev/ttyACM0");
std::ofstream outputCSV;



/*
EXAMPLE1: Timing the things. 


tools used; std::chrono

creating a 'time' object
chrono::steady_clock::time_point <name of variable> = chrono::steady_clock::now();
<-- this sets the time_points value to the "current time" (current time being since the Linux epoch but don't worry since its all relative anyway)
to get the time between two points

chrono::duration_cast<chrono::(seconds/milleseconds/etc) ( time_point_1 - time_point_2).count() to return integer difference between two time points
###note: in above example the < and > are needed as its a typecasting format




EXAMPLE2: for reading serial values
float * arrayOfVals;

arrayOfVals = ampSerial.getSerialVals(<number of values to read>)



EXAMPLE3:


*/
double calcCurrent(TalonPair *mc){

    double MotorVoltage= mc->getVoltage();
    //cout<< "motor voltage= " << MotorVoltage<< endl;  // Output the current reading from the arduino

    
    double quadVel = mc->getQuadVelocity();
  //  cout<< "quad vel= " << quadVel<< endl;  // Output the current reading from the arduino


    double Km = .0008796;
    double armResistance = .22642;
    double quadReadTime = .1; //seconds
    double I = (MotorVoltage-quadVel/4096*210*Km/(quadReadTime/60))/armResistance;
    return I;

    
}



void setup(string loggingFile){
    string interface;
    interface = "can0";
	int temp; 
	if (temp = (ctre::phoenix::platform::can::SetCANInterface(interface.c_str())) == -1){
		perror("");
		std::_Exit(0);
	}

    float *limits = (float*)malloc(sizeof(float)*2);
    float *PID_vals = (float*)malloc(sizeof(float)*4);

    limits[0] = 0;
    limits[1]= 1;

    PID_vals[PID_P] = .22;
    PID_vals[PID_I] = .05;
    PID_vals[PID_D] = 0;
    PID_vals[PID_F] = 0;
    
    buckets = new TalonPair(5, VELOCITY,limits, PID_vals);
    screwdriver = new TalonPair(6);

    //setup_ard_Thread(&ampSerial);


    string fileOpen = "./logs/logs_"+ loggingFile + ".csv";
	outputCSV.open(fileOpen.c_str());
	cout<<fileOpen<<" has been opened for logging"<<endl;
    outputCSV<<"Time, quadratureVelocity, outputVoltage"<<endl;
    /* To write into the outputCSV do following

    outputCSV<<Time + "," + talon.getQuardratureVelocity + "," + talon.getVoltage <<endl;


    */

	//Might need to add this into main loop
	
    return;
    
}
void cleanup(){
    pthread_mutex_destroy(&readLock);

}

int main(int argc, char* argv[]){
    
    if (argc != 2){
        cout<<"USAGE './testbed <targetFileName>'"<<endl;
        std::_Exit(1);
    }
    else{
        string loggingFile = str(argv[1]);
    }

    setup(loggingFile);
    chrono::steady_clock::time_point start = chrono::steady_clock::now();
    chrono::steady_clock::time_point end = chrono::steady_clock::now();
    while(chrono::duration_cast<chrono::seconds>(start - end).count()){

    }
    
    
    
    
    wiringPiSetup(); //intialize GPIO pins

    pinMode(HSwitch, INPUT);  // twlls gpio pins what is gettign pluged in
    pinMode(LSwitch, INPUT);

    pullUpDnControl(HSwitch, PUD_DOWN); // sets pull down resistors
    pullUpDnControl(LSwitch, PUD_DOWN);



    char   x;       // X button input
    float  I;       // Current input
    float  Ie = 10;// Current epected
    

    readSerial ampSerial((char*)"/dev/ttyACM0");


    while(chrono::duration_cast<chrono::seconds>(start - end).count() < 20){
       
       
        double targetVelocity_UnitsPer100ms = true * 500.0 * 4096 / 600;
        
        
        buckets->SETSPEED(targetVelocity_UnitsPer100ms);       // Sets robot bucket rotation speed
        double Speeds = buckets->getQuadVelocity();
        cout<< "Speds = " <<  Speeds << endl;
        screwdriver->SETSPEED(0);  // Sets the robot to lower screw
        
        outputCSV
        
    }
    
}

   


    
       