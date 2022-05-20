///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2021, STEREOLABS.
//
// All rights reserved.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
///////////////////////////////////////////////////////////////////////////
#include <sl/Camera.hpp>
#include <iostream>
#include <math.h>
#include <vector>
#include <unistd.h>
#include "/usr/include/python2.7/Python.h"

#define WIDTH 90 //width of tesselated map
#define HEIGHT 90 //height of tesselated map
#define IMAGEWIDTH 1280 //x size of image
#define IMAGEHEIGHT 720 //y size of image
#define ACCURACY 10 //how many times it will ZED grab images for point cloud
float XZedRelativeToRobot = -1.778; //where the zed is relative to the robot pov CENTIMETERS/10
float YZedRelativeToRobot = 6.096; //where the zed is relative to the robot pov CENTIMETERS/10
float XFidCamRelativeToRobot = -2.0955; //where fiducial cam is relative to center of robot
float YFidCamRelativeToRobot = -3.302;
float fiducialPositionX = 45;
float fiducialPositionY = 0;

float PI = 3.14159265;


float depositionX = 0;
float depositionY = 0;

float robotPositionX = 45; //position from fiducial
float robotPositionY = 0;

using namespace std;
using namespace sl;

// Create a ZED camera object
Camera zed;


//initializations for chassis
#include "chassis.h"
float *PID_chassis= (float*)malloc(sizeof(float)*4);
chassis locomotion(PID_chassis);

#include "Mining.h" //mining code
TalonPair* buckets;
TalonPair* screwdriver;

#include "deposition.h" //deposition code


//our .h Files
#include "AStarCode.h" //contains all code pertaining to AStar algorithm
#include "OccupancyMap.h" //contains all relevant occupancy map code
#include "PathFollowing.h" //contains code for following a path
#include "detect_markers.hpp"
#include "fiducial.h"

void miningSetup()
{
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
	buckets = new TalonPair(5,VELOCITY,limits, PID_valsBuck);
	//TalonPair* screwdriver= new TalonPair(5,VELOCITY,limits, PID_valsScrew);
	screwdriver = new TalonPair(6);
	cout<<"miningSetup Complete "<<endl;
}

void makeRowIntraversable()
{
	for (int i=0; i<10; i++)
	{
		mapOfPit[15][i]->isTraversable = false;
	}
}

void testMovement(float speed)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	std::chrono::steady_clock::time_point   end = std::chrono::steady_clock::now();
	
	while (std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() < 5)
	{
		end = std::chrono::steady_clock::now();
		locomotion.SETSPEED(-speed, speed);
	}
	locomotion.SETSPEED(0,0);
	begin = std::chrono::steady_clock::now();
	while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() < 5)
	{
		end = std::chrono::steady_clock::now();
		locomotion.SETSPEED(speed, speed);
	}
	locomotion.SETSPEED(0,0);
	begin = std::chrono::steady_clock::now();
	while(std::chrono::duration_cast<std::chrono::seconds>(end-begin).count() < 5)
	{
		end = std::chrono::steady_clock::now();
		locomotion.SETSPEED(-speed, -speed);
	}
}

int main(int argc, char **argv)
{
	cout << "welcome to my lets play\n";
	//initialization for mining and deposition
	readSerial* ampSerial = new readSerial((char*)"/dev/ttyUSB0");
	TalonPair * Motor_hopper_belt = new TalonPair(7);
	
	if (argc > 1 and  strcmp(argv[1], "mining") == 0)
	{
		
		miningSetup();
//		cout <<"begining "<<endl;
//		actuatorPos(ampSerial,0.00) ;
//		LimitSwitchTest();
		MiningTime1(ampSerial, buckets, screwdriver);		
		//preMining(ampSerial,buckets,screwdriver);
		/*
		actuatorCalibration(ampSerial) ;
		sleep(2);
		actuatorCalibration(ampSerial) ;
		//actuatorPos(ampSerial,0.00) ;
		*/
		
		/*
		cout<< "Set to zero"<< endl;
		actuatorPos(ampSerial,0);
		sleep(2);
		actuatorPos(ampSerial,1.00f);
		sleep(2);
		actuatorPos(ampSerial,0.50f);
		sleep(1);
		cout<< "Set to 0.5"<< endl;
		*/
		//actuatorPos(ampSerial,0.50f);
		
				
		
		
		
		return 1;
		/*sleep(1);
		//actuatorPos(ampSerial,1) ;
		sleep(1);
		actuatorPos(ampSerial,1.4) ;
		sleep(1);
		actuatorPos(ampSerial,.8) ;

		*/

		

		//MiningTime1(ampSerial, buckets, screwdriver);
		return 1;
	}
	else if (argc > 1 and  strcmp(argv[1], "limitSwitchTest") == 0)
	{
		LimitSwitchTest();
		return 1;
	}
	else if (argc > 1 and  strcmp(argv[1], "deposition") == 0)
	{
		//deposition
		deposition(ampSerial, Motor_hopper_belt);
	}
	else if (argc > 1 and strcmp(argv[1], "fiducial") == 0)
	{
		if (fiducial(argc, argv) == false) cout << "help\n";
		//frameTranslation(XFidCamRelativeToRobot, -YFidCamRelativeToRobot, 0);
		cout << robotPositionX << " " << robotPositionY << endl;
		return 1;
	}
	else if (argc > 1 and strcmp(argv[1], "movement") == 0)
	{
		testMovement(400);
		return 1;	
	}
  
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Use PERFORMANCE depth mode
    init_parameters.coordinate_units = UNIT::CENTIMETER; // Use millimeter units (for depth measurements)
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP; // Use a right-handed Y-up coordinate system
    init_parameters.sdk_verbose = true;
    
    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS)
    {
        cout << "Error " << returned_state << ", exit program." << endl;
        return EXIT_FAILURE;
    }
    
    //initialization
	initializeTesselatedMap();
	initializeOccupancyMapXYZVal();
	initializePositionalTracking();
	
	int scale = 10; //bigger is more zoomed out, cm/scale
	
	int threshVal = 40;
	float confidenceZedThreshhold = 75;
	OccThresh = -.3;
	if (argc > 4 and strcmp(argv[1], "Vision") == 0) //order of args is Vision threshVal OccThresh confidenceZedThreshhold
	{
		cout << "setting threshhold values\n";
		threshVal = atoi(argv[2]);
		confidenceZedThreshhold = atof(argv[3]);
		OccThresh = atof(argv[4]);
	}
	
	
	
	
	//generate map
	getCloudAndPlane(scale, confidenceZedThreshhold, threshVal);
	startNode = mapOfPit[(int)robotPositionY][(int)robotPositionX];
	endNode = mapOfPit[8][(int)robotPositionX+2];
	cmdLineOccupancyMap();
	cmdLineNobs();
	//thiccOccupancymap(3);

	FindPath(startNode);
	
	
	//generate with thickening
	cmdLineOccupancyMap();
	
	if (argc > 1 and  strcmp(argv[1], "Vision") == 0)
	{
		// Close the camera
		zed.close();
		return EXIT_SUCCESS;	
	}
	//mining initialization
	//miningSetup();
	
	//do stuff
	cout << "beginning movement in 5 seconds\n";
	
	sleep(5);
	while(true)
	{
		
		getTranslationImage(&zedCurrent);
		if (followPath(startNode, &zedCurrent, &zedGoal, &zedNextGoal, true) == false) //it got stuck in path
		{
			unstuckRobot(true);
			//localize with fiducial
			//redo A*
			continue;
		}
		//mining
		//MiningTime1(ampSerial, buckets, screwdriver);
		
		//redu a* where
		startNode = mapOfPit[(int)zedCurrent.ty][(int)zedCurrent.tx];
		endNode = mapOfPit[(int)depositionY+3][(int)depositionX];
		FindPath(startNode);
		while(true)
		{
			if (followPath(startNode, &zedCurrent, &zedGoal, &zedNextGoal, false) == false)
			{
				unstuckRobot(false);
				//localize with fiducial
				//redo A*
				continue;
			}
			break;
		}
		//deposition
		//deposition(ampSerial, Motor_hopper_belt);
		break;	
	}
	
    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}
