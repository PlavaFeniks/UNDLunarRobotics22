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
#include <chassis.h>

#define WIDTH 90 //width of tesselated map
#define HEIGHT 90 //height of tesselated map
#define IMAGEWIDTH 1280 //x size of image
#define IMAGEHEIGHT 720 //y size of image
#define ACCURACY 10 //how many times it will ZED grab images for point cloud
#define XJETSONRELATIVETOROBOT -3.49 //where the jetson is relative to the robot pov CENTIMETERS/10
#define YJETSONRELATIVETOROBOT 3.49 //where the jetson is relative to the robot pov CENTIMETERS/10
#define PI 3.14159265 //the latest recipe

float zedPositionX = 45;
float zedPositionY = 0;

using namespace std;
using namespace sl;

// Create a ZED camera object
Camera zed;

//initializations for chassis
#include "chassis.h"


float *PID_chassis= (float*)malloc(sizeof(float)*4);
chassis locomotion(PID_chassis);

#include "Mining.h" //mining code
#include "deposition.h" //deposition code


//our .h Files
#include "AStarCode.h" //contains all code pertaining to AStar algorithm
#include "OccupancyMap.h" //contains all relevant occupancy map code
#include "PathFollowing.h" //contains code for following a path

void makeRowIntraversable()
{
	for (int i=0; i<10; i++)
	{
		mapOfPit[15][i]->isTraversable = false;
	}
}

int main(int argc, char **argv)
{
	// Set configuration parameters
	std::string interface;
	interface = "can0";
	int temp; 
	if (temp = (ctre::phoenix::platform::can::SetCANInterface(interface.c_str())) == -1){
		perror("");
		std::_Exit(0);
	}    
//LimitSwitchTest();
//return 1;	
//depo_start();
//return 1;
//	MiningTime1(new readSerial((char*)"/dev/ttyUSB0"));
//	return 1;
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

	//initialization for mining and deposition
	readSerial* ampSerial = new readSerial((char*)"/dev/ttyUSB0");
	TalonPair * Motor_hopper_belt = new TalonPair(7);
	
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
	TalonPair* buckets = new TalonPair(6,VELOCITY,limits, PID_valsBuck);
	//TalonPair* screwdriver= new TalonPair(5,VELOCITY,limits, PID_valsScrew);
	TalonPair* screwdriver = new TalonPair(5);
    
    //initialization
	initializeTesselatedMap();
	initializeOccupancyMapXYZVal();
	initializePositionalTracking();
	
	//generate map
	getCloudAndPlane();
	startNode = mapOfPit[(int)zedPositionY][(int)zedPositionX];
	endNode = mapOfPit[15][(int)zedPositionX];
	cmdLineOccupancyMap();
	thiccOccupancymap(3);

	FindPath(startNode);
	
	
	//generate with thickening
	cmdLineOccupancyMap();
	
	if (argc > 1)
	{
		// Close the camera
		zed.close();
		return EXIT_SUCCESS;	
	}
	
	//do stuff
	cout << "beginning movement in 5 seconds\n";
	
	sleep(5);
	while(true)
	{
		
		getTranslationImage(&zedCurrent);
		followPath(startNode, &zedCurrent, &zedGoal, &zedNextGoal, true);
		//mioning
		MiningTime1(ampSerial, buckets, screwdriver);
		followPath(endNode, &zedCurrent, &zedGoal, &zedNextGoal, false);
		//deposition
		deposition(ampSerial, Motor_hopper_belt);
		break;	
	}
	
    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}
