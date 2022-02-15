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

using namespace std;
using namespace sl;

// Create a ZED camera object
Camera zed;

//our .h Files
#include "AStarCode.h" //contains all code pertaining to AStar algorithm
#include "OccupancyMap.h" //contains all relevant occupancy map code

int main(int argc, char **argv) {
	zedTr = {0, 0, 0, 0, 0, 5};
	cout << zedTr.rx << zedTr.rz << "\n";
	
	initializeTesselatedMap();
	initializeOccupancyMapXYZVal();
	
	// Set configuration parameters
    InitParameters init_parameters;
    init_parameters.depth_mode = DEPTH_MODE::PERFORMANCE; // Use PERFORMANCE depth mode
    init_parameters.coordinate_units = UNIT::CENTIMETER; // Use millimeter units (for depth measurements)
    init_parameters.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Z_UP; // Use a right-handed Y-up coordinate system
    init_parameters.sdk_verbose = true;
    
    // Open the camera
    auto returned_state = zed.open(init_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Error " << returned_state << ", exit program." << endl;
        return EXIT_FAILURE;
    }
    
    getCloudAndPlane();
	
	cmdLineOccupancyMap();
	
	while(true)
	{
		break;
		//PrintPosition();
	}
	
    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
}
