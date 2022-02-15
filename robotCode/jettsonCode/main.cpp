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
#include <iostream>
#include <math.h>
#include <vector>
#include <unistd.h>
#define WIDTH 90 //width of map
#define HEIGHT 90 //height of map
#include "/usr/include/python2.7/Python.h"
#include "/home/rmc/Documents/GitRepos/UNDLunarRobotics22/robotCode/jettsonCode/matplotlibcpp.h"


const int ACCURACY = 10; //how many times it will grab imagines
using namespace std;
using namespace sl;
namespace plt = matplotlibcpp;
double OccThresh = .5f; // -1 to 1 where .5 is 75% chance it's occupied and 1 is totally tubularly occupied

class AStarNode;

int calculateDistance(int x, int y, AStarNode* targetNode);
int calculateDistance(int x, int y, int targetX, int targetY);

class AStarNode
{
	public:
	float gCost = -1; //distance to starting node
	float hCost = -1; //distance to endNode
	float fCost = -1; //f = G + H
	int x,y,z; //position, y|x used for index
	
	//OccMapVals
	int Nobs = 0; //Number of times observed
	int OBS = 0; //Running value of observation
	double Pocc = 1.0; //Probability of occupancy
	int showOcc = 0;
	//PointCloud Processing
	float Zsum = 0;
	int Npoints = 0;
	float Zval = 0;
	float Disthere = 0;
	
	
	bool isTraversable = true;
	bool isClose = false;
	AStarNode* parent = NULL;
	
	AStarNode(int x, int y, int z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	
	void setXYZ(int x, int y, int z)
	{
		this->x = x;
		this->y = y;
		this->z = z;
	}
	
	void setGCost(int gCost, AStarNode* endNode)
	{
		this->gCost = gCost;
		hCost = calculateDistance(x, y, endNode);
		fCost = hCost + gCost;
	}
	
};
int calculateDistance(int x, int y, AStarNode* targetNode)
{
	int targetX = targetNode->x;
	int targetY = targetNode->y;
	float distance = sqrt(pow((targetX - x), 2) + pow((targetY - y), 2));
	return (int)(distance*10);
}

int calculateDistance(int x, int y, int targetX, int targetY)
{
	float distance = sqrt(pow((targetX - x), 2) + pow((targetY - y), 2));
	return (int)(distance*10);
}
//AStarNode closedNodes = new AStarNode**[HEIGHT]; shouldnt need because class itself keeps track of if it's closed
AStarNode ***mapOfPit = new AStarNode**[HEIGHT];

int imageWidth = 1280;
int imageHeight = 720;

float ***Zval = new float**[imageWidth];//float Zval[1280][720];
float ***Xval = new float**[imageWidth];//float Xval[1280][720];
float ***Yval = new float**[imageWidth];//float Yval[1280][720];

float gplaneA;
float gplaneB;
float gplaneC;
float gplaneD;

double Dist;
double threshVal = 20.0;

Plane groundPlane;
/*ground plane = ax + by + cz = d FOR REFERENCE :: https://www.stereolabs.com/docs/api/classsl_1_1Plane.html#ae5080d63d8703462b5db4d1eb4333942
	groundPlane.x = a;
	groundPlane.y = b;
	groundPlane.z = c;
	groundPlane.w = d;
 
*/



// Create a ZED camera object
Camera zed;

//to get value from point cloud by doing pointcloud.x, .y, or .z
float GetDistance (float x, float y, float z)
{
	return sqrt(x*x + y*y + z*z);
}

//
void FillArray(float array[][720], float value)
{
	for (int i=0; i<imageWidth; i++)
	{
		for (int j=0; j<imageHeight; j++)
		{
			array[i][j] = value;
			
		}
	}
}
void Mapper1(){
	
	//Read in the sensor data and update the average value for the cell
	// i and j for the image points, will need to smooth the data only taking verified points
	
	float cellSize = 1; //to go from mm to meter
	/*float a = groundPlane.x; 
	float b = groundPlane.y;
	float c = groundPlane.z;
	float d = groundPlane.w;*/
	for (int i=0; i<imageWidth; i++)
		{
			for (int j=0; j<imageHeight; j++)
			{
				if (Xval[i][j] == NULL) continue;
				
				int xhere = int(*Xval[i][j]/cellSize+45);
				int yhere = int(*Yval[i][j]/cellSize);
				float zhere = *Zval[i][j];
				
				//cout <<xhere << "  " << yhere << "  " << zhere << "\n";
				
				if (xhere >= 0 and xhere < WIDTH and yhere >= 0 and yhere < HEIGHT)
				{
					mapOfPit[yhere][xhere]->Zsum += zhere;
					mapOfPit[yhere][xhere]->Nobs ++;
					mapOfPit[yhere][xhere]->Zval = mapOfPit[yhere][xhere]->Zsum/float(mapOfPit[yhere][xhere]->Nobs);
					
					Dist = abs(gplaneA*(*Xval[i][j])*10+gplaneB*(*Yval[i][j])*10+gplaneC*(*Zval[i][j])*10+gplaneD)/(sqrt(pow(gplaneA,2)+pow(gplaneB,2)+pow(gplaneC,2)));
					mapOfPit[yhere][xhere]->Disthere = Dist;
					if (abs(Dist) >= threshVal)
					{
						mapOfPit[yhere][xhere]->OBS --;
					}else
					{
						mapOfPit[yhere][xhere]->OBS ++;
					}
					mapOfPit[yhere][xhere]->Pocc = mapOfPit[yhere][xhere]->OBS/mapOfPit[yhere][xhere]->Nobs;
					
				}
				
				Xval[i][j] = NULL;
				Yval[i][j] = NULL;
				Zval[i][j] = NULL;
			}
		}
		
		
		for (int i = 0; i<WIDTH; i++)
		{
			for (int j = 0; j<HEIGHT; j++)
			{
				if(mapOfPit[i][j]->Pocc < OccThresh)
				{
					mapOfPit[i][j]->isTraversable = false;
				}
				else
				{
					mapOfPit[i][j]->isTraversable = true;
				}
			}
		}
	
		
		//make x and y values into the same size as the map
		//floor the value to use as the indexes for the occmap
		//check if point will be an inlier or an outlier of the plane for navigation
		//		do this with the z value comparing with the plane
		//if inlier
		//	perform inlier stuff using the indexes found in x,y
		//if outlier
		//	perform outlier stuff
		//exit
	
	
}

float maxDifference = 0;
float minDifference = std::numeric_limits<float>::infinity();
//gets ground plane and point cloud
//affects global variables
void getCloudAndPlane()
{
	double cellSize = .1; //meters
	// Set runtime parameters after opening the camera
    RuntimeParameters runtime_parameters;
    runtime_parameters.sensing_mode = SENSING_MODE::STANDARD; // Use STANDARD sensing mode

    // Capture 50 images and depth, then stop
    sl::Mat image, depth, point_cloud, confidence_map;
    
    // Enable positional tracking before starting spatial mapping
    //needed for ground plane stuff
    zed.enablePositionalTracking();
    
    //loops a set amount of times, doesnt count if zed cant grab image
    int k=0;
        float smallestX = 10000;
        float largestX = -1;
        float smallestY = 10000;
        float largestY = -1;
    while (k < ACCURACY) {
        // A new image is available if grab() returns ERROR_CODE::SUCCESS
        if (!(zed.grab(runtime_parameters) == ERROR_CODE::SUCCESS)) continue;
        
        // Retrieve left image
        zed.retrieveImage(image, VIEW::RIGHT);
        // Retrieve depth map. Depth is aligned on the left image
        zed.retrieveMeasure(depth, MEASURE::DEPTH);
        // Retrieve colored point cloud. Point cloud is aligned on the left image.
        zed.retrieveMeasure(point_cloud, MEASURE::XYZ);
        zed.retrieveMeasure(confidence_map, MEASURE::CONFIDENCE);
        
        //NEW PLANE DATA
		sl::Plane plane;
		sl::Transform resetTrackingFloorFrame;

		ERROR_CODE find_plane_status = zed.findFloorPlane(plane, resetTrackingFloorFrame);
		if(find_plane_status == ERROR_CODE::SUCCESS and plane.type == PLANE_TYPE::HORIZONTAL){
			// Reset positional tracking to align it with the floor plane frame
			//zed.resetPositionalTracking(resetTrackingFloorFrame);
			sl::float4 theThing = plane.getPlaneEquation();
				gplaneA = theThing.x;
				gplaneB = theThing.y;
				gplaneC = theThing.z;
				gplaneD = theThing.w;
		}
		else
		{
			continue;
		}
        
		sl::Pose zed_pose;
        POSITIONAL_TRACKING_STATE state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);
        //cout << "Translation: " << zed_pose.getTranslation().tx << " TX:"  <<" TY: " << zed_pose.getTranslation().ty<< " TZ: "<< zed_pose.getTranslation().tz;
        //this loop will iterate through every pixel on zed camera and get it's "z" value
		for (int i=0; i<imageWidth; i++)
		{
			for (int j=0; j<imageHeight; j++)
			{	
				float confidenceZED = 0;
				confidence_map.getValue(i, j, &confidenceZED);
				if (confidenceZED >50) continue; //1-100+
				
				
				sl::float4 point_cloud_value;
				point_cloud.getValue(i,j,&point_cloud_value);
				if (std::isfinite(point_cloud_value.z))
				{
					
					if (smallestX > point_cloud_value.x) smallestX = point_cloud_value.x;
					if (smallestY > point_cloud_value.y) smallestY = point_cloud_value.y;
					if (largestX < point_cloud_value.x) largestX = point_cloud_value.x;
					if (largestY < point_cloud_value.y) largestY = point_cloud_value.y;
					Zval[i][j] = new float(point_cloud_value.z/10);
					Xval[i][j] = new float(point_cloud_value.x/10);
					Yval[i][j] = new float(point_cloud_value.y/10);
					int x = point_cloud_value.x/10;
					int y = point_cloud_value.y/10;
					int z = point_cloud_value.z/10;
					if (x >=0 and x <WIDTH and y >=0 and y <HEIGHT) mapOfPit[y][x]->setXYZ(x, y, z);
					/*
					cout << "Distance: " << GetDistance(i, j, point_cloud_value.z) << "\n";
					cout << "X: " << x << "\n";
					cout << "Y: " << y << "\n";
					cout << "Z: " << z << "\n";
					*/
				}
			}
		}
		/*
		 
		//gets ground plane
		Transform resetTrackingFloorFrame;
		Plane tempPlane;
		zed.findFloorPlane(tempPlane, resetTrackingFloorFrame);
		if (tempPlane.type == PLANE_TYPE::HORIZONTAL) //only stores ground plane if the given plane is horizontal
		{
			groundPlane = tempPlane;
			/*cout << "A: " << groundPlane.getPlaneEquation().x<< "\n";
			cout << "B: " << groundPlane.getPlaneEquation().y<< "\n";
			cout << "C: " << groundPlane.getPlaneEquation().z<< "\n";
			cout << "D: " << groundPlane.getPlaneEquation().w<< "\n";
			cout << "Plane Type: " << groundPlane.type << "\n";
			
		}
		if (groundPlane.type == PLANE_TYPE::UNKNOWN) //stores the plane if ground plane is unkown type I.E. if zed's unsure if it's horizontal or vertical
		{
			groundPlane = tempPlane;			
		}
		*/
		
		
		
		
		
		Mapper1();
		k += 1;       
	}/*
	gplaneA = plane.getPlaneEquation(x);
	gplaneB = theThing.y;
	gplaneC = theThing.z;
	gplaneD = theThing.w;*/
		cout << "\nheres stuff " << smallestX << " " << smallestY << " " << largestX << " " << largestY << "\n";
	cout << maxDifference << "\n";
	cout << minDifference << "\n";/*
	float a = tempPlane.getPlaneEquation().x; 
	float b = tempPlane.getPlaneEquation().y;
	float c = groundPlane.getPlaneEquation().z;
	float d = groundPlane.getPlaneEquation().w;*/
}

//getPosition
void PrintPosition()
{
	bool isImu = true;
	
	sl::PositionalTrackingParameters tracking_parameters;
    tracking_parameters.enable_area_memory = true;
    
    auto returned_state = zed.enablePositionalTracking(tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Enabling positionnal tracking failed: " <<  returned_state << "\n";
        zed.close();
        return;
    }
    
    sl::Transform initialPosition;
    string output = "";
    float x=0;
    float otherX = 0;
	while (true)
	{
		if (zed.grab() == ERROR_CODE::SUCCESS)
		{
			if (isImu)
			{
				SensorsData sensorsData;
				SensorsData::IMUData imuData;
				if (zed.getSensorsData(sensorsData, TIME_REFERENCE::CURRENT) == ERROR_CODE::SUCCESS) {
					
					sl::Pose zed_pose;
					zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);
					
					zed_pose = sensorsData.imu.pose;
					
					x += zed_pose.getTranslation().x;
					cout << "X: " << x << " vs " << zed_pose.getTranslation().x;
					//sImu = false;
					cout << "\n";
				}
			}
			else
			{
				sl::Pose zed_pose;
				auto state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);
				cout << zed_pose.getTranslation().tx << "\n";
				isImu = true;
				// Get the pose of the camera relative to the world frame
				// Display translation and timestamp
				//printf("Translation: tx: %.3f, ty:  %.3f, tz:  %.3f, timestamp: %llu\r",
				//zed_pose.getTranslation().tx, zed_pose.getTranslation().ty, zed_pose.getTranslation().tz, zed_pose.timestamp);
				// Display orientation quaternion
				//printf("Orientation: ox: %.3f, oy:  %.3f, oz:  %.3f, ow: %.3f\r",
				//zed_pose.getOrientation().ox, zed_pose.getOrientation().oy, zed_pose.getOrientation().oz, zed_pose.getOrientation().ow);
			}
		}
		else sleep_ms(1);
	}
}


void initializeTesselatedMap()
{
	//initialization of map with default values
	for (int i=0; i< HEIGHT; i++)
	{
		mapOfPit[i] = new AStarNode*[WIDTH];
		for (int j=0; j<WIDTH; j++)
		{
			mapOfPit[i][j] = new AStarNode(j, i, 0); //set 0 to height
		}
	}
}
void initializeOccupancyMapXYZVal()
{
	for (int i=0; i<imageWidth; i++)
	{
		Zval[i] = new float*[imageHeight];
		Xval[i] = new float*[imageHeight];
		Yval[i] = new float*[imageHeight];
		for (int j=0; j<imageHeight; j++)
		{
			Zval[i][j] = NULL;
			Xval[i][j] = NULL;
			Yval[i][j] = NULL;
		}
	}
}


void ayoCheckIfMapWorking()
{
	cout << "start of map\n";
	for (int i=HEIGHT-1; i>=0; i--)
	{
		for (int j=0; j<WIDTH; j++)
		{
			// /*
			if (mapOfPit[i][j]->Nobs==0)
			{
				//cout << mapOfPit[i][j]->Zval << "\t";
				cout << " ";				
			}
			else if (mapOfPit[i][j]->isTraversable)
			{
				//cout << "0?\t";
				cout << " ";
			}
			else
			{
				cout << "1";
			}
			//*/
			//std::cout.precision(1);
			//cout << mapOfPit[i][j]->Zval << "\t";
		}
		cout << "\n";
	}
	cout << "end of map\n";
	sleep(1);
}

int main(int argc, char **argv) {
	
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
    /*
	for (int i=0; i<1; i++)
	{
		getCloudAndPlane();
		cout << i <<"\n";
	}
	*/
	cout << "Printing Map of Pit, sleeping for 5 seconds\n";
	sleep(1);
	
	ayoCheckIfMapWorking();
	
	while(true)
	{
		break;
		//PrintPosition();
	}
	mapOfPit[1][1]->Zsum += 5;
	mapOfPit[1][1]->Zsum += 5;
	//plt::plot_surface(x, y, z);
    //plt::show();
	cout << mapOfPit[25][25]->Zsum <<"\n";
	cout << mapOfPit[25][25]->Zval <<"\n";
    // Close the camera
    zed.close();
    return EXIT_SUCCESS;
    
}
