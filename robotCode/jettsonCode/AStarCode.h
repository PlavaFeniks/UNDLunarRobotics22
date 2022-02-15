

class AStarNode;
int calculateDistance(int x, int y, AStarNode* targetNode);
int calculateDistance(int x, int y, int targetX, int targetY);

AStarNode ***mapOfPit = new AStarNode**[HEIGHT]; //100 by 100 size of map

struct //used for storing zed position
{
	float tx;
	float ty;
	float tz;
	
	//https://www.stereolabs.com/docs/api/classsl_1_1Rotation.html
	float rx;
	float ry;
	float rz;
} zedTr;

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
				//https://www.stereolabs.com/docs/tutorials/using-sensors/
				//https://www.stereolabs.com/docs/tutorials/positional-tracking/
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
