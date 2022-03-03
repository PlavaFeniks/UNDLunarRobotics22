#define PI 3.14159265
//https://github.com/stereolabs/zed-examples/blob/master/positional%20tracking/cpp/src/main.cpp

void getTranslationImage(TransformationData* updateOrient) //sets position and angle change using images
{   
	float x,y,z;
	float rx, ry, rz;
	while (true)
	{
		if (zed.grab() == ERROR_CODE::SUCCESS)
		{
			sl::Pose zed_pose;
			auto state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);
			if (state == POSITIONAL_TRACKING_STATE::OK)
			{
				x = updateOrient->tx=zed_pose.getTranslation().x/10;
				y = updateOrient->ty=zed_pose.getTranslation().y/10;
				z = updateOrient->tz=zed_pose.getTranslation().z/10;
				rx = updateOrient->rx = zed_pose.getEulerAngles(false).x;
				ry = updateOrient->ry = zed_pose.getEulerAngles(false).y;
				rz = updateOrient->rz = zed_pose.getEulerAngles(false).z;
				
				//orientation set for the robot's position
				float theta, gamma, alpha = 0;
				theta = rz;
				alpha = abs(atan2(YJETSONRELATIVETOROBOT, XJETSONRELATIVETOROBOT)* 180 / PI - 90);
				gamma = alpha + theta;
				
				updateOrient->tx = x-cos((gamma+90) / 180 * PI);
				updateOrient->ty = y-sin((gamma+90) / 180 * PI);
				
				//cout << zed_pose.getEulerAngles(false).x << " " << zed_pose.getEulerAngles(false).y << " " << zed_pose.getEulerAngles(false).z << "\n";
				//cout << setprecision(3) << zed_pose.getTranslation().x << " " <<  << " " <<zed_pose.getTranslation().z << "\n";
				/*cout << setprecision(3) << 
				" X: " << x <<
				" Y:" << y <<
				" Z:" << z <<
				
				" Rx: "<< rx <<
				" Ry:" << ry <<
				" Rz:" << rz <<
				"\n";*/
				break;
			}
		}
	}
	return;
}

void getTranslationIMU() //sets position and angle change using IMU
{
	float x,y,z;
	float rx, ry, rz;
	while (true)
	{
		if (zed.grab() == ERROR_CODE::SUCCESS)
		{
			sl::Pose zed_pose;
			SensorsData sensors_data;
			auto state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);
			if (zed.getSensorsData(sensors_data, TIME_REFERENCE::IMAGE) == sl::ERROR_CODE::SUCCESS)
			{
				x=sensors_data.imu.pose.getTranslation().x;
				y=sensors_data.imu.pose.getTranslation().y;
				z=sensors_data.imu.pose.getTranslation().z;
				rx = sensors_data.imu.pose.getEulerAngles(false).x;
				ry = sensors_data.imu.pose.getEulerAngles(false).y;
				rz = sensors_data.imu.pose.getEulerAngles(false).z;
				/*cout << zed_pose.getEulerAngles(false).x << " " << zed_pose.getEulerAngles(false).y << " " << zed_pose.getEulerAngles(false).z << "\n";
				//cout << setprecision(3) << zed_pose.getTranslation().x << " " <<  << " " <<zed_pose.getTranslation().z << "\n";
				cout << setprecision(3) << 
				" X: " << x <<
				" Y:" << y <<
				" Z:" << z <<
				
				" Rx: "<< rx <<
				" Ry:" << ry <<
				" Rz:" << rz <<
				"\n";*/
				break;
			}
		}
	}
}

void initializePositionalTracking()
{
	sl::PositionalTrackingParameters tracking_parameters;
    tracking_parameters.enable_area_memory = true;
    
    auto returned_state = zed.enablePositionalTracking(tracking_parameters);
    if (returned_state != ERROR_CODE::SUCCESS) {
        cout << "Enabling positionnal tracking failed: " <<  returned_state << "\n";
        zed.close();
        return;
    }
}

void determineAngleToGoal(TransformationData current, TransformationData* goalState) //finds the angle to the goal and stores it in goal state
{
	float adjustGoalx = goalState->tx - current.tx;
	float adjustGoaly = goalState->ty - current.ty;
	
	float angle = atan2(adjustGoaly, -adjustGoalx) * 180 / PI - 90; //dont touch
	goalState->rz = -angle;
	cout << angle << "\n";
	
}

float getAngleDifference(TransformationData current, TransformationData goalState) //gets angle that robot needs to rotate
{
	float angleDifference = goalState.rz - current.rz;
	cout << angleDifference << "\n";
	return angleDifference;
}

float getDistanceDifference(TransformationData current, TransformationData goalState) //gets distance robot needs to move 
{
	float adjustGoalx = goalState.tx - current.tx;
	float adjustGoaly = goalState.ty - current.ty;
	
	float distance = sqrt(pow(adjustGoalx, 2) + pow(adjustGoaly, 2));
	cout << distance << "\n";
	return distance;
}

void turnMove(TransformationData* current, TransformationData* goalState) //robot will turn and move towards desired location
{
	//find initial angle
	getTranslationImage(current);
	determineAngleToGoal(*current, goalState);
	
	float previousAngle, previousDistance = -1;
	while(true)//periot
	{
		getTranslationImage(current);
		float angleDiff = getAngleDifference(*current, *goalState);
		if (abs(angleDiff) < 1 or previousAngle < abs(angleDiff)+.5)
		{
			//locomotion.SETSPEED(0, 0);
			break;
		}
		else if (angleDiff > 0);// locomotion.SETSPEED(-.40, .40);
		else if (angleDiff < 0);// locomotion.SETSPEED(.40, -.40);
		previousAngle = abs(angleDiff);
		
	}
	while(true)//walking
	{
		getTranslationImage(current);
		float distance = getDistanceDifference(*current, *goalState);
		if (distance<2 or previousDistance < distance + .5)
		{
			//locomotion.SETSPEED(0,0);
			break;
		}
		else;//locomotion.SETSPEED(.20, .20);
		previousDistance = distance;
	}
}
void followPath(AStarNode* startingNode, TransformationData* current, TransformationData* goalState)
{
	AStarNode* currentNode = startNode;
	
	if (currentNode == NULL) return;
	
	while(currentNode != NULL)
	{
		AStarNode* nextNode = currentNode->parent;
		
		current->tx = currentNode->x;
		current->ty = currentNode->y;
		
		goalState->tx = nextNode->x;
		goalState->ty = nextNode->y;
		
		turnMove(current, goalState);
		currentNode = nextNode;
		
	}
}

