//https://github.com/stereolabs/zed-examples/blob/master/positional%20tracking/cpp/src/main.cpp
const int ERRORRATEDISTANCE = 2;
const int ERRORRATEANGLE = 1;

void getTranslationImage(TransformationData* updateOrient, bool driveForward = true) //sets position and angle change using images
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
				x = updateOrient->tx=zed_pose.getTranslation().x/10 + XJETSONRELATIVETOROBOT;
				y = updateOrient->ty=zed_pose.getTranslation().y/10 + YJETSONRELATIVETOROBOT;

				z = updateOrient->tz=zed_pose.getTranslation().z/10;
				rx = updateOrient->rx = zed_pose.getEulerAngles(false).x;
				ry = updateOrient->ry = zed_pose.getEulerAngles(false).y;
				rz = updateOrient->rz = zed_pose.getEulerAngles(false).z;
				
				
				//orientation set for the robot's position
				float theta, gamma, alpha = 0;
				theta = rz;
				alpha = atan2(YJETSONRELATIVETOROBOT, XJETSONRELATIVETOROBOT)* 180 / PI;
				gamma = alpha + theta;
				
				float distanceToCenterofRobot = sqrt(pow(XJETSONRELATIVETOROBOT, 2) + pow(YJETSONRELATIVETOROBOT, 2));
				updateOrient->tx = x-distanceToCenterofRobot*cos((gamma) / 180 * PI);
				updateOrient->ty = y-distanceToCenterofRobot*sin((gamma) / 180 * PI);
				updateOrient->rz = updateOrient->rz;
				
				
				//cout << "x: " << updateOrient->tx << " y " << updateOrient->ty  << "\n";
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
	if (!driveForward) updateOrient->rz = updateOrient->rz+180;
	return;
}

//absolute garbage, dont use
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

void determineAngleToGoal(TransformationData current, TransformationData* goalState, bool driveForward = true) //finds the angle to the goal and stores it in goal state
{
	float adjustGoalx = goalState->tx - current.tx;
	float adjustGoaly = goalState->ty - current.ty;
	float angle = atan2(adjustGoaly, -adjustGoalx) * 180 / PI; //dont touch
	if (!driveForward) angle = angle - PI;
	goalState->rz = -angle;
	
}

float getAngleDifference(TransformationData current, TransformationData goalState) //gets angle that robot needs to rotate
{
	float angleDifference = goalState.rz - current.rz + 90;
	
	return fmod(angleDifference, 360);
}

float getDistanceDifference(TransformationData current, TransformationData goalState) //gets distance robot needs to move 
{
	float adjustGoalx = goalState.tx - current.tx;
	float adjustGoaly = goalState.ty - current.ty;
	
	float distance = sqrt(pow(adjustGoalx, 2) + pow(adjustGoaly, 2));
	
	return distance;
}

void turnMoveForward(TransformationData* current, TransformationData* goalState, TransformationData* nextGoalState) //robot will turn and move towards desired location
{
	//find initial angle
	getTranslationImage(current);
	determineAngleToGoal(*current, goalState);
	float angleDiff = getAngleDifference(*current, *goalState);
	cout << "-----\n";	
	cout << "Goal X: " << goalState->tx << " Y: " << goalState->ty << "\nCurrent X:" << current->tx << " Y: " << current->ty << "\n\n";
	cout << "rotating " << angleDiff << " amount\n";
	
	while(true)//periot
	{
		getTranslationImage(current);
		angleDiff = getAngleDifference(*current, *goalState);
		if (abs(angleDiff) < ERRORRATEANGLE)
		{
			cout << "hit target angle, leftover" << angleDiff << "\n\n";
			locomotion.SETSPEED(0, 0);
			break;
		}
		else if (angleDiff > 0)locomotion.SETSPEED(750, 750);
		else if (angleDiff < 0)locomotion.SETSPEED(-750, -750);

		
	}
	
	getTranslationImage(current);
	float distanceToGoal = getDistanceDifference(*current, *goalState);
	cout << "moving " << distanceToGoal << "\n";
	while(true)//walking
	{
		getTranslationImage(current);
		distanceToGoal = getDistanceDifference(*current, *goalState);
		float distanceToNextGoal = getDistanceDifference(*current, *nextGoalState);
		if (distanceToGoal<ERRORRATEDISTANCE)
		{
			cout << "hit target distance, leftover" << distanceToGoal << "\n\n";
			locomotion.SETSPEED(0,0);
			break;
		}
		else if (distanceToNextGoal<distanceToGoal)
		{
			cout << "overshot target, distance " << distanceToGoal << "\n\n";
			locomotion.SETSPEED(0,0);
			break;
		}
		else locomotion.SETSPEED(-750, 750);
	}
}
void followPathForwards(AStarNode* startingNode, TransformationData* current, TransformationData* goalState, TransformationData* nextGoalState) //goes from start to end node
{
	cout << "moving forwards\nSleeping for 5 seconds\n";
	sleep(5);
	AStarNode* currentNode = startNode;
	
	if (currentNode == NULL) return;
	
	while(currentNode != NULL)
	{
		AStarNode* nextNode = currentNode->child;
		if (nextNode == NULL) break;
		goalState->tx = nextNode->x;
		goalState->ty = nextNode->y;
		
		AStarNode* nextNextNode = nextNode->child;
		if (nextNextNode == NULL)
		{
			nextGoalState->tx = goalState->tx + (goalState->tx - currentNode->x);
			nextGoalState->ty = goalState->ty + (goalState->ty - currentNode->y);
		}
		else
		{
			nextGoalState->tx = nextNextNode->x;
			nextGoalState->ty = nextNextNode->y;
		}
		
		float distance;
		getTranslationImage(current);
		distance = getDistanceDifference(*current, *goalState);
		if (distance < ERRORRATEDISTANCE)
		{
			currentNode = nextNode;
			cout << "skipping next node\n";
			continue;
		}
		
		turnMoveForward(current, goalState, nextGoalState);
		currentNode = nextNode;
	}
}

//------------------------------------backwards
void turnMoveBackwards(TransformationData* current, TransformationData* goalState, TransformationData* nextGoalState) //robot will turn and move towards desired location
{
	//find initial angle
	getTranslationImage(current, false);
	determineAngleToGoal(*current, goalState);
	float angleDiff = getAngleDifference(*current, *goalState);
	cout << "-----\n";
	cout << "Goal X: " << goalState->tx << " Y: " << goalState->ty << "\nCurrent X:" << current->tx << " Y: " << current->ty << "\n\n";
	cout << "rotating " << angleDiff << " amount\n";
	
	while(true)//periot
	{
		getTranslationImage(current, false);
		angleDiff = getAngleDifference(*current, *goalState);
		if (abs(angleDiff) < ERRORRATEANGLE)
		{
			cout << "hit target angle, leftover" << angleDiff << "\n\n";
			locomotion.SETSPEED(0, 0);
			break;
		}
		else if (angleDiff > 0) locomotion.SETSPEED(750, 750);
		else if (angleDiff < 0) locomotion.SETSPEED(-750, -750);
		
	}
	
	getTranslationImage(current, false);
	float distanceToGoal = getDistanceDifference(*current, *goalState);
	cout << "moving " << distanceToGoal << "\n";
	while(true)//walking
	{
		getTranslationImage(current, false);
		distanceToGoal = getDistanceDifference(*current, *goalState);
		float distanceToNextGoal = getDistanceDifference(*current, *nextGoalState);
		if (distanceToGoal<ERRORRATEDISTANCE)
		{
			cout << "hit target distance, leftover" << distanceToGoal << "\n\n";
			locomotion.SETSPEED(0,0);
			break;
		}
		else if (distanceToNextGoal<distanceToGoal)
		{
			cout << "overshot target, distance " << distanceToGoal << "\n\n";
			locomotion.SETSPEED(0,0);
			break;
		}
		else locomotion.SETSPEED(750, -750);
	}
}
void followPathBackwards(AStarNode* startingNode, TransformationData* current, TransformationData* goalState, TransformationData* nextGoalState) //goes from start to end ndoe
{
	cout << "moving backwards\nSleeping for 5 seconds\n";
	sleep(5);
	AStarNode* currentNode = startingNode;
	
	if (currentNode == NULL) {cout << "big bad error\n";return;}
	
	while(currentNode != NULL)
	{
		AStarNode* nextNode = currentNode->parent;
		if (nextNode ==NULL) {cout << "done\n";break;}
		goalState->tx = nextNode->x;
		goalState->ty = nextNode->y;
		
		AStarNode* nextNextNode = nextNode->parent;
		if (nextNextNode == NULL)
		{
			nextGoalState->tx = goalState->tx + (goalState->tx - currentNode->x);
			nextGoalState->ty = goalState->ty + (goalState->ty - currentNode->y);
		}
		else
		{
			nextGoalState->tx = nextNextNode->x;
			nextGoalState->ty = nextNextNode->y;
		}
		
		float distance;
		getTranslationImage(current);
		distance = getDistanceDifference(*current, *goalState);
		if (distance < ERRORRATEDISTANCE)
		{
			currentNode = nextNode;
			cout << "skipping next node\n";
			continue;
		}
		
		turnMoveBackwards(current, goalState, nextGoalState);
		currentNode = nextNode;
	}
}

