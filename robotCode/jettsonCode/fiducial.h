
void frameTranslation(float horizontalDisplacement, float depth, float angle) //translate position from fiducial to what robot position is
{
	float x = horizontalDisplacement/100 ;//+ XFidCamRelativeToRobot;
	float y = -depth/100;//+ YFidCamRelativeToRobot;
	
	float rz = angle;
	
	float theta, gamma, alpha = 0;
	theta = angle;
	alpha = atan2(YFidCamRelativeToRobot, XFidCamRelativeToRobot) * 180 / PI;
	gamma = alpha + theta;
	
	float distanceToCenterOfRobot = sqrt(pow(XFidCamRelativeToRobot, 2) + pow(YFidCamRelativeToRobot, 2));
	robotPositionX = x - distanceToCenterOfRobot * cos((gamma) / 180 * PI) + fiducialPositionX;
	robotPositionY = y - distanceToCenterOfRobot * sin((gamma) / 180 * PI) + fiducialPositionY;
	
}

bool fiducial(int argc, char **argv)
{
	bool poseFound = false;
	float x, z, theta = 0;
	tie(x, z, theta, poseFound) = fiducialNums(argc, argv);
	if (poseFound == false) return false;
	frameTranslation(x, z, theta);
	return true;
}
