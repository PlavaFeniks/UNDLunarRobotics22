
//https://github.com/stereolabs/zed-examples/blob/master/positional%20tracking/cpp/src/main.cpp
float x,y,z;
void GetTranslationImage() //sets position and angle change using images
{    
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
			sl::Pose zed_pose;
			auto state = zed.getPosition(zed_pose, REFERENCE_FRAME::CAMERA);
			if (state == POSITIONAL_TRACKING_STATE::OK)
			{
				x+=zed_pose.getTranslation().x;
				y+=zed_pose.getTranslation().y;
				z+=zed_pose.getTranslation().z;
				//cout << zed_pose.getEulerAngles(false).x << " " << zed_pose.getEulerAngles(false).y << " " << zed_pose.getEulerAngles(false).z << "\n";
				//cout << setprecision(3) << zed_pose.getTranslation().x << " " <<  << " " <<zed_pose.getTranslation().z << "\n";
				cout << setprecision(3) << x << " " << y << " " << z << "\n";
				break;
			}
		}
	}
}

void GetTranslationIMU() //sets position and angle change using IMU
{
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
	while (true)
	{
		if (zed.grab() == ERROR_CODE::SUCCESS)
		{
			//https://www.stereolabs.com/docs/tutorials/using-sensors/
			//https://www.stereolabs.com/docs/tutorials/positional-tracking/
			SensorsData sensorsData;
			SensorsData::IMUData imuData;
			sl::Pose zed_pose;
			auto state = zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);
			if (zed.getSensorsData(sensorsData, TIME_REFERENCE::IMAGE) == ERROR_CODE::SUCCESS)
			{				
				cout << sensorsData.imu.pose.getTranslation().tz << "\t";
				cout << sensorsData.imu.pose.getEulerAngles().z << "\n";
				break;
			}
		}
	}
}
