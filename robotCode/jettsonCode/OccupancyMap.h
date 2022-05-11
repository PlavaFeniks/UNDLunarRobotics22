float ***Zval = new float**[IMAGEWIDTH];//float Zval[1280][720];
float ***Xval = new float**[IMAGEWIDTH];//float Xval[1280][720];
float ***Yval = new float**[IMAGEWIDTH];//float Yval[1280][720];

//Plane groundPlane;
/*ground plane = ax + by + cz = d FOR REFERENCE :: https://www.stereolabs.com/docs/api/classsl_1_1Plane.html#ae5080d63d8703462b5db4d1eb4333942
	groundPlane.x = a;
	groundPlane.y = b;
	groundPlane.z = c;
	groundPlane.w = d;
 
		 
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
float gplaneA;
float gplaneB;
float gplaneC;
float gplaneD;

double OccThresh = 1.0f; // -1 to 1 where .5 is 75% chance it's occupied and 1 is totally tubularly occupied
void occupancyMap(int scale=10, double threshVal=100)
{

	//Read in the sensor data and update the average value for the cell
	// i and j for the image points, will need to smooth the data only taking verified points
	
	for (int i=0; i<IMAGEWIDTH; i++)
		{
			for (int j=0; j<IMAGEHEIGHT; j++)
			{
				if (Xval[i][j] == NULL) continue;
				
				int xhere = int(*Xval[i][j]);
				int yhere = int(*Yval[i][j]);
				float zhere = *Zval[i][j];
				
				if (xhere >= 0 and xhere < WIDTH and yhere >= 0 and yhere < HEIGHT)
				{
					mapOfPit[yhere][xhere]->Zsum += zhere;
					mapOfPit[yhere][xhere]->Nobs ++;
					mapOfPit[yhere][xhere]->Zval = mapOfPit[yhere][xhere]->Zsum/float(mapOfPit[yhere][xhere]->Nobs);
					
					float dist = abs(gplaneA*(*Xval[i][j])*scale+gplaneB*(*Yval[i][j])*scale+gplaneC*(*Zval[i][j])*scale+gplaneD)/(sqrt(pow(gplaneA,2)+pow(gplaneB,2)+pow(gplaneC,2)));
					mapOfPit[yhere][xhere]->Disthere = dist;
					if (abs(dist) >= threshVal)
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


//gets ground plane and point cloud
//affects global variables
void getCloudAndPlane(int scale)
{
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
        //this loop will iterate through every pixel on zed camera and get it's "z" value
		for (int i=0; i<IMAGEWIDTH; i++)
		{
			for (int j=0; j<IMAGEHEIGHT; j++)
			{	
				float confidenceZED = 0;
				confidence_map.getValue(i, j, &confidenceZED);
				if (confidenceZED >50) continue; //1-100+
				
				
				sl::float4 point_cloud_value;
				point_cloud.getValue(i,j,&point_cloud_value);
				if (std::isfinite(point_cloud_value.z))
				{
					Xval[i][j] = new float(point_cloud_value.x/scale + zedPositionX);
					Yval[i][j] = new float(point_cloud_value.y/scale + zedPositionY);
					Zval[i][j] = new float(point_cloud_value.z/scale);
					int x = int(*Xval[i][j]);
					int y = int(*Yval[i][j]);
					int z = int(*Zval[i][j]);
					if (x >=0 and x <WIDTH and y >=0 and y <HEIGHT) mapOfPit[y][x]->setXYZ(x, y, z);
				}
			}
		}
		occupancyMap(scale);
		k += 1;       
	}
	cout << "point cloud defined\n";
}

void initializeOccupancyMapXYZVal()
{
	for (int i=0; i<IMAGEWIDTH; i++)
	{
		Zval[i] = new float*[IMAGEHEIGHT];
		Xval[i] = new float*[IMAGEHEIGHT];
		Yval[i] = new float*[IMAGEHEIGHT];
		for (int j=0; j<IMAGEHEIGHT; j++)
		{
			Zval[i][j] = NULL;
			Xval[i][j] = NULL;
			Yval[i][j] = NULL;
		}
	}
}

/*
 * Problem: each tile in the occupancy map is 10cm by 10cm, robot is bigger than that
 * Solution: expand the not traversable parts of the occupancy map
 * */
void thiccOccupancymap(int thickenAmount)
{
	for (int i=HEIGHT-1; i>=0; i--) //y of map
	{
		for (int j=0; j<WIDTH; j++) //x of map
		{
			
			bool backout = false;
			if (mapOfPit[i][j]->isTraversable == false) continue;
			
			for (int r=-thickenAmount; r<=thickenAmount; r++) //y around map
			{
				for (int q=-thickenAmount; q<=thickenAmount; q++)
				{
					if (q==0 and r == 0) continue;
					int newY = i+q;
					int newX = j+r;
					
					if (newX < 0 or newX > WIDTH-1 or newY < 0 or newY > HEIGHT-1) continue;
					
					
					if (mapOfPit[newY][newX]->Pocc < OccThresh)
					{
						mapOfPit[i][j]->isTraversable = false;
						backout = true;
						break;
					}
				}
				if (backout ==true) break;
			}
		}
	}
}

void cmdLineOccupancyMap() //displays the occupancy map in cmdline
{
	cout << "start of map\n";
	for (int i=HEIGHT-1; i>=0; i--)
	{
		for (int j=0; j<WIDTH; j++)
		{
			// /*
			if (mapOfPit[i][j]->child != NULL) cout << "\x1B[33m-";
			else if (mapOfPit[i][j] == endNode) cout << "\x1B[94me";
			else if (mapOfPit[i][j] == startNode) cout << "\x1B[95ms";				
			else if (mapOfPit[i][j]->isTraversable) cout << " ";//if traversable
			else cout << "\x1B[91m1"; //if not traversable
			cout << "\033[0m";
		}
		cout << "\n";
	}
	cout << "end of map\n";
	sleep(1);
}

void cmdLineNobs()
{
	cout << "start of map\n";
	for (int i=HEIGHT-1; i>=0; i--)
	{
		for (int j=0; j<WIDTH; j++)
		{
			// /*
			if (mapOfPit[i][j]->Nobs > 0) cout << "\x1B[91m1"; //if not traversable
			else cout << " ";
			cout << "\033[0m";
		}
		cout << "\n";
	}
	cout << "end of map\n";
	sleep(1);
}
