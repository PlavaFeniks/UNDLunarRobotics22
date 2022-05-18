/*
By downloading, copying, installing or using the software you agree to this
license. If you do not agree to this license, do not download, install,
copy or use the software.

                          License Agreement
               For Open Source Computer Vision Library
                       (3-clause BSD License)

C///opyright (C) 2013, OpenCV Foundation, all rights reserved.
Third party copyrights are property of their respective owners.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  * Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

  * Redistributions in binary form must reproduce the above copyright notice,
    this list of conditions and the following disclaimer in the documentation
    and/or other materials provided with the distribution.

  * Neither the names of the copyright holders nor the names of the contributors
    may be used to endorse or promote products derived from this software
    without specific prior written permission.

This software is provided by the copyright holders and contributors "as is" and
any express or implied warranties, including, but not limited to, the implied
warranties of merchantability and fitness for a particular purpose are
disclaimed. In no event shall copyright holders or contributors be liable for
any direct, indirect, incidental, special, exemplary, or consequential damages
(including, but not limited to, procurement of substitute goods or services;
loss of use, data, or profits; or business interruption) however caused
and on any theory of liability, whether in contract, strict liability,
or tort (including negligence or otherwise) arising in any way out of
the use of this software, even if advised of the possibility of such damage.
*/


#include <opencv4/opencv2/highgui.hpp>
#include <opencv4/opencv2/aruco.hpp>
#include <opencv4/opencv2/opencv.hpp>
#include <iostream>
#include "aruco_samples_utility.hpp"



using namespace std;
using namespace cv;

namespace {
const char* about = "Basic marker detection";

//! [aruco_detect_markers_keys]
const char* keys  =
        "{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
        "DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
        "DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
        "DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16,"
        "DICT_APRILTAG_16h5=17, DICT_APRILTAG_25h9=18, DICT_APRILTAG_36h10=19, DICT_APRILTAG_36h11=20}"
        "{cd       |       | Input file with custom dictionary }"
        "{v        |       | Input from video or image file, if ommited, input comes from camera }"
        "{ci       | 0     | Camera id if input doesnt come from video (-v) }"
        "{c        |       | Camera intrinsic parameters. Needed for camera pose }"
        "{l        | 0.1   | Marker side length (in meters). Needed for correct scale in camera pose }"
        "{dp       |       | File of marker detector parameters }"
        "{r        |       | show rejected candidates too }"
        "{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1,"
        "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}";
}
tuple<vector<Vec3d>,vector<Vec3d>, bool> getFiducialPose(int argc, char **argv);

void fiducial(int argc, char **argv)
{
    vector< Vec3d > marker2cameraRVec, marker2cameraTVec;
    bool poseFound = false;
    Mat marker2cameraRMat = (Mat1d(3, 3) << 1, 0, 0, 	0, 1, 0, 	0, 0, 0);
	tie(marker2cameraRVec, marker2cameraTVec, poseFound) = getFiducialPose(argc, argv);
    Rodrigues(marker2cameraRVec, marker2cameraRMat);
    cout << marker2cameraRVec[0] << " " << marker2cameraTVec[0] << " " << marker2cameraRMat << endl;
}

tuple<vector<Vec3d>,vector<Vec3d>, bool> getFiducialPose(int argc, char **argv)
{
    vector< Vec3d > rvecs, tvecs;
        
    CommandLineParser parser(argc, argv, keys);
    parser.about(about);
	Mat camMatrix = (Mat1d(3, 3) << 1364.1, 0, 0, 0, 1360.8, 0, 1004.9, 511.6639, 1);
	Mat distCoeffs = (Mat1d(1, 4) << -0.0449, 0.0650, 0, 0);
/*    if(argc < 2) {					//print needed parameter
        parser.printMessage();
        return 0;
    }
*/
    bool showRejected = parser.has("r");
    bool estimatePose = parser.has("c");
    float markerLength = parser.get<float>("l");

    Ptr<aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(20);
    if (parser.has("d")) {
        int dictionaryId = parser.get<int>("d");
        dictionary = aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
    }

    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
    if(parser.has("dp")) {
        FileStorage fs(parser.get<string>("dp"), FileStorage::READ);
        bool readOk = dictionary->aruco::Dictionary::readDictionary(fs.root());
        if(!readOk) {
            cerr << "Invalid detector parameters file" << endl;
            return make_tuple(rvecs, tvecs, false);
        }
    }

   if (parser.has("refine")) {
        //override cornerRefinementMethod read from config file
        detectorParams->cornerRefinementMethod = parser.get<int>("refine");
    }
 //   std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;			//corner refinement method

    int camId = parser.get<int>("ci");

    String video;
    if(parser.has("v")) {
        video = parser.get<String>("v");
    }

    if(!parser.check()) {
        parser.printErrors();
        return make_tuple(rvecs, tvecs, false);
    }

    
    else if (parser.has("cd")) {
        FileStorage fs(parser.get<std::string>("cd"), FileStorage::READ);
        bool readOk = dictionary->aruco::Dictionary::readDictionary(fs.root());
        if(!readOk) {
            std::cerr << "Invalid dictionary file" << std::endl;
            return make_tuple(rvecs, tvecs, false);
        }
    }
    
    
	cv::VideoCapture inputVideo("/dev/video0");

    int waitTime;
    
    /*
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }
    */

    double totalTime = 0;
    int totalIterations = 0;

    while(inputVideo.grab()) {

        Mat image, imageCopy;
        inputVideo.retrieve(image);

        double tick = (double)getTickCount();

        vector< int > ids;
        vector< vector< Point2f > > corners, rejected;
        
        // detect markers and estimate pose
        aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
        
        
       if(ids.size() > 0)
            aruco::estimatePoseSingleMarkers(corners, markerLength, camMatrix, distCoeffs, rvecs, tvecs);
            
        // draw results
        image.copyTo(imageCopy);
        if(ids.size() > 0) {
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            for(unsigned int i = 0; i < ids.size(); i++)
            {
				cv::drawFrameAxes(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i], 1, 2);
			}
			cout << endl;
			return make_tuple(rvecs, tvecs, true);
			//break;
        }
        imshow("out", imageCopy);
        
    if (cv::waitKey(10) >= 0) break;
	}
}
