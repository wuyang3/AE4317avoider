/*
 * opencv_aruco.cpp
 * Call Aruco library functionality.
 *  Created on: Jun 29, 2017
 *      Author: wuyang
 */

#include "opencv_aruco.h"

using namespace std;
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "modules/computer_vision/opencv_image_functions.h"
#include "aruco.hpp"
using namespace cv;

const float arucoSquareDimension = 0.250f;
const Ptr<aruco::Dictionary> markerDictionary =
			aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
/*
float camera[] = {1273.7650, 0, 534.5183,
							0, 1477.9645, 696.2918,
							0, 0, 1};
float distortion[] = {-0.41297, -1.68895, 6.26034, -5.97930};

Mat cameraMatrix(3, 3, CV_64F, camera);
Mat distortionCoefficients(4, 1, CV_64F, distortion);
*/

int aruco_func(char *img, int width, int height){
	Mat M(height, width ,CV_8UC2, img);
	Mat image;

	vector<int> markerIds;
	vector<vector<Point2f> > markerCorners;
	
	//vector<Vec3d> rotationVectors, translationVectors;

	cvtColor(M, image, CV_YUV2BGR_Y422);

	aruco::detectMarkers(image, markerDictionary, markerCorners, markerIds);
	//aruco::estimatePoseSingleMarkers(
			//markerCorners, arucoSquareDimension, cameraMatrix, distortionCoefficients, rotationVectors, translationVectors);
	if (markerIds.size() > 0){
		aruco::drawDetectedMarkers(image, markerCorners, markerIds);
		//for (uint16_t i = 0; i < markerIds.size(); i++){
			//aruco::drawAxis(image, cameraMatrix, distortionCoefficients, rotationVectors[i], translationVectors[i], 5.0f);
		//}
	}
	colorrgb_opencv_to_yuv422(image, img, width, height);

	return 0;
}
