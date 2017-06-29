#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/aruco.hpp>

#include <sstream>
#include <iostream>
#include <fstream>
#include <chrono>

using namespace std;
using namespace cv;

template<typename Diff>
void log_progress(Diff d, string name){
	cout << name << chrono::duration_cast<chrono::milliseconds>(d).count()
			<< "ms passed" << endl;
}

const float calibrationSquareDimension = 0.02433f; //meters
const float arucoSquareDimension = 0.050f; //meters 0.0392f
const Size chessboardDimensions = Size(9, 7); //meters

void createArucoMarkers(){
	Mat outputMarker;

	Ptr<aruco::Dictionary> markerDictionary =
			aruco::getPredefinedDictionary(aruco::DICT_4X4_50);
	for(int i=0; i<50; i++){
		aruco::drawMarker(markerDictionary, i, 500, outputMarker, 1);
		ostringstream convert;
		string imageName = "4x4Marker_";
		convert << imageName << i << ".jpg";
		imwrite(convert.str(), outputMarker);
	}
}

void createKnownBoardPosition(Size boardSize, float squareEdgeLength, vector<Point3f>& corners){
	for (int i = 0; i < boardSize.height; i++){
		for (int j = 0; j < boardSize.width; j++){
			corners.push_back(Point3f(j * squareEdgeLength, i * squareEdgeLength, 0.0f));
		}
	}

}

/* Process a set of images! Not used when reading from stream. */
void getChessboardCorners(vector<Mat> images, vector<vector<Point2f> >& allFoundCorners, bool showResults=false){
	for(vector<Mat>::iterator iter = images.begin(); iter !=images.end(); iter++){
			vector<Point2f> pointBuf; //Buffer to hold points found in the image
			bool found = findChessboardCorners(
					*iter, Size(9,7), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
			if (found){
				allFoundCorners.push_back(pointBuf);
			}
			if (showResults){
				drawChessboardCorners(*iter, Size(9,7), pointBuf, found);
				imshow("Looking for corners", *iter);
				waitKey(0);
			}
	}
}

int startWebcamMonitoring(const Mat& cameraMatrix, const Mat& distanceCoefficients, float arucoSquareDimension){

	Mat frame;

	vector<int> markerIds;
	vector<vector<Point2f> > markerCorners, rejectedCandidates;

	aruco::DetectorParameters parameters;

	Ptr<aruco::Dictionary> markerDictionary =
			aruco::getPredefinedDictionary(aruco::DICT_4X4_50);

	VideoCapture vid(0);

	if (!vid.isOpened()){
		return -1;
	}

	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	vector<Vec3d> rotationVectors, translationVectors;

	while(true){
		if(!vid.read(frame)){
			break;
		}
		auto t_start_1 = chrono::high_resolution_clock::now();
		aruco::detectMarkers(frame, markerDictionary, markerCorners, markerIds);
		auto t_now_1 = chrono::high_resolution_clock::now();

		auto t_start_2 = chrono::high_resolution_clock::now();
		aruco::estimatePoseSingleMarkers(
				markerCorners, arucoSquareDimension, cameraMatrix, distanceCoefficients, rotationVectors, translationVectors);
		auto t_now_2 = chrono::high_resolution_clock::now();

		if (markerIds.size() > 0){
			log_progress(t_now_1 - t_start_1, "Marker detection: ");
			log_progress(t_now_2 - t_start_2, "Pose estimation: ");
			aruco::drawDetectedMarkers(frame, markerCorners, markerIds);
			// Note that there might be several markers in one frame.
			for (unsigned int i = 0; i < markerIds.size(); i++){
				aruco::drawAxis(frame, cameraMatrix, distanceCoefficients, rotationVectors[i], translationVectors[i], 0.1f);
			}
		}
		imshow("Webcam", frame);
		if (waitKey(30) >= 0) break;
	}
	return 1;
}

void cameraCalibration(vector<Mat> calibrationImages, Size boardSize, float squareEdgeLength, Mat& cameraMatrix, Mat& distanceCoefficients){
	vector<vector<Point2f> > checkeroboardImageSpacePoints;
	getChessboardCorners(calibrationImages, checkeroboardImageSpacePoints, false);
	vector<vector<Point3f> > worldSpaceCornersPoints(1); //size 1

	createKnownBoardPosition(boardSize, squareEdgeLength, worldSpaceCornersPoints[0]);
	worldSpaceCornersPoints.resize(checkeroboardImageSpacePoints.size(), worldSpaceCornersPoints[0]);

	vector<Mat> rVectors, tVectors;
	distanceCoefficients = Mat::zeros(8,1,CV_64F);

	calibrateCamera(worldSpaceCornersPoints, checkeroboardImageSpacePoints, boardSize, cameraMatrix, distanceCoefficients, rVectors, tVectors);
}

bool saveCameraCalibration(string name, Mat cameraMatrix, Mat distanceCoefficients){
	ofstream outStream(name);
	if (outStream){
		uint16_t rows = cameraMatrix.rows;
		uint16_t cols = cameraMatrix.cols;
		// push out number of rows and columns for easy reading.
		outStream << rows << endl;
		outStream << cols << endl;

		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				double value = cameraMatrix.at<double>(r, c);
				outStream << value << endl;
			}
		}

		rows = distanceCoefficients.rows;
		cols = distanceCoefficients.cols;
		// number of rows and columns.
		outStream << rows << endl;
		outStream << cols << endl;

		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				double value = distanceCoefficients.at<double>(r, c);
				outStream << value << endl;
			}
		}
		outStream.close();
		return true;
	}
	return false;
}

bool loadCameraCalibration(string name, Mat& cameraMatrix, Mat& distanceCoefficients){
	ifstream imStream(name);
	if (imStream){
		uint16_t rows;
		uint16_t cols;
		// Take in rows and columns for better cameraMatrix initialization.
		imStream >> rows;
		imStream >> cols;

		cameraMatrix = Mat(Size(cols, rows), CV_64F);

		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				double read = 0.0f;
				imStream >> read;
				cameraMatrix.at<double>(r, c) = read;
				cout << cameraMatrix.at<double>(r, c) << endl;
			}
		}
		// Distance coefficients. Take in rows and columns.
		imStream >> rows;
		imStream >> cols;

		distanceCoefficients = Mat::zeros(rows, cols, CV_64F);
		for (int r = 0; r < rows; r++){
			for (int c = 0; c < cols; c++){
				double read = 0.0f;
				imStream >> read;
				distanceCoefficients.at<double>(r, c) = read;
				cout << distanceCoefficients.at<double>(r, c) << endl;
			}
		}
		imStream.close();
		return true;
	}
	return false;
}

void cameraCalibrationProcess(Mat& cameraMatrix, Mat& distanceCoefficients){
	Mat frame;
	Mat drawToFrame;

	vector<Mat> savedImages;
	vector<vector<Point2f> > markerCorners, rejectCandidates;

	VideoCapture vid(0);

	if (!vid.isOpened()){
		return;
	}

	int framesPerSecond = 20;

	namedWindow("Webcam", CV_WINDOW_AUTOSIZE);

	while(true){
		if(!vid.read(frame)){
			break;
		}

		vector<Vec2f> foundPoints;
		bool found = false;

		found = findChessboardCorners(frame, chessboardDimensions, foundPoints, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_NORMALIZE_IMAGE);
		frame.copyTo(drawToFrame);
		drawChessboardCorners(drawToFrame, chessboardDimensions, foundPoints, found);
		if (found){
			imshow("Webcam", drawToFrame);
		}
		else
			imshow("Webcam", frame);
		char character = waitKey(1000/framesPerSecond); //capture the pressed character.

		switch(character){
		case ' ':
			//saving images
			if (found){
				Mat temp;
				frame.copyTo(temp);
				savedImages.push_back(temp);
				cout << "image found" << endl;
			}
			break;
		case 10:
			//start calibrations.
			if (savedImages.size() > 15){
				cout << "enough image!" << endl;
				cameraCalibration(savedImages, chessboardDimensions, calibrationSquareDimension, cameraMatrix, distanceCoefficients);
				saveCameraCalibration("cameraCalibration", cameraMatrix, distanceCoefficients);
				cout << "calibrate and save!" << endl;
			}
			break;
		case 27:
			//exit program
			cout << "exit!" << endl;
			return;
			break;
		}
	}
}

int main(int argv, char** argc){
	// createArucoMarkers();

	Mat cameraMatrix = Mat::eye(3, 3, CV_64F);
	Mat distanceCoefficients;

	//cameraCalibrationProcess(cameraMatrix, distanceCoefficients);
	loadCameraCalibration("cameraCalibration", cameraMatrix, distanceCoefficients);
	startWebcamMonitoring(cameraMatrix, distanceCoefficients, arucoSquareDimension);

	return 0;
}
