// chess-bot-cv.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

	//Mat resized_image;
	//resize(image, resized_image, Size(), 0.25, 0.25);

//Uncomment the following line if you are compiling this code in Visual Studio
//#include "stdafx.h"

#include <opencv2/opencv.hpp>
#include <iostream>

using namespace cv;
using namespace std;

class Chess_CV_Test {
	Mat image;

	int iLowH = 0;
	int iHighH = 179;

	int iLowS = 0;
	int iHighS = 255;

	int iLowV = 0;
	int iHighV = 255;

	int display_image(Mat, String, double);
	int process();
	int create_control_window();

	Chess_CV_Test(Mat input_img) {
		Mat image = input_img;
		create_control_window();
		process();
	}
};

int Chess_CV_Test::display_image(Mat image, String windowName, double scale) {
	// Display image
	namedWindow(windowName); // Create a window
	moveWindow(windowName, 40, 30);
	Mat resized_image;
	resize(image, resized_image, cv::Size(), scale, scale);
	imshow(windowName, resized_image); // Show our image inside the created window.
	waitKey(0); // Wait for any keystroke in the window
	destroyWindow(windowName); //destroy the created window
	return 0;
}

int Chess_CV_Test::process() {
	Mat imgHSV;
	cvtColor(image, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV

	Mat imgThresholded;
	inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

	//morphological opening (remove small objects from the foreground)
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));

	//morphological closing (fill small holes in the foreground)
	dilate(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));
	erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, cv::Size(5, 5)));

	Mat image_to_display;
	cvtColor(imgHSV, image_to_display, COLOR_HSV2BGR);

	String windowName = "Thresholded Image";
	destroyWindow(windowName);
	display_image(image_to_display, windowName, 0.15);
}

int Chess_CV_Test::create_control_window() {
	namedWindow("Control"); //create a window called "Control"

	//Create trackbars in "Control" window
	createTrackbar("LowH", "Control", &iLowH, 179, process); //Hue (0 - 179)
	createTrackbar("HighH", "Control", &iHighH, 179, process);

	createTrackbar("LowS", "Control", &iLowS, 255, process); //Saturation (0 - 255)
	createTrackbar("HighS", "Control", &iHighS, 255, process);

	createTrackbar("LowV", "Control", &iLowV, 255, process); //Value (0 - 255)
	createTrackbar("HighV", "Control", &iHighV, 255, process);

	return 0;
}



int main(int argc, char** argv)
{
	// Read the image file
	Mat image = imread("./chess_cv_test.jpg");

	// Check for failure
	if (image.empty())
	{
		cout << "Could not open or find the image" << endl;
		cin.get(); //wait for any key press
		return -1;
	}

	Chess_CV_Test test = new Chess_CV_Test(image);

}