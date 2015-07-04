#include "EwynCamera.h"

#include <cv.h>
#include <iostream>
#include <sys/stat.h>

using namespace cv;
using namespace std;

void handleControlChange(int newValue, void* userData) {
	EwynCamera* camera = (EwynCamera*) userData;
	cout << "New Value" << newValue << ", low hue value: " << camera->getLowHueThreshold() << endl;
}

EwynCamera::EwynCamera(VideoCapture videoDevice, double scaleFactor)
	: morphSize(5,5), scaleFactor(scaleFactor) {
	controlWindowName = "Ewyn Camera Controls";
	setDefaultThresholding();
	videoFeed = videoDevice;
	imageLoaded = videoFeed.read(originalImage);
	if (imageLoaded) {
		scaleOriginalImage(scaleFactor);
	}
}

EwynCamera::EwynCamera(const std::string testFileName, double scaleFactor)
	: morphSize(5,5), scaleFactor(scaleFactor) {
	struct stat buffer;

	controlWindowName = "Ewyn Camera Controls";
	setDefaultThresholding();
	fileName = testFileName;
	if (stat (testFileName.c_str(), &buffer) == 0) {
		originalImage = imread(fileName);
		imageLoaded = !originalImage.empty();
		if (imageLoaded) {
			scaleOriginalImage(scaleFactor);
		}
	} else {
		imageLoaded = false;
	}
}

EwynCamera::~EwynCamera() {
	fileName = "";
	videoFeed = VideoCapture();
}

void EwynCamera::createControlWindow() {
	namedWindow(controlWindowName.c_str(), CV_WINDOW_AUTOSIZE);
	cvCreateTrackbar2("Low Hue", controlWindowName.c_str(), &lowHueThreshold, 179, handleControlChange, this);
	cvCreateTrackbar2("High Hue", controlWindowName.c_str(), &highHueThreshold, 179, handleControlChange, this);
	cvCreateTrackbar2("Low Saturation", controlWindowName.c_str(), &lowSaturationThreshold, 255, handleControlChange, this);
	cvCreateTrackbar2("High Saturation", controlWindowName.c_str(), &highSaturationThreshold, 255, handleControlChange, this);
	cvCreateTrackbar2("Low Value", controlWindowName.c_str(), &lowValueThreshold, 255, handleControlChange, this);
	cvCreateTrackbar2("High Value", controlWindowName.c_str(), &highValueThreshold, 255, handleControlChange, this);
}

void EwynCamera::scaleOriginalImage(double scaleFactor) {
	if (!originalImage.empty() && (scaleFactor != 1.0)) {
		Size scaleSize = originalImage.size();
		scaleSize.height *= scaleFactor;
		scaleSize.width *= scaleFactor;
		resize(originalImage, originalImage, scaleSize);
	}
}

void EwynCamera::setDefaultThresholding() {
	lowHueThreshold = kLowHueThreshold;
	highHueThreshold = kHighHueThreshold;
	lowSaturationThreshold = kLowSaturationThreshold;
	highSaturationThreshold = kHighSaturationThreshold;
	lowValueThreshold = kLowValueThreshold;
	highValueThreshold = kHighValueThreshold;
}

void EwynCamera::thresholdImage() {
	Mat hsvImage;
	cvtColor(originalImage, hsvImage, COLOR_BGR2HSV);
	inRange(hsvImage, 
			Scalar(lowHueThreshold, lowSaturationThreshold, lowValueThreshold),
			Scalar(highHueThreshold, highSaturationThreshold, highValueThreshold),
			thresholdedImage);

	// Remove small objects from the forground.
	erode(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
	dilate(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));

	// Fill small holes in the background;
	dilate(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
	erode(thresholdedImage, thresholdedImage, getStructuringElement(MORPH_ELLIPSE, morphSize));
}

void EwynCamera::updateOriginalImage() {
	if (imageLoaded) {
		if (fileName.size() > 0) {
			originalImage = imread(fileName);
			imageLoaded = !originalImage.empty();
			if (imageLoaded) {
				scaleOriginalImage(scaleFactor);
			}
		} else {
			imageLoaded = videoFeed.read(originalImage);
			if (imageLoaded) {
				scaleOriginalImage(scaleFactor);
			}
		}
	}
}