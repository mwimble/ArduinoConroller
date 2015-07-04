#ifndef __EWYN_CAMERA_H
#define __EWYN_CAMERA_H

#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class EwynCamera {
public:
	EwynCamera(cv::VideoCapture videoDevice, double scaleFactor = 1.0);

	EwynCamera(const std::string testFileName, double scaleFactor = 1.0);

	~EwynCamera();

	void createControlWindow();

	int getLowHueThreshold() { return lowHueThreshold; }

	cv::Mat getOriginalImage() { return originalImage; }

	cv::Mat getThresholdedImage() { return thresholdedImage; }

	bool imageFound() { return imageLoaded; }

	void scaleOriginalImage(double scaleFactor);

	void setDefaultThresholding();

	void thresholdImage();

	void updateOriginalImage();

private:
	// Either use a test file name or a video feed.
	std::string fileName;
	cv::VideoCapture videoFeed;
	bool imageLoaded;
	double scaleFactor;

	// OpenCV objects.
	cv::Mat originalImage;
	cv::Mat hsvImage;
	cv::Mat	thresholdedImage;

	// Thresholding limits.
	static const int		kLowHueThreshold = 75;
	static const int		kHighHueThreshold = 179;
	static const int 		kLowSaturationThreshold = 0;
	static const int 		kHighSaturationThreshold = 170;
	static const int 		kLowValueThreshold = 0;
	static const int 		kHighValueThreshold = 106;

	int			lowHueThreshold;
	int			highHueThreshold;
	int 		lowSaturationThreshold;
	int 		highSaturationThreshold;
	int 		lowValueThreshold;
	int 		highValueThreshold;
	cv::Size	morphSize;

	// Control window variables.
	std::string controlWindowName;

};
#endif