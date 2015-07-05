#ifndef __EWYN_CAMERA_H
#define __EWYN_CAMERA_H

#include <iostream>
#include <string>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

class EwynCamera {
public:
	typedef enum { V, H } TLINE_DIRECTION;

	static const char* TLINE_DIRECTION_STRINGS[2];

	struct TLINE_SEGMENT {
	    int x;
	    int y;
	    int length;
	    TLINE_DIRECTION dir;

	    TLINE_SEGMENT(int xx, int yy, int ll, TLINE_DIRECTION dd) :
	        x(xx), y(yy), length(ll), dir(dd) {};

	    std::string toString() const;
	};

	struct TLINE {
	public:
		struct CURVE_FIT {
			float a;
			float b;
			CURVE_FIT(float a_, float b_) : a(a_), b(b_) {}
		};

		static const int kCapturedMidpoints = 10;
	    int number;
	    int lastMidpoints[kCapturedMidpoints];
	    int midpointIndex;
	    long sumLengths;
	    TLINE_DIRECTION dir;
	    std::vector<TLINE_SEGMENT> lineSegments;

	    static int nextNumber;

	    TLINE();

	    // Return the average length (width) of all the TLINE_SEGMENTs in the collection.
	    int averageLength() const { return lineSegments.size() == 0 ? 0 : sumLengths / lineSegments.size(); }

	    static const bool debugAddLineElement = false;
	    void addLineSegment(TLINE_SEGMENT lineSegment);

	    bool isVerticalLine() { return lineSegments.size() > averageLength(); }

	    static const bool debugLinearCurveFit = true;
		CURVE_FIT linearCurveFit();

	    // Return the average midpoint (x-coordinate) of all the TLINE_SEGMENTs in the collection.
	    int midpoint() const;

	    std::string toString();
	};

	EwynCamera(cv::VideoCapture videoDevice, double scaleFactor = 1.0);

	EwynCamera(const std::string testFileName, double scaleFactor = 1.0);

	~EwynCamera();

	void createControlWindow();

	void detectLines();

	std::vector<TLINE> getHorizontalLines() { return horizontalLines; }

	int getLowHueThreshold() { return lowHueThreshold; }

	cv::Mat getOriginalImage() { return originalImage; }

	cv::Mat getThresholdedImage() { return thresholdedImage; }

	std::vector<TLINE> getVerticalLines() { return verticalLines; }

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

	// For line detection.
	static const int kMinimumLineSegmentLength = 15;

	std::vector<TLINE> verticalLines;
	std::vector<TLINE> horizontalLines;

	void computeVerticalLines();
	void insertLineSegment(std::vector<TLINE>& arrayOfLines, const TLINE_SEGMENT newLine);
	bool linesOverlap(const TLINE_SEGMENT l1, TLINE_SEGMENT l2);
	bool pixelRepresentsALine(uchar pixel) { return pixel == 255; }
};
#endif