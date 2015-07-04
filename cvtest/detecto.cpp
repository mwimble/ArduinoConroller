#include <chrono>
#include <ctime>
#include <iostream>
#include <string>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "EwynCamera.h"

using namespace cv;
using namespace std;

Scalar colorArray[] = {Scalar(0, 0, 255),Scalar(255, 0, 0),Scalar(0, 255, 0),Scalar(255, 128, 64),Scalar(64, 128, 255),Scalar(128, 255, 64),Scalar(77,77,77),Scalar(164,124,68),Scalar(204,196,132),Scalar(164,148,147),Scalar(163,123,67),Scalar(26,122,26), Scalar(195,195,50),Scalar(193,193,193),Scalar(255,248,73),Scalar(243,243,243)};

int main( int argc, char** argv ) {
    std::chrono::time_point<std::chrono::system_clock> start, end, loopStart;

    EwynCamera  camera("/home/pi/Robotics/cvtest/floor1.jpg", 0.25);
    if (!camera.imageFound()) {
        throw -1;
    }

    camera.createControlWindow();
    camera.thresholdImage();

    camera.detectLines();
    std::vector<EwynCamera::TLINE> verticalLines = camera.getVerticalLines();
    cout << "VERTICAL LINES all count: " << verticalLines.size() << endl;
    Mat ti = camera.getOriginalImage();
    int colorArrayLength = sizeof(colorArray) / sizeof(colorArray[0]);
    for (int i = 0; i < verticalLines.size(); i++) {
        const EwynCamera::TLINE line = verticalLines[i];
        const EwynCamera::TLINE_SEGMENT& first = line.lineSegments.front();
        const EwynCamera::TLINE_SEGMENT& last = line.lineSegments.back();
        if (line.lineSegments.size() > 5) {
            cout << verticalLines[i].toString() << endl;
            EwynCamera::TLINE::CURVE_FIT curveFit = verticalLines[i].linearCurveFit();
            cout << "...Curve fit a: " << curveFit.a << ", b: " << curveFit.b << endl;
            for (int segx = 0; segx < line.lineSegments.size(); segx++) {
                const EwynCamera::TLINE_SEGMENT seg = line.lineSegments[segx];
                cv::line(ti, Point(seg.x, seg.y), Point(seg.x - 1, seg.y + 1), colorArray[i % colorArrayLength], 1, 8);
                cv::line(ti, Point(seg.x, seg.y), Point(seg.x + seg.length - 1, seg.y + 1), colorArray[i % colorArrayLength], 1, 8);
            }
            //linearCurveFit(line);
            /*
            cv::line(ti, Point(first.x, first.y), Point(last.x, last.y), colorArray[i % colorArrayLength], 2, 8);
            cv::line(ti, Point(last.x, last.y), Point(last.x + last.length, last.y), colorArray[i % colorArrayLength], 2, 8);
            cv::line(ti, Point(last.x + last.length, last.y), Point(first.x + first.length, first.y), colorArray[i % colorArrayLength], 2, 8);
            cv::line(ti, Point(first.x + first.length, first.y), Point(first.x, first.y), colorArray[i % colorArrayLength], 2, 8);
            */
        }
    }

    Scalar axisColor = Scalar(0, 255, 255);
    Scalar verticalLineColor = Scalar(0, 136, 255);
    Size size = camera.getOriginalImage().size();
    EwynCamera::TLINE::CURVE_FIT curveFit = verticalLines[0].linearCurveFit();
    cv::line(ti, Point(size.width / 2, size.height), Point (size.width / 2, 0), axisColor, 1, 8);
    cv::line(ti, Point(0, size.height / 2), Point(size.width, size.height / 2), axisColor, 1, 8);
    cv::line(ti, Point(curveFit.a + curveFit.b * size.height, size.height), Point(curveFit.a , 0), verticalLineColor, 1, 8);
    imshow("Original Image", camera.getOriginalImage());
    imshow("Thresholded Image", camera.getThresholdedImage());
    waitKey(0);

    /*
    double accumTime = 0.0;
    std::chrono::duration<double> elapsed_seconds;
    for (int i = 0; i < 10; i++) {
        start = std::chrono::system_clock::now();
        camera.updateOriginalImage();
        end = std::chrono::system_clock::now();
        elapsed_seconds = end-start;
        accumTime += elapsed_seconds.count();
    }
    Size size = camera.getOriginalImage().size();
    cout << "fps for still image: " << (10.0 / accumTime) << ", imageSize height: " << size.height << ", width: " << size.width << endl;

    //waitKey(0);

    camera = EwynCamera(VideoCapture(0), 1.0);
    accumTime = 0.0;
    for (int i = 0; i < 10; i++) {
        start = std::chrono::system_clock::now();
        camera.updateOriginalImage();
        end = std::chrono::system_clock::now();
        elapsed_seconds = end-start;
        accumTime += elapsed_seconds.count();
    }
    cout << "fps for video image: " << (10.0 / accumTime) << endl;
    
    accumTime = 0.0;
    int count = 0;
    loopStart = std::chrono::system_clock::now();
    while (true) {
        start = std::chrono::system_clock::now();
        camera.updateOriginalImage();
        if (!camera.imageFound()) {
            throw -1;
        }

        //imshow("Original Video", camera.getOriginalImage());
        if (waitKey(3) == 27) {
            break;
        }
        end = std::chrono::system_clock::now();
        elapsed_seconds = end - start;
        accumTime += elapsed_seconds.count();
        count++;
        if (count >= 100) { break;}
    }

    end = std::chrono::system_clock::now();
    elapsed_seconds = end - loopStart;
    double loopSeconds = elapsed_seconds.count();
    size = camera.getOriginalImage().size();

    cout << "fps for video image: " << ((count * 1.0) / accumTime)  << ", count: " << count << ", accum time: " << accumTime
         << ", total loop time: " << loopSeconds << ", imageSize height: " << size.height << ", width: " << size.width << endl;
    */

    return 0;
/*
    VideoCapture cap(0); // Capture the video.
    cout << "About to namedWindow" << endl;
    namedWindow("Control", CV_WINDOW_AUTOSIZE); //create a window called "Control"

    int iLowH = 0;
    int iHighH = 179;

    int iLowS = 0; 
    int iHighS = 255;

    int iLowV = 0;
    int iHighV = 255;

    // Window width: 100px

    cout << "About to cvCreateTrackbar" << endl;
    //Create trackbars in "Control" window
    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);


    Mat imgOriginal;
    Mat imgHSV;
    Mat imgThresholded;

    imgOriginal = imread("/home/pi/Robotics/cvtest/floor1.jpg");
    bool bSuccess = !imgOriginal.empty();
    Size rescale(648,486);
    resize(imgOriginal, imgOriginal, rescale);

    bool verticalLinesDrawn = false;
    bool horizontalLinesDrawn = false;

    while (true) {
        //bool bSuccess = cap.read(imgOriginal);
        if (!bSuccess) {
            // If not success, break loop
            cout << "Cannot read a frame from video stream" << endl;
            return -1;
        }

    //#####
    iLowH = 75;
    iHighH = 179;
    iLowS = 0;
    iHighS = 170;
    iLowV = 0;
    iHighV = 106;
    //#####

        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV); //Convert the captured frame from BGR to HSV
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image

        // Morphological opening (remove small objects from the foreground)
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 

        // Morphological closing (fill small holes in the foreground)
        dilate( imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) ); 
        erode(imgThresholded, imgThresholded, getStructuringElement(MORPH_ELLIPSE, Size(5, 5)) );


        vector<TLINE> verticalLineStarters;
        vector<TLINE> horizontalLineStarters;

        cout << "Starting vertical line detection" << endl;
        for (int row  = imgThresholded.rows - 1; row >= (imgThresholded.rows / 2); row--) {
            //cout << "ROW: " << row << endl;
            uchar* rowData = imgThresholded.ptr(row);
            int start = -1;
            int length = 0;
            for (int col = 0; col < imgThresholded.cols; col++) {
                if (pixelRepresentsALine(*rowData)) {
                    if (start == -1) {
                        start = col;
                        length = 1;
                    } else {
                        length++;
                    }
                } else {
                    if ((start >= 0) && (length >= 20)) {
                        if (length > 20) {
                            insertLineElement(verticalLineStarters, TLINE_SEGMENT(start, row, length, V));
                        }
                    }

                    start = -1;
                    length = 0;
                }

                rowData++;
            }
        }

        cout << "Starting horizontal line detection" << endl;
        for (int col = 0; col < imgThresholded.cols; col++) {
            int start = -1;
            int length = 0;
            for (int row = imgThresholded.rows / 2; row < imgThresholded.rows; row++) {
                if (pixelRepresentsALine(imgThresholded.at<uchar>(row, col))) {
                    if (start == -1) {
                        start = row;
                        length = 1;
                    } else {
                        length++;
                    }
                } else {
                    if ((start >= 0) && (length >= 20)) {
                        if (length > 20) {
                            insertLineElement(horizontalLineStarters, TLINE_SEGMENT(start, col, length, H));
                        }
                    }

                    start = -1;
                    length = 0;
                }
            }
        }

        cout << "VERTICAL LINES all count: " << verticalLineStarters.size() << endl;
        for (int i = 0; i < verticalLineStarters.size(); i++) {
            cout << TLINE::dump(verticalLineStarters, i) << endl;
            const TLINE line = verticalLineStarters[i];
            const TLINE_SEGMENT& first = line.lineSegments.front();
            const TLINE_SEGMENT& last = line.lineSegments.back();
           if (line.lineSegments.size() > 5) {
                linearCurveFit(line);
                if (!verticalLinesDrawn) {
                    cv::line(imgOriginal, Point(first.x, first.y), Point(last.x, last.y), Scalar(0, 0, 255), 2, 8);
                    cv::line(imgOriginal, Point(last.x, last.y), Point(last.x + last.length, last.y), Scalar(0, 0, 255), 2, 8);
                    cv::line(imgOriginal, Point(last.x + last.length, last.y), Point(first.x + first.length, first.y), Scalar(0, 0, 255), 2, 8);
                    cv::line(imgOriginal, Point(first.x + first.length, first.y), Point(first.x, first.y), Scalar(0, 0, 255), 2, 8);
                    verticalLinesDrawn = true;
                }
            }
        }

        cout << "HORIZONTAL LINES all count: " << horizontalLineStarters.size() << endl;
        for (int i = 0; i < horizontalLineStarters.size(); i++) {
            cout << TLINE::dump(horizontalLineStarters, i) << endl;
            const TLINE line = horizontalLineStarters[i];
            const TLINE_SEGMENT& first = line.lineSegments.front();
            const TLINE_SEGMENT& last = line.lineSegments.back();
            if (line.lineSegments.size() > 5) {
                linearCurveFit(line);
                if (!horizontalLinesDrawn) {
                    cv::line(imgOriginal, Point(first.y, first.x), Point(last.y, last.x), Scalar(0, 0, 255), 2, 8);
                    cv::line(imgOriginal, Point(last.y, last.x), Point(last.y, last.x + last.length), Scalar(0, 0, 255), 2, 8);
                    cv::line(imgOriginal, Point(last.y, last.x + last.length), Point(first.y, first.x + first.length), Scalar(0, 0, 255), 2, 8);
                    cv::line(imgOriginal, Point(first.y, first.x + first.length), Point(first.y, first.x), Scalar(0, 0, 255), 2, 8);
                    horizontalLinesDrawn = true;
                }
            }
        }

        imshow("Thresholded Image", imgThresholded); //show the thresholded image
        imshow("Original", imgOriginal); //show the original image

        if (waitKey(30) == 27) {
            // Wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
            cout << "esc key is pressed by user" << endl;
            return 0; 
        }
    }

    cout << "imgThresholded rows: " << imgThresholded.rows
        << ", cols: " << imgThresholded.cols
        << ", elemSize: " << imgThresholded.elemSize()
        << ", isContinuous: " << imgThresholded.isContinuous()
        << ", type: " << imgThresholded.type()
        << endl;

  return 0;
*/
}
