#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

using namespace cv;
using namespace std;

typedef enum { V, H } TLINE_DIRECTION;

static const char* TLINE_DIRECTION_STRINGS[] = { "V", "H" };

struct TLINE_SEGMENT {
    int x;
    int y;
    int length;
    TLINE_DIRECTION dir;

    TLINE_SEGMENT(int xx, int yy, int ll, TLINE_DIRECTION dd) :
        x(xx), y(yy), length(ll), dir(dd) {};

    String dump() const {
        stringstream ss;
        ss << "{"
             << "x:" << x
             << ", y:" << y
             << ", l:"  << length
             << ", d: " << dir
             << "}";
        return ss.str();
    }
};

struct TLINE {
    int number;
    int lastMidpoints[10];
    int midpointIndex;
    long sumLengths;
    TLINE_DIRECTION dir;
    vector<TLINE_SEGMENT> lineSegments;

    static int nextNumber;

    TLINE() {
        for (int i = 0; i < 10; i++) { lastMidpoints[i] = -1; }
        midpointIndex = 0;
        sumLengths = 0;
        number = nextNumber++;
    }

    static const bool debugAddLineElement = true;
    
    int length() const {
        return lineSegments.size() == 0 ? 0 : sumLengths / lineSegments.size();
    }

    int midpoint() const {
        int result = 0;
        int count = 0;
        for (int i = 0; i < 10; i++) {
            if (lastMidpoints[i] != -1) {
                result += lastMidpoints[i];
                count++;
            }
        }

        return count == 0 ? 0 : result / count;
    } 

    void addLineSegment(TLINE_SEGMENT lineSegment) {
        lineSegments.push_back(lineSegment);
        lastMidpoints[midpointIndex++] = lineSegment.x + (lineSegment.length / 2);
        if (midpointIndex >= 10) midpointIndex = 0;
        sumLengths += lineSegment.length;
        if (debugAddLineElement) {
            cout << "Push to existing line[" << number
                 << "] back: " << lineSegments.back().dump()
                 << ", newLine: " << lineSegment.dump()
                 << ", avg midpoint: " << midpoint()
                 << ", line midpoint: " << (lineSegment.x + lineSegment.length / 2)
                 << ", segment count: " << lineSegments.size()
                 << endl;
        }
    }

    static String dump(const vector<TLINE>& arrayOfLines, int row) {
        stringstream ss;
        const TLINE_SEGMENT& first = arrayOfLines[row].lineSegments.front();
        const TLINE_SEGMENT& last = arrayOfLines[row].lineSegments.back();
        ss << "{Line " << row 
           << ", dir: " << TLINE_DIRECTION_STRINGS[arrayOfLines[row].dir]
           << ", segCount: " << arrayOfLines[row].lineSegments.size()
           << ", front: " << first.dump()
           << ", back: " << last.dump()
           << ", avg midpoint: " << arrayOfLines[row].midpoint()
           << ", avg length: " << arrayOfLines[row].length()
           << ", box ll: " << first.x << "," << first.y
           << ", ul: " << last.x << "," << last.y
           << ", lr: " << (first.x + first.length - 1) << "," << first.y
           << ", ur: " << (last.x + last.length - 1) << "," << last.y
           << "}";
        return ss.str();
    }
};

int TLINE::nextNumber = 0;


static const bool debugLinesOverlap = true;
bool linesOverlap(const TLINE_SEGMENT l1, TLINE_SEGMENT l2) {
    bool result = false;
    if (l1.x <= l2.x) {
        result = l2.x <= (l1.x + l1.length);
    } else {
        result = l1.x <= (l2.x + l2.length);
    }

    if (debugLinesOverlap) {
        cout << "linesOverlap result: " << result << ", l1: " << l1.dump() << ", l2: " << l2.dump() << endl;
    }

    return result;
}

void insertLineElement(vector<TLINE>& arrayOfLines, const TLINE_SEGMENT newLine) {
    bool lineFound = false;

    // Iterate over all existing lines as this new line might be part of several
    // existing lines if the lines in the image are not perfectly N, S, E, W.
    for (int i = 0; i < arrayOfLines.size(); i++) {
        if (linesOverlap(arrayOfLines[i].lineSegments.back(), newLine) &&
            (newLine.x >= arrayOfLines[i].midpoint() - arrayOfLines[i].length()) &&
            (newLine.x <= arrayOfLines[i].midpoint() + arrayOfLines[i].length())) {
            arrayOfLines[i].addLineSegment(newLine);
            lineFound = true;
        }
    }

    if (!lineFound) {
        cout << "Creating new line" << endl;
        TLINE oneElementVector;
        oneElementVector.addLineSegment(newLine);
        oneElementVector.dir = newLine.dir;
        arrayOfLines.push_back(oneElementVector);
    }
}

static const bool debugLinearCurveFit = false;
void linearCurveFit(const TLINE& data) {
    if (debugLinearCurveFit) {
        cout << "linearCurveFit START data.lineSegments.size(): " << data.lineSegments.size() << endl;
    }

    long sumx = 0;
    long sumx2 = 0;
    long sumy = 0;
    long sumxy = 0;
    int n = data.lineSegments.size();
    for (int i=0; i <= n - 1; i++) {
        sumx = sumx + data.lineSegments[i].x;
        sumx2 = sumx2 + data.lineSegments[i].x * data.lineSegments[i].x;
        sumy = sumy + data.lineSegments[i].y;
        sumxy = sumxy + data.lineSegments[i].x * data.lineSegments[i].y;
    }

    float t1 = ((sumx2 * 1.0) * (sumy * 1.0));
    float t2 = ((sumx * 1.0) * (sumxy * 1.0));
    float num = t1 - t2;
    float t3 = ((n * 1.0) * (sumx2 * 1.0));
    float t4 = ((sumx * 1.0) * (sumx * 1.0));
    float denom = t3 - t4;
    float a = num / denom;
    float b = ((n * sumxy) - (sumx * sumy * 1.0)) / ((n * sumx2) - (sumx * sumx * 1.0));
    if (debugLinearCurveFit) {
        cout << "n: " << n << ", t1: " << t1 << ", t2: " << t2 << ", num: " << num << ", t3: " << t3 << ", t4: " << t4 << ", denom: " << denom << ", a: " << a << ", b: " << b << endl;
    }
}

bool pixelRepresentsALine(uchar pixel) {
    return pixel == 255;
}

int main( int argc, char** argv )
{
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
}
