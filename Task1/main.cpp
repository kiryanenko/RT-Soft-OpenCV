#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
    cout << "start" << endl;
    VideoCapture cap("video.mov"); // open the video file for reading
    if ( !cap.isOpened() ) return -1;
    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Frame per seconds : " << fps << endl;
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    Ptr<LineSegmentDetector> ls = createLineSegmentDetector(LSD_REFINE_STD);
    Mat edges;
    while(1) {
        Mat frame;
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess) {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        cvtColor(frame, edges, COLOR_BGR2GRAY); // Перевод в градации серого
        std::vector<Vec4f> lines_std;
        ls->detect(edges, lines_std); // Detect the line

        for (size_t i = 0; i < lines_std.size(); ++i) {
            Point2f p1, p2;
            p1.x = lines_std[i][0]; p1.y = lines_std[i][1];
            p2.x = lines_std[i][2]; p2.y = lines_std[i][3];
            line(frame, p1, p2, Scalar(25, 255, 25));
        }

        imshow("MyVideo", frame); //show the frame in "MyVideo" window
        if(waitKey(30) == 27) {
            break;
        }
    }
}
