#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "MotionDetector.h"

static const double CYCLE_TIME = 0.02;

#define US_PER_S 1000000.0
#define MS_PER_S 1000.0

int main(int argc, char* argv[]) {
    std::cout << "start" << std::endl;
    cv::VideoCapture cap("video.mov"); // open the video file for reading
    if ( !cap.isOpened() ) return -1;
    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    std::cout << "Frame per seconds : " << fps << std::endl;
    cv::namedWindow("SmartCamera",CV_WINDOW_AUTOSIZE); //create a window called "SmartCamera"

    cv::Mat frame;
    cap.read(frame);
    MotionDetector motionDetector(frame.size(), CYCLE_TIME);

    auto start = std::chrono::system_clock::now();

    while(1) {
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess) {
            std::cout << "Cannot read the frame from video file" << std::endl;
            break;
        }

        auto end = std::chrono::system_clock::now();
        std::vector<cv::Rect> targets;
        cv::Mat mask;
        motionDetector.detectMovingRegions(frame,
                                           std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count(),
                                           targets, mask);

        // Рисую bounding box
        for(const auto& rect : targets)
            cv::rectangle(frame, rect, cv::Scalar( 255, 255, 0 ) );

        cv::imshow("SmartCamera", frame); //show the frame in window
        if(cv::waitKey(30) == 27) {
            break;
        }
    }
    return 0;
}
