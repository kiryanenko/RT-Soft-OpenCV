#include <iostream>
#include <opencv2/opencv.hpp>
#include <chrono>
#include "MotionDetector.h"
#include "Map.h"

static const double CYCLE_TIME = 20;

int main(int argc, char* argv[]) {
    std::cout << "start" << std::endl;
    cv::VideoCapture cap("video.avi"); // open the video file for reading
    if ( !cap.isOpened() ) return -1;
    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    std::cout << "Frame per seconds : " << fps << std::endl;
    cv::namedWindow("SmartCamera",CV_WINDOW_AUTOSIZE); //create a window called "SmartCamera"

    cv::Mat frame;
    cap.read(frame);
    MotionDetector motionDetector(frame.size(), CYCLE_TIME);

    Map map("map.png",              // Изображение карты
            cv::Point(300, 380),    // Положение камеры на карте
            cv::Point(140, 100),    // Положение точки на карте, куда смотрит центр камеры
            25,                     // Угол наклона камеры по вертикали
            frame.size(),           // Размер фрейма с камеры
            700,                    // Количество пикселей на 1 градус на фрейме с камеры
            100                     // Поправка по OY (выделяется туловище но не ноги, для устранения этого ввёл эту поправку)
    );

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

        cv::Mat mapImg;
        map.drawPositions(mapImg, targets);

        cv::imshow("SmartCamera", frame);   // show the frame in window
        cv::imshow("Map", mapImg);          // show the map in window

        if(cv::waitKey(30) == 27) {
            break;
        }
    }
    return 0;
}
