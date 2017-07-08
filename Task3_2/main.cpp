// Task3_2. Реализовать программу “кротовая нора”
// - читает изображение с доской и стикером;
// - читает изображение “динозавра”
// - находит розовый стикер на доске
// - отображает динозавра “привязанного” к точке розового стикера

#include <iostream>
#include <vector>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include <math.h>


// Функция записывает в stickersCoords координаты стикеров
void recogniseStickersByThreshold(cv::Mat image, std::vector<std::vector<cv::Point>> &stickersCoords) {
    cv::Mat image_hsv;
    cv::cvtColor(image, image_hsv, cv::COLOR_BGR2HSV ); // Преобразуем в hsv
    cv::Mat tmp_img(image.size(), CV_8U);
    // Выделение подходящих по цвету областей. Цвет задается константой :)
    cv::inRange(image_hsv, cv::Scalar(100, 100, 100), cv::Scalar(255, 255, 255), tmp_img);
    // "Замазать" огрехи в при выделении по цвету
    cv::dilate(tmp_img,tmp_img,cv::Mat(), cv::Point(-1,-1), 3);
    cv::erode(tmp_img,tmp_img,cv::Mat(), cv::Point(-1,-1), 1);
    //Выделение непрерывных областей
    cv::findContours(tmp_img, stickersCoords, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_NONE);
}

bool cmpPoint(const cv::Point &a, const cv::Point &b) {
    return a.x < b.x || a.x == b.x && a.y < b.y;
}

int main() {
    using namespace cv;
    using namespace std;

    cout << "start" << endl;
    VideoCapture cap("video.mov"); // open the video file for reading
    if ( !cap.isOpened() ) return -1;
    double fps = cap.get(CV_CAP_PROP_FPS); //get the frames per seconds of the video
    cout << "Frame per seconds : " << fps << endl;
    namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

    // Считываю изображение динозавра
    Mat dino = imread("dino.png");

    std::vector<std::vector<cv::Point>> stickersCoords;
    while(1) {
        Mat frame;
        bool bSuccess = cap.read(frame); // read a new frame from video
        if (!bSuccess) {
            cout << "Cannot read the frame from video file" << endl;
            break;
        }

        recogniseStickersByThreshold(frame, stickersCoords);
        cv::Point sticker1 = *max_element(stickersCoords.front().begin(), stickersCoords.front().end(), cmpPoint);
        cv::Point sticker2 = *min_element(stickersCoords.back().begin(), stickersCoords.back().end(), cmpPoint);

        // Масштабирование
        int side = abs(sticker1.x - sticker2.x);
        cv::resize(dino, dino, cv::Size(side, side));
        // Слияние коартинок
        Mat roi = frame(Rect(min(sticker1.x, sticker2.x), min(sticker1.y, sticker2.y), side, side));    // подматрица frame
        dino.copyTo(roi);

        imshow("MyVideo", frame); //show the frame in "MyVideo" window
        if(waitKey(30) == 27) {
            break;
        }
    }
    return 0;
}