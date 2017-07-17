//
// Created by kiryanenko on 16.07.17.
//

#ifndef SMARTCAMERA_MAP_H
#define SMARTCAMERA_MAP_H

#include <opencv2/opencv.hpp>
#include <vector>


class Map {
    cv::Mat m_map;                      // Изображение карты
    cv::Point m_cameraPos;              // Положение камеры на карте
    cv::Point m_cameraView;             // Положение точки на карте, куда смотрит центр камеры
    double m_horisontalAngle;           // Угол поворота камеры по горизонтали (налево - положительный)
    double m_verticalAngle;             // Угол наклона камеры по вертикали (относительно горизонта)
    double m_height;                    // Высота камеры (усл. ед.)
    cv::Size m_frameSize;               // Размер фрейма с камеры
    double m_frameScale;                // Количество пикселей на 1 градус на фрейме с камеры

    int m_yCorrection;                  // Поправка по OY (выделяется туловище но не ноги, для устранения этого ввёл эту поправку)

public:
    Map(const char *img, cv::Point cameraPos, cv::Point cameraView, double verticalAngle,
        cv::Size frameSize, double frameScale, int yCorrection = 0);
    void drawPositions(cv::Mat &dst, const std::vector<cv::Rect> &targets);

private:
    void drawCameraPoint(cv::Mat &dst);
};


#endif //SMARTCAMERA_MAP_H
