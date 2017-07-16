//
// Created by kiryanenko on 16.07.17.
//

#include "Map.h"

#include <math.h>
#define PI 3.14159265

Map::Map(const char *img, cv::Point cameraPos, cv::Point cameraView, double verticalAngle,
         cv::Size frameSize, double frameScale) :
        m_cameraPos(cameraPos), m_cameraView(cameraView),
        m_horisontalAngle(atan( (cameraPos.x - cameraView.x + 0.0) / (cameraPos.y - cameraView.y) )),
        m_verticalAngle(abs(verticalAngle) * PI / 180),
        m_frameSize(frameSize), m_frameScale(frameScale)
{
    m_map = cv::imread(img);

    // Расстояние между cameraPos и cameraView
    double distance = sqrt(
            (cameraPos.x - cameraView.x) * (cameraPos.x - cameraView.x) +
            (cameraPos.y - cameraView.y) * (cameraPos.y - cameraView.y)
    );
    m_height = distance * tan(m_verticalAngle);
}

void Map::drawPositions(cv::Mat &dst, const std::vector<cv::Rect> &targets) {
    drawCameraPoint(dst);
    for(const auto& rect : targets) {
        cv::Point posOnCamera(              // Нижняя середина rect (Ноги человека)
                rect.x + rect.width / 2,
                rect.y + rect.height + 100
        );
        double hAngle = m_horisontalAngle - (posOnCamera.x - m_frameSize.width / 2.0) / m_frameScale; // угол по горизонтали
        double vAngle = m_verticalAngle + (posOnCamera.y - m_frameSize.height / 2.0) / m_frameScale;  // угол по вертикали

        if (vAngle > 0) {   // Проверка, что не уходит за линию горизонта
            // Определяю примерное расстояние до объекта по углу по вертикали
            double distance = m_height / tan(vAngle);
            cv::Point pos(
                    m_cameraPos.x - tan(hAngle) * distance,
                    m_cameraPos.y - distance
            );

            cv::circle(dst, pos, 10, cv::Scalar(0, 0, 255));
        }
    }
}

void Map::drawCameraPoint(cv::Mat &dst) {
    dst = m_map.clone();
    cv::circle(dst, m_cameraPos, 5, cv::Scalar(0, 0, 0));
    cv::circle(dst, m_cameraView, 2, cv::Scalar(0, 0, 0));
}
