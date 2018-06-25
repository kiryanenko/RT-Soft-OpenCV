Индивидуальное задание: проект SmartCamera
==========================================

В ходе выполнения итогового проекта, индивидуальная тема которого
заранее обсуждалась с преподавателем, была разработана программа,
которая считывает видеопоток с камеры видеонаблюдения, установленной
внутри помещения. И на этом видеопотоке обнаруживает и выделяет людей,
находящихся внутри помещения. А также позволяет показать их положение на
плане здания.

*Листинг 1 -- Файл «MotionDetector.cpp»*
```c++
#include "MotionDetector.h"
#include <opencv2/optflow/motempl.hpp>

void MotionDetector::detectMovingRegions(const cv::Mat &currentFrame, double timestamp, std::vector<cv::Rect> &targets,
    cv::Mat &mask)
{
    // 1. Сглаживаем текущий кадр (currentFrame) фильтром Гаусса, чтобы избавиться от шумов.
    cv::Mat bluredImage;
    cv::cvtColor(currentFrame, bluredImage, CV_BGR2GRAY);
    cv::GaussianBlur(bluredImage, bluredImage, cv::Size(3, 3), -1);

    // 2. Из сглаженного текущего кадра (m_bluredImage ) вычитаем предыдущий (m_previousImage).
    // Если искомые изображения были в градациях серого, то и значения пикселей в разности (mask)
    // будут изменяться от нуля до 255 (при восьми битной глубине цвета).
    cv::absdiff(bluredImage, m_previousImage, mask);

    // 3. Сравниваем значения полученной разности с некоторым пороговым значением (MOTION_THRESHOLD).
    // Если значение пикселя больше порогового, то этот пиксель принадлежит движущемуся объекту, иначе отбрасываем его.
    // Теперь мы получили бинарное изображение (mask), где ноль означает, что пиксель не движется,
    // отличное от нуля значение – пиксель движется.
    cv::threshold(mask, mask, MOTION_THRESHOLD, 255, cv::THRESH_BINARY);

    // 4. Применяем морфологические операции закрытия и открытия, чтобы избавиться от движущихся регионов малого размера
    // (шумы камеры). Полученное изображение (mask) и есть движущийся контур.
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, cv::Mat());
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, m_openingKernel,
        cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, cv::Scalar(0));

    // 5. Наносим бинарное изображение на так называемое изображение истории движения (motionHistoryImage).
    // На нём нарисованы движущиеся контуры за последние, например, 200 мс (m_motionHistoryDuration).
    // Контуры были получены через постоянные промежутки времени. Интенсивность пикселей контура
    // обратно пропорциональна времени, которое прошло от измерения контура до данного момента.
    // Т.е. чем раньше был получен движущийся контур, тем он бледнее изображён на изображение истории движения.
    cv::motempl::updateMotionHistory(mask, m_motionHistoryImage, timestamp, m_motionHistoryDuration);

    // 6. Выделяем регионы (targets) с различными движениями на изображение истории движения.
    cv::motempl::segmentMotion(m_motionHistoryImage, m_segmask, targets, timestamp, m_maxMotionGradient);

    // 7. Отбрасываем все регионы, площадь которых меньше некоторого значения (MIN_CONTOUR_AREA).
    std::vector<cv::Rect>::iterator endIt = targets.end();
    for (std::vector<cv::Rect>::iterator it = targets.begin(); it != endIt; ++it) {
        if (cv::countNonZero(mask(*it)) < MIN_CONTOUR_AREA || (*it).width * (*it).width < MIN_CONTOUR_AREA)
            targets.erase(it);
    }
    bluredImage.copyTo(m_previousImage);
}
```

![](/docs/media/image7.jpeg)

*Рисунок 1 -- Результат работы программы «SmartCamera»*

Таким образом, в результате работы над проектом было разработано
полностью функционирующее приложение, соответствующее техническому заданию.
