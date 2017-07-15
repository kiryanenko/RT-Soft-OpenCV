#include "MotionDetector.h"

#include <opencv2/optflow/motempl.hpp>

const double MotionDetector::MOTION_THRESHOLD = 10;
const double MotionDetector::MIN_CONTOUR_AREA = 150;

MotionDetector::MotionDetector(const cv::Size &imageSize, double cycleTime) :
        m_previousImage( imageSize, CV_8UC1 ),
        m_motionHistoryImage( imageSize, CV_32FC1 ),
        m_openingKernel( 5, 5, CV_8UC1 ),
        m_segmask( imageSize, CV_32FC1 ),
        m_maxMotionGradient( 1.5 * cycleTime ),
        m_motionHistoryDuration( 7 * cycleTime )
{
}

void MotionDetector::detectMovingRegions(const cv::Mat &currentFrame, double timestamp, std::vector<cv::Rect> &targets,
                                         cv::Mat &mask)
{
    // 1. Сглаживаем текущий кадр (currentFrame) фильтром Гаусса, чтобы избавиться от шумов.
    cv::Mat bluredImage;
    cv::cvtColor( currentFrame, bluredImage, CV_BGR2GRAY );
    cv::GaussianBlur( bluredImage, bluredImage, cv::Size( 3, 3 ), -1 );

    // 2. Из сглаженного текущего кадра (m_bluredImage ) вычитаем предыдущий (m_previousImage).
    // Если искомые изображения были в градациях серого, то и значения пикселей в разности (mask)
    // будут изменяться от нуля до 255 (при восьми битной глубине цвета).
    cv::absdiff( bluredImage, m_previousImage, mask );
    // 3. Сравниваем значения полученной разности с некоторым пороговым значением (MOTION_THRESHOLD).
    // Если значение пикселя больше порогового, то этот пиксель принадлежит движущемуся объекту, иначе отбрасываем его.
    // Теперь мы получили бинарное изображение (mask), где ноль означает, что пиксель не движется,
    // отличное от нуля значение – пиксель движется.
    cv::threshold( mask, mask,  MOTION_THRESHOLD, 255, cv::THRESH_BINARY );
    // 4. Применяем морфологические операции закрытия и открытия, чтобы избавиться от движущихся регионов малого размера
    // (шумы камеры). Полученное изображение (mask) и есть движущийся контур.
    cv::morphologyEx( mask, mask, cv::MORPH_CLOSE, cv::Mat() );
    cv::morphologyEx( mask, mask, cv::MORPH_OPEN, m_openingKernel,
                      cv::Point( -1, -1 ), 1, cv::BORDER_CONSTANT, cv::Scalar( 0 ) );
    // 5. Наносим бинарное изображение на так называемое изображение истории движения (motionHistoryImage).
    // На нём нарисованы движущиеся контуры за последние, например, 200 мс (m_motionHistoryDuration).
    // Контуры были получены через постоянные промежутки времени. Интенсивность пикселей контура
    // обратно пропорциональна времени, которое прошло от измерения контура до данного момента.
    // Т.е. чем раньше был получен движущийся контур, тем он бледнее изображён на изображение истории движения.
    cv::motempl::updateMotionHistory( mask, m_motionHistoryImage, timestamp, m_motionHistoryDuration );
    // 6. Выделяем регионы (targets) с различными движениями на изображение истории движения.
    cv::motempl::segmentMotion( m_motionHistoryImage, m_segmask, targets, timestamp, m_maxMotionGradient );

    // 7. Отбрасываем все регионы, площадь которых меньше некоторого значения (MIN_CONTOUR_AREA).
    std::vector<cv::Rect>::iterator endIt = targets.end();
    for( std::vector<cv::Rect>::iterator it = targets.begin( ); it != endIt; ++it ) {
        if( cv::countNonZero( mask( * it ) ) < MIN_CONTOUR_AREA )
            targets.erase( it );
    }

    bluredImage.copyTo( m_previousImage );
}
