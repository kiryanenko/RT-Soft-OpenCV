#ifndef SMARTCAMERA_MOTIONDETECTOR_H
#define SMARTCAMERA_MOTIONDETECTOR_H

#include <opencv2/opencv.hpp>

class MotionDetector {
public:
    MotionDetector( const cv::Size& imageSize, double cycleTime );

    void detectMovingRegions( const cv::Mat& currentFrame, double timestamp,
                              std::vector<cv::Rect>& targets , cv::Mat& mask );

    double getMotionHistoryDuration() const {
        return m_motionHistoryDuration;
    }

private:
    MotionDetector& operator=( const MotionDetector& );
    MotionDetector( const MotionDetector& );

    cv::Mat m_previousImage;
    cv::Mat m_motionHistoryImage;
    cv::Mat m_openingKernel;
    cv::Mat m_segmask;

    double m_maxMotionGradient;

    double m_motionHistoryDuration;

    static const double MOTION_THRESHOLD;
    static const double MIN_CONTOUR_AREA;
};


#endif //SMARTCAMERA_MOTIONDETECTOR_H
