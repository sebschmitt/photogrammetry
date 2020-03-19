#ifndef ESSENTIAL_MATRIX_H
#define ESSENTIAL_MATRIX_H

#include "calibration.hpp"
#include "opencv2/core.hpp"

class EssentialMatrix {

    private:
        Calibration cameraCalibration;
        int pointsInfrontCamera(const cv::Mat inliners1,
                                 const cv::Mat inliners2,
                                 const cv::Mat &cameraRotation,
                                 const cv::Mat &cameraTranslation);

        void decomposeEssentialMatrix(const cv::Mat &essentialMatrix,
                                      const cv::Mat &inliers1,
                                      const cv::Mat &inliers2,
                                      cv::Mat &rotation,
                                      cv::Mat &translation);

        template<class T> 
        std::vector<T> applyMask(const std::vector<T>& InputArray,
                            const std::vector<uchar>& mask);

        cv::Mat normalizePoints(const std::vector<cv::Point2f>& imagePoint);
                                      
        
        double calculateDepth(const cv::Mat &rotationRow1,
                              const cv::Mat &rotationRow2,
                              const cv::Mat &translation,
                              const double respectivePoint,
                              const cv::Mat &imagePoint1);

    public:
        EssentialMatrix(Calibration cameraCalibration);
        void determineEssentialMatrix(const std::vector<cv::Point2f> &pointsCamera1,
                                      const std::vector<cv::Point2f> &pointsCamera2,
                                      cv::Mat &cameraRotation,
                                      cv::Mat &cameraTranslation);

};

#endif
