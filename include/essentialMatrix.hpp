#ifndef ESSENTIAL_MATRIX_H
#define ESSENTIAL_MATRIX_H

#include "calibration.hpp"
#include "opencv2/core.hpp"

class EssentialMatrix {

    private:
        Calibration cameraCalibration;
        int pointsInfrontCamera(const cv::Mat1f& inliners1,
                                const cv::Mat1f& inliners2,
                                const cv::Mat1d& cameraRotation,
                                const cv::Mat1d& cameraTranslation,
                                const cv::Mat1d& extrincts,
                                const cv::Mat1d& projectionMatrix,
                                cv::Mat1f& worldPoints);

        void decomposeEssentialMatrix(const cv::Mat1d& essentialMatrix,
                                      const cv::Mat1f& inliers1,
                                      const cv::Mat1f& inliers2,
                                      cv::Mat1d& extrincts,
                                      cv::Mat1d& projectionMatrix,
                                      cv::Mat1f& worldPoints);

        template<class T> 
        std::vector<T> applyMask(const std::vector<T>& InputArray,
                                 const std::vector<uchar>& mask);

        cv::Mat1f homogenizePoints(const std::vector<cv::Point2f>& imagePoint);
                                      
        
        /* double calculateDepth(const cv::Mat1d& rotationMatrix, */
        /*                       const cv::Mat1d& translationVector, */
        /*                       const cv::Mat1f& imagePoint2, */
        /*                       const int pointIndex); */

    public:
        EssentialMatrix(Calibration cameraCalibration);

        void determineEssentialMatrix(const std::vector<cv::Point2f> &pointsCamera1,
                                      const std::vector<cv::Point2f> &pointsCamera2,
                                      cv::Mat1d& extrincts,
                                      cv::Mat1d& projectionMatrix,
                                      cv::Mat1f& worldPoints);

};

#endif
