#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "calibration.hpp"

class FeatureMatching {
    private:
        Calibration calibration;
        void preprocessImage(const cv::Mat& image,
                             cv::Mat& processedImage);

        void filterMatches(const std::vector<cv::KeyPoint>& keypoints1,
                           const std::vector<cv::KeyPoint>& keypoints2,
                           const std::vector<cv::DMatch>& matches,
                           std::vector<cv::DMatch> &filteredMatches,
                           cv::Size imageSize);

    public:
        FeatureMatching(Calibration cameraCalibration);

        void findMatches(const cv::Mat& img1,
                         const cv::Mat& img2,
                         std::vector<cv::Point2f> &imagePoints1,
                         std::vector<cv::Point2f> &imagePoints2);
};

#endif
