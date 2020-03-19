#ifndef FEATURE_MATCHING_H
#define FEATURE_MATCHING_H

#include "calibration.hpp"

class FeatureMatching {
    private:
        Calibration calibration;
        void preprocessImage(cv::Mat image, cv::OutputArray processedImage);
        void filterMatches(const std::vector<cv::KeyPoint> keypoints1, const std::vector<cv::KeyPoint> keypoints2, const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch> &filteredMatches, cv::Size imageSize);
/* void drawEpipolarLines(cv::Mat& image_out, */
/*                        cv::Mat& image1, */
/*                        cv::Mat& image2, */
/*                        std::vector<cv::Point2f>& points1, // keypoints 1 */
/*                        std::vector<cv::Point2f>& points2, // keypoints 2 */
/*                        std::vector<cv::Vec3f> lines); // image to compute epipolar lines in */

    public:
        FeatureMatching(Calibration cameraCalibration);
        void findMatches(cv::Mat img1, cv::Mat img2, std::vector<cv::Point2f> &imagePoints1, std::vector<cv::Point2f> &imagePoints2);
};

#endif
