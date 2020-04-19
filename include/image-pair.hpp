//
// Created by Sebastian Schmitt on 08.04.2020.
//

#ifndef YAPGT_IMAGE_PAIR_H
#define YAPGT_IMAGE_PAIR_H

#include "image.hpp"
#include "calibration.hpp"
#include "colors.hpp"

#include <opencv2/core.hpp>

#include <map>
#include <string>
#include <tuple>
#include <vector>

namespace Scene {
    class ImagePair {
        friend class SceneSequence;
        friend class ImageSequenceIterator;

    private:
        Image leftImage;
        Image rightImage;
        ImagePair* prevPair = nullptr;
        ImagePair* nextPair = nullptr;

        Calibration cameraCalibration;
        
        // std::vector<std::tuple<cv::Point2f, cv::Point2f>> matches;
        std::vector<size_t> matchedKeypointsLeft;
        std::vector<size_t> matchedKeypointsRight;
        std::map<size_t, size_t> rightKeypointsToMatch;

        cv::Mat1d projection;
        cv::Mat1d tranform;
        cv::Mat1f worldPoints = cv::Mat1f(0, 0);
        std::map<size_t, size_t> matchIdxToWorldPoint;


    public:
        ImagePair(Image leftImage, Image rightImage, std::vector<size_t> matchedKeypointsLeft, std::vector<size_t> matchedKeypointsRight);
        ImagePair(Image leftImage, Image rightImage, std::vector<size_t> matchedKeypointsLeft, std::vector<size_t> matchedKeypointsRight, Calibration cameracalibration);
    
        std::vector<cv::Point2f> &getLeftPoints();
        std::vector<cv::Point2f> &getRightPoints();

        std::vector<cv::Point2f> getLeftMatches();
        std::vector<cv::Point2f> getRightMatches();

        std::map<size_t, cv::Point3f> ImagePair::getMatchingWorldPoints(const std::vector<size_t> &reconstructedMatchIndixes);
        std::map<size_t, cv::Point3f> getWorldPointsFromRightKeypoints(const std::map<size_t, size_t> &rightKeypointIndexToNextMatchIndex);

        std::string getLeftImageName();
        std::string getRightImageName();
        cv::Mat& getLeftImage();
        cv::Mat& getRightImage();
        cv::Mat getProjection();
        cv::Mat getTransform();
        cv::Mat& getWorldPoints();
        std::vector<Colors::Color> getColors();

        const cv::Mat getPreviousTransform();
        cv::Mat getPreviousProjection();

        bool isFirstImagePair();

        void setReconstruction(cv::Mat projection, cv::Mat transform, cv::Mat worldPoints, std::vector<uchar> reconstructionMask);

    };


}

#endif //YAPGT_IMAGE_PAIR_H
