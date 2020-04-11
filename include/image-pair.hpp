//
// Created by Sebastian Schmitt on 08.04.2020.
//

#ifndef YAPGT_IMAGE_PAIR_H
#define YAPGT_IMAGE_PAIR_H

#include "image.hpp"

#include <opencv2/core.hpp>
#include <map>
#include <tuple>
#include <vector>

namespace Scene {
    class ImagePair {
        // friend ImagePairSequence;
    private:
        Image leftImage;
        Image rightImage;
        ImagePair* prevPair;
        ImagePair* nextPair;
        
        // std::vector<std::tuple<cv::Point2f, cv::Point2f>> matches;
        std::vector<size_t> matchedKeypointsLeft;
        std::vector<size_t> matchedKeypointsRight;
        std::map<size_t, size_t> rightKeypointsToMatch;

        cv::Mat1d projection;
        cv::Mat1d tranform;
        cv::Mat1f worldPoints;
        std::map<size_t, size_t> matchIdxToWorldPoint;



    public:
        ImagePair(Image leftImage, Image rightImage, std::vector<size_t> matchedKeypointsLeft, std::vector<size_t> matchedKeypointsRight);
    
        std::vector<cv::Point2f> getLeftPoints();
        std::vector<cv::Point2f> getRightPoints();

        std::map<size_t, cv::Point3f> getMatchingWorldPoints(std::vector<size_t> reconstructedMatchIndixes);
        std::map<size_t, cv::Point3f> getWorldPointsFromRightKeypoints(std::vector<size_t> rightKeypointIndixes);

        cv::Mat getLeftImage();
        cv::Mat getRightImage();
        cv::Mat& getProjection();
        cv::Mat& getTransform();
        cv::Mat& getWorldPoints();

        void setReconstruction(cv::Mat projection, cv::Mat transform, cv::Mat worldPoints, std::vector<uchar> reconstructionMask);
    };

    class ImagePairSequence {
        ImagePair* head;
        ImagePair* tail;

    public:
        void append(Image image); 
        const ImagePair& first();
        const ImagePair& last();
        const ImagePair& at(size_t index);
    };


}

#endif //YAPGT_IMAGE_PAIR_H
