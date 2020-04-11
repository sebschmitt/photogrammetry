//
// Created by Sebastian Schmitt on 08.04.2020.
//

#include "image-pair.hpp"

#include <utility>

namespace Scene {
        /*
            matchedKeypointsLeft: vector of indixes for the keypoints in the left image, that have been machted.
            matchedKeypointsRight:vector of indixes for the keypoints in the right image, that have been machted.

            matchedKeypointsLeft[i] is matched to matchedKeypointsRight[i]

        */
        ImagePair::ImagePair(Image leftImage, Image rightImage, std::vector<size_t> matchedKeypointsLeft, std::vector<size_t> matchedKeypointsRight) {
        this->leftImage = std::move(leftImage);
        this->rightImage = std::move(rightImage);
        this->prevPair = nullptr;
        this->nextPair = nullptr;

        this->matchedKeypointsLeft = std::move(matchedKeypointsLeft);
        this->matchedKeypointsRight = std::move(matchedKeypointsRight);

        // create mapping of keypoint to the respective match only for the right image
        for (size_t index = 0; index < matchedKeypointsRight.size(); index++) {
            rightKeypointsToMatch[matchedKeypointsRight[index]] = index;
        }
    }

    cv::Mat ImagePair::getLeftImage() {
        return this->leftImage.image;
    }

    cv::Mat ImagePair::getRightImage() {
        return this->rightImage.image;
    }

    std::vector<cv::Point2f> ImagePair::getLeftPoints() {
        return this->leftImage.imagePoints;
    }

    std::vector<cv::Point2f> ImagePair::getRightPoints() {
        return this->rightImage.imagePoints;
    }

    cv::Mat& ImagePair::getProjection() {
        return this->projection;
    }

    cv::Mat& ImagePair::getWorldPoints() {
        return this->worldPoints;
    }

    cv::Mat& ImagePair::getTransform() {
        return this->tranform;
    }

    // reconstructionMask is a list of 0 and 1, indicating which elements from the Match vectors have been used
    void ImagePair::setReconstruction(cv::Mat projection, cv::Mat transform, cv::Mat worldPoints, std::vector<uchar> reconstructionMask) {
        // TODO: check if projection sizes are correct
        assert(projection.cols == 4 && projection.rows == 4);
        assert(transform.cols == 4 && transform.rows == 4);
        assert(worldPoints.rows == 4);

        assert(projection.type() == CV_64FC1);
        assert(transform.type() == CV_64FC1);
        assert(worldPoints.type() == CV_32FC1);

        assert(reconstructionMask.size() == matchedKeypointsLeft.size());

        this->projection = std::move(projection);
        this->projection = std::move(tranform);


        // normalize found world points and copy them
        this->worldPoints = cv::Mat1f(3, worldPoints.cols);
        for (size_t i = 0; i < worldPoints.cols; i++) {
            float divisor = worldPoints.at<float>(3, i);

            this->worldPoints.at<float>(0, i) = worldPoints.at<float>(0, i) / divisor;
            this->worldPoints.at<float>(1, i) = worldPoints.at<float>(1, i) / divisor;
            this->worldPoints.at<float>(2, i) = worldPoints.at<float>(2, i) / divisor;
        }


        // create mapping from keypointIndex to worldPoint
        size_t worldPointIndex = 0;
        for (size_t matchIndex = 0; matchIndex < reconstructionMask.size(); matchIndex++) {
            if (reconstructionMask.at(matchIndex)) {
                assert(worldPointIndex < worldPoints.cols);

                matchIdxToWorldPoint[matchIndex] = worldPointIndex;
                worldPointIndex++;
            }
        }
    }


	std::map<size_t, cv::Point3f> ImagePair::getMatchingWorldPoints(std::vector<size_t> reconstructedMatchIndixes) {

		// setReconstruction must be called before this function is called
		assert(!matchIdxToWorldPoint.empty());

		std::vector<size_t> leftKeypointIndixes;
		for (auto& mapping : matchIdxToWorldPoint) {
			leftKeypointIndixes.push_back(matchedKeypointsLeft.at(mapping.first));
		}

		return prevPair->getWorldPointsFromRightKeypoints(leftKeypointIndixes);
	}


	std::map<size_t, cv::Point3f> ImagePair::getWorldPointsFromRightKeypoints(std::vector<size_t> rightKeypointIndixes) {
		std::map<size_t, cv::Point3f> foundWorldPoints;

		for (size_t& keypointIndex : rightKeypointIndixes) {

			size_t matchIndex = this->rightKeypointsToMatch[keypointIndex];
			size_t worldPointIndex = this->matchIdxToWorldPoint[matchIndex];

			cv::Point3f point = cv::Point3f(worldPoints.at<float>(0, worldPointIndex), worldPoints.at<float>(1, worldPointIndex), worldPoints.at<float>(2, worldPointIndex));

            foundWorldPoints[matchIndex] = point;
		}
		return foundWorldPoints;
	}
}

