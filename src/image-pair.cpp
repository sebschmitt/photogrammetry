#include "image-pair.hpp"
#include "colors.hpp"

#include <numeric>
#include <iostream>
#include <exception>
#include <utility>

namespace Scene {
	/*
		matchedKeypointsLeft: vector of indexes for the keypoints in the left image, that have been matched.
		matchedKeypointsRight:vector of indexes for the keypoints in the right image, that have been matched.

		matchedKeypointsLeft[i] is matched to matchedKeypointsRight[i]

	*/
	ImagePair::ImagePair(Image leftImage, Image rightImage, std::vector<size_t> matchedKeypointsLeft, std::vector<size_t> matchedKeypointsRight) {
		this->leftImage = leftImage;
		this->rightImage = rightImage;
		this->prevPair = nullptr;
		this->nextPair = nullptr;

		this->matchedKeypointsLeft = matchedKeypointsLeft;
		this->matchedKeypointsRight = matchedKeypointsRight;

		// create mapping of keypoint to the respective match only for the right image
		for (size_t index = 0; index < matchedKeypointsRight.size(); index++) {
			if (rightKeypointsToMatch.count(matchedKeypointsRight[index]) > 0) {
				std::cout << "keypoint has multiple matches" << std::endl;
			}
			rightKeypointsToMatch[matchedKeypointsRight[index]] = index;
		}
	}

	ImagePair::ImagePair(Image leftImage, Image rightImage, std::vector<size_t> matchedKeypointsLeft, std::vector<size_t> matchedKeypointsRight, Calibration cameraCalibration) {
		this->leftImage = leftImage;
		this->rightImage = rightImage;
		this->prevPair = nullptr;
		this->nextPair = nullptr;

		this->matchedKeypointsLeft = matchedKeypointsLeft;
		this->matchedKeypointsRight = matchedKeypointsRight;

		// create mapping of keypoint to the respective match only for the right image
		for (size_t index = 0; index < matchedKeypointsRight.size(); index++) {
			if (rightKeypointsToMatch.count(matchedKeypointsRight[index]) > 0) {
				std::cout << "keypoint has multiple matches" << std::endl;
			}
			
			rightKeypointsToMatch[matchedKeypointsRight[index]] = index;
		}

		this->cameraCalibration = cameraCalibration;

	}

	cv::Mat &ImagePair::getLeftImage() {
		return this->leftImage.image;
	}

	cv::Mat &ImagePair::getRightImage() {
		return this->rightImage.image;
	}

	std::vector<cv::Point2f> &ImagePair::getLeftPoints() {
		return this->leftImage.imagePoints;
	}

	std::vector<cv::Point2f> &ImagePair::getRightPoints() {
		return this->rightImage.imagePoints;
	}

	cv::Mat ImagePair::getProjection() {
		return this->projection;
	}

	std::vector<cv::Point3f> &ImagePair::getWorldPoints() {
		return this->worldPoints;
	}

	std::vector<Colors::Color> ImagePair::getColors() {

		std::vector<size_t> matchIndexes;
		for (auto it = matchIdxToWorldPoint.begin(); it != matchIdxToWorldPoint.end(); it++) {
			matchIndexes.push_back(it->first);
		}

		std::sort(matchIndexes.begin(), matchIndexes.end());

		std::vector<Colors::Color> colors;
		for (auto& matchIndexWorldPointPair : matchIdxToWorldPoint) {
			size_t matchIndex = matchIndexWorldPointPair.first;
			size_t leftPointIndex = matchedKeypointsLeft.at(matchIndex);
			size_t rightPointIndex = matchedKeypointsRight.at(matchIndex);

			colors.push_back(Colors::ColorPicker::getAverageColor(leftImage.image, rightImage.image, leftImage.imagePoints.at(leftPointIndex), rightImage.imagePoints.at(rightPointIndex), 4));
		}
		return colors;
	}

	const cv::Mat ImagePair::getPreviousTransform() {
		if (prevPair == nullptr) {
			return cv::Mat::eye(cv::Size(4, 4), CV_64FC1);
		}

		return prevPair->getTransform();
	}

	cv::Mat ImagePair::getPreviousProjection() {
		if (prevPair == nullptr) {
			return cameraCalibration.getCameraMatrix() * cv::Mat::eye(3, 4, CV_64FC1);
		}
		return prevPair->getProjection();
	}


	bool ImagePair::isFirstImagePair() {
		return prevPair == nullptr;
	}

	cv::Mat ImagePair::getTransform() {
		return this->tranform;
	}

	// reconstructionMask is a list of 0 and 1, indicating which elements from the Match vectors have been used
	void ImagePair::setReconstruction(cv::Mat projection, cv::Mat transform, cv::Mat worldPoints, std::vector<uchar> reconstructionMask) {
		assert(projection.cols == 4 && projection.rows == 3);
		assert(transform.cols == 4 && transform.rows == 4);
		assert(worldPoints.rows == 4);

		assert(projection.type() == CV_64FC1);
		assert(transform.type() == CV_64FC1);
		assert(worldPoints.type() == CV_32FC1);

		assert(reconstructionMask.size() == matchedKeypointsLeft.size());
		assert(reconstructionMask.size() == matchedKeypointsRight.size());

		assert(reconstructionMask.size() == worldPoints.cols);
		this->projection = projection;
		this->tranform = transform;

		// normalize, filter and copy found world points 
		// also create the worldPoint to match mapping
		size_t worldPointIndex = 0;
		for (size_t matchIndex = 0; matchIndex < reconstructionMask.size(); matchIndex++) {
			if (reconstructionMask.at(matchIndex)) {
				if ((worldPoints.at<float>(2, matchIndex) > 0 && worldPoints.at<float>(3, matchIndex) > 0) ||
					(worldPoints.at<float>(2, matchIndex) < 0 && worldPoints.at<float>(3, matchIndex) < 0)) {

					float divisor = worldPoints.at<float>(3, matchIndex);
					this->worldPoints.push_back(cv::Point3f(worldPoints.at<float>(0, matchIndex) / divisor, worldPoints.at<float>(1, matchIndex) / divisor, worldPoints.at<float>(2, matchIndex) / divisor));

					matchIdxToWorldPoint[matchIndex] = worldPointIndex;
					worldPointIndex++;
				}
			}
		}
		std::cout << "Reconstructed " << worldPointIndex << " points." << std::endl;
	}


	std::vector<cv::Point2f> ImagePair::getLeftMatches() {
		std::vector<cv::Point2f> matches;
		for (auto index : matchedKeypointsLeft) {
			matches.push_back(leftImage.imagePoints.at(index));
		}
		return matches;
	}

	std::vector<cv::Point2f> ImagePair::getRightMatches()
	{
		std::vector<cv::Point2f> matches;
		for (auto index : matchedKeypointsRight) {
			matches.push_back(rightImage.imagePoints.at(index));
		}
		return matches;
	}

	std::map<size_t, cv::Point3f> ImagePair::getMatchingWorldPoints(const std::vector<size_t>& reconstructedMatchIndixes) {
		std::map<size_t, size_t> keypointIndexToMatchIndex;

		for (auto& matchIndex : reconstructedMatchIndixes) {
			keypointIndexToMatchIndex.emplace(matchedKeypointsLeft.at(matchIndex), matchIndex);
		}

		return prevPair->getWorldPointsFromRightKeypoints(keypointIndexToMatchIndex);
	}


	std::map<size_t, cv::Point3f> ImagePair::getWorldPointsFromRightKeypoints(const std::map<size_t, size_t>& keypointIndexToNextMatchIndex) {
		std::map<size_t, cv::Point3f> nextMatchIndexToWorldPoint;

		for (auto keypointMatchPair : keypointIndexToNextMatchIndex) {
			// check if the  provided keypoint has has a match
			if (rightKeypointsToMatch.count(keypointMatchPair.first) > 0) {

				size_t matchIndex = this->rightKeypointsToMatch[keypointMatchPair.first];

				// ensure the match has a world point
				if (matchIdxToWorldPoint.count(matchIndex) > 0) {
					size_t worldPointIndex = this->matchIdxToWorldPoint[matchIndex];
					nextMatchIndexToWorldPoint[keypointMatchPair.second] = worldPoints.at(worldPointIndex);
				}
			}
		}
		return nextMatchIndexToWorldPoint;
	}

	std::string ImagePair::getLeftImageName() {
		return leftImage.name;
	}

	std::string ImagePair::getRightImageName() {
		return rightImage.name;
	}

	}

