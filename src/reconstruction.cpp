#include "reconstruction.hpp"

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

#include "iostream"


SceneReconstructor::SceneReconstructor(Calibration calibration) {
	this->calibration = calibration;
}

cv::Mat SceneReconstructor::getProjection(const cv::Mat &globalRotation, const cv::Mat &globalTranslation) {
	cv::Mat projection = cv::Mat(3, 4, CV_64FC1);
	// copy rotation and translation into the matrix, multiply with K
	cv::hconcat(globalRotation,  globalTranslation, projection);
	return calibration.getCameraMatrix() * projection;
}

cv::Mat SceneReconstructor::combineToTransformation(const cv::Mat &rotation, const cv::Mat &translation) {
	// create a 4x4 transform matrix using the given rotation and translation.
	// the resulting matrix has the form; [ R; 0 | t; 1]
	cv::Mat t = cv::Mat::eye(4, 4, CV_64FC1);

	rotation.copyTo(t(cv::Range(0, 3), cv::Range(0, 3)));
	translation.copyTo(t(cv::Range(0, 3), cv::Range(3, 4)));

	return t;
}

void SceneReconstructor::getFromTransformation(const cv::Mat& transformation, cv::Mat& rotation, cv::Mat& translation) {
	assert(transformation.cols == 4 && transformation.rows == 4);

	rotation = cv::Mat1d(3, 3);
	translation = cv::Mat1d(1, 3);
	
	transformation(cv::Range(0, 3), cv::Range(0, 3)).copyTo(rotation);
	transformation(cv::Range(0, 3), cv::Range(3, 4)).copyTo(translation);
}

cv::Point3f SceneReconstructor::toPoint(cv::Mat& column) {
	assert(column.cols == 1);
	assert(column.type() == CV_32FC1);
		
	return cv::Point3f(column.at<float>(0, 0), column.at<float>(1, 0), column.at<float>(2, 0));
}

double SceneReconstructor::getDistance(cv::Point3f& pointA, cv::Point3f& pointB) {
	cv::Point3f line = pointB - pointA;
	return std::sqrt(line.x * line.x + line.y * line.y + line.z * line.z);
}

void SceneReconstructor::computPoseAndProjection(const cv::Mat& localRotation, const cv::Mat& localTranslation, const cv::Mat& prevTransform, cv::Mat& globalTransformation, cv::Mat& projection) {
		globalTransformation = prevTransform * combineToTransformation(localRotation,  localTranslation);

		// compute the transformation for tranforming the current scene into the the coordinate system of the first scene
		cv::Mat globalRotation, globalTranslation;
		getFromTransformation(globalTransformation, globalRotation, globalTranslation);

		projection = getProjection(globalRotation, globalTranslation);
}

void SceneReconstructor::reconstructScenes(Iterator<Scene::ImagePair>* pairSequence) {
	std::cout << "Reconstructing scenes..." << std::endl;

	int sceneCounter = 0;
	while (pairSequence->hasNext()) {
		sceneCounter++;
		std::cout << "image pair " << sceneCounter << ": ";
		Scene::ImagePair* currentScene = pairSequence->next();

		std::vector<cv::Point2f> leftMatches = currentScene->getLeftMatches();
		std::vector<cv::Point2f> rightMatches = currentScene->getRightMatches();


		// compute the essential matrix
		std::vector<uchar> matchMask;

		cv::Mat essentialMatrix = cv::findEssentialMat(leftMatches, rightMatches, this->calibration.getCameraMatrix(), cv::RANSAC, 0.999, 1, matchMask);

		// compute R and t from the essential matrix
		std::vector<uchar> poseMask;
		cv::Mat localRotation, localTranslation;
		int numberOfInliers = cv::recoverPose(essentialMatrix, leftMatches, rightMatches, calibration.getCameraMatrix(), localRotation, localTranslation, matchMask);

		cv::Mat projection;
		cv::Mat globalTransform;
		computPoseAndProjection(localRotation, localTranslation, currentScene->getPreviousTransform(), globalTransform, projection);

		cv::Mat worldPoints;
		cv::Mat previousProjection;
		cv::Mat currentProjection;

		// according to the opencv documentation all arguments for triangualtePoints must be of type float
		// therefore convert them here 
		currentScene->getPreviousProjection().convertTo(previousProjection, CV_32FC1);
		projection.convertTo(currentProjection, CV_32FC1);
		
		// triangulate and get the world points
		cv::triangulatePoints(previousProjection, currentProjection, leftMatches, rightMatches, worldPoints);

		// scale if necessary and set reconstruction result for the image pair.
		if (currentScene->isFirstImagePair()) {
			std::cout << "No scaling necessary." << std::endl;
			currentScene->setReconstruction(projection, globalTransform, worldPoints, matchMask);
		} else {
			// normalize points
			std::vector<size_t> usedKeypointIndexesForReconstruction;
			for (int i = 0; i < worldPoints.cols; i++) {
				// only continue with world points, which are inliers and have a positive z value
				// mark points with negative z value as outliers in the mask 
				if (matchMask.at(i)) {
					if ((worldPoints.at<float>(2, i) > 0 && worldPoints.at<float>(3, i) > 0) ||
						 worldPoints.at<float>(2, i) < 0 && worldPoints.at<float>(3, i) < 0) {

						float divisor = worldPoints.at<float>(3, i);

						worldPoints.at<float>(0, i) /= divisor;
						worldPoints.at<float>(1, i) /= divisor;
						worldPoints.at<float>(2, i) /= divisor;
						worldPoints.at<float>(3, i) /= divisor;

						// store index of a good match in order to get reconstructed world points from the previous image pair
						usedKeypointIndexesForReconstruction.push_back(i);
					} else {
						matchMask.at(i) = 0;
					}
				}
			}

			std::map<size_t, cv::Point3f> matchingWorldPoints = currentScene->getMatchingWorldPoints(usedKeypointIndexesForReconstruction);
			std::vector<std::tuple<double, double>> distances;

			std::cout << "Found " << matchingWorldPoints.size() << " overlapping worldpoints. ";

			if (matchingWorldPoints.size() < 2) {
				std::cerr << std::endl << "Not enought referenze points were found to scale the current iamge pair. Stopping reconstruction early." << std::endl;
				break;
			}

			// create pairs of points and mesure the distances

			double sumofScales = 0;
			int distancesCount = 0;
			for (auto outerMatchPointPair = matchingWorldPoints.begin(); outerMatchPointPair != matchingWorldPoints.end()--; outerMatchPointPair++) {
				cv::Point3f unscaledA = toPoint(worldPoints.col(outerMatchPointPair->first));
				cv::Point3f scaledA = outerMatchPointPair->second;

				for (auto innerMatchPointPair = std::next(outerMatchPointPair); innerMatchPointPair != matchingWorldPoints.end(); innerMatchPointPair++) {
					cv::Point3f unscaledB = toPoint(worldPoints.col(innerMatchPointPair->first));
					cv::Point3f scaledB = innerMatchPointPair->second;

					double scaledDistance = getDistance(scaledA, scaledB);
					double unscaledDistance = getDistance(unscaledA, unscaledB);
					
					// avoid division through 0
					if (unscaledDistance == 0)
						continue;

					double currentScale = scaledDistance / unscaledDistance;
					// avoid double overflow 
					if (currentScale > std::numeric_limits<double>::max() - sumofScales)
						break;

					sumofScales += currentScale;
					distancesCount++;
				}
			}
			double scale = sumofScales / distancesCount;

			std::cout << "Using scale " << scale << std::endl;

			worldPoints.release();

			// recalculate translation and projection with the right scale
			computPoseAndProjection(localRotation, localTranslation * scale, currentScene->getPreviousTransform(), globalTransform, projection);

			projection.convertTo(currentProjection, CV_32FC1);

			cv::triangulatePoints(previousProjection, currentProjection, leftMatches, rightMatches, worldPoints);

			currentScene->setReconstruction(projection, globalTransform, worldPoints, matchMask);
		}
	}
	std::cout << "Done." << std::endl << std::endl;
}