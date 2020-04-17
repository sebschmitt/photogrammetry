#include "reconstruction.hpp"

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

#include "iostream"


SceneReconstructor::SceneReconstructor(Calibration calibration) {
	this->calibration = calibration;
}

cv::Mat SceneReconstructor::getProjection(const cv::Mat &globalRotation, const cv::Mat &globalTranslation) {
	cv::Mat projection = cv::Mat(3, 4, CV_64FC1);

	// globalRotation.copyTo(projection(cv::Range(0, 3), cv::Range(0, 3)));
	// globalTranslation.copyTo(projection(cv::Range(0, 3), cv::Range(3, 4)));
	cv::hconcat(globalRotation.t(), -globalRotation.t() * globalTranslation, projection);
	// cv::hconcat(globalRotation,  globalTranslation, projection);

	return calibration.getCameraMatrix() * projection;
}

// create a 4x4 transform matrix using the given rotation and translation.
// the resulting matrix has the form; [ R; 0 | t; 1]
cv::Mat SceneReconstructor::combineToTransformation(const cv::Mat &rotation, const cv::Mat &translation) {
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
		// cv::Mat previousTransform = currentScene.getPreviousTransform();
		// cv::Mat localTransform = combineToTransformation(localRotation, localTranslation);
		// cv::Mat globalTransform = previousTransform * localTransform;
		 globalTransformation = prevTransform * combineToTransformation(localRotation,  localTranslation);

		// compute the transformation for tranforming the current scene into the the coordinate system of the first scene
		cv::Mat globalRotation, globalTranslation;
		getFromTransformation(globalTransformation, globalRotation, globalTranslation);

		projection = getProjection(globalRotation, globalTranslation);
}

void SceneReconstructor::combineMask(const std::vector<uchar>& input, std::vector<uchar> &output) {
	assert(input.size() == output.size());
	for (int i = 0; i < input.size(); i++) {
		if (!input.at(i))
			output.at(i) = 0;
	}
}

void SceneReconstructor::reconstructScenes(Iterator<Scene::ImagePair>* pairSequence) {

	while (pairSequence->hasNext()) {
		Scene::ImagePair* currentScene = pairSequence->next();

		std::vector<cv::Point2f> leftMatches = currentScene->getLeftMatches();
		std::vector<cv::Point2f> rightMatches = currentScene->getRightMatches();


		// compute the essential matrix
		// TODO: play around with parameters: https://docs.opencv.org/4.1.1/d9/d0c/group__calib3d.html#ga13f7e34de8fa516a686a56af1196247f
		std::vector<uchar> matchMask;

		cv::Mat essentialMatrix = cv::findEssentialMat(leftMatches, rightMatches, this->calibration.getCameraMatrix(), cv::RANSAC, 0.999, 3, matchMask);

		// compute R and t from the essential matrix
		// using https://docs.opencv.org/4.1.1/d9/d0c/group__calib3d.html#ga13f7e34de8fa516a686a56af1196247f
		std::vector<uchar> poseMask;
		cv::Mat localRotation, localTranslation;
		int numberOfInliers = cv::recoverPose(essentialMatrix, leftMatches, rightMatches, calibration.getCameraMatrix(), localRotation, localTranslation, matchMask);

		// combineMask(poseMask, matchMask);

		cv::Mat projection;
		cv::Mat globalTransform;

		computPoseAndProjection(localRotation, localTranslation, currentScene->getPreviousTransform(), globalTransform, projection);

		// TODO: check output and maybe convert the input types to float 
		// triangulate and get the world points
		cv::Mat worldPoints;
		cv::Mat previousProjection;
		cv::Mat currentProjection;

		currentScene->getPreviousProjection().convertTo(previousProjection, CV_32FC1);
		projection.convertTo(currentProjection, CV_32FC1);
		
		cv::triangulatePoints(previousProjection, currentProjection, leftMatches, rightMatches, worldPoints);

		// normalize points
		std::vector<size_t> usedKeypointIndexesForReconstruction;
		for (int i = 0; i < worldPoints.cols; i++) {

			// only continue with world points, which are inliers and have a positive z value
			if (matchMask.at(i) && ((worldPoints.at<float>(2, i) > 0 && worldPoints.at<float>(3, i) > 0) || 
				                     worldPoints.at<float>(2, i) < 0 && worldPoints.at<float>(3, i) < 0)) {
				float divisor = worldPoints.at<float>(3, i);

				worldPoints.at<float>(0, i) /= divisor;
				worldPoints.at<float>(1, i) /= divisor;
				worldPoints.at<float>(2, i) /= divisor;
				worldPoints.at<float>(3, i) /= divisor;

				// std::cout << worldPoints.col(i) << std::endl;

				usedKeypointIndexesForReconstruction.push_back(i);
			} else {
				// ensure that the keypoint is set to 0 for outliers
				matchMask.at(i) = 0;
			}
		}
		std::cout << "triangulated " << usedKeypointIndexesForReconstruction.size() << " Points" << std::endl;

		if (currentScene->isFirstImagePair())
			currentScene->setReconstruction(projection, globalTransform, worldPoints, matchMask);
		else {
			std::map<size_t, cv::Point3f> matchingWorldPoints = currentScene->getMatchingWorldPoints(usedKeypointIndexesForReconstruction);
			std::vector<std::tuple<double, double>> distances;


			std::cout << "Found " << matchingWorldPoints.size() << " overlapping worldpoints." << std::endl;

			if (matchingWorldPoints.size() < 2) {
				std::cerr << "Not enought points were found for scaling. Stopping reconstruction." << std::endl;
				break;
			}

			auto matchWorldPointPair = matchingWorldPoints.begin();
			while(matchWorldPointPair != matchingWorldPoints.end()) {

				cv::Point3f unscaledA = toPoint(worldPoints.col(matchWorldPointPair->first));
				cv::Point3f scaledA = matchWorldPointPair->second;

				matchWorldPointPair++;
				if (matchWorldPointPair == matchingWorldPoints.end())
					break;

				cv::Point3f unscaledB = toPoint(worldPoints.col(matchWorldPointPair->first));
				cv::Point3f scaledB = matchWorldPointPair->second;

				distances.push_back(std::make_tuple(getDistance(unscaledA, unscaledB), getDistance(scaledA, scaledB)));
			}

			double scale = 0;
			std::vector<double> scales;
			for (int i = 0; i < distances.size(); i++) {
				if (std::get<1>(distances.at(i)) <= 0 || std::get<0>(distances.at(i)) <= 0) {
					continue;
				}
				scales.push_back(std::get<1>(distances.at(i)) / std::get<0>(distances.at(i)));

				scale += std::get<1>(distances.at(i)) / std::get<0>(distances.at(i));
			}

			scale /= distances.size();

			worldPoints.release();

			computPoseAndProjection(localRotation, localTranslation * scale, currentScene->getPreviousTransform(), globalTransform, projection);

			projection.convertTo(currentProjection, CV_32FC1);
			cv::triangulatePoints(previousProjection, currentProjection, leftMatches, rightMatches, worldPoints);

			//cv::Mat filteredWorldPoints(3, usedKeypointIndexesForReconstruction.size(), CV_32FC1);
			//for (int i = 0; i < scaledWorldPoints.cols; i++) {
			//	filteredWorldPoints.at<float>(0, i) /= scaledWorldPoints.at<float>(3, i);
			//	filteredWorldPoints.at<float>(1, i) /= scaledWorldPoints.at<float>(3, i);
			//	filteredWorldPoints.at<float>(2, i) /= scaledWorldPoints.at<float>(3, i);
			//}

			currentScene->setReconstruction(projection, globalTransform, worldPoints, matchMask);
		}
	}
}