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

	std::cout << transformation << std::endl;
	std::cout << rotation << std::endl;
	std::cout << translation << std::endl;
}

void SceneReconstructor::reconstructScenes(Iterator<Scene::ImagePair>* pairSequence) {

	while (pairSequence->hasNext()) {
		Scene::ImagePair currentScene = pairSequence->next();

		std::vector<cv::Point2f> leftMatches = currentScene.getLeftMatches();
		std::vector<cv::Point2f> rightMatches = currentScene.getRightMatches();


		// compute the essential matrix
		// TODO: play around with parameters: https://docs.opencv.org/4.1.1/d9/d0c/group__calib3d.html#ga13f7e34de8fa516a686a56af1196247f
		std::vector<uchar> keypointMatchMask;
		
		cv::Mat essentialMatrix = cv::findEssentialMat(leftMatches, rightMatches, this->calibration.getCameraMatrix(), cv::RANSAC, 0.99, 1.0, keypointMatchMask);

		// compute R and t from the essential matrix
		// using https://docs.opencv.org/4.1.1/d9/d0c/group__calib3d.html#ga13f7e34de8fa516a686a56af1196247f
		cv::Mat localRotation, localTranslation;
		int numberOfInliers = cv::recoverPose(essentialMatrix, leftMatches, rightMatches, calibration.getCameraMatrix(), localRotation, localTranslation, keypointMatchMask);

		// cv::Mat previousTransform = currentScene.getPreviousTransform();
		// cv::Mat localTransform = combineToTransformation(localRotation, localTranslation);
		// cv::Mat globalTransform = previousTransform * localTransform;
		 cv::Mat globalTransform = currentScene.getPreviousTransform() * combineToTransformation(localRotation,  localTranslation);

		// compute the transformation for tranforming the current scene into the the coordinate system of the first scene
		cv::Mat globalRotation, globalTranslation;
		getFromTransformation(globalTransform, globalRotation, globalTranslation);

		// TODO: check output and maybe convert the input types to float 
		// triangulate and get the world points
		cv::Mat worldPoints;
		cv::triangulatePoints(currentScene.getPreviousProjection(), getProjection(globalRotation, globalTranslation), leftMatches, rightMatches, worldPoints);

		if (!currentScene.isFirstImagePair()) {
			assert(worldPoints.cols == keypointMatchMask.size());

			std::vector<size_t> usedKeypointIndexesForReconstruction;
			
			for (int i = 0; i < worldPoints.cols; i++) {
				float divisor = worldPoints.at<double>(3, i);

				worldPoints.at<double>(0, i) /= divisor;
				worldPoints.at<double>(1, i) /= divisor;
				worldPoints.at<double>(2, i) /= divisor;
				worldPoints.at<double>(3, i) /= divisor;

				if (keypointMatchMask.at(i) && divisor < 0)
					std::cout << "Found negative z in with inlier mask" << std::endl;
				if (keypointMatchMask.at(i) == 0 && divisor > 0)
					std::cout << "Found outlier with posivtive z." << std::endl;
			}
		}





	}
}