#include "reconstruction.hpp"

#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"

#include "iostream"


SceneReconstructor::SceneReconstructor(Calibration calbiration) {
	this->calibration = calibration;
}

cv::Mat SceneReconstructor::getProjection(const cv::Mat &globalRotation, const cv::Mat &globalTranslation, int scale) {
	cv::Mat rAndT = cv::Mat(3, 4, CV_64FC1);
	cv::hconcat(globalRotation.t(), -globalRotation.t() * (globalTranslation * scale), rAndT);
	return calibration.getCameraMatrix() * rAndT;
}

// create a 4x4 transform matrix using the given rotation and translation.
// the resulting matrix has the form; [ R; 0 | t; 1]
cv::Mat SceneReconstructor::combineToTransformation(const cv::Mat &rotation, const cv::Mat &translation) {
	cv::Mat1d t = cv::Mat1d(3, 4);

	cv::hconcat(rotation, translation, t);
	t.push_back(std::vector<double>{0, 0, 0, 1});

	assert(t.cols == 4 && t.rows == 4);
	return t;
}

void SceneReconstructor::getFromTransformation(const cv::Mat& transformation, cv::Mat& rotation, cv::Mat& translation) {
	assert(transformation.cols == 4 && transformation.rows == 4);

	rotation = cv::Mat1d(3, 3);
	translation = cv::Mat1d(1, 3);
	
	transformation(cv::Range(0, 3), cv::Range(0, 3)).copyTo(rotation);
	transformation(cv::Range(3, 3), cv::Range(0, 3)).copyTo(translation);
}

void SceneReconstructor::reconstructScenes(Iterator<Scene::ImagePair>* pairSequence) {

	while (pairSequence->hasNext()) {
		Scene::ImagePair currentScene = pairSequence->next();

		std::vector<cv::Point2f> leftMatches = currentScene.getLeftMatches();
		std::vector<cv::Point2f> rightMatches = currentScene.getRightMatches();


		// compute the essential matrix
		// TODO: play around with parameters: https://docs.opencv.org/4.1.1/d9/d0c/group__calib3d.html#ga13f7e34de8fa516a686a56af1196247f
		std::vector<uchar> keypointMatchMask;
		cv::Mat essentialMatrix = cv::Mat(3, 3, CV_64FC1);
		essentialMatrix = cv::findEssentialMat(leftMatches, rightMatches, this->calibration.getCameraMatrix(), cv::RANSAC, 0.999, 2.0, keypointMatchMask);

		std::cout << essentialMatrix.empty() << std::endl;
		std::cout << essentialMatrix.isContinuous() << std::endl;
		std::cout << essentialMatrix << std::endl;

		// compute R and t from the essential matrix
		// using https://docs.opencv.org/4.1.1/d9/d0c/group__calib3d.html#ga13f7e34de8fa516a686a56af1196247f
		cv::Mat localRotation, localTranslation;
		int numberOfInliers = cv::recoverPose(essentialMatrix, leftMatches, rightMatches, calibration.getCameraMatrix(), localRotation, localTranslation, keypointMatchMask);

		std::cout << localRotation.isContinuous() << std::endl;
		std::cout << localTranslation.isContinuous() << std::endl;

		// compute the transformation for tranforming the current scene into the the coordinate system of the first scene
		cv::Mat globalTransform = currentScene.getPreviousTransform() * combineToTransformation(localRotation,  localTranslation);

		cv::Mat globalRotation, globalTranslation;
		getFromTransformation(globalTransform, globalRotation, globalTranslation);

		// TODO: check output and maybe convert the input types to float 
		// triangulate and get the world points
		cv::Mat1d worldPoints;
		cv::triangulatePoints(currentScene.getPreviousProjection(), getProjection(globalRotation, globalTransform), leftMatches, rightMatches, worldPoints);



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