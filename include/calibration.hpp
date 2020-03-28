#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <filesystem>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/calib3d.hpp>

class Calibration {

private:	
    cv::Mat1d cameraMatrix = cv::Mat1d(3, 3); //= cv::Mat1d(3, 3);
    cv::Mat1d optimalCameraMatrix = cv::Mat1d(3, 3); // = cv::Mat1d(3, 3);
    cv::Mat1d distortionCoefficients;
    std::vector<cv::Mat> rotationVectors;
    std::vector<cv::Mat> translationVectors;

    std::string cameraMatrixSerName = "cameraMatrix";
    std::string optimalMatrixSerName = "optimalMatrix";
    std::string distortionCoefficientsSerName = "distortionCoefficients";

public:
	Calibration();

    const cv::Mat& getCameraMatrix() const {return cameraMatrix;}
    const cv::Mat& getInstrincs() const {return cameraMatrix;}

	void calibrate(std::vector<std::filesystem::path> imageFiles, cv::Size boardSize);

	void loadCalibration(std::filesystem::path filepath);

	void saveCalibration(std::filesystem::path filepath);

    void undistortImage(const cv::Mat& image, cv::Mat& undistortedImage);
	
};

#endif
