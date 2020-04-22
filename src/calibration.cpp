#include "calibration.hpp"

#include <opencv2/imgproc.hpp>

#include <exception>
#include <fstream>
#include <iostream>


using namespace std;
using namespace cv;


Calibration::Calibration() {
    cameraMatrix.at<double>(0, 0) = 1;
    cameraMatrix.at<double>(1, 1) = 1;
}


void Calibration::saveCalibration(filesystem::path filepath) {
    if (!filepath.has_filename())
        throw runtime_error("The provided filepath " + filepath.string() + "  is not a file.");

    filesystem::path directoryPath = filepath.parent_path();
    
    if (!directoryPath.empty() && !filesystem::exists(directoryPath))
        throw runtime_error("The provieded directoryPath " + directoryPath.string() + " does not exist.");

    cout << "Saving calibration to " << filepath.string().c_str() << endl;
    
    cv::FileStorage fs(filepath.string(), cv::FileStorage::WRITE);
    fs << cameraMatrixSerName << cameraMatrix;
    fs << distortionCoefficientsSerName << distortionCoefficients;
    fs.release();

    cout << "Successfully saved calibration!" << endl;
}

void Calibration::undistortImage(const Mat& image, Mat& undistortedImage) {
    cv::undistort(image, undistortedImage, cameraMatrix, distortionCoefficients);
}

void Calibration::loadCalibration(filesystem::path filepath) {
    if (!filesystem::exists(filepath))
        throw runtime_error("The file does not exist.");

    cout << "Loading calibration from file " << filepath.string().c_str() << endl;

    FileStorage fs(filepath.string(), FileStorage::READ);
    cameraMatrix = fs[cameraMatrixSerName].mat();
    distortionCoefficients = fs[distortionCoefficientsSerName].mat();
    fs.release();

    assert(cameraMatrix.type() == CV_64FC1);
    assert(distortionCoefficients.type() == CV_64FC1);

    cout << "Successfully loaded calibration!" << endl;
}

void Calibration::calibrate(vector<filesystem::path> imageFiles, Size boardSize) {
    vector<vector<Point3f>> objectPoints;
    vector<vector<Point2f>> imagePoints;
    vector<Point2f> corners;
    vector<Point3f> object;

    for (int tileCornerCounter = 0; tileCornerCounter < boardSize.area(); tileCornerCounter++) {
        object.push_back(Point3f(tileCornerCounter / boardSize.width, tileCornerCounter % boardSize.width, 0.0));
    }

    Mat currentImage;
    for (auto const& imagePath: imageFiles) {
        currentImage = imread(imagePath.string());
        resize(currentImage, currentImage, currentImage.size() / IMAGE_DOWNSAMPLE);
        cvtColor(currentImage, currentImage, COLOR_BGR2GRAY);

        std::cout << "Trying to find corners in file " << imagePath.filename() << ": ";

        bool found = findChessboardCorners(currentImage, boardSize, corners,
                                           CALIB_CB_ADAPTIVE_THRESH | CALIB_CB_FILTER_QUADS);

        if (found) {
            cout << "Found corners" << endl;

            cornerSubPix(currentImage, corners, Size(5, 5), Size(-1, -1),
                         TermCriteria(TermCriteria::EPS | TermCriteria::MAX_ITER, 30, 0.001));

            imagePoints.push_back(corners);
            objectPoints.push_back(object);
        } else {
            cout << "Couldn't find corners." << endl;
        }
    }

    cout << "calibrating" << endl;
    double calibrationOutput = calibrateCamera(objectPoints,
                                               imagePoints,
                                               currentImage.size(),
                                               cameraMatrix,
                                               distortionCoefficients,
                                               rotationVectors,
                                               translationVectors);
    assert(cameraMatrix.type() == CV_64FC1);
    assert(distortionCoefficients.type() == CV_64FC1);

    cout << "Camera calibrated with output " << calibrationOutput << endl;
}
