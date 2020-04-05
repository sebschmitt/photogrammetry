#include "essentialMatrix.hpp"
#include <opencv2/calib3d.hpp>
#include "model-exporters.hpp"
#include "stdlib.h"

#include <iostream>

using namespace std;
using namespace cv;

EssentialMatrix::EssentialMatrix(Calibration cameraCalibration) {
    this->cameraCalibration = cameraCalibration;
}


Mat1f EssentialMatrix::homogenizePoints(const vector<Point2f>& imagePoints) {
    Mat1f homogenPoints(3, imagePoints.size());

    for (int i = 0; i < imagePoints.size(); i++) {
        homogenPoints(0, i) = imagePoints[i].x;
        homogenPoints(1, i) = imagePoints[i].y;
        homogenPoints(2, i) = 1;
    }
    return homogenPoints;
}

template<class T>
vector<T> EssentialMatrix::applyMask(const vector<T>& input,
                                     const vector<uchar>& mask) {
    vector<T> output;
    for(size_t i = 0; i < mask.size(); i++) {
        if (mask[i] == 1)
            output.push_back(input[i]);
    }
    return output;
}

void EssentialMatrix::determineEssentialMatrix(const vector<Point2f> &pointsCamera1,
                                               const vector<Point2f> &pointsCamera2, 
                                               Mat1d& extrincts,
                                               Mat1d& projectionMatrix,
                                               Mat1f& worldPoints) {

    vector<uchar> fundamentalMask;
    Mat1d fundametalMatrix = findFundamentalMat(pointsCamera1, pointsCamera2, FM_RANSAC, 0.1, 0.99, fundamentalMask);

    cout << "points count: " << pointsCamera1.size() << endl;
    Mat1f inliners1 = homogenizePoints(applyMask<Point2f>(pointsCamera1, fundamentalMask));
    Mat1f inliners2 = homogenizePoints(applyMask<Point2f>(pointsCamera2, fundamentalMask));
    /* Mat1f inliners1 = homogenizePoints(pointsCamera1); */
    /* Mat1f inliners2 = homogenizePoints(pointsCamera2); */
    cout << "inliers1 count: " << inliners1.cols << endl;

    Mat1d essentialMatrix = cameraCalibration.getInstrincs().t() * fundametalMatrix * cameraCalibration.getInstrincs();

    decomposeEssentialMatrix(essentialMatrix, inliners1, inliners2, extrincts, projectionMatrix, worldPoints);

    for (int i = 0; i < worldPoints.cols; i++) {
        cout << "=====================" << endl;
        cout << worldPoints.col(i) <<  " " << endl;
        float dividend = worldPoints.at<float>(3, i);
        worldPoints.at<float>(0, i) = worldPoints.at<float>(0, i) / dividend;
        worldPoints.at<float>(1, i) = worldPoints.at<float>(1, i) / dividend;
        worldPoints.at<float>(2, i) = worldPoints.at<float>(2, i) / dividend;
        worldPoints.at<float>(3, i) = worldPoints.at<float>(3, i) / dividend;

        cout << worldPoints.col(i) << endl;
    }

    cout << worldPoints << endl;
}


void EssentialMatrix::decomposeEssentialMatrix(const Mat1d& essentialMatrix,
                                               const Mat1f& inliners1,
                                               const Mat1f& inliners2,
                                               Mat1d& extrincts,
                                               Mat1d& projectionMatrix,
                                               Mat1f& worldPoints) {

    assert(essentialMatrix.rows == 3 && essentialMatrix.cols == 3);
    assert(inliners1.rows == 3);
    assert(inliners2.rows == 3);
    
    Mat1d rotation1, rotation2, trans;
    decomposeEssentialMat(essentialMatrix, rotation1, rotation2, trans);

    vector<Mat> rotations = {rotation1, rotation1, rotation2, rotation2};
    vector<Mat> translations = {trans, -trans, trans, -trans};

    int pointsInfront = 0;
    int currentPointsInfront = 0;

    Mat1d currentExtrincs;
    Mat1d currentProjectionMat;
    Mat1f currentWorldPoints(3, 4);


    // TODO: fix change determineEssentalMatrix parameters
    // TODO: fix image Points to always be float
    // TODO: check other variable types
    for (size_t i = 0; i < rotations.size(); i++) {
        
        currentExtrincs = Mat1d(3,4);
        hconcat(rotations[i], translations[i], currentExtrincs); 
        currentProjectionMat = cameraCalibration.getInstrincs() * currentExtrincs;
        currentWorldPoints.release();

        currentPointsInfront = pointsInfrontCamera(inliners1, inliners2, rotations[i], translations[i], currentExtrincs, currentProjectionMat, currentWorldPoints);

        if (currentPointsInfront > pointsInfront) {
            pointsInfront = currentPointsInfront;
            extrincts = currentExtrincs;
            projectionMatrix = currentProjectionMat ;
            worldPoints = currentWorldPoints;
        }
    }
    cout << "Selected matrices with " << pointsInfront;
}


int EssentialMatrix::pointsInfrontCamera(const Mat1f& inliners1,
                                         const Mat1f& inliners2,
                                         const Mat1d& cameraRotation,
                                         const Mat1d& cameraTranslation,
                                         const Mat1d& extrincts,
                                         const Mat1d& projectionMatrix,
                                         Mat1f& worldPoints) {

    /* siehe https://answers.opencv.org/question/27155/from-fundamental-matrix-to-rectified-images/ */
    /* und */ 
    /* https://en.wikipedia.org/wiki/Essential_matrix */
    assert(inliners1.rows == 3);
    assert(inliners2.rows == 3);
    assert(inliners1.rows == inliners2.rows);
    assert(cameraRotation.cols == 3 && cameraRotation.rows == 3);
    assert(cameraTranslation.rows == 3 && cameraTranslation.cols == 1);

    Mat extrincts1;
    Mat extrincts2;

    hconcat(Mat::diag(Mat::ones(1, 3, CV_64FC1)), Mat::zeros(3, 1, CV_64FC1), extrincts1); 
    extrincts.convertTo(extrincts2, CV_32FC1);

    Mat p1 = cameraCalibration.getInstrincs() * extrincts1;
    Mat p2;

    p1.convertTo(p1, CV_32FC1);
    projectionMatrix.convertTo(p2, CV_32FC1);
    extrincts1.convertTo(extrincts1, CV_32FC1);

    triangulatePoints(p1, projectionMatrix, inliners1.rowRange(0, 2), inliners2.rowRange(0, 2), worldPoints);

    int numberOfPointsInFront = 0;
    vector<int> pointMask;

    for (int i = 0; i < worldPoints.cols; i++) {
        if ((worldPoints.at<float>(2, i) > 0 && worldPoints.at<float>(3, i) > 0) ||  (worldPoints.at<float>(2, i) < 0 && worldPoints.at<float>(3, i) < 0)) {
            pointMask.push_back(1);
            numberOfPointsInFront++;
        } else
            pointMask.push_back(0);
    }
    return numberOfPointsInFront;
}

