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


Mat EssentialMatrix::normalizePoints(const vector<Point2f>& imagePoints) {
    Mat_<double> normalizedPoints(imagePoints.size(), 3);

    for (const auto &point: imagePoints) {
        Mat x = (Mat_<double>(1,3) << point.x, point.y, 1.0);
        normalizedPoints.push_back(x);
    }
    return normalizedPoints;
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
                                               Mat &cameraRotation,
                                               Mat &cameraTranslation) {

    cout << "Determining essentialMatrix" << endl;

    /* Mat fundamentalMask(pointsCamera1.size(), 1, CV_8UC1); */
    vector<uchar> fundamentalMask;
    Mat fundametalMatrix = findFundamentalMat(pointsCamera1, pointsCamera2, FM_RANSAC, 0.1, 0.99, fundamentalMask);

    /* cout << " fundametalMatrix:" << endl; */
    /* cout << fundametalMatrix << endl; */

    Mat inliners1 = normalizePoints(applyMask<Point2f>(pointsCamera1, fundamentalMask));
    Mat inliners2 = normalizePoints(applyMask<Point2f>(pointsCamera2, fundamentalMask));

    Mat essentialMatrix = cameraCalibration.getInstrincs().t() * fundametalMatrix * cameraCalibration.getInstrincs();

    /* cout << "Essential matrix: " << endl; */
    /* cout << essentialMatrix << endl; */

    // decomposes the essential matrix into 2 possible rotation matrices and 1 translation vector
    decomposeEssentialMatrix(essentialMatrix, inliners1, inliners2,  cameraRotation,  cameraTranslation);

    /* cout << "found rotation and camera rotation" << endl; */
    /* Mat extrincts1; */
    /* Mat extrincts2; */

    /* cv::hconcat(Mat::diag(Mat::ones(1, 3, CV_64FC1)), Mat::zeros(3, 1, CV_64FC1), extrincts1); */ 
    /* cv::hconcat(cameraRotation, cameraTranslation, extrincts2); */ 

    /* cout << "calculating projection matrices .." << endl; */
    /* cout << cameraCalibration.getInstrincs() << endl; */
    /* cout << cameraCalibration.getCameraMatrix() << endl; */
    /* Mat p1 = cameraCalibration.getInstrincs() * extrincts1; */
    /* Mat p2 = cameraCalibration.getInstrincs() * extrincts2; */

    /* cout << "copying points.. " << endl; */
    /* Mat points1 = Mat_<double>(inliners1.cols, inliners1.rows);; */
    /* Mat points2= Mat_<double>(2, inliners1.rows); */
    /* for (int i = 0; i < inliners1.rows; i++) { */
    /*     points1.at<double>(0, i) = inliners1.at<double>(i, 0); */
    /*     points1.at<double>(1, i) = inliners1.at<double>(i, 1); */
    /*     points2.at<double>(0, i) = inliners2.at<double>(i, 0); */
    /*     points2.at<double>(1, i) = inliners2.at<double>(i, 1); */
    /* } */

    /* pointsCamera1.convertTo(pointsCamera1, CV_32FC1); */
    /* pointsCamera2.convertTo(pointsCamera2, CV_32FC1); */
    /* Mat1f worldPoints; */
    /* worldPoints.convertTo(worldPoints, CV_32FC1); */

    /* cv::triangulatePoints(p1, p2, pointsCamera1, pointsCamera2, worldPoints); */
    /* cout << pointsCamera1.size() << endl; */
    /* cout << worldPoints.size() << endl; */

    /* cout << worldPoints.type() << " " << worldPoints.cols << endl; */

    /* cout << worldPoints.col(0) << endl; */

    /* cout << "exporting.." << endl; */
    /* Mat newWorldPoints(3, worldPoints.cols, CV_32FC1); */
    /* for (int i = 0; i < worldPoints.cols; i++) { */
    /*     newWorldPoints.at<float>(0, i) = (worldPoints.at<float>(0, i) / worldPoints.at<float>(3, i)) * 100; */
    /*     newWorldPoints.at<float>(1, i) = (worldPoints.at<float>(1, i) / worldPoints.at<float>(3, i)) * 100; */
    /*     newWorldPoints.at<float>(2, i) = (worldPoints.at<float>(2, i) / worldPoints.at<float>(3, i)) * 100; */
        /* cout << worldPoints.at<float>(0, i) / worldPoints.at<float>(3, i) << endl; */
        /* cout << worldPoints.at<float>(1, i) / worldPoints.at<float>(3, i) << endl; */
        /* cout << worldPoints.at<float>(2, i) / worldPoints.at<float>(3, i) << endl; */
        /* cout << worldPoints.at<float>(3, i) / worldPoints.at<float>(3, i) << endl; */
    /* } */


    /* PlyModelExporter exporter; */
    /* exporter.exportPointCloud("./resources/testPoints2.ply", newWorldPoints); */
    /* cout << "exported" << endl; */
}


void EssentialMatrix::decomposeEssentialMatrix(const Mat &essentialMatrix,
                                               const Mat &inliners1,
                                               const Mat &inliners2,
                                               Mat &rotation,
                                               Mat &translation) {

    assert(essentialMatrix.rows == 3 && essentialMatrix.cols == 3);
    assert(inliners1.cols == 3);
    assert(inliners2.cols == 3);
    
    Mat rotation1, rotation2, trans;
    decomposeEssentialMat(essentialMatrix, rotation1, rotation2, trans);

    int pointsInfront = 0;
    int currentPointsInfront = 0;
    vector<Mat> rotations = {rotation1, rotation1, rotation2, rotation2};
    vector<Mat> translations = {trans, -trans, trans, -trans};

    for (size_t i = 0; i < rotations.size(); i++) {
        currentPointsInfront = pointsInfrontCamera(inliners1, inliners2, rotations[i], translations[i]);
        cout << "found " << currentPointsInfront << " points in front of camera." << endl;

        if (currentPointsInfront > pointsInfront) {
            pointsInfront = currentPointsInfront;
            rotation = rotations[i];
            translation = translations[i];
        }
    }

    cout << "Counted " << pointsInfront << " points" << endl;
    cout << "Seleced rotation and translation: " << endl;
    cout << rotation << endl;
    cout << translation << endl;
}


int EssentialMatrix::pointsInfrontCamera(const cv::Mat inliners1,
                                          const cv::Mat inliners2,
                                          const cv::Mat &cameraRotation,
                                          const cv::Mat &cameraTranslation) {
    /* siehe https://answers.opencv.org/question/27155/from-fundamental-matrix-to-rectified-images/ */
    /* und */ 
    /* https://en.wikipedia.org/wiki/Essential_matrix */
    assert(inliners1.cols == 3);
    assert(inliners2.cols == 3);
    assert(inliners1.rows == inliners2.rows);
    assert(cameraRotation.cols == 3 && cameraRotation.rows == 3);
    assert(cameraTranslation.rows == 3 && cameraTranslation.cols == 1);
    
    int count = 0;
    for (size_t index = 0; index < inliners1.rows; index++) {
        double z1 = calculateDepth(cameraRotation.row(0), cameraRotation.row(2), cameraTranslation, inliners2.at<double>(index, 0), inliners1.row(index).t());

        double z2 = calculateDepth(cameraRotation.row(1), cameraRotation.row(2), cameraTranslation, inliners2.at<double>(index, 1), inliners1.row(index).t());

        double x1 = inliners1.at<double>(index, 0) * z1;
        double y1 = inliners1.at<double>(index, 1) * z1;

        double x2 = inliners1.at<double>(index, 0) * z2;
        double y2 = inliners1.at<double>(index, 1) * z2;

        if (z1 > 0 && z2 > 0) { 
            count++;
        }
    }

    return count;
}


double EssentialMatrix::calculateDepth(const Mat &rotationRow1,
                                       const Mat &rotationRow2,
                                       const Mat &translation,
                                       const double respectivePoint,
                                       const Mat &imagePoint1) {
    assert(rotationRow1.rows == 1 && rotationRow1.cols == 3);
    assert(rotationRow2.rows == 1 && rotationRow2.cols == 3);
    assert(translation.rows == 3 && translation.cols == 1);
    assert(imagePoint1.rows == 3 && imagePoint1.cols == 1);
    
    Mat point = (rotationRow1 - respectivePoint * rotationRow2).t();

    return point.dot(translation) / point.dot(imagePoint1);
}
