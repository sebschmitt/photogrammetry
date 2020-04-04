#include "feature-matching.hpp"
#include "calibration.hpp"
#include "essentialMatrix.hpp" 

#include <string>
#include <filesystem>
#include "model-exporters.hpp"

#include <iostream>

#include "argparser.h"



using namespace std;

int main(int argc, char *argv[]) {
    argparser::ArgumentParser parser("yapgt (yet another photogrammetry tool)");

    argparser::Argument a_loadCalibration("loadcalibration", "Filepath to load calibration from");
    argparser::Argument a_saveCalibration("savecalibration", "Filepath to save calibration to");
    argparser::Argument a_calibrationImages("calibrationImages", "Images for Calibration");

    parser.addArgument(&a_loadCalibration);
    parser.addArgument(&a_saveCalibration);
    parser.addArgument(&a_calibrationImages);

    parser.parseArguments(argc, argv);

    Calibration calb = Calibration();

    if (a_loadCalibration.isFound()) {
        calb.loadCalibration(filesystem::path(a_loadCalibration.getValue<string>()));
    }

    if (a_calibrationImages.isFound()) {
        filesystem::path path(a_calibrationImages.getValue<string>());
        vector<filesystem::path> filepaths;
        for (const auto& entry : filesystem::directory_iterator(path)) {
            filepaths.push_back(entry.path());
        }

        // number of inner corners per chessboard row and column
        // TODO add argument
        unsigned int cornersRow = 6;
        unsigned int cornersColumn = 9;

        calb.calibrate(filepaths, cv::Size(cornersColumn, cornersRow));
    }

     // TODO: check if either a_loadCalibration or a_calibrationImages was found

    if (a_saveCalibration.isFound()) {
         calb.saveCalibration(filesystem::path(a_saveCalibration.getValue<string>()));
    }

    // TODO: add further arguments

    /* cv::Mat img1 = cv::imread("./resources/guitar_images/20200223_125417880_iOS.jpg"); */
    cv::Mat img1 = cv::imread("./resources/guitar_images/20200223_125427418_iOS.jpg");
    cv::Mat img2 = cv::imread("./resources/guitar_images/20200223_125447874_iOS.jpg");

    /* cv::namedWindow("Matches", cv::WINDOW_FREERATIO); */
    /* cv::imshow("Matches", img1); */
    /* cv::waitKey(0); */
    /* cv::imshow("Matches", img2); */
    /* cv::waitKey(0); */

    vector<cv::Point2f> imagePoints1;
    vector<cv::Point2f> imagePoints2;
    FeatureMatching featureMatching = FeatureMatching(calb);
    featureMatching.findMatches(img1, img2, imagePoints1, imagePoints2);

    cout << "matching successful" << endl;
    EssentialMatrix essentialMatrixComputer(calb);

    cv::Mat1d extrincts, projectionMatrix;
    cv::Mat1f worldPoints;
    essentialMatrixComputer.determineEssentialMatrix(imagePoints1,
                                                     imagePoints2,
                                                     extrincts,
                                                     projectionMatrix,
                                                     worldPoints);
    cout << worldPoints << endl;;
    worldPoints = worldPoints;
    cout << worldPoints << endl;

    worldPoints.pop_back();

    PlyModelExporter exporter;
    exporter.exportPointCloud("./resources/test.ply", worldPoints);
}
