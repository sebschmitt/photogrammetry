#include "feature-matching.hpp"
#include "calibration.hpp"
#include "essentialMatrix.hpp" 

#include <string>
#include <filesystem>
#include "model-exporters.hpp"

#include <iostream>


using namespace std;

int main(int argc, char *argv[]) {
    Calibration calb = Calibration();


    filesystem::path calibrationFile("./resources/calibration.xml");
    filesystem::path path("./resources/chessboard_images");

    if (!filesystem::exists(calibrationFile)) {
        vector<filesystem::path> filepaths;
        for (const auto& entry : filesystem::directory_iterator(path)) {
            filepaths.push_back(entry.path());
        }

        // number of inner corners per chessboard row and column
        unsigned int cornersRow = 6;
        unsigned int cornersColumn = 9;

        calb.calibrate(filepaths, cv::Size(cornersColumn, cornersRow));
        calb.saveCalibration(calibrationFile);
    } else {
        calb.loadCalibration(calibrationFile);
    }

    /* calb.loadCalibration("./resources/calibration.xml"); */

    cout << "calibration successful" << endl;


    cv::Mat img1 = cv::imread("./resources/guitar_images/20200223_125417880_iOS.jpg");
    /* cv::Mat img1 = cv::imread("./resources/guitar_images/20200223_125427418_iOS.jpg", cv::IMREAD_GRAYSCALE); */
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

    cv::Mat cameraRotation, cameraTranslation;
    essentialMatrixComputer.determineEssentialMatrix(imagePoints1,
                                                     imagePoints2,
                                                     cameraRotation,
                                                     cameraTranslation);




    cv::Mat test = (cv::Mat_<float>(3,5) << 5, 5, 5,
                                             10, 20, 10,
                                             15, 13, 15,
                                             30, 20, 20,
                                             20, 10, 25);

   PlyModelExporter exporter;

   exporter.exportPointCloud("./resources/test.ply", test);

}
