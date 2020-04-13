#include "feature-matching.hpp"
#include "calibration.hpp"
#include "essentialMatrix.hpp" 

#include <string>
#include <filesystem>
#include "model-exporters.hpp"

#include <iostream>

#include "argparser.h"
#include <sequence-matcher.hpp>
#include <reconstruction.hpp>



using namespace std;


int main(int argc, char *argv[]) {


    



    argparser::ArgumentParser parser("yapgt (yet another photogrammetry tool)");

    /* Calibration Arguments */
    argparser::Argument a_loadCalibration("loadcalibration", "Filepath to load calibration from");
    argparser::Argument a_saveCalibration("savecalibration", "Filepath to save calibration to");
    argparser::Argument a_calibrationImages("calibrationImages", "Images for Calibration");
    argparser::Argument a_calibrationCalibrateRow("calibrateRow", "corners row for Calibration");
    argparser::Argument a_calibrationCalibrateColumn("calibrateColumn", "corners column for Calibration");

    argparser::Argument a_matchImages("matchImages", "Images to use for matching");
    argparser::Argument a_outFile("out", "Output file");


    parser.addArgument(&a_loadCalibration);
    parser.addArgument(&a_saveCalibration);
    parser.addArgument(&a_calibrationImages);
    parser.addArgument(&a_calibrationCalibrateRow);
    parser.addArgument(&a_calibrationCalibrateColumn);
    parser.addArgument(&a_matchImages);
    parser.addArgument(&a_outFile);

    parser.parseArguments(argc, argv);
    
    // iphone test
    Calibration calb = Calibration();

    if (!(a_loadCalibration.isFound() || a_calibrationImages.isFound())) {
        cout << "Neither " << a_loadCalibration.getName() << " nor " << a_loadCalibration.getName() << " was supplied" << endl;
        return -1;
    }

    if (a_loadCalibration.isFound() && a_calibrationImages.isFound()) {
        cout << "You can't use  " << a_loadCalibration.getName() << " and " << a_loadCalibration.getName() << " at the same time" << endl;
        return -1;
    }

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

        if (!a_calibrationCalibrateRow.isFound()) {
            cout << "Argument " << a_calibrationCalibrateRow.getName() << " is required for calibration" << endl;
            return -1;
        }

        if (!a_calibrationCalibrateColumn.isFound()) {
            cout << "Argument " << a_calibrationCalibrateColumn.getName() << " is required for calibration" << endl;
            return -1;
        }

        unsigned int cornersRow = a_calibrationCalibrateRow.getValue<int>();
        unsigned int cornersColumn = a_calibrationCalibrateColumn.getValue<int>();

        calb.calibrate(filepaths, cv::Size(cornersColumn, cornersRow));
    }

    if (a_saveCalibration.isFound()) {
         calb.saveCalibration(filesystem::path(a_saveCalibration.getValue<string>()));
    }

    /* Prototype for later use */
    // if (a_matchImages.isFound()) {
        // for (const auto& entry : filesystem::directory_iterator(a_matchImages.getValue<string>())) {
        // }
    // }



    if (!a_matchImages.isFound()) { // TODO proper handling?
        cout << "Argument " << a_matchImages.getName() << " was not found" << endl;
        return -1;
    }

    if (!a_outFile.isFound()) { // TODO proper handling?
        cout << "Argument " << a_outFile.getName() << " was not found" << endl;
        return -1;
    }


    // TODO: validate file format
    std::vector<filesystem::path> inputImagePaths;
    for (const auto& entry : filesystem::directory_iterator(a_matchImages.getValue<string>())) {
        inputImagePaths.push_back(entry.path());
    }

    string testFolder = "./resources/ps4_controller";
    // string testFolder = "./resources/pokemon_ball";

    SequenceMatcher sequenceMatcher(calb);
    Scene::SceneSequence sequence = sequenceMatcher.generateSequence(testFolder);

    SceneReconstructor reconstructor(calb);
    reconstructor.reconstructScenes(sequence.createIterator());

    PlyModelExporter exporter;
    exporter.exportPointCloudSequence(a_outFile.getValue<string>(), sequence.createIterator());

    return 0;
}
