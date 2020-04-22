#include "feature-matching.hpp"
#include "calibration.hpp"
#include "essentialMatrix.hpp"

#include <string>
#include <filesystem>
#include "model-exporters.hpp"

#include <iostream>

#include "argparser.hpp"
#include <sequence-matcher.hpp>
#include <reconstruction.hpp>

using namespace std;

int main(int argc, char *argv[]) {
    argparser::ArgumentParser parser("yapgt (yet another photogrammetry tool)");

    /* Calibration Arguments */
    argparser::Argument a_loadCalibration("loadCalibration", "Filepath to load calibration from");
    argparser::Argument a_saveCalibration("saveCalibration", "Filepath to save calibration to");
    argparser::Argument a_calibrationImages("calibrationImages", "Folder with images for calibration");
    argparser::Argument a_calibrationCalibrateRow("calibrateRow", "corners row for Calibration");
    argparser::Argument a_calibrationCalibrateColumn("calibrateColumn", "corners column for Calibration");

    argparser::Argument a_matchImages("matchImages", "Folder with images to match");
    argparser::Argument a_outFile("out", "Output file");
    argparser::Argument a_matchOutputDir("matchOutput", "Folder to store match images in");


    parser.addArgument(&a_loadCalibration);
    parser.addArgument(&a_saveCalibration);
    parser.addArgument(&a_calibrationImages);
    parser.addArgument(&a_calibrationCalibrateRow);
    parser.addArgument(&a_calibrationCalibrateColumn);
    parser.addArgument(&a_matchImages);
    parser.addArgument(&a_outFile);
    parser.addArgument(&a_matchOutputDir);

    parser.parseArguments(argc, argv);

    // iphone test
    Calibration calb = Calibration();

    if (!(a_loadCalibration.isFound() || a_calibrationImages.isFound())) {
        cout << "Neither " << a_loadCalibration.getName() << " nor " << a_calibrationImages.getName() << " was supplied" << endl;
        return -1;
    }

    if (a_loadCalibration.isFound() && a_calibrationImages.isFound()) {
        cout << "You can't use  " << a_loadCalibration.getName() << " and " << a_calibrationImages.getName() << " at the same time" << endl;
        return -1;
    }

    if (a_loadCalibration.isFound()) {
        filesystem::path path(a_loadCalibration.getValue<string>());

        if (!filesystem::exists(path) || !filesystem::is_regular_file(path)) {
            cout << "Value supplied for " << a_loadCalibration.getName() << " is not a valid path" << endl;
            return -1;
        }

        calb.loadCalibration(path);
    }

    if (a_calibrationImages.isFound()) {
        filesystem::path path(a_calibrationImages.getValue<string>());

        if (!filesystem::exists(path) || !filesystem::is_directory(path)) {
            cout << "Value supplied for " << a_calibrationImages.getName() << " is not a valid path" << endl;
            return -1;
        }

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
        filesystem::path saveCalibrationFilePath(a_saveCalibration.getValue<string>());
        filesystem::path outFileFolder = saveCalibrationFilePath.parent_path();
        if (!filesystem::exists(outFileFolder) || !filesystem::is_directory(outFileFolder)) {
            if (!filesystem::create_directories(outFileFolder)) {
                cout << "Could not create directory for " << a_saveCalibration.getName() << endl;
                return -1;
            }
        }

        calb.saveCalibration(saveCalibrationFilePath);
    }

    if (a_matchImages.isFound()) {
        filesystem::path matchImagesPath(a_matchImages.getValue<string>());
        if (!filesystem::exists(matchImagesPath) || !filesystem::is_directory(matchImagesPath)) {
            cout << "Value supplied for " << a_matchImages.getName() << " is not a valid path" << endl;
            return -1;
        }

        // TODO: validate file format
        std::vector<filesystem::path> inputImagePaths;
        for (const auto &entry : filesystem::directory_iterator(matchImagesPath)) {
            inputImagePaths.push_back(entry.path());
        }

        filesystem::path matchOutputPath(a_matchOutputDir.getValue<string>());
        if (a_matchOutputDir.isFound() && !filesystem::exists(matchOutputPath)) {
            if (!filesystem::create_directories(matchOutputPath)) {
                cout << "Could not create directory for " << a_matchOutputDir.getName() << endl;
                return -1;
            }
        }

        SequenceMatcher sequenceMatcher(calb);
        Scene::SceneSequence sequence = sequenceMatcher.generateSequence(matchImagesPath, matchOutputPath);

        SceneReconstructor reconstructor(calb);
        reconstructor.reconstructScenes(sequence.createIterator());


        if (a_outFile.isFound()) {
            filesystem::path outputFilePath(a_outFile.getValue<string>());
            filesystem::path outputFileFolder = outputFilePath.parent_path();
            if (!filesystem::exists(outputFileFolder) || !filesystem::is_directory(outputFileFolder)) {
                if (!filesystem::create_directories(outputFileFolder)) {
                    cout << "Could not create directory for " << a_outFile.getName() << endl;
                    return -1;
                }
            }

            PlyModelExporter exporter;
            exporter.exportPointCloudSequence(outputFilePath, sequence.createIterator());
        }
    }

    return 0;
}
