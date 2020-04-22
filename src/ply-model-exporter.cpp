#include "model-exporters.hpp"
#include "colors.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <vector>


using namespace std;
using namespace cv;


void PlyModelExporter::exportPointCloudSequence(const filesystem::path& filepath,  Iterator<Scene::ImagePair>* imageSequence) {

    cv::Mat worldPoints = cv::Mat(3, 0, CV_32FC1);
    std::vector<Colors::Color> colors;

    while (imageSequence->hasNext()) {
        Scene::ImagePair *currentScene = imageSequence->next();

        cv::Mat points = currentScene->getWorldPoints();
        
        if (points.cols == 0)
            break;

        auto currentColors = currentScene->getColors();

        if (currentColors.size() != points.cols)
            std::cout << "Number of colors and reconstructed points are not equal." << std::endl;

        colors.insert(colors.end(), currentColors.begin(), currentColors.end());

        cv::hconcat(worldPoints, points, worldPoints);
    }

    exportPointCloud(filepath, worldPoints, colors);
}

void PlyModelExporter::exportPointCloud(const filesystem::path& filepath,
                                        const Mat& worldPoints,
                                        const std::vector<Colors::Color> &colors) {
    assert(worldPoints.rows == 3);

    if (filepath.has_parent_path() && !filesystem::exists(filepath.parent_path()))
        throw runtime_error("The directory path for the file " + filepath.string() + " does not exist.");

    if (filepath.extension() != ".ply")
        throw runtime_error("Invalid file extension " + filepath.extension().string() + ". Exptected .ply");

    if (worldPoints.channels() != 1) 
        throw runtime_error("Only 1 channel matrices are supported.");

    if (worldPoints.type() != CV_32FC1)
        throw runtime_error("Only matrices with floating point values are supported.");

    ofstream outputfile(filepath);

    if (outputfile.is_open()) {
        writeHeader(outputfile, worldPoints);
        writeVertexList(outputfile, worldPoints, colors);
        outputfile.close();
    } else {
        throw runtime_error("Failed to open file " + filepath.string());
    }
}

void PlyModelExporter::writeHeader(ofstream& outputfile, const Mat& vertices) {
    outputfile << "ply" << endl;
    outputfile << "format ascii 1.0" << endl;
    outputfile << "element vertex " << vertices.cols << endl;
    outputfile << "property float x" << endl;
    outputfile << "property float y" << endl;
    outputfile << "property float z" << endl;
    outputfile << "property uchar red" << endl;
    outputfile << "property uchar green" << endl;
    outputfile << "property uchar blue" << endl;
    /* outputfile << "element face 7" << endl; */
    /* outputfile << "property list uchar int vertex_index" << endl; */
    /* outputfile << "element edge 5" << endl; */
    /* outputfile << "property int vertex1" << endl; */
    /* outputfile << "property int vertex2" << endl; */
    /* outputfile << "property uchar red" << endl; */
    /* outputfile << "property uchar green" << endl; */
    /* outputfile << "property uchar blue" << endl; */
    outputfile << "end_header" << endl;
}

void PlyModelExporter::writeVertexList(ofstream& outputfile, const Mat& vertices, const vector<Colors::Color> &colors) {
    for (size_t col = 0; col < vertices.cols; col++) {
        outputfile << vertices.at<float>(0, col) << " ";
        outputfile << vertices.at<float>(1, col) << " ";
        outputfile << vertices.at<float>(2, col) << " ";
        outputfile << (int) colors.at(col).Red << " ";
        outputfile << (int) colors.at(col).Green << " ";
        outputfile << (int) colors.at(col).Blue << endl;
    }

}
