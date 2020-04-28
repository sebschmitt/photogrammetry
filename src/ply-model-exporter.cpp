#include "model-exporters.hpp"
#include "colors.hpp"

#include <exception>
#include <fstream>
#include <iostream>
#include <vector>


using namespace std;
using namespace cv;


void PlyModelExporter::exportPointCloudSequence(const filesystem::path& filepath,  Iterator<Scene::ImagePair>* imageSequence) {
    std::vector<cv::Point3f> worldPoints;
    std::vector<Colors::Color> colors;

    while (imageSequence->hasNext()) {
        Scene::ImagePair *currentScene = imageSequence->next();
        std::vector<cv::Point3f> points = currentScene->getWorldPoints();
        
        if (points.size() == 0)
            break;

        auto currentColors = currentScene->getColors();

        if (currentColors.size() != points.size())
            std::cout << "Number of colors and reconstructed points are not equal." << std::endl;

        colors.insert(colors.end(), currentColors.begin(), currentColors.end());
        worldPoints.insert(worldPoints.end(), points.begin(), points.end());

    }

    exportPointCloud(filepath, worldPoints, colors);
}

void PlyModelExporter::exportPointCloud(const filesystem::path& filepath,
                                        const std::vector<cv::Point3f>& worldPoints,
                                        const std::vector<Colors::Color> &colors) {

    if (filepath.has_parent_path() && !filesystem::exists(filepath.parent_path()))
        throw runtime_error("The directory path for the file " + filepath.string() + " does not exist.");

    if (filepath.extension() != ".ply")
        throw runtime_error("Invalid file extension " + filepath.extension().string() + ". Exptected .ply");

    ofstream outputfile(filepath);

    if (outputfile.is_open()) {
        writeHeader(outputfile, worldPoints);
        writeVertexList(outputfile, worldPoints, colors);
        outputfile.close();
    } else {
        throw runtime_error("Failed to open file " + filepath.string());
    }
}

void PlyModelExporter::writeHeader(ofstream& outputfile, const std::vector<cv::Point3f>& vertices) {
    outputfile << "ply" << endl;
    outputfile << "format ascii 1.0" << endl;
    outputfile << "element vertex " << vertices.size() << endl;
    outputfile << "property float x" << endl;
    outputfile << "property float y" << endl;
    outputfile << "property float z" << endl;
    outputfile << "property uchar red" << endl;
    outputfile << "property uchar green" << endl;
    outputfile << "property uchar blue" << endl;
    outputfile << "end_header" << endl;
}

void PlyModelExporter::writeVertexList(ofstream& outputfile, const std::vector<cv::Point3f>& vertices, const vector<Colors::Color> &colors) {
    for (size_t col = 0; col < vertices.size(); col++) {
        outputfile << vertices.at(col).x << " ";
        outputfile << vertices.at(col).y << " ";
        outputfile << vertices.at(col).z << " ";
        outputfile << (int) colors.at(col).Red << " ";
        outputfile << (int) colors.at(col).Green << " ";
        outputfile << (int) colors.at(col).Blue << endl;
    }

}
