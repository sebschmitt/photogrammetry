#ifndef MODEL_EXPORTERS_H
#define MODEL_EXPORTERS_H

#include "model-exporter.hpp"
#include "colors.hpp"

class PlyModelExporter : ModelExporter {
    
    public:
        void exportPointCloud(const std::filesystem::path& filepath,
                              const std::vector<cv::Point3f>& worldPoints,
                              const std::vector<Colors::Color>& colors) override;
        void exportPointCloudSequence(const std::filesystem::path& filepath, Iterator<Scene::ImagePair>* imageSequence) override;

    private:
        cv::Mat worldPoints;

        void writeHeader(std::ofstream& outputfile,
                         const std::vector<cv::Point3f>& vertices);
        void writeVertexList(std::ofstream& outputfile,
                             const std::vector<cv::Point3f>& vertices,
                             const std::vector<Colors::Color> &colors);

};

#endif
