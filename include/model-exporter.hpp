#ifndef MODEL_EXPORTER_H
#define MODEL_EXPORTER_H

// #include "imagepair-sequence-iterator.hpp"
#include "iterator.hpp"
#include "image-pair.hpp"
#include <opencv2/core.hpp>
#include <filesystem>


struct Color {
    unsigned char Red;
    unsigned char Green;
    unsigned char Blue;
};

class ModelExporter {
    public:
        virtual void exportPointCloud(const std::filesystem::path& filepath,
                                      const cv::Mat& worldPoints,
                                      const std::vector<Color>& colors) = 0;
        virtual void exportPointCloudSequence(const std::filesystem::path& filepath, Iterator<Scene::ImagePair>* imageSequence) = 0;

};

#endif
