#ifndef MODEL_EXPORTER_H
#define MODEL_EXPORTER_H

#include <opencv2/core.hpp>
#include <filesystem>

class ModelExporter {
    public:
        virtual void exportPointCloud(const std::filesystem::path& filepath,
                                      const cv::Mat& worldPoints) = 0;

};

#endif
