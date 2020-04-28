#ifndef YAPGT_IMAGE_H
#define YAPGT_IMAGE_H

#include <string>
#include <opencv2/core/mat.hpp>

namespace Scene {
    struct Image {
        std::string name;
        cv::Mat image;
        std::vector<cv::Point2f> imagePoints;
    };
}

#endif //YAPGT_IMAGE_H
