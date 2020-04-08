//
// Created by Sebastian Schmitt on 08.04.2020.
//

#ifndef YAPGT_IMAGE_PAIR_H
#define YAPGT_IMAGE_PAIR_H

#include "image.h"

namespace Scene {
    class ImagePair {
    private:
        Image leftImage;
        Image rightImage;
        ImagePair *prevPair;
        ImagePair *nextPair;
        std::vector<unsigned int> overlappingPoints;
    public:
        ImagePair(Image leftImage, Image rightImage);
        std::vector<cv::Point2f> getLeftPoints();
        std::vector<cv::Point2f> getRightPoints();
        std::vector<unsigned int> getOverlappingPoints();
        cv::Mat getLeftImage();
        cv::Mat getRightImage();
    };

}

#endif //YAPGT_IMAGE_PAIR_H
