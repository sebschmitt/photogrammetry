//
// Created by Sebastian Schmitt on 08.04.2020.
//

#include "image-pair.h"

#include <utility>

namespace Scene {
    ImagePair::ImagePair(Scene::Image leftImage, Scene::Image rightImage) {
        this->leftImage = std::move(leftImage);
        this->rightImage = std::move(rightImage);
        this->prevPair = nullptr;
        this->nextPair = nullptr;
    }

    cv::Mat ImagePair::getLeftImage() {
        return this->leftImage.image;
    }

    cv::Mat ImagePair::getRightImage() {
        return this->rightImage.image;
    }

    std::vector<cv::Point2f> ImagePair::getLeftPoints() {
        return this->leftImage.imagePoints;
    }

    std::vector<cv::Point2f> ImagePair::getRightPoints() {
        return this->rightImage.imagePoints;
    }
}
