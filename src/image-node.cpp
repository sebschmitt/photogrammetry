#include "image-node.hpp"


ImageNode::ImageNode() {
}

cv::Mat ImageNode::setTransform(cv::Mat trans) {
    assert(trans.cols == 4 && trans.rows == 4);
    assert(trans.type == CV_64F1);
    transform = trans;
}


cv::Mat ImageNode::getTransform() {
    return transform;
}

cv::Mat ImageNode::getPreviousTransform() {
    if (prevNode == nullptr) {
        return cv::Mat::eye(4, 4, cv::CV_64F1);
    }
    return prevNode->transform;
}

cv::Mat ImageNode::getWorldPoints() {
    return worldPoints();
}


cv::Mat ImageNode::getPreviousWorldPoints() {
    if (prevNode == nullptr) {
        return 
    }
}




