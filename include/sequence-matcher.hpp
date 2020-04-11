#ifndef YAPGT_SEQUENCE_MATCHER_H
#define YAPGT_SEQUENCE_MATCHER_H

#include "image-pair.hpp"

#include <filesystem>
#include <vector>

struct ImageContainer {
    std::string name;
    cv::Mat image;
    std::vector<cv::KeyPoint> keypoints;
    cv::Mat descriptors;
    // keypoint index, image_index, key point index
    std::map<size_t, std::map<size_t, size_t>> keypointMatches;
};

class SequenceMatcher {

private:
	void getKeypointIndexes(const ImageContainer& leftImageContainer, const size_t& rightImageIndex, std::vector<size_t>& leftKeypointIndexes, std::vector<size_t> rightKeypointIndexes);
	Scene::Image createImageFromContainer(ImageContainer container);

public:
    //SequenceMatcher();

    // TODO: change return value
    void generateSequence(std::filesystem::path folderpath);
 };


#endif

