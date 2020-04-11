#include "sequence-matcher.hpp"

#include "image-pair.hpp"
#include "image.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>

#include <cassert>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>



void SequenceMatcher::generateSequence(std::filesystem::path folderPath) {

    ImageContainer leftBestMatch, rightBestMatch;
    int bestMatchCount = 0;

	assert(std::filesystem::exists(folderPath));
	assert(std::filesystem::is_directory(folderPath));


    std::cout << "Loading images" << std::endl;

    std::vector<ImageContainer> images;
    for (const auto& imageEntry : std::filesystem::directory_iterator(folderPath)) {
        ImageContainer image;

        image.name = imageEntry.path().string();

        try {
            // TODO: addtional testing for e.g image size, filetype etc.
            image.image = cv::imread(imageEntry.path().string());
            assert(!image.image.empty());
        }
        catch (const cv::Exception e) {
            std::cerr << "Failed to open the file " << image.name << ". Error Message: " << e.msg << std::endl;
            continue;
        }

        try {
            // nfeature, nOctaveLayers, contrastThreshold, edgeThreshould, sigma=1.6
            cv::Ptr<cv::xfeatures2d::SIFT> featureFinder = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.6);
            featureFinder->detectAndCompute(image.image, cv::noArray(), image.keypoints, image.descriptors);
        }
        catch (const cv::Exception e) {
            std::cerr << "Failed to  detect features and compute descriptors in image " << image.name << ". Error message: " << e.msg << ". Continuing without this image." << std::endl;
            continue;
        }
               
        images.emplace_back(image);
    }

    // only continue with the programm, when we at least have 2 images
    if (images.size() < 2) {
        std::cerr << "Less than 2 images have been loaded. At least 2 images are necessary. Aborting Programm.";
        return;
    }


    std::cout << "Loading done." << std::endl;
    std::cout << "Matching features..." << std::endl;

    // Use NORM_L1 or NORM_L2, because we are using SIFT 
    // enable cross check
    cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_L1, true);
    // cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_L1, false);

    // match each image with each other.
    for (size_t leftImageIndex=0; leftImageIndex < images.size()-1; leftImageIndex++) {
        ImageContainer &leftImage = images[leftImageIndex];
        std::cout << "Matching Image " << leftImageIndex << std::endl;

        for (size_t rightImageIndex=leftImageIndex+1; rightImageIndex < images.size(); rightImageIndex++) {
			ImageContainer &rightImage = images[rightImageIndex];

            if (leftImageIndex == rightImageIndex)
                continue;

            std::vector<cv::Point2f> leftImagePoints, rightImagePoints;
            std::vector<size_t> leftKeypointIndexes, rightKeypointIndexes;

            std::vector<cv::DMatch> matches;

            // query = left image, train = right image
            // enable normal machting
            matcher->match(leftImage.descriptors, rightImage.descriptors, matches);

            // used for normal matcher
            for (auto& match : matches) {
				leftImagePoints.push_back(leftImage.keypoints[match.queryIdx].pt);
				rightImagePoints.push_back(rightImage.keypoints[match.trainIdx].pt);

				leftKeypointIndexes.push_back(match.queryIdx);
				rightKeypointIndexes.push_back(match.trainIdx);
            }

            // use with knnMatching 
            // std::vector<std::vector<cv::DMatch>> knnMatches;
            // matcher->knnMatch(leftImage.descriptors, rightImage.descriptors, knnMatches, 2);
            
            // use wiht knnMathcing and crossCheck
            // matcher->knnMatch(leftImage.descriptors, rightImage.descriptors, knnMatches, 1);

            // use with general knnMatching
            // for (auto& match : knnMatches) {
                // knnMatch with ratio test
                //if (match[0].distance < 0.7 * match[1].distance) {
             
                // knnMatch with cross check
                // if (match.size()) {
                     // leftImagePoints.push_back(leftImage.keypoints[match[0].queryIdx].pt);
                     // rightImagePoints.push_back(rightImage.keypoints[match[0].trainIdx].pt);

                     // leftKeypointIndexes.push_back(match[0].queryIdx);
                     // rightKeypointIndexes.push_back(match[0].trainIdx);

                    // matches.push_back(match[0]);
                // }
            // }

            // create fundamental mask for further constrains
            std::vector<uchar> mask;
            cv::findFundamentalMat(leftImagePoints, rightImagePoints, cv::FM_RANSAC, 3, 0.99, mask);

            std::vector<cv::DMatch> maskedMatches;

            // add masked matches to the keyppoint mappings of both images
            for (size_t kpIndex = 0; kpIndex < mask.size(); kpIndex++) {
                if (mask[kpIndex]) {
                    leftImage.keypointMatches[leftKeypointIndexes[kpIndex]][rightImageIndex] = rightKeypointIndexes[kpIndex];
                    rightImage.keypointMatches[rightKeypointIndexes[kpIndex]][leftImageIndex] = leftKeypointIndexes[kpIndex];

                    maskedMatches.push_back(matches[kpIndex]);
                }
            }

            cv::Mat canvas;
            cv::drawMatches(leftImage.image, leftImage.keypoints, rightImage.image, rightImage.keypoints, maskedMatches, canvas);

            std::stringstream imgNameStream;
            imgNameStream << "./resources/pokemon_normal_test/image " << leftImageIndex << " on " << rightImageIndex << " " << matches.size() << " " << maskedMatches.size() << ".jpg";
            std::string imgName = imgNameStream.str();

            cv::imwrite(imgName, canvas);
        }
    }
    std::cout << "Done" << std::endl;

    std::cout << "Building sequence..." << std::endl;

    Scene::Image currentLeft, currentRight;

    currentLeft = createImageFromContainer(images.at(0));
    currentRight = createImageFromContainer(images.at(1));

    Scene::ImagePairSequence sequence;


    
    // sequence.append(Scene::ImagePair(currentLeft,currentRight, ))

}

// vector<size_t> getMatchIndexes(size_t leftImage, size_t rightImage, std::map < size_t, std::map<size_t, size_t> matchMappings) {

// }

void SequenceMatcher::getKeypointIndexes(const ImageContainer& leftImageContainer, const size_t& rightImageIndex, std::vector<size_t>& leftKeypointIndexes, std::vector<size_t> rightKeypointIndexes) {

    for (size_t leftKeypointIndex = 0; leftKeypointIndex < leftImageContainer.keypointMatches.size(); leftKeypointIndex++) {
        // iterate over possible keypoint indexes for the left image, and check 
        if (leftImageContainer.keypointMatches.count(leftKeypointIndex) && leftImageContainer.keypointMatches.at(leftKeypointIndex).count(rightImageIndex)) {
            leftKeypointIndexes.push_back(leftKeypointIndex);
            rightKeypointIndexes.push_back(leftImageContainer.keypointMatches.at(leftKeypointIndex).at(rightImageIndex));
        }
    }
}

Scene::Image SequenceMatcher::createImageFromContainer(ImageContainer container) {
    Scene::Image image;
    image.image = container.image;
    image.name = container.name;

    for (auto& kp : container.keypoints) {
        image.imagePoints.push_back(kp.pt);
    }

    return image;
}
