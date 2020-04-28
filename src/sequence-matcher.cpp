#include "sequence-matcher.hpp"

#include "image-pair.hpp"
#include "image.hpp"
#include "scene-sequence.hpp"

#include <opencv2/xfeatures2d.hpp>

#include <cassert>
#include <iostream>
#include <string>
#include <vector>
#include <opencv2/imgproc.hpp>


namespace Scene {
    SequenceMatcher::SequenceMatcher(Calibration calibration) {
        this->calibration = calibration;
    }

    Scene::SceneSequence
    SequenceMatcher::generateSequence(std::filesystem::path folderPath, std::filesystem::path matchOutputDirectory) {
        bool outputMatches = std::filesystem::exists(folderPath) && std::filesystem::is_directory(folderPath);
        ImageContainer leftBestMatch, rightBestMatch;

        assert(std::filesystem::exists(folderPath));
        assert(std::filesystem::is_directory(folderPath));


        std::cout << "Loading images" << std::endl;

        std::vector<ImageContainer> images;
        for (const auto &imageEntry : std::filesystem::directory_iterator(folderPath)) {
            ImageContainer image;

            image.name = imageEntry.path().string();

            try {
                std::cout << "Loading " << imageEntry.path().string() << std::endl;

                cv::Mat input = cv::imread(imageEntry.path().string());
                assert(!input.empty());

                resize(input, input, input.size() / IMAGE_DOWNSAMPLE);

                calibration.undistortImage(input, image.image);
            }
            catch (const cv::Exception e) {
                std::cerr << "Failed to open the file " << image.name << ". Error Message: " << e.msg << std::endl;
                continue;
            }

            try {
                cv::Ptr<cv::xfeatures2d::SIFT> featureFinder = cv::xfeatures2d::SIFT::create(0, 3, 0.04, 10, 1.6);
                featureFinder->detectAndCompute(image.image, cv::noArray(), image.keypoints, image.descriptors);
            }
            catch (const cv::Exception e) {
                std::cerr << "Failed to  detect features and compute descriptors in image " << image.name
                          << ". Error message: " << e.msg << ". Continuing without this image." << std::endl;
                continue;
            }

            images.emplace_back(image);
        }

        // only continue with the program, when we have more than two images
        if (images.size() < 2) {
            std::cerr << "Less than 2 images have been loaded. At least 2 images are necessary. Aborting Programm.";
            exit(-1);
        }


        std::cout << "Loading done." << std::endl;
        std::cout << "Matching features..." << std::endl;

        // Use NORM_L1 or NORM_L2, because we are using SIFT
        // enable cross check
        cv::Ptr<cv::BFMatcher> matcher = cv::BFMatcher::create(cv::NORM_L2, true);

        for (size_t leftImageIndex = 0; leftImageIndex < images.size() - 1; leftImageIndex++) {
            ImageContainer &leftImage = images[leftImageIndex];

            size_t rightImageIndex = leftImageIndex + 1;
            ImageContainer &rightImage = images[rightImageIndex];

            std::cout << "Matching Image " << leftImage.name << " on " << rightImage.name << ": ";

            std::vector<cv::Point2f> leftImagePoints, rightImagePoints;
            std::vector<size_t> leftKeypointIndexes, rightKeypointIndexes;

            std::vector<cv::DMatch> matches;

            // enable normal matching
            matcher->match(leftImage.descriptors, rightImage.descriptors, matches);

            // used for normal matcher
            for (auto &match : matches) {
                leftImagePoints.push_back(leftImage.keypoints[match.queryIdx].pt);
                rightImagePoints.push_back(rightImage.keypoints[match.trainIdx].pt);

                leftKeypointIndexes.push_back(match.queryIdx);
                rightKeypointIndexes.push_back(match.trainIdx);
            }

            // create fundamental mask for further constrains
            std::vector<uchar> mask;
            cv::findFundamentalMat(leftImagePoints, rightImagePoints, cv::FM_RANSAC, 1, 0.999, mask);

            std::vector<cv::DMatch> maskedMatches;

            // add masked matches to the keyppoint mappings of both images
            int matchCount = 0;
            for (size_t kpIndex = 0; kpIndex < leftImagePoints.size(); kpIndex++) {
                if (mask[kpIndex]) {
                    leftImage.keypointMatches[leftKeypointIndexes[kpIndex]][rightImageIndex] = rightKeypointIndexes[kpIndex];
                    rightImage.keypointMatches[rightKeypointIndexes[kpIndex]][leftImageIndex] = leftKeypointIndexes[kpIndex];

                    maskedMatches.push_back(matches[kpIndex]);
                    matchCount++;
                }
            }

            std::cout << " found " << matchCount << " matches" << std::endl;

            if (outputMatches) {
                cv::Mat canvas;

                cv::drawMatches(leftImage.image, leftImage.keypoints, rightImage.image, rightImage.keypoints,
                                maskedMatches, canvas);

                std::stringstream imgNameStream;
                imgNameStream << matchOutputDirectory.string() << "/image " << leftImageIndex << " on "
                              << rightImageIndex << " " << matches.size() << " " << maskedMatches.size() << ".jpg";
                std::string imgName = imgNameStream.str();

                cv::imwrite(imgName, canvas);
            }
        }

        std::cout << "Done" << std::endl;
        std::cout << "Building sequence..." << std::endl;

        Scene::SceneSequence sequence = Scene::SceneSequence();

        for (size_t imageIndex = 0; imageIndex < images.size() - 1; imageIndex++) {
            Scene::Image currentLeft, currentRight;

            std::vector<size_t> leftKeypointMatches, rightKeypointMatches;
            currentLeft = createImageFromContainer(images.at(imageIndex));
            currentRight = createImageFromContainer(images.at(imageIndex + 1));

            this->getKeypointIndexes(images.at(imageIndex), imageIndex + 1, leftKeypointMatches, rightKeypointMatches);

            if (imageIndex == 0)
                sequence.append(
                        new Scene::ImagePair(currentLeft, currentRight, leftKeypointMatches, rightKeypointMatches,
                                             calibration));
            else
                sequence.append(
                        new Scene::ImagePair(currentLeft, currentRight, leftKeypointMatches, rightKeypointMatches));
        }

        return sequence;
    }

    void SequenceMatcher::getKeypointIndexes(const ImageContainer &leftImageContainer, const size_t &rightImageIndex,
                                             std::vector<size_t> &leftKeypointIndexes,
                                             std::vector<size_t> &rightKeypointIndexes) {


        // iterate over each map entry and collect all keypoints, who have a match with righImageIndex
        for (auto kpToImg : leftImageContainer.keypointMatches) {
            if (kpToImg.second.count(rightImageIndex) > 0) {
                leftKeypointIndexes.push_back(kpToImg.first);
                rightKeypointIndexes.push_back(kpToImg.second.at(rightImageIndex));
            }
        }
    }

    Scene::Image SequenceMatcher::createImageFromContainer(ImageContainer container) {
        Scene::Image image;
        image.image = container.image;
        image.name = container.name;

        for (auto &kp : container.keypoints) {
            image.imagePoints.push_back(kp.pt);
        }

        return image;
    }
}