#include "feature-matching.hpp"

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>

using namespace std;
using namespace cv;

FeatureMatching::FeatureMatching(Calibration cameraCalibration) {
    calibration = cameraCalibration;
}

void FeatureMatching::findMatches(const Mat& image1,
                                  const Mat& image2,
                                  vector<Point2f> &imagePoints1, vector<Point2f> &imagePoints2) {
    Mat processedImage1; 
    Mat processedImage2;

    // apply filters and undistortion
    preprocessImage(image1, processedImage1);
    preprocessImage(image2, processedImage2);

    vector<KeyPoint> keyPoints1;
    vector<KeyPoint> keyPoints2;
    Mat descriptors1;
    Mat descriptors2;
    // parameters: nfeatures=500, scaleFactor=1.2f, nlevels=8, edgeThreshold=31, firstLevel=0, WTA_K=2, scoreType=ORB::HARRIS_SCORE, patchSize=31, fastThreshold=20
    Ptr<ORB> orb = ORB::create(2000, 1.2f, 8, 31, 0, 4, ORB::HARRIS_SCORE, 31, 30);

    // compute keypoints and descriptors using ORB
    orb->detectAndCompute(processedImage1, noArray(), keyPoints1, descriptors1);
    orb->detectAndCompute(processedImage2, noArray(), keyPoints2, descriptors2);

    // use NORM_HAMMING2 when WTA_K = 3 or 4 for ORB
    Ptr<BFMatcher> matcher =  BFMatcher::create(NORM_HAMMING2, true);

    // find matches using bruteforce
    vector<DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);

    // filter matches
    vector<DMatch> filteredMatches;
    filterMatches(keyPoints1, keyPoints2, matches, filteredMatches, image1.size());

    /* // show matches in a new window */
    /* Mat drawnMatches; */
    /* drawMatches(processedImage1, keyPoints1, processedImage2, keyPoints2, filteredMatches, drawnMatches);; */
    /* namedWindow("Matches", WINDOW_FREERATIO); */
    /* imshow("Matches", drawnMatches); */
    /* waitKey(0); */

    // crate index mask for keypoints from the filtered matches
    vector<int> keypointIndexes1;
    vector<int> keypointIndexes2;
    
    for (size_t i=0; i < filteredMatches.size(); i++) {
        keypointIndexes1.push_back(filteredMatches[i].trainIdx);;
        keypointIndexes2.push_back(filteredMatches[i].queryIdx);
    }

    // return masked keypoints as points2f
    KeyPoint::convert(keyPoints1, imagePoints1, keypointIndexes1);
    KeyPoint::convert(keyPoints2, imagePoints2, keypointIndexes2);
    cout << "Found " << imagePoints1.size() << " matching features." << endl;
}


void FeatureMatching::preprocessImage(const Mat& image, Mat& processedImage) {

    // remove distortion from image so that straight lines are straigt
    calibration.undistortImage(image, processedImage);

    // transform to grey color, with only one channel (required by histogramm)
    cvtColor(processedImage, processedImage, COLOR_BGR2GRAY);   

    // apply histogram filter to incease contrast
    equalizeHist(processedImage, processedImage);

    // TODO: apply more filters
}


void FeatureMatching::filterMatches(const vector<KeyPoint>& keypoints1,
                                    const vector<KeyPoint>& keypoints2,
                                    const vector<DMatch>& matches,
                                    vector<DMatch>& filteredMatches,
                                    Size imageSize){
    
    // max distance for matches is 25% of the max distance / image diagonal
    double distanceLimit = 0.25f * sqrt(pow(imageSize.height, 2) + pow(imageSize.width, 2));
    double maxHeightDifference = 0.05 * imageSize.height;
    double minHeightDifference = 100;


    int filterCounter = 0;
    for (size_t index = 0; index < matches.size(); index++) {
        DMatch match = matches[index];
        KeyPoint queryKeyPoint = keypoints1[match.queryIdx];
        KeyPoint trainKeyPoint = keypoints2[match.trainIdx];

        double currentDistance = sqrt(pow(queryKeyPoint.pt.x - trainKeyPoint.pt.x, 2) + pow(queryKeyPoint.pt.y - trainKeyPoint.pt.y, 2));
        double currentHeightDifference = abs(queryKeyPoint.pt.y - trainKeyPoint.pt.y);

        // TODO: use better filters
        // this test is not applicable to other images
        if (currentDistance < distanceLimit && currentHeightDifference < maxHeightDifference && queryKeyPoint.pt.y - trainKeyPoint.pt.y > minHeightDifference) {
            filteredMatches.push_back(match);
        } else {
            filterCounter++;
        }
    }
}
