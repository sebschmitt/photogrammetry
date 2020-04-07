#ifndef IMAGE_MAPPING_H 
#define IMAGE_MAPPING_H 

#include <opencv2/core.hpp>

#include <string>
#include <vector>


class ImagePair {
private:
	
	ImagePair *prevPair;
	ImagePair *nextPair;

	std::string leftImageName;
	std::string rightImageName;
	cv::Mat leftImage;
	cv::Mat rightImage;

	std::vector<Point2f> leftImagePoints;
	std::vector<Point2f> rightImagePoints;


	cv::Mat projection;
	cv::Mat worldPoints;

	std::vector<uchar> mask;
		
public:
	void setCameraTransform(cv::Mat transform);
	void setProjectionMat(cv::Mat projection);
	void setWorldPoints(cv::Mat worldPoints);
		

};

#endif
