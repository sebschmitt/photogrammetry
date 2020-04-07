#ifndef YAPGT_IMAGE_NODE_H
#define YAPGT_IMAGE_NODE_H

#include "image-pair.hpp"

class ImageNode : ImagePair {
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
#endif // !YAPGT_IMAGE_NODE_H
