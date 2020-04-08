#ifndef YAPGT_IMAGE_NODE_H
#define YAPGT_IMAGE_NODE_H

#include "image-pair.hpp"


class ImageNode : ImagePair {
private:
	ImageNode *prevNode;
	ImageNode *nextNode;

	std::string leftImageName;
	std::string rightImageName;

	cv::Mat leftImage;
	cv::Mat rightImage;

	std::vector<Point2f> leftImagePoints;
	std::vector<Point2f> rightImagePoints;
    sdt::vector<unsigned int> overlappingPointIndexes;
		
public:
    friend class LinkedPairList;

    ImageNode(cv::Mat leftImage, cv::Mat rightImage) { }
	~ImageNode();

	const std::vector<Point2f> &getLeftImagePoints(); 
	const std::vector<Point2f> &getRightImagePoints();
    const sdt::vector<unsigned int> &getOverlappingPointIndexes();

    

};



class LinkedPairList {
private:
    ImageNode *head;
    ImageNode *tail;
    
public:

    void append();
    void prepend();

	ImagePairIterator createIterator();
}
#endif // !YAPGT_IMAGE_NODE_H
