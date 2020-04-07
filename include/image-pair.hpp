#ifndef IMAGE_MAPPING_H 
#define IMAGE_MAPPING_H 

#include <opencv2/core.hpp>

#include <string>
#include <vector>


// interface definition
class ImagePair {

public:
	virtual cv::Mat getPreviousTransform();
	virtual cv::Mat getPreviousWorldPoints();
	virtual std::vector<uchar> getCommonPointsMaks();

	virtual void setResults(cv::Mat transform, cv::Mat projection, cv::Mat worldPoints);
	virtual ~ImagePair();
	virtual ImagePairIterator createIterator();
};


class ImagePairIterator {
public:
	virtual ~ImagePairIterator();
	virtual ImagePair currentItem();
	virtual bool isDone();
	virtual void next();
	virtual void first();
};


#endif
