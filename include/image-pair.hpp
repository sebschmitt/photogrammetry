#ifndef IMAGE_MAPPING_H 
#define IMAGE_MAPPING_H 

#include <opencv2/core.hpp>

#include <string>
#include <vector>

class ImagePairIterator {
public:
	virtual ~ImagePairIterator();
	virtual ImagePair currentItem();
	virtual bool isDone();
	virtual void next();
	virtual void first();
};

// interface definition
class ImagePair {

public:
	/* virtual ~ImagePair() = 0; */
    virtual cv::Mat getLeftImage() = 0;
    virtual cv::Mat getRightImage() = 0;
    virtual std::vector<cv::Point2f> getLeftImagePoints() = 0;
    virtual std::vector<cv::Point2f> getRightImagePoints() = 0;
	virtual cv::Mat getPreviousTransform() = 0;
	virtual cv::Mat getPreviousWorldPoints() = 0;
	virtual std::vector<uchar> getCommonPointsMaks() = 0;
	virtual void setResults(cv::Mat transform, cv::Mat projection, cv::Mat worldPoints) = 0;
	virtual ImagePairIterator createIterator() = 0;
};




#endif
