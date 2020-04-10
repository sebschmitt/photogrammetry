#ifndef YAPGT_RECONSTRUCTION_H
#define YAPGT_RECONSTRUCTION_H

#include "calibration.hpp"

#include <opencv2/core.hpp>

class SceneReconstructor {
private:
	Calibration calibration;
	cv::Mat getProjection(cv::Mat R, cv::Mat t, int scale = 1);


public:
	SceneReconstructor(Calibration calibration);
	void reconstructScenes();
};
#endif // !YAPGT_RECOnstruction_h
