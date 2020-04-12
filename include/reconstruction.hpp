#ifndef YAPGT_RECONSTRUCTION_H
#define YAPGT_RECONSTRUCTION_H

#include "calibration.hpp"
#include "iterator.hpp"
#include "image-pair.hpp"

#include <opencv2/core.hpp>




class SceneReconstructor {
	private:
		Calibration calibration;
		cv::Mat getProjection(const cv::Mat &R, const cv::Mat &t, int scale = 1);
		cv::Mat combineToTransformation(const cv::Mat &rotation, const cv::Mat &translation);
		void getFromTransformation(const cv::Mat& transformation, cv::Mat &rotation, cv::Mat& translation);


	public:
		SceneReconstructor(Calibration calibration);
		void reconstructScenes(Iterator<Scene::ImagePair> *pairSequence);
	};
#endif // !YAPGT_RECOnstruction_h
