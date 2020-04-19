#ifndef YAPGT_COLORS_H
#define YAPGT_COLORS_H

#include <opencv2/core.hpp>

namespace Colors {
	struct Color {
		unsigned char Red;
		unsigned char Green;
		unsigned char Blue;
	};


	class ColorPicker {
	private:
		static Color getAverageColor(const cv::Mat& image, const cv::Point2f& pointInImg, unsigned int radius);

	public:
		static Color getAverageColor(const cv::Mat& image1, const cv::Mat& image2, const cv::Point2f& pointInImg1, const cv::Point& pointInImg2, unsigned int radius); 
	};

}

#endif // !YAPGT_COLORS_H
