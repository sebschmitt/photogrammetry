#include "colors.hpp"

Colors::Color Colors::ColorPicker::getAverageColor(const cv::Mat& image, const cv::Point2f& pointInImg, unsigned int radius) {
	assert(image.type() == CV_8UC3);
	assert((image.cols / 2) >= radius);
	assert((image.rows / 2) >= radius);

	// create rectange with keypoint as the center
	cv::Rect roi(pointInImg.x - radius, pointInImg.y - radius, radius * 2, radius * 2);

	// limit the rect to the image size
	roi = roi &cv::Rect(0, 0, image.cols, image.rows);
	cv::Mat imagePart(image, roi);

	unsigned int redCount = 0;
	unsigned int blueCount = 0;
	unsigned int greenCount = 0;
	unsigned int pixelCounter = 0;

	// enumerate all points inside the smaller region
	for (int row = 0; row < imagePart.rows; row++) {
		for (int col = 0; col < imagePart.rows; col++) {

			// ensure current pixel is inside the radius
			// calulate distance from the pixel (col, row) to the circle center (radius, radius)
			if (std::pow(col - radius, 2) + std::pow(row - radius, 2) < std::pow(radius, 2)) {
				cv::Vec3b pixel = imagePart.at<cv::Vec3b>(row, col);
				blueCount += pixel[0];
				greenCount += pixel[1];
				redCount += pixel[2];
				pixelCounter++;
			}
		}
	}
	return Color{ (unsigned char)(redCount / pixelCounter),
				  (unsigned char)(greenCount / pixelCounter),
				  (unsigned char)(blueCount / pixelCounter)};
}

Colors::Color Colors::ColorPicker::getAverageColor(const cv::Mat& image1, const cv::Mat& image2, const cv::Point2f& pointInImg1, const cv::Point& pointInImg2, unsigned int radius) {
	assert(image1.size() == image2.size());

	Colors::Color color1 = Colors::ColorPicker::getAverageColor(image1, pointInImg1, radius);
	Colors::Color color2 = Colors::ColorPicker::getAverageColor(image2, pointInImg2, radius);

	return Color{ (unsigned char) ((color1.Red + color2.Red) / 2),
				  (unsigned char) ((color1.Green + color2.Green) / 2),
				  (unsigned char) ((color1.Blue + color2.Blue) / 2)};
}
