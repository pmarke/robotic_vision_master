#pragma once

#include <opencv2/opencv.hpp>
#include <iostream>

namespace robotic_vision {

	class ImgDiffTracker {

	public:
		ImgDiffTracker();

		void get_features(const cv::Mat img);

	private:

		// Keeps track if at least one image has been received. 
		bool first_image_;

		// Different images
		cv::Mat gray_image_;
		cv::Mat background_image_;
		cv::Mat diff_image_;

		// blob
		cv::SimpleBlobDetector::Params params_;
		cv::Ptr<cv::SimpleBlobDetector> detector_;
		std::vector<cv::KeyPoint> keypoints_;


		void display_image();




	};




}