#pragma once

#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>

// Extensions
#include "feature_manager/feature_manager.h"

namespace robotic_vision {


	class Frontend {

	public:

		Frontend();

	private:

		// Open cv frames
		cv::Mat img_;
		cv::Mat cropped_image_;
		cv::Mat translated_image_;
		cv::Mat rotated_image_;

		bool display_;

		// Extenstions
		FeatureManager feature_manager_;

		// Crops an image according to the roi
		void crop_image(const cv::Mat& img, cv::Mat& cropped_image, cv::Rect roi);

		// rotates the image according to the angle in degrees
		void rotate_image(const cv::Mat& img, cv::Mat& rotated_image, float angle);

		// Init used for visual_odometry
		void visual_odometry_init();

		// implement visual odometry
		void visual_odometry_implement();
	};



}