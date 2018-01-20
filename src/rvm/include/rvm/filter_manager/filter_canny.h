#pragma once

#include<opencv2/opencv.hpp>
#include<ros/ros.h>
#include"filter_manager/filter_base.h"

namespace robotic_vision {

	class FilterCanny : public FilterBase
	{
	public:
		FilterCanny();

		// implement the canny filter
		void implement_filter(const cv::Mat& img, cv::Mat& filteredImg);

		void set_parameters(rvm::filterManagerConfig &config, uint32_t level);

	private:

		struct CannyFilterParameters{
			double threshold_min;       // min threshold value for Hysteresis Thresholding
			double threshold_max;       // max threshold value for Hysteresis Thresholding
			int apertureSize=3;         // aperture size for the Sobel() operator
			bool L2gradient = false;    // a flag to indicate wheter to use L2 norm or L1 to find gradient intensity.
		} canny_parameters;
	};


}

/* The canny filter is a multi state algorithm
	1. Noise Reduction: remove noise in the image with 5x5 Gaussin filter
	2. Find Intensity Gradient of the Image: Filtered using Sobel Kernel both hoz and vert
											 Take the norm of the two to get the edge gradient
	3. Non-Maximum Suppression: Supress non local maximums
	4. Hysteresis Thresholding: Determines what are edges. */
