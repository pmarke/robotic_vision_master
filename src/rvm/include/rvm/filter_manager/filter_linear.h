#pragma once

#include<ros/ros.h>
#include<ros/console.h>
#include"filter_manager/filter_base.h"
#include<opencv2/opencv.hpp>

namespace robotic_vision {

	enum LinearFilter_Type{BILATERAL, GAUSSIAN, MEDIAN};

	// This class allows the user to implement various filters:
	// bilateralFilter, Gaussian Filter, and the Median Filter
	// 
	class FilterLinear : public FilterBase 
	{
	public:
		FilterLinear();

		//implement a filter assortment
		void implement_filter(const cv::Mat& img, cv::Mat& filteredImg);
		
		void set_parameters(rvm::filterManagerConfig &config, uint32_t level);

	private:

		LinearFilter_Type linear_filter_type_ = BILATERAL;

		// gaussian parameters
		struct GaussianParameters
		{
			cv::Size ksize;  // Gaussian kernel size. If zero, dimensions will be computed from sigma
			double sigmaX;   // Gaussian kernel standard deviation in X direction
			double sigmaY;   // Gaussian kernel standard deviation in Y direction. If 0, sigmaY = sigmaX
			int borderType;  // Method to pad the image
		} gaussian_parameters_;


		// bilateral parameters
		struct BilateralParameters   //h(x) = k^-1 $$f(x0)c(x0 - xn)s(f(x0) - f(xn))
		{
			int d;                // Diameter of each pixel neighborhood that is used during filtering
			double sigmaColor;    // Filter sigma weight in the color space
			double sigmaSpace;    // Filter sigma weight in the coordinate space
			int borderType;       // Method to pad the image
		} bilateral_parameters_;


		// medianBlur parameters
		struct MedianBlurParameters
		{
			int ksize;       // The size of the kernel. Must be odd and greater than 1
		} median_parameters_;

		// Implements a bilateral filter on the image
		// It tries to eliminate noise while preserving edges. 
		// Instead of using distance as the weight, it uses difference in light intensity and pixel distance as the weight.
		void bilateral_filter(const cv::Mat& img, cv::Mat& filteredImg);

		// Implements a gausssian filter on the image
		// useful in elimiating noise
		// uses pixel distance as the weight
		void gaussian_filter(const cv::Mat& img, cv::Mat& filteredImg);

		// Implements a median filter on the image
		void median_filter(const cv::Mat& img, cv::Mat& filteredImg);

	};

}