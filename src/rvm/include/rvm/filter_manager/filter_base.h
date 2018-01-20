#pragma once

#include"rvm/filterManagerConfig.h"
#include<opencv2/opencv.hpp>


namespace robotic_vision{

	class FilterBase
	{
	public:

		//This method receives and image, applies a filter to it, and 
		// returns the filtered image.
		virtual void implement_filter(const cv::Mat& img, cv::Mat& filteredImg) = 0;
	

		// Used to update filter parameters
		virtual void set_parameters(rvm::filterManagerConfig &config, uint32_t level)=0;
	};



}