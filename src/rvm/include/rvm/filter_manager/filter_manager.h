#pragma once

#include<ros/ros.h>
#include<opencv2/opencv.hpp>
#include"filter_manager/filter_base.h"
#include"filter_manager/filter_canny.h"
#include"filter_manager/filter_linear.h"
#include<ros/console.h>
#include<stdint.h>

// rqt_reconfigure
#include<rvm/filterManagerConfig.h>
#include <dynamic_reconfigure/server.h>



// Manages the filters so you can switch between different types.
namespace robotic_vision {

	enum Filter_Type{FILTER_CANNY, FILTER_LINEAR};

	class FilterManager
	{
	public:
		FilterManager();

		void implement_filter(const cv::Mat& img, cv::Mat& filteredImg);

		// sets the filter to different ones
		void set_filter(Filter_Type type);


	private:

		// ensures that a filter has been selected before updating parameters
		bool filterSet_ = false;

		// dynamic reconfigure
		dynamic_reconfigure::Server<rvm::filterManagerConfig> server;

		// keeps track of the current filter in use
		Filter_Type filter_type_;

		// image filter
		std::shared_ptr<FilterBase> img_filter_;

		void reconfigureCallback(rvm::filterManagerConfig &config, uint32_t level);

	};



}