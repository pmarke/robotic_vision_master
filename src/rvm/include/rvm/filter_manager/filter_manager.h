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

		void implement_filter(const cv::Mat& img);

		// sets the filter to different ones
		void set_filter(Filter_Type type);

		cv::Mat filteredImg;


	private:

		// if true, gausian noise will be added to the image
		bool add_gaussian_noise_ = false;

		// if true, the filter name will be displayed on the altered image
		bool display_filter_name_ = false;

		// ensures that a filter has been selected before updating parameters
		bool filterSet_ = false;

		// dynamic reconfigure
		dynamic_reconfigure::Server<rvm::filterManagerConfig> server;

		// keeps track of the current filter in use
		Filter_Type filter_type_;

		// image filter
		std::shared_ptr<FilterBase> img_filter_;

		void reconfigureCallback(rvm::filterManagerConfig &config, uint32_t level);


		// draws the filter's name on the image
		void drawFilterNameOnImage();

		// add gaussian noise to the image
		void add_gaussian_noise(cv::Mat& img);

	

	};



}