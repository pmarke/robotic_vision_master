#include"filter_manager/filter_canny.h"


namespace robotic_vision {

	FilterCanny::FilterCanny(){
		ros::NodeHandle nh("~");

		nh.param<double>("canny_threshold_min", canny_parameters.threshold_min, 100);
		nh.param<double>("canny_threshold_max", canny_parameters.threshold_max,200);
		nh.param<int>("canny_apertureSize", canny_parameters.apertureSize,3);
		nh.param<bool>("canny_L2gradient", canny_parameters.L2gradient,false);
	

	}

	void FilterCanny::implement_filter(const cv::Mat& img, cv::Mat& filteredImg){


		// implement canny filter
		cv::Canny(img, filteredImg, 
			canny_parameters.threshold_min, 
			canny_parameters.threshold_max, 
			canny_parameters.apertureSize, 
			canny_parameters.L2gradient);

	}

	void FilterCanny::set_parameters(rvm::filterManagerConfig &config, uint32_t level){


		// update parameters
		canny_parameters.threshold_min = config.canny_threshold_min;
		canny_parameters.threshold_max = config.canny_threshold_max;
		canny_parameters.apertureSize = config.canny_apertureSize;
		canny_parameters.L2gradient = config.canny_L2gradient;

		// verify that the threshold min is less than the max
		if(canny_parameters.threshold_max <= canny_parameters.threshold_min)
			canny_parameters.threshold_min = canny_parameters.threshold_max -1;
	}





}