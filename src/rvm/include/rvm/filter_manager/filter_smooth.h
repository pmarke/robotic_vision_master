#pragma once

#include"filter_manager/filter_base.h"
#include<ros/ros.h>
#include<opencv2/opencv>

namespace robotic_vision {

	class FilterSmooth : public FilterBase
	{
	public:
		FilterSmooth();

		// implement the smooth filter
		void implement_filter(const cv::Mat& img, cv::Mat& filteredImg);

		void set_parameters(rvm::filterManagerConfig &config, uint32_t level);

	private:

		struct SmoothFilterParameters
		{
			int smoothtype;  // Type of smoothing: CV_BLUR_NO_SCALE, CV_BLUR, CV_GAUSSIAN, CV_MEDIAN, CV_BILATERAL
			int size1;       // The aperture width: 1,3,5,7
			int size2;       // The aperture height. Ignored by CV_MEDIAN and CV_BILATERAL. Can be set to 0
			double sigma1;   // Sigma for Gaussian parameter
			double sigma2;  // ?
		};

	}
}