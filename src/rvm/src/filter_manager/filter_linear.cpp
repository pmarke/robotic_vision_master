#include"filter_manager/filter_linear.h"

namespace robotic_vision {


	FilterLinear::FilterLinear(){

		int ksize;

		ros::NodeHandle nh("~");

		// init gaussian fitler parameters
		nh.param<int>("gaussian_ksize", ksize, 3);
		nh.param<double>("gaussian_sigmaX", gaussian_parameters_.sigmaX,0.0);
		nh.param<double>("gaussian_sigmaY", gaussian_parameters_.sigmaY,0.0);
		nh.param<int>("borderType", gaussian_parameters_.borderType,0);
		gaussian_parameters_.ksize = cv::Size(ksize,ksize);

		// init bilateral filter parameters
		nh.param<int>("bilateral_d", bilateral_parameters_.d, 9);
		nh.param<double>("bilateral_sigmaColor", bilateral_parameters_.sigmaColor, 75);
		nh.param<double>("bilateral_sigmaSpace", bilateral_parameters_.sigmaSpace, 75);
		nh.param<int>("borderType", bilateral_parameters_.borderType,0);

		// init median filter parameters
		nh.param<int>("median_ksize", median_parameters_.ksize,0);
	}

	void FilterLinear::implement_filter(const cv::Mat& img, cv::Mat& filteredImg)
	{
		// Implement the filter depending on the one chosen
		switch(linear_filter_type_)
		{
			case BILATERAL:
			{
				bilateral_filter(img,filteredImg);
				break;
			}
			case GAUSSIAN:
			{
				gaussian_filter(img, filteredImg);
				break;
			}
			case MEDIAN:
			{
				median_filter(img, filteredImg);
				break;
			}
			default:
			{
				ROS_WARN("FilterLinear: Linear filter type unknown.\n");
				break;
			}
		}
	}


	// update the parameters
	void FilterLinear::set_parameters(rvm::filterManagerConfig &config, uint32_t level)
	{
		// linear filter type
		linear_filter_type_ = static_cast<enum LinearFilter_Type>(config.linear_filter_type);

		switch(linear_filter_type_)
		{
			case BILATERAL:
			{

				// bilateral parameters
				bilateral_parameters_.d = config.bilateral_d;
				bilateral_parameters_.sigmaColor = config.bilateral_sigmaColor;
				bilateral_parameters_.sigmaSpace = config.bilateral_sigmaSpace;
				bilateral_parameters_.borderType = config.borderType;
				break;
			}
			case GAUSSIAN:
			{

				// gaussian parameters
				int ksize = config.gaussian_ksize;
				gaussian_parameters_.ksize = cv::Size(ksize,ksize);
				gaussian_parameters_.sigmaX = config.gaussian_sigmaX;
				gaussian_parameters_.sigmaY = config.gaussian_sigmaY;
				gaussian_parameters_.borderType = config.borderType;
				break;
			}
			case MEDIAN:
			{

				// median parameters
				median_parameters_.ksize = config.median_ksize;
				break;
			}
			default :
			{
				ROS_WARN("FilterLinear: Linear filter type unknown. \n");
			}
		}

	}

	void FilterLinear::gaussian_filter(const cv::Mat& img, cv::Mat& filteredImg){

		cv::GaussianBlur(img,filteredImg,
						gaussian_parameters_.ksize,
						gaussian_parameters_.sigmaX,
						gaussian_parameters_.sigmaY,
						gaussian_parameters_.borderType);

	}

	void FilterLinear::median_filter(const cv::Mat& img, cv::Mat& filteredImg){

		cv::medianBlur(img,filteredImg, median_parameters_.ksize);


	}

	void FilterLinear::bilateral_filter(const cv::Mat& img, cv::Mat& filteredImg)
	{
		cv::bilateralFilter(img, filteredImg,
							 bilateral_parameters_.d,
							 bilateral_parameters_.sigmaColor,
							 bilateral_parameters_.sigmaSpace,
							 bilateral_parameters_.borderType);
	}

}