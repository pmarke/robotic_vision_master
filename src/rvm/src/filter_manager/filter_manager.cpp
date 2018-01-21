#include"filter_manager/filter_manager.h"


namespace robotic_vision {

	FilterManager::FilterManager(){


		auto f = std::bind(&FilterManager::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
		server.setCallback(f);
		// set the filter
		set_filter(FILTER_LINEAR);
	};


	void FilterManager::implement_filter(const cv::Mat& img){


		filteredImg.release();

		// add gaussian noise to image;
		if(add_gaussian_noise_)
		{
			cv::Mat img2 = img.clone();
			add_gaussian_noise(img2);

			// implement filter
			img_filter_->implement_filter(img2,filteredImg);
		}
		else{
			// implement filter
			img_filter_->implement_filter(img,filteredImg);
		}

		if(display_filter_name_)
			drawFilterNameOnImage();



	}

	void FilterManager::set_filter(Filter_Type type){


		// change the filter pointer to the right object
		switch(type)
		{
			case FILTER_CANNY :
			{
				img_filter_ = std::make_shared<FilterCanny>();
				
				ROS_INFO("Setting filter to Canny Filter. \n");

				break;
			}
			case FILTER_LINEAR:
			{
				img_filter_  = std::make_shared<FilterLinear>();

				ROS_INFO("Setting filter to Linear Filter, \n");
				break;
			}
			default :
			{
				img_filter_ = std::make_shared<FilterCanny>();
				ROS_WARN("FILTER_MANAGER: Filter type unknown. Using Filter_Canny\n");
				break;
			}
		}

		if(!filterSet_) filterSet_ = true;
	}

	void FilterManager::reconfigureCallback(rvm::filterManagerConfig &config, uint32_t level){

		// see if the filter should display a name. 
		display_filter_name_ = config.display_filter_name;

		// if it has changed, either add or remove noise and 
		// create or destory the opencv window
		if(add_gaussian_noise_ != config.add_gaussian_noise)
		{
			add_gaussian_noise_ = config.add_gaussian_noise;

			// if noise is being added create the window
			if(add_gaussian_noise_)
			{
				cv::namedWindow( "image with noise", cv::WINDOW_NORMAL);
				cv::moveWindow("image with noise", 45, 580);
				cv::resizeWindow("image with noise", 600, 500);
			}
			
			// else destroy the winidow
			else
				cv::destroyWindow("image with noise");

		}



		// make sure the filter is set
		if(!filterSet_) 
			set_filter(static_cast<enum Filter_Type>(config.filter_type));
		else{
			// If the type of filter has changed, change the pointer
			if(config.filter_type != filter_type_)
				set_filter(static_cast<enum Filter_Type>(config.filter_type));
			
			// Set the filter's parameters
			img_filter_->set_parameters(config, level);
		}
			

	}

	void FilterManager::drawFilterNameOnImage()
	{
		std::string filterName;

		if(filter_type_ == FILTER_CANNY)
		{
			filterName = "Canny Edge Detector";
		}
		else{
			if(img_filter_->get_filter_type() == BILATERAL)
				filterName = "Bilateral Filter";
			else if(img_filter_->get_filter_type() == GAUSSIAN)
				filterName = "Gaussian Filter";
			else
				filterName = "Median Filter";
		}
		cv::rectangle(filteredImg, cv::Point2f(0,0),cv::Point2f(400,50), cv::Scalar(255,229,204), -1);
		cv::putText(filteredImg, filterName, cv::Point2f(10,30),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0,0,0),2,8);
	}

	void FilterManager::add_gaussian_noise(cv::Mat& img){

		cv::Mat gaussianNoise(img.size(),img.type());
		cv::Scalar mean = cv::Scalar::all(0);
		cv::Scalar stddev = cv::Scalar(1,5,50);

		cv::randn(gaussianNoise,mean,stddev);

		img += gaussianNoise;

		cv::imshow("image with noise", img);
		cv::waitKey(1);
	}
		
	
}