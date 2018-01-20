#include"filter_manager/filter_manager.h"


namespace robotic_vision {

	FilterManager::FilterManager(){


		auto f = std::bind(&FilterManager::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
		server.setCallback(f);
		// set the filter
		set_filter(FILTER_CANNY);
	};


	void FilterManager::implement_filter(const cv::Mat& img, cv::Mat& filteredImg){


		img_filter_->implement_filter(img,filteredImg);



	}

	void FilterManager::set_filter(Filter_Type type){


		switch(type)
		{
			case FILTER_CANNY :
			{
				img_filter_ = std::make_shared<FilterCanny>();
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

		if(filterSet_) img_filter_->set_parameters(config, level);

	}
		
	
}