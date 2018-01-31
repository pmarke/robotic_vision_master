#include "feature_manager/feature_manager.h"

namespace robotic_vision {

	FeatureManager::FeatureManager()
	{


		// Initialize feature matcher with default type.
		set_matcher(OPTICAL_FLOW_PYR_LK);
	}




	void FeatureManager::find_correspoinding_features(const cv::Mat& img)
	{

		// clear history
		prev_features_.clear();
		matched_features_.clear();

		feature_matcher_->find_correspoinding_features(img, prev_image_, prev_features_, matched_features_, mask_);
	
		calculate_pixel_velocity();

		prev_image_ = img.clone();
	}



	void FeatureManager::set_matcher(enum FeatureMatcherType type){


		if(type == OPTICAL_FLOW_PYR_LK)
		{

			//Retrieve the needed parameters for the cv::OpticalFlowPyrLK
			//get variables
			
			feature_matcher_ = std::make_shared<LK_Matcher>();
		}
		else if(type == BF_MATCHER)
		{
			ROS_WARN("BF_MATCHER not implemented");
		}

		feature_matcher_type_ = type;
	}


	void FeatureManager::calculate_pixel_velocity() {

		pixel_velocity_.clear();

		for (int i = 0; i < prev_features_.size(); i++) {

			pixel_velocity_.push_back(matched_features_[i]-prev_features_[i]);
		
		}


	}



}