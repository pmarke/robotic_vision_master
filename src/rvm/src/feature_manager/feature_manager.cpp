#include "feature_manager.h"

namespace robotic_vision {

	FeatureManager::FeatureManager();
	{

		// Initialize feature matcher with default type.
		set_matcher(OPTICAL_FLOW_PRY_LK);
	}




	void FeatureManager::find_correspoinding_features(cv::Mat& frame){

		// clear history
		prev_features_.clear();
		new_matched_.clear();

		feature_matcher_->find_correspoinding_features(img, prev_features_, new_matched_);
	}



	void FeatureManager::set_matcher(enum FeatureMatcherType type){


		// Create an instance of a private node handle
		ros::NodeHandle nh("~");

		if(type == OPTICAL_FLOW_PYR_LK)
		{

			//Retrieve the needed parameters for the cv::OpticalFlowPyrLK
			get variables

			feature_matcher_ = std::make_shared<LKMATCHER>();
		}
		else if(type == BF_MATCHER)
		{
			ROS_WARN("BF_MATCHER not implemented");
		}

		feature_matcher_type_ = type;
	}


}