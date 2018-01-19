#pragma once

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <feature_manager/feature_matcher.h>
#include <feature_manager/lk_matcher.h>
#include <memory>


// Available Feature Matchers/Trackers



/* Uses either cv::OpticalFlowPyrLK or cv::BFMatcher() to get
corresponding features between Image(n-1) and Image(n). These
corresponding features can then be used to calculate the 
Hormography [H(n-1)(n)] between the two images */ 
namespace robotic_vision{

enum FeatureMatcherType{OPTICAL_FLOW_PYR_LK, BF_MATCHER};

class FeatureManager
{
public:
	FeatureManager();

	std::vector<cv::Point2f> prev_features_;    // Features obtained by a feature detection method on old frame
	std::vector<cv::Point2f> matched_features_; // Corresponding features on new frame

	// This method finds the corresponding features in the new frame
	// That were found the in previous frame.
	void find_correspoinding_features(const cv::Mat& img);


	cv::Mat mask_;

private:

	// Feature detector/tracker
	std::shared_ptr<FeatureMatcher> feature_matcher_;

	enum FeatureMatcherType feature_matcher_type_;


	void set_matcher(enum FeatureMatcherType type);


};



}


// Notes
// Implement set parameters to change feature matcher type