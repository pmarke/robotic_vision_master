#pragma once

#include<opencv2/opencv.hpp>
#include "base_feature_matcher.h" 


namespace robotic_vision {

	

	class ORB_Matcher : public BaseFeatureMatcher 
	{
	public:
		ORB_Matcher();

		void find_correspoinding_features(const cv::Mat& img, const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& matched_features, const cv::Mat& mask);

	private:


		std::vector<cv::KeyPoint> prev_keypoints_;
		std::vector<cv::KeyPoint> new_keypoints_;

		cv::Mat prev_descriptors_;
		cv::Mat new_descriptors_;

		// Vector that will hold descriptor good matches
		std::vector< cv::DMatch > good_matches_;

	
		cv::Ptr<cv::ORB> orb_;
		cv::Ptr<cv::DescriptorMatcher> matcher_;
		const double nn_match_ratio_ = 0.8f;    // Nearest-neighbour matching ratio

		bool first_image_;

		// Display 
		bool display_; // If true, the image will be displayed
		
		
		// find features using ORB algorithm
		void detect_features_ORB(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints,  cv::Mat& descriptors);
	
		// find matches between prev and new keypoints and descriptors
		void find_matches(std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& matched_features);

		// displays the matched key points
		void display(const cv::Mat& img, const cv::Mat& prev_image);
	

	};
}