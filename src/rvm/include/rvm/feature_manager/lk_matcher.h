#pragma once

#include<opencv2/opencv.hpp>
#include "feature_matcher.h" 

namespace robotic_vision{

	enum Feature_Detector_Type ={GFTT FAST};

	class LK_Matcher : FeatureMatcher 
	{
	public:
		LK_Matcher();

		virtual void find_correspoinding_features(const cv::Mat& img, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_matched, const cv::Mat& mask);

	private:
		std::vector<cv::Point2f> prev_features_;


		// gftt algorithm
		cv::Ptr<cv::GFTTDetector> gftt;
		struct Gftt_Parameters{
			int maxCorners;
			double qualityLevel;
			double minDistance;
			double blockSize;
			bool useHarrisDetector;
			double k;
		}gftt_parameters;

		// fast parameters
		

		// find features using GFTT algorithm
		void detect_features_GFTT(const cv::Mat& img, std::vector<cv::Point2f>& features);

		// find features using FAST algorithm
		void detect_features_FAST(const cv::Mat& img, std::vector<cv::Point2f>& features);
	};
}