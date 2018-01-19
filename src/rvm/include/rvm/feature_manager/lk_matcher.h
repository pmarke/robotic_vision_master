#pragma once

#include<opencv2/opencv.hpp>
#include "feature_matcher.h" 

namespace robotic_vision{

	enum LK_Feature_Detector_Type ={GFTT FAST};

	class LK_Matcher : FeatureMatcher 
	{
	public:
		LK_Matcher();

		virtual void find_correspoinding_features(const cv::Mat& img, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_matched, const cv::Mat& mask);

	private:
		std::vector<cv::Point2f> prev_features_ = NULL;

		LK_Feature_Detector_Type feature_detector_type_;

		cv::Mat prev_image_;


		// gftt algorithm
		cv::Ptr<cv::GFTTDetector> gftt;
		struct GFTT_Parameters{
			int maxCorners;          // Max number of features that will be detected
			double qualityLevel;     // Minimal accepted quality of image corners
			double minDistance;      // Minimum possible Euclidean distance between the returned corners
			double blockSize;        // Neighborhood size: block_size X block_size (pixels)
			bool useHarrisDetector;  // Harris detector free parameter
			double k;
		}gftt_parameters_;

		// fast parameters
		struct FAST_Parameters{
			int threshold;           // Threshold difference between intensity of the central pixel and the pixels of a circle around this pixel [0,255]
			bool nonmaxSuppression;  // If true, non-maximum suppression is applied to keypoints (features)
			int type;                // One of the three neighbors: FastFeatureDetector::TYPE_9_16, FastFeatureDetector::TYPE_7_12, FastFeatureDetector::TYPE_5_8
		} fast_parameters_;
		

		// find features using GFTT algorithm
		void detect_features_GFTT(const cv::Mat& img, std::vector<cv::Point2f>& features);

		// find features using FAST algorithm
		void detect_features_FAST(const cv::Mat& img, std::vector<cv::Point2f>& features);
	};
}