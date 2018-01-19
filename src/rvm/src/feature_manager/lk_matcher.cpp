#include "lk_matcher"

namespace robotic_vision{

	LK_Matcher::LK_Matcher(){

		ros::NodeHandle nh("~");

		// get parameters
		nh.param("gftt_maxCorners", gftt_parameters.maxCorners, 500);
		nh.param("gftt_qualityLevel", gftt_parameters.qualityLevel, 0.05);
		nh.param("gftt_minDistance", gftt_parameters.minDistance, 10);
		nh.param("gftt_blockSize", gftt_parameters.blockSize, 9);
		nh.param("gftt_useHarrisDetector", gftt_parameters.gftt_useHarrisDetector, true);
		nh.param("gftt_k", gftt_parameters.k, 0.05);

		nh.param("fast_threshold", fast_parameters.threshold,100 );
		nh.param("fast_nonmaxSuppression", fast_parameters.nonmaxSuppression, true );
		nh.param("fast_type", fast_parameters.type, cv::FastFeatureDetector::TYPE_9_16);

		nh.param("LK_Feature_Detector_Type", feature_detector_type_, GFTT);

		// Create gftt object with given parameters
		gftt.create(gftt_parameters.maxCorners,gftt_parameters.qualityLevel,gftt_parameters.minDistance,gftt_parameters.blockSize, gftt_parameters.gftt_useHarrisDetector,gftt_parameters.k);
	}

	virtual void LK_Matcher::find_correspoinding_features(const cv::Mat& img, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& new_matched, const cv::Mat& mask){

		// First Image
		if(prev_features_ == NULL)
		{
			// Get features from the image
			(feature_detector_type_ == GFTT) ? detect_features_GFTT(img, prev_features_) : detect_features_FAST(prev_features_);

			prev_image_ = img;
		}
		else // There has been at least one image processed
		{
			cv::calcOpticalFlowPyrLK(prev_image_, img, prev_features_, new_matched, status, err, cv::Size(21,21),3,0,0.0001);
		}
	}

	void LK_Matcher::detect_features_GFTT(const cv::Mat& img, std::vector<cv::Point2f>& features){

		//clear history
		features.clear();

		// get the features
		cv::detect(img,features);
	}


	void LK_Matcher::detect_features_FAST(const cv::Mat& img, std::vector<cv::Point2f>& features){

		//clear history
		features.clear();

		// get the features
		cv::FAST(img,features,fast_parameters_.threshold, fast_parameters_.nonmaxSuppression, fast_parameters_.type);
	}




}