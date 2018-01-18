#include "lk_matcher"

namespace robotic_vision{

	LK_Matcher::LK_Matcher(){

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
		cv::FAST(img,features,FAST_threshold_, FAST_nonmaxSupression_);
	}


}