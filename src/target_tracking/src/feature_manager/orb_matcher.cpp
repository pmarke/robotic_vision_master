#include "feature_manager/orb_matcher.h"

namespace robotic_vision {

ORB_Matcher::ORB_Matcher(){

	// Used for display
	display_ = true;
	cv::namedWindow("ORB_Matches", CV_WINDOW_NORMAL);
	cv::resizeWindow("ORB_Matches", 600, 300);

	// Indicate that no image has been received
	first_image_ = true;

	// Create the ORB feature detector and the matcher
	orb_ = cv::ORB::create(100,1.2f,8,31,0,2);
	matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
	// BruteForce (it uses L2 )
	// BruteForce-L1
	// BruteForce-Hamming
	// BruteForce-Hamming(2)
	// FlannBased

}

void ORB_Matcher::find_correspoinding_features(const cv::Mat& img, const cv::Mat& prev_image, std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& matched_features, const cv::Mat& mask){

	// Detect keypoints and descriptons on the new image
	detect_features_ORB(img,new_keypoints_,new_descriptors_);

	// Receiving the first image
	if (first_image_) {


		first_image_ = false;

	}
	else {

		// Find matches between the previous frame and the new frame
		find_matches(prev_features, matched_features);

		if (display_) 
			display(img, prev_image);
	}

	// Copy the keypoints from the new frame to those of the old frame. 
	prev_keypoints_ = new_keypoints_;
	prev_descriptors_ = new_descriptors_;
	

}



void ORB_Matcher::detect_features_ORB(const cv::Mat& img, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors){

	// clear history
	keypoints.clear();

	// get the features and descriptors of an image
	orb_->detectAndCompute(img,cv::noArray(), keypoints, descriptors);
}

void ORB_Matcher::find_matches(std::vector<cv::Point2f>& prev_features, std::vector<cv::Point2f>& matched_features) {

	// clear history
	good_matches_.clear();
	prev_features.clear();
	matched_features.clear();


	// Vector that will hold descriptor matches
	std::vector< cv::DMatch > matches;



	// Find the best 2 matches in the new frame for each previous descriptor
	matcher_->match(prev_descriptors_, new_descriptors_, matches);


	double max_dist = 0; double min_dist = 100;

	 //-- Quick calculation of max and min distances between keypoints
	 for( int i = 0; i < new_descriptors_.rows; i++ )
	 { double dist = matches[i].distance;
	   if( dist < min_dist ) min_dist = dist;
	   if( dist > max_dist ) max_dist = dist;
	 }

	 // Filter out bad discriptors
	 for( int i = 0; i < new_descriptors_.rows; i++ )
	 { if( matches[i].distance <= 2*min_dist )
	    { good_matches_.push_back( matches[i]); }
	 }


	// extract the good points from keypoints
	for (unsigned i = 0; i < good_matches_.size(); i++ ) {

		prev_features.push_back(prev_keypoints_[good_matches_[i].queryIdx].pt);
		matched_features.push_back(new_keypoints_[good_matches_[i].trainIdx].pt);

	}

	// cv::KeyPoint::convert(prev_matched_keypoints,prev_features );
	// cv::KeyPoint::convert(new_matched_keypoints,matched_features );

	


}


// displays the matched key points
void ORB_Matcher::display(const cv::Mat& img, const cv::Mat& prev_image) {

	cv::Mat altered_image;

	cv::drawMatches(prev_image,prev_keypoints_, img, new_keypoints_, good_matches_, altered_image);

	cv::imshow("ORB_Matches", altered_image);
	cv::waitKey(1);

}








}