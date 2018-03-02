#include "frontend.h"

namespace robotic_vision {

Frontend::Frontend() {

	display_ = true;

	visual_odometry_init();

	Frontend::visual_odometry_implement();



}

void Frontend::crop_image(const cv::Mat& img, cv::Mat& cropped_image, cv::Rect roi) {

	cropped_image = cv::Mat(img, roi);

}

void Frontend::rotate_image(const cv::Mat& img, cv::Mat& rotated_image, float angle) {

	// get size of image
	cv::Size s = img.size();

	// Construct rotation matrix
	cv::Mat r = cv::getRotationMatrix2D(cv::Point2f(s.width,s.height)/2, angle,1);

	// Construct rotated image
	cv::warpAffine(img, rotated_image, r, s); 

}

void Frontend::visual_odometry_init() {

	// Image file path
	std::string HOME = getenv("HOME");
	std::string source = HOME+"/projects/robotic_vision_master/src/play_ground/images/darth_revan.jpg";
	std::cout << "Image source path: " << source << std::endl;

	// Read in image
	img_ = cv::imread(source);

	// Select the ROIs
	cv::Size s = img_.size();
	std::cout << "Size of image: " << s << std::endl;

	cv::Rect roi_1, roi_2;
	float scale = 0.95;
	if (s.height*scale > s.width) {  // Image is landscape view

		float dim = s.width*scale;
		roi_1 = cv::Rect(0,0,dim,dim);
		roi_2 = cv::Rect(s.width*(1-scale), s.width*(1-scale),dim,dim);
	}
	else  {                         // Image is portrait view

		float dim = s.height*0.8;
		roi_1 = cv::Rect(0,0,dim,dim);
		roi_2 = cv::Rect(s.height*(1-scale), s.height*(1-scale),dim,dim);
	}

	// Get cropped image, translated image, and rotated image
	crop_image(img_, cropped_image_, roi_1);
	crop_image(img_, translated_image_, roi_2);
	rotate_image(cropped_image_, rotated_image_, 10);

	// Display the images
	if (display_) {

		cv::imshow("Original Image", img_);
		cv::imshow("Cropped Image", cropped_image_);
		cv::imshow("Translated Image", translated_image_);
		cv::imshow("Rotated Image", rotated_image_);
		// cv::waitKey(0);


	}



}

void Frontend::visual_odometry_implement() {

	feature_manager_.find_correspoinding_features(cropped_image_);
	feature_manager_.find_correspoinding_features(rotated_image_);

	// rotation and translation
	cv::Mat R,T, E;
	E = cv::findEssentialMat(feature_manager_.prev_features_, feature_manager_.matched_features_);

	int check = cv::recoverPose(E, feature_manager_.prev_features_, feature_manager_.matched_features_, R, T);

	std::cout << "R: " << R << std::endl;
	std::cout << "T: " << T << std::endl;

	cv::waitKey(0);


}








}