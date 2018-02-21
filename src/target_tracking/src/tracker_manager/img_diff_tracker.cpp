#include "tracker_manager/img_diff_tracker.h"

namespace robotic_vision {

ImgDiffTracker::ImgDiffTracker() {


	display_ = true;
	cv::namedWindow("Difference Image", CV_WINDOW_NORMAL);
	cv::resizeWindow("Difference Image", 1680/3, 1050/3);
    cv::moveWindow("Difference Image",   2*1680/3, 0);



	first_image_ = true;

	// Set up blob detector

	// Change thresholds
	params_.minThreshold = 10;
	params_.maxThreshold = 255;

	// Filter by color
	params_.filterByColor = true;
	params_.blobColor = 255;
	 
	// Filter by Area.
	params_.filterByArea = true;
	params_.minArea = 100;
	 
	// Filter by Circularity
	params_.filterByCircularity = false;
	params_.minCircularity = 0.1;
	 
	// Filter by Convexity
	params_.filterByConvexity = false;
	params_.minConvexity = 0.87;
	 
	// Filter by Inertia
	params_.filterByInertia = false;
	params_.minInertiaRatio = 0.01;

	// create detector
	detector_ = cv::SimpleBlobDetector::create(params_);


}

void ImgDiffTracker::get_features(const cv::Mat img) {

	// convert image to grayscale
	gray_image_ = img.clone();

	// smooth the image
	// cv::medianBlur(gray_image_, gray_image_, 7);
	cv::GaussianBlur(gray_image_, gray_image_, cv::Size(7,7), 1.5);



	if (first_image_) {


		first_image_ = false;

		// Make a deep copy of the image. 
		background_image_ = gray_image_.clone();
	}
	else {

		
		// Compute the abs diff with saturation between the images
		cv::absdiff(gray_image_, background_image_, diff_image_);

		// Threshold the image
		cv::threshold(diff_image_,diff_image_, 40, 255, cv::THRESH_BINARY);

		// Dialate the image
		cv::dilate(diff_image_,diff_image_, cv::Mat(), cv::Point(-1,-1),4);

		// erode the image
		cv::erode(diff_image_,diff_image_,cv::Mat(), cv::Point(-1,-1),5);

		// detect key points
		detector_->detect(diff_image_,keypoints_);

		// Clean history and Extract features from key points
		new_features_.clear();
		cv::KeyPoint::convert(keypoints_,new_features_);


		// UPdate background_image
		background_image_ = gray_image_.clone();

		// Display the image
		if (display_)
			display_image();


	}



}


void ImgDiffTracker::display_image() {


	// draw key points. 
	cv::drawKeypoints( diff_image_, keypoints_, diff_image_, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS );

	// display the diff_image_
	cv::imshow("Difference Image", diff_image_);
	cv::waitKey(1);
}

}