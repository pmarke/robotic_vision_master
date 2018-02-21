#include "tracker_manager/meanshift_tracker.h"


namespace robotic_vision {

MeanShiftTracker::MeanShiftTracker() {

	set_parameters();


	if (display_) {
		// Create windows

		cv::namedWindow("MeanShift", CV_WINDOW_NORMAL);
		cv::namedWindow("Mean Shift Init", CV_WINDOW_NORMAL);
		cv::namedWindow("hue roi", CV_WINDOW_NORMAL);
		cv::namedWindow("mask roi", CV_WINDOW_NORMAL);

		cv::resizeWindow("MeanShift", 1680/3, 1050/3);
		cv::resizeWindow("Mean Shift Init", 1680/3, 1050/3);
		
		cv::moveWindow("Mean Shift Init", 1680/3, 0);
		cv::moveWindow("MeanShift",       1680/3, 1050/3);
		cv::moveWindow("hue roi",         1680/3, 2*1050/3);
		cv::moveWindow("mask roi",    1.5*1680/3, 2*1050/3);




	}

	
}

void MeanShiftTracker::set_parameters() {

	bins_ = 30;
	hist_range_[0] = 0;
	hist_range_[1] = 180;
	term_criteria_ = cv::TermCriteria( cv::TermCriteria::EPS | cv::TermCriteria::COUNT, 10, 1 );
	display_ = true;
	roi_width_ = 30;
	roi_height_ = 60;
	drift_threshold_ = 35;

}

cv::Mat MeanShiftTracker::get_gray_image(const cv::Mat img) {

	cv::Mat gray_image;

	// convert image to gray image
	cv::cvtColor(img,gray_image, CV_BGR2GRAY);

	return gray_image;


}

void MeanShiftTracker::back_projection(cv::Mat& img, Target * target) {

	const float* ranges = {hist_range_};

	// Get back projection
	cv::calcBackProject(&img,1,0,target->roi_hist,target->back_projection, &ranges,1,true);
	

}

void MeanShiftTracker::get_features(const cv::Mat& img, std::vector<Tracks> tracks) {


	// clear history
	new_features_.clear();

	// Get gray image
	cv::Mat gray_image = get_gray_image(img);

	// Get new location of each target using mean shift
	for (int i = 0; i < targets_.size(); i++) {

		back_projection(gray_image, &targets_[i]);
		apply_meanshift(&targets_[i]);

		// Find the distance between the feature in the previous image and 
		// the new image
		float norm = sqrt(pow(targets_[i].x - targets_[i].new_pos.x, 2)+
			pow(targets_[i].y - targets_[i].new_pos.y, 2));

		// std::cout << "norm" << norm << std::endl;

		// Push new points onto new features if the norm < drift_threshold_
		if (norm < drift_threshold_)
			new_features_.push_back(targets_[i].new_pos);

	}


	if (display_)
		display(img);

	
}

void MeanShiftTracker::init_targets(const cv::Mat& img, std::vector<Tracks> tracks) {

	// convert image to HSV
	cv::Mat gray_image = get_gray_image(img);

	// cv::imshow("hue image", gray_image);

	// clear history
	targets_.clear();

	// Init a target for every model rransac is tracking
	for (int i = 0; i < tracks.size(); i++) {

		// temp target
		init_target(gray_image, tracks[i]);


	}

	if (display_)
		display_init(img);



}

void MeanShiftTracker::init_target(cv::Mat& img, Tracks track) {


	// Temp target
	Target target;

	// Initialize target
	target.id = track.id;
	target.x = (int)track.x;
	target.y = (int)track.y;


	set_roi(img, &target, &track);


	// hsv region of interest
	cv::Mat image_roi = img(target.roi);


	// Get a mask to indicate a sub region of the roi.
	cv::Mat mask;
	cv::inRange(image_roi, 10, 100, mask);


	// Dialate and erode the mask
	cv::dilate(mask,mask, cv::Mat(), cv::Point(-1,-1),1);	
	cv::erode(mask,mask,cv::Mat(), cv::Point(-1,-1),1);

	if (display_)
		cv::imshow("hue roi", image_roi);
		cv::imshow("mask roi", mask);



	// Create the histogram
	histSize_ = MAX(bins_,2);
	const float* ranges = {hist_range_};

	// Get the Histogram and normalize it
	cv::calcHist(&image_roi, 1,0,mask,target.roi_hist, 1, &histSize_, &ranges, true,false );
	

	cv::normalize(target.roi_hist,target.roi_hist,0,255,cv::NORM_MINMAX);


	targets_.push_back(target);

}

void MeanShiftTracker::set_roi(cv::Mat& img,Target* target, Tracks* track) {


	// Get roi parameters
	int x = (int)track->x -roi_width_/2;
	int y = (int)track->y -roi_height_/2;

	// Make sure roi stays within the image
	if (x < 0) {x = 0;}
	if (y < 0) {y = 0;}
	if (x + roi_width_ > img.cols) {x = img.cols-roi_width_;}
	if (y + roi_height_ > img.rows) {y = img.rows-roi_height_;}

	// set ROI
	target->roi = cv::Rect(x,y, roi_width_, roi_height_ );
}

void MeanShiftTracker::apply_meanshift(Target* target) {

	// Find feature in new image
	cv::CamShift(target->back_projection, target->roi, term_criteria_);

	// Set new position to the new feature
	target->new_pos = cv::Point2f(target->roi.x + target->roi.width/2, target->roi.y+target->roi.height/2);



}


void MeanShiftTracker::display(const cv::Mat img) {

	cv::Mat altered_img = img.clone();


	for (int i = 0; i < targets_.size(); i++) {
		cv::rectangle(altered_img, 
			cv::Point(targets_[i].roi.x,targets_[i].roi.y ),
			cv::Point(targets_[i].roi.x+targets_[i].roi.width,targets_[i].roi.y+targets_[i].roi.height), cv::Scalar(0,0,255),2);
	
		cv::circle(altered_img, 
			cv::Point(targets_[i].roi.x+targets_[i].roi.width/2,targets_[i].roi.y+targets_[i].roi.height/2),
			3,cv::Scalar(255,0,0),2);
	}

	cv::imshow("MeanShift", altered_img);

	cv::waitKey(1);


}

void MeanShiftTracker::display_init(const cv::Mat img) {

	cv::Mat altered_img = img.clone();

	for (int i = 0; i < targets_.size(); i++) {
		cv::rectangle(altered_img, 
			cv::Point(targets_[i].roi.x,targets_[i].roi.y ),
			cv::Point(targets_[i].roi.x+targets_[i].roi.width,targets_[i].roi.y+targets_[i].roi.height), cv::Scalar(0,0,255),2);
	
		cv::circle(altered_img, 
			cv::Point(targets_[i].roi.x+targets_[i].roi.width/2,targets_[i].roi.y+targets_[i].roi.height/2),
			3,cv::Scalar(255,0,0),2);
	}

	cv::imshow("Mean Shift Init", altered_img);

}


}