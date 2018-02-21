#pragma once

#include <opencv2/opencv.hpp>
#include <rransac/tt_rransac.h>
#include <rransac/tracks.h>
#include <iostream>
#include <vector>


namespace robotic_vision {


	class MeanShiftTracker {

	public:

		MeanShiftTracker();

		void get_features(const cv::Mat& img, std::vector<Tracks> tracks);

		void init_targets(const cv::Mat& img, std::vector<Tracks> tracks);

		std::vector<cv::Point2f> new_features_;

	private:

		// A target that is being tracked
		struct Target {

			int id;  // ID of the target

			// center of the target being tracked
			int x,y;

			// region of interest
			cv::Rect roi;

			// Target's histogram
			cv::Mat roi_hist;

			// Targets back projection
			cv::Mat back_projection;

			// new position
			cv::Point2f new_pos;

		};

		// Parameters
		int bins_;                       // Num of bins in histogram
		int histSize_;                   // Size of histogram
		float hist_range_[2];            // Hist range
		bool display_;			         // If true, images will be displayed
		cv::TermCriteria term_criteria_; // Term criteria for cam/mean shift
		int roi_width_;                  // ROI Width
		int roi_height_;                 // ROI Height
		double drift_threshold_;         // If the new feature is farther than drift_threshold_ delete it



		// Histogram 
		cv::Mat hist_;





		std::vector<Target> targets_;

		// get the back projection of an image
		void back_projection(cv::Mat& img, Target * target);

		// display the found feature
		void display(const cv::Mat img);

		// disply where the targets are initiated
		void display_init(const cv::Mat img);

		// Inits a target
		void init_target(cv::Mat& img, Tracks track );

		// set the region of interest for a target
		void set_roi(cv::Mat& img, Target* target, Tracks* track);

		// Apply mean shift to find the target in the new image
		void apply_meanshift(Target* target);

		// Returns a filtered gray image
		cv::Mat get_gray_image(const cv::Mat img);

		// Set parameters
		void set_parameters();


	};




}

/* MeanShift

RotatedRect MeanShift(Input Array probImage, Rect& window, TermCriteria criteria)

probImage - Back projection of the object histogram. See calcBackProject()
window - initial search window
criteria - stop criteria for the underlying meanShift()


Back Projection: a way of recording how well the pixels of a given image 
fit the distribution of pixels in a histogram model.*/