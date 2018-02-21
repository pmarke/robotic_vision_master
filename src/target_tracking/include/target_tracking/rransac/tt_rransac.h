#pragma once

#include <rransac/tracker.h>
#include <rransac/access_type.h>
#include <rransac/tracks.h>

#include <vector>

#include <opencv2/opencv.hpp>

#include <iostream>

#include <Eigen/Dense>

#include <rransac/open_cv_access.h>

namespace robotic_vision {

	class TTRRansac {

	public:


		std::vector<Tracks> tracks_;


		TTRRansac();

		// Add new measurments to rransac
		void add_measurments(std::vector<cv::Point2f>& pos, std::vector<cv::Point2f> vel, int source );

		// Add new measurments to rransac
		void add_measurments(std::vector<cv::Point2f>& pos, int source );

		// Run rransac and update model
		void run_tracker();

		// Bring all previous data into the current frame. 
		// void apply_transformation(Eigen::Projective2d& T);

		void draw_tracks(const cv::Mat img);

		void set_parameters();



	private:
		rransac::Tracker tracker_;

		rransac::core::Parameters params_;

		std::vector<rransac::core::ModelPtr> models_;

		std::vector<cv::Scalar> colors_;
	 	double text_scale_;	




	};


}