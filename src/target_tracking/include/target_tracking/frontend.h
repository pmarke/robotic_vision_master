#pragma once

// std libraries
#include <iostream>
#include <string>
#include <vector>
#include <stdlib.h> 

// ROS libraries
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

// Other libraries
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <rransac/tracker.h>


// Extensions
#include "tracker_manager/img_diff_tracker.h"
#include "feature_manager/feature_manager.h"


namespace robotic_vision {

	class Frontend
	{
	public: 
		Frontend();

	private:
		// ROS
		ros::NodeHandle nh_;
		image_transport::CameraSubscriber sub_video;
		image_transport::Publisher pub_video;
		image_transport::Subscriber sub_video_dos;
		std::string image_transport_hint_;

		// camera parameters
		ros::Time timestamp_frame_;
		cv::Mat camera_matrix_;
		cv::Mat dist_coeff_;
		bool cinfo_received_ = false;  // camera info received once or no

		// cv::VideoCapture
		cv::VideoCapture cap_;

		// frames
		cv::Mat img_;
		cv::Mat alteredImg_;
		cv::Mat grayImg_;
		cv::Mat hsvImg_;

		// user options
		bool publish_video_; // If true, the altered video will be published by ROS
		bool use_cv_imShow_; // If true, the altered video will be displayed by cv::imShow
		bool use_webcam_;    // If true, use webcam instead of ROS to get video
		bool has_camera_info_;

		// RRANSAC
		rransac::Tracker tracker;

		// extensions
		ImgDiffTracker img_diff_tracker_;
		FeatureManager feature_manager_;

		// Used to indicate how many images to throw away
		unsigned img_count_;
		unsigned img_use_; // Throw out img_use_ out of img_use_+1 images


		// sub_video callback if camera info is provided
		void callback_sub_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo);

		// video callback when no camera info is provided
		void callback_sub_video_dos(const sensor_msgs::ImageConstPtr& data);

		// Publish Video
		void publish_video();

		// use webcam instead of ros subscribe to get video
		void using_webcam();

		// called to implement extensions such as FilterManager, FeatureManager, etc
		void implementExtensions();

		
	};
}
