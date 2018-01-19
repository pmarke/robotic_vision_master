#pragma once

//libraries
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "feature_manager/feature_manager.h"
#include <iostream>
#include <vector>


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


		// camera parameters
		ros::Time timestamp_frame_;
		cv::Mat camera_matrix_;
		cv::Mat dist_coeff_;
		bool cinfo_received_ = false;  // camera info received once or no

		// cv::VideoCapture
		cv::VideoCapture cap();


		// cv::VideoWriter
		cv::VideoWriter videoWriter_; 
		char videoFileName_[1000];    // = filePath/fileName
		int codec_ = cv::VideoWriter::fourcc('M','J','P','G');                   // fourcc codec

		// frames
		cv::Mat img_;
		cv::Mat alteredImg_;
		cv::Mat grayImg_;
		cv::Mat hsvImg_;


		// user options
		bool publish_video_; // If true, the altered video will be published by ROS
		bool use_cv_imShow_; // If true, the altered video will be displayed by cv::imShow
		float camera_fps_;   // The fps of the camera. Not webcam
		bool use_webcam_;    // If true, use webcam instead of ROS to get video
		bool record_video_;  // If true, record altered vido

		// extensions
		FeatureManager feature_manager_;




		// sub_video callback
		void callback_sub_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo);

		// Publish Video
		void publish_video();

		// openCV imshow
		void displayVideo_imShow();



		
	};
}
