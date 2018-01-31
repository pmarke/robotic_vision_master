#pragma once

//libraries
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include "feature_manager/feature_manager.h"
#include "filter_manager/filter_manager.h"
#include <ros_holodeck/command.h>
#include <ros_holodeck/state.h>
// #include "rvm/frontendConfig.h"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <stdlib.h> 


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
		ros::Subscriber state_sub_;
		ros::Publisher command_pub_;

		// controller
		ros_holodeck::state state_;
		ros_holodeck::command command_;
		float yaw_rate_ = 0;



		// camera parameters
		ros::Time timestamp_frame_;
		cv::Mat camera_matrix_;
		cv::Mat dist_coeff_;
		bool cinfo_received_ = false;  // camera info received once or no

		// cv::VideoCapture
		cv::VideoCapture cap_;


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
		FilterManager filter_manager_;

		// dynamic reconfigure
		// dynamic_reconfigure::Server<rvm::frontendConfig> server_;
		bool use_feature_manager_; // if true, use feature manager
		bool use_filter_manager_;  // if true, use filter manager
		bool display_features_;    // if true, features will be displayed
		bool has_camera_info_;
		// void reconfigureCallback(rvm::frontendConfig &config, uint32_t level);

		// sub_video callback if camera info is provided
		void callback_sub_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo);

		// video callback when no camera info is provided
		void callback_sub_video_dos(const sensor_msgs::ImageConstPtr& data);

		// get states from holodeck
		void callback_state(const ros_holodeck::state& data);

		// publish commands to holodeck
		void publish_command();

		// Publish Video
		void publish_video();

		// openCV imshow
		void displayVideo_imShow();

		// use webcam instead of ros subscribe to get video
		void using_webcam();

		// called to implement extensions such as FilterManager, FeatureManager, etc
		void implementExtensions();

		// draws the features onto the image.
		void drawFeatures();

		void calculate_FOE(cv::Point2f& pt);

		void calculate_direction();

		
	};
}
