#include "frontend.h"



namespace robotic_vision {

Frontend::Frontend()
{
	// create private node
	ros::NodeHandle nh_private("~/test");
	// dynamic_reconfigure::Server<rvm::frontendConfig> server(nh_private);

	// get parameters
	nh_private.param("publish_video", publish_video_, true);
	nh_private.param("use_cv_imShow", use_cv_imShow_, false);
	nh_private.param("camera_fps", camera_fps_, 30.0f);
	nh_private.param("use_webcam", use_webcam_, false);
	nh_private.param("record_video", record_video_,false);

	// dynamic reconfigure parameters
	nh_private.param("use_feature_manager", use_feature_manager_, false);
	nh_private.param("use_filter_manager", use_filter_manager_, false);
	nh_private.param("display_features", display_features_, false);



	// ROS communication
	image_transport::ImageTransport it(nh_);
	pub_video = it.advertise("altered_video",10);
	/* Notes need to implement webcam and record video*/


	// use webcam else use ros subscriber to get image
	if(use_webcam_)
		using_webcam();
	else{
		sub_video = it.subscribeCamera("video",10, &Frontend::callback_sub_video, this);
	}

	// dynamic reconfigure
	// auto f = std::bind(&Frontend::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
	// server.setCallback(f);
}


void Frontend::callback_sub_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo){
	// printf("here \n");
	// get newest image
	try{
		img_ = cv_bridge::toCvCopy(data,sensor_msgs::image_encodings::BGR8)->image;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge execption: %s", e.what());
		return;
	}

	// get camera frame timestamp
	timestamp_frame_ = data->header.stamp;

	// save the camera parameters one time
	if(!cinfo_received_)
	{
		// camera_matrix_ (K) is 3x3
		// dist_coeff_    (D) is a column vector of 4, 5, or 8 elements
	    camera_matrix_ = cv::Mat(              3, 3, CV_64FC1);
    	dist_coeff_    = cv::Mat(cinfo->D.size(), 1, CV_64FC1);

	    // convert rosmsg vectors to cv::Mat
	    for(int i=0; i<9; i++)
	      camera_matrix_.at<double>(i/3, i%3) = cinfo->K[i];

	    for(int i=0; i<cinfo->D.size(); i++)
	      dist_coeff_.at<double>(i, 0) = cinfo->D[i];

	  cinfo_received_ = true;

	}

	implementExtensions();


}

void Frontend::publish_video(){

	std::string image_encoding;

	// get the image encoding for cv_bridge depending on the CV:MAT type
	switch(alteredImg_.type()){
		case 0: // CV_8UC1
		{
			image_encoding = sensor_msgs::image_encodings::MONO8;
			break;
		}
		case 16:// CV_8UC3
		{
			image_encoding = sensor_msgs::image_encodings::BGR8;

			break;
		}
		case 24:
		{
			image_encoding = sensor_msgs::image_encodings::BGRA8;

			break;
		}
		default:
		{
			image_encoding = sensor_msgs::image_encodings::MONO8;

			ROS_WARN_ONCE("Frontend: image type not found. \n");
			break;
		}
	}

	// publish the altered video
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(),image_encoding, alteredImg_).toImageMsg();
	pub_video.publish(msg);

}

void Frontend::displayVideo_imShow(){

	// display the altered video using imShow
	cv::imshow("altered_video", alteredImg_);
	cv::waitKey(1);
}

void Frontend::using_webcam(){

	// try to open the camera
	if(!cap_.open(0))
		ROS_WARN("Frontend: no webcam device found");

	// Camera device found
	else{

		ROS_INFO("Frontend: webcam device found.\n");
		// ROS_INFO("Press any key to terminate.\n");

		// keep grabbing video from the webcam until the user presses a key
		while(cv::waitKey(1) <=0 ){

			// if there is a new image
			if(cap_.read(img_))
				implementExtensions();

			ros::spinOnce();

		}
	}
}

void Frontend::implementExtensions(){


		filter_manager_.implement_filter(img_);
		alteredImg_ = filter_manager_.filteredImg;


		// convert color to gray
		if( ((int) alteredImg_.type()) == 16)
		{
			cv::cvtColor(alteredImg_, grayImg_, cv::COLOR_BGR2GRAY);

			feature_manager_.find_correspoinding_features(grayImg_);
			drawFeatures();
		}

		


	

	// user options
	if(publish_video_) publish_video();
	if(use_cv_imShow_) displayVideo_imShow();
}

void Frontend::drawFeatures(){

	// deep copy the image 
	// alteredImg_ = img_.clone();

	// add circles to the detected features
	for( auto&& pt : feature_manager_.matched_features_){
		cv::circle(alteredImg_,pt, 6, cv::Scalar(255,100,0), 2,0,0);
	}
	
}

// void Frontend::reconfigureCallback(rvm::frontendConfig &config, uint32_t level)
// {
// 	use_feature_manager_ = config.use_feature_manager;
// 	use_filter_manager_ = config.use_filter_manager;
// 	display_features_ = config.display_features;
// }


}