#include "frontend.h"



namespace robotic_vision {

Frontend::Frontend()
{
	//create a private node handle for use with param server
	ros::NodeHandle nh_private("~");

	// get parameters
	nh_.param("publish_video", publish_video_, true);
	nh_.param("use_cv_imShow", use_cv_imShow_, false);
	nh_.param("camera_fps", camera_fps_, 30.0f);
	nh_.param("use_webcam", use_webcam_, false);
	nh_.param("record_video", record_video_,false);

	// ROS communication
	image_transport::ImageTransport it(nh_);
	sub_video = it.subscribeCamera("video",10, &Frontend::callback_sub_video, this);
	pub_video = it.advertise("altered_video",10);

	/* Notes need to implement webcam and record video*/
}


void Frontend::callback_sub_video(const sensor_msgs::ImageConstPtr& data, const sensor_msgs::CameraInfoConstPtr& cinfo){

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

	if(publish_video_) publish_video();
	if(use_cv_imShow_) displayVideo_imShow();


}

void Frontend::publish_video(){
	// publish the altered video
	sensor_msgs::ImagePtr msg;
	msg = cv_bridge::CvImage(std_msgs::Header(),sensor_msgs::image_encodings::BGR8, img_).toImageMsg();
	pub_video.publish(msg);

}

void Frontend::displayVideo_imShow(){

	// display the altered video using imShow
	cv::imshow("altered_video", img_);
	cv::waitKey(1);
}



}