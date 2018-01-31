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
	nh_private.param("has_camera_info", has_camera_info_, false);



	// ROS communication
	image_transport::ImageTransport it(nh_);
	pub_video = it.advertise("altered_video",10);
	


	// use webcam else use ros subscriber to get image
	if(use_webcam_)
		using_webcam();
	else{
		if (has_camera_info_)
			sub_video = it.subscribeCamera("video",10, &Frontend::callback_sub_video, this);
		else {

			sub_video_dos = it.subscribe("video", 10, &Frontend::callback_sub_video_dos,this,image_transport::TransportHints("raw"));
		}
	}

	state_sub_ = nh_.subscribe("holodeck/state", 10, &Frontend::callback_state, this);

	command_pub_ = nh_.advertise<ros_holodeck::command>("holodeck/command",10);

	// publish first command to start the holodeck
	command_.pitch = 0;
	command_.roll = 0;
	command_.yaw_rate = 0;
	command_.altitude = -5;

	publish_command();

	// dynamic reconfigure
	// auto f = std::bind(&Frontend::reconfigureCallback, this, std::placeholders::_1, std::placeholders::_2);
	// server.setCallback(f);
}

void Frontend::callback_sub_video_dos(const sensor_msgs::ImageConstPtr& data) {
	try{
		img_ = cv_bridge::toCvCopy(data,sensor_msgs::image_encodings::BGR8)->image;
	}
	catch(cv_bridge::Exception& e){
		ROS_ERROR("cv_bridge execption: %s", e.what());
		return;
	}


	implementExtensions();

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

void Frontend::callback_state(const ros_holodeck::state& data) {

}

void Frontend::publish_command() {

	command_pub_.publish(command_);
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


	// filter_manager_.implement_filter(img_);
	// alteredImg_ = filter_manager_.filteredImg;




	cv::cvtColor(img_, grayImg_, cv::COLOR_BGR2GRAY);

	feature_manager_.find_correspoinding_features(grayImg_);
	drawFeatures();
	calculate_direction();	
	publish_command();

	// user options
	if(publish_video_) publish_video();
	if(use_cv_imShow_) displayVideo_imShow();
}

void Frontend::drawFeatures(){

	// deep copy the image 
	alteredImg_ = img_.clone();

	// add circles to the detected features
	for( int i = 0; i < feature_manager_.matched_features_.size(); i++) {

			cv::arrowedLine(alteredImg_,
			 feature_manager_.prev_features_[i],
			 feature_manager_.matched_features_[i], 
			 cv::Scalar(255,0,0), 3 );
	}

	cv::Point2f pt;

	// calculate_FOE(pt);

	// cv::circle(alteredImg_, pt, 5, cv::Scalar(0,0,255),4);



	
	
}

void Frontend::calculate_FOE(cv::Point2f& pt) {

	Eigen::Matrix<float, 50,2> m;
	Eigen::Matrix<float,50,1> b;
	Eigen::Matrix<float,2,1> result;



	if (feature_manager_.pixel_velocity_.size() >= 50) {

		for (int i = 0; i < feature_manager_.prev_features_.size() && i < 50; i++) {

			m(i,0) = feature_manager_.pixel_velocity_[i].y;

			m(i,1) = feature_manager_.pixel_velocity_[i].x;


			b(i) = feature_manager_.prev_features_[i].x*feature_manager_.pixel_velocity_[i].y 
				  -feature_manager_.prev_features_[i].y*feature_manager_.pixel_velocity_[i].x;


		}


		Eigen::ColPivHouseholderQR<Eigen::Matrix<float,Eigen::Dynamic,Eigen::Dynamic>> temp(m.transpose()*m);
		result = temp.solve(m.transpose()*b);
		pt.x = result(0);
		pt.y = result(1);

	}
	else {
		pt.x = 0;
		pt.y = 0;
	}

	std::cout << pt.x << " " << pt.y << std::endl;


}

void Frontend::calculate_direction() {


	float sum = 0;
	float avg_mag = 0;
	float scale = 1;

	for (int i = 0; i < feature_manager_.pixel_velocity_.size(); i++) {

		if ( abs(feature_manager_.pixel_velocity_[i].x) < 100 ) {

			sum += feature_manager_.pixel_velocity_[i].x;

			avg_mag += abs(feature_manager_.pixel_velocity_[i].x);
		}

	}

	sum = sum/feature_manager_.pixel_velocity_.size();
	avg_mag = avg_mag/feature_manager_.pixel_velocity_.size();

	std::cout << "sum " << sum << std::endl;
	std::cout << "avg_mag " << avg_mag << std::endl;

	// low pass filter
	if (avg_mag > 4) {
		command_.pitch = 0;
		command_.roll = 0.1;
		command_.yaw_rate = 0;
		command_.altitude = -5;
	}
	else if (avg_mag > 2) {
		yaw_rate_ = 0.8*yaw_rate_ + 0.2*sum*scale;

		if (yaw_rate_ > 1)
			yaw_rate_ = 1;
		else if (yaw_rate_ < -1)
			yaw_rate_ = -1;

		command_.pitch = -0.1;
		command_.roll = 0;
		command_.yaw_rate = yaw_rate_;
		command_.altitude = -5;
	}
	else {
		command_.pitch = -0.3;
		command_.roll = 0;
		command_.yaw_rate = 0;
		command_.altitude = -5;
	}





}

// void Frontend::reconfigureCallback(rvm::frontendConfig &config, uint32_t level)
// {
// 	use_feature_manager_ = config.use_feature_manager;
// 	use_filter_manager_ = config.use_filter_manager;
// 	display_features_ = config.display_features;
// }


}