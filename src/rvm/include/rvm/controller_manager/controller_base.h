#pragma once


#include <opencv2/opencv.hpp>


namespace robotic_vision{


	class FilterBase
	{
	public:



		//This method receives and image and states, and calculates next 
		// commands
		virtual void get_commands(const cv::Mat& img, cv::Mat& filteredImg) = 0;
	
		// commands
		virtual float Vx (void) = 0;
		virtual float Vy (void) = 0;
		virtual float yaw_rate (void) = 0;
		virtual float altitude (void) = 0;


	};



}