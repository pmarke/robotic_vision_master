#include "tracker_manager/camshift_tracker.h"


namespace robotic_vision {

CamShiftTracker::CamShiftTracker() {

	hranges_ = {0,180};

	namedWindow("Histogram", CV_WINDOW_NORMAL);
	namedWindow("CamShift", CV_WINDOW_NORMAL);
	cv::resizeWindow("Histogram", 600, 400);
	cv::resizeWindow("CamShift", 600, 400);
	
}

}