#pragma once

#include <opencv2/opencv.hpp>


namespace robotic_vision {


	class CamShiftTracker {

	public:

		CamShiftTracker();

	private:

		cv::Rect trackWindow_;

		int hsize_;
		float hranges_[2];
		const float* phranges_ = hranges_;



	};




}

/* CamShift

RotatedRect CamShift(Input Array probImage, Rect& window, TermCriteria criteria)

probImage - Back projection of the object histogram. See calcBackProject()
window - initial search window
criteria - stop criteria for the underlying meanShift()