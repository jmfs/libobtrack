#include "CamShiftTracker.h"
#include "RotatedRect.h"
#include <cv.h>
#include <iostream>
#include <vector>

namespace obt {

/*! The constructor.
	See cv::CamShift documentation or OpenCV's camshiftdemo.c for details.
	The parameter values are defaults considered "sane" by the author. :)

	\param bins The number of histogram bins to use when taking the hue histogram.

	\param sMin The minimum (inclusive) saturation to be included in the hue histogram. 
	Pixels having a saturation higher than this will not be used in the 
	histogram's calculation. This is needed, since hue is ill-defined at low saturation.

	\param vMin The minimum (inclusive) Value (the V in HSV) to be considered in the hue histogram.
	Pixels having a value lower than this will not be used in the histogram's calculation.
	This is needed, since hue is ill-defined at a low value.

	\param vMax The maximum (exclusive) Value (the V in HSV) to be considered in the hue histogram.
	Pixels having a value lower than this will not be used in the histogram's calculation.
	This is needed, since whitish colors have ambiguous hues at high values.
*/
CamShiftTracker::CamShiftTracker(int bins, int sMin, int vMin, int vMax):
		Tracker(true),
		bins(bins),
		sMin(sMin),
		vMin(std::min(vMin, vMax)),
		vMax(std::max(vMin, vMax)) {
	hist.create(1, &(this->bins), CV_8UC1);
}

/*! Trains the CamShiftTracker with an image, and the region where the object is present.
	\param ti A vector of image/object pairs. In this tracker, only the first element 
	is considered.
*/
bool CamShiftTracker::train(const std::vector<const TrainingInfo>& ti) {
	std::vector<const TrainingInfo>::size_type size = ti.size();
	if(size == 0 || ti[0].objects.empty()) {
		std::clog << "ERROR: CamShiftTracker::train: Training vector is empty "
				<< "or has no objects." << std::endl;
		return false;
	}
	
	// CamShift prep

	// Extract the object's region and convert to HSV
	searchWindow = ti[0].objects[0].shape().boundingRect();
	cv::Mat roi = ti[0].img(searchWindow);
	cv::Mat hsv;
	cv::cvtColor(roi, hsv, CV_RGB2HSV);

	// Threshold both saturation and value. See constructor docs for details.
	cv::Mat mask;
	// In 8-bit images, OpenCV has hues from 0 to 180.
	// Upper bounds are exclusive.
	cv::inRange(hsv, cv::Scalar(0, sMin, vMin, 0), cv::Scalar(181, 256, vMax, 0), mask);

	const int channel = 0;
	float range[] = {0, 181};
	const float* ranges[] = {range};
	cv::calcHist(&hsv, 1, &channel, mask, hist, 1, &bins, ranges, true, false);

	double histMax;
	cv::minMaxLoc(hist, NULL, &histMax);	
	// Couldn't find cv::convertScale. That's what should be here.
	// Maybe the OpenCV devs forgot to include it in the C++ interface (as of 2010-10-03).
	// Anyway, there are no negative values here, so this should work.
	cv::convertScaleAbs(hist, hist, histMax ? 255.0 / histMax : 0.0);

	_objects.push_back(Object(new Rect(searchWindow)));

	trained = true;

	return true;
}


int CamShiftTracker::start(const cv::Mat& img) {
	return feed(img);
}

int CamShiftTracker::feed(const cv::Mat& img) {
	if(!trained) {
		std::clog << "ERROR: CamShiftTracker::train: Training vector is empty "	<< 
				"or has no objects." << std::endl;
		return 0;
	}
	int channel = 0;
	float range[] = {0, 256};
	const float* ranges[] = {range};
	cv::Mat bp;
	cv::calcBackProject(&img, 1, &channel, hist, bp, ranges, 1, true);
	RotatedRect* foundObject = new RotatedRect();
	*foundObject = cv::CamShift(bp, searchWindow, 
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 1));
	_objects.front().setShape(foundObject);

	searchWindow = foundObject->boundingRect();	
}

}