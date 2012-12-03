#include "CamShiftTracker.h"
#include "RotatedRect.h"
#include "Rect.h"
#include <cv.h>
#include <algorithm>
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
		_bins(bins),
		_sMin(sMin),
		_vMin(std::min(vMin, vMax)),
		_vMax(std::max(vMin, vMax)) {
}

/*! Trains the CamShiftTracker with an image, and the region where the object is present.
	\param ti A vector of RGB image/object pairs. In this tracker, only the first element 
	is considered.
*/
bool CamShiftTracker::train(const std::vector<TrainingInfo>& ti) {
	std::vector<TrainingInfo>::size_type size = ti.size();
	if(size == 0 || ti[0].shapes.empty()) {
		std::clog << "ERROR: CamShiftTracker::train: Training vector is empty "
				<< "or has no objects." << std::endl;
		return false;
	}

	mask = cv::Mat::zeros(ti[0].img.rows, ti[0].img.cols, CV_8UC1);
	
	// CamShift prep

	// Extract the object's region and convert to HSV
	Rect searchWindow = ti[0].shapes[0]->boundingRect();
	cv::Mat maskROI = mask(searchWindow);
	cv::Mat hsv;
	cv::cvtColor(ti[0].img, hsv, CV_RGB2HSV);
	cv::Mat roi = hsv(searchWindow);

	// Threshold both saturation and value. See constructor docs for details.

	// In 8-bit images, OpenCV has hues from 0 to 180.
	// Upper bounds are exclusive.
	cv::inRange(hsv, cv::Scalar(0, _sMin, _vMin, 0), cv::Scalar(181, 256, _vMax, 0), mask);

	const int channel[] = {0};
	float range[] = {0, 181};
	const float* ranges[] = {range};
	cv::calcHist(&roi, 1, channel, maskROI, hist, 1, &_bins, ranges, true, false);

	double histMax;
	cv::minMaxLoc(hist, NULL, &histMax);
	hist *= histMax > 0 ? 255.0 / histMax : 0.0;

	if(shapes.empty())
		shapes.push_back(RotatedRect(searchWindow));
	else
		shapes[0] = RotatedRect(searchWindow);

	trained = true;

	return true;
}


int CamShiftTracker::start(const cv::Mat& img) {
	return feed(img);
}

int CamShiftTracker::feed(const cv::Mat& img) {
	if(!trained) {
		std::clog << "ERROR: CamShiftTracker::feed: tracker has not been trained." << std::endl;
		return 0;
	}

	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_RGB2HSV);
	cv::inRange(hsv, cv::Scalar(0, _sMin, _vMin, 0), cv::Scalar(181, 256, _vMax, 0), mask);

	int channel = 0;
	float range[] = {0, 256};
	const float* ranges[] = {range};
	cv::Mat bp;
	cv::calcBackProject(&hsv, 1, &channel, hist, bp, ranges);	
	cv::bitwise_and(bp, mask, bp);
	Rect searchWindow = shapes[0].boundingRect();
	sanitizeWindow(searchWindow, hsv.cols, hsv.rows);
	
	RotatedRect foundObject = cv::CamShift(bp, searchWindow, 
		cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 1));

	shapes[0] = foundObject;
	
	return 1;
}

int CamShiftTracker::bins() const {
	return _bins;
}

/*/*! Sets the number of histogram bins.
	Will only take effect once train() is called again.
	\sa train()
*
void CamShiftTracker::setBins(int numBins) {
	_bins = std::max(0, numBins);
}*/

int CamShiftTracker::sMin() const {
	return _sMin;
}

/*! Sets the mimimum saturation. Will only take effect once train() is called.
	\sa train()
*/
void CamShiftTracker::setSMin(int sMin) {
	_sMin = sMin < 0 ? 0 : std::min(sMin, 255);
}

int CamShiftTracker::vMin() const {
	return _vMin;
}

/*! Sets the minimum value. If the new minimum value is greater than or 
	equal to the maximum value, the maximum value is reset to 256.
	Will only take effect once train() is called.
*/
void CamShiftTracker::setVMin(int vMin) {
	if(vMin < 0)
		vMin = 0;
	else if(vMin >= _vMax) // >= and 256, since vMax is exclusive
		_vMax = 256;		
	else
		_vMin = std::min(vMin, 255);
}


int CamShiftTracker::vMax() const {
	return _vMax;
}

/*! Sets the maximum value. If the new maximum value is less than the 
	minimum value, the minimum value is reset to 0.
	Will only take effect once train() is called.
*/
void CamShiftTracker::setVMax(int vMax) {
	if(vMax > 256) 
		_vMax = 256;
	else if(vMax < _vMin)
		_vMin = 0;
	else
		_vMax = std::max(0, _vMax);
}

void CamShiftTracker::objectShapes(std::vector<const Shape*>& out) const {	
	for(std::vector<RotatedRect>::const_iterator i = shapes.begin(); i != shapes.end(); i++)
		out.push_back(static_cast<const Shape*>(&(*i)));
}

void CamShiftTracker::sanitizeWindow(Rect& rect, int width, int height) {
	if(rect.x < 0) {
		// x is negative, so in reality we are subtracting from the width
		rect.width += rect.x;
		rect.x = 0;
	}
	else if(rect.x >= width)
		rect.x = width - 1;

	if(rect.y < 0) {
		// y is negative, so in reality we are subtracting from the height
		rect.height += rect.y;
		rect.y = 0;
	}
	else if(rect.y >= height)
		rect.y = height - 1;

	if(rect.x + rect.width > width)
		rect.width = width - rect.x;
	if(rect.y + rect.height > height)
		rect.height = height - rect.y;
}
	

}