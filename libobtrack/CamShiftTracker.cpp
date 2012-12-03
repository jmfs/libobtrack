#include "CamShiftTracker.h"
#include "RotatedRect.h"
#include "Rect.h"
#include "Shape.h"
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
		Tracker(false, true),
		_bins(bins),
		_sMin(sMin),
		_vMin(std::min(vMin, vMax)),
		_vMax(std::max(vMin, vMax)) {
}

/*! \sa Tracker::start
*/
int CamShiftTracker::start(const TrainingInfo* ti, int idx) {
	assert(shapes.size() == masks.size() && masks.size() == hists.size());

	if(ti == NULL || ti->img.rows <= 0 || ti->img.cols <= 0 || ti->shapes.empty()) {
		std::cerr << "ERROR: CamShiftTracker::start: TrainingInfo has "
			"no objects." << std::endl;
		return NO_HINT;
	}

	if(idx > static_cast<int>(shapes.size())) {
		std::cerr << "WARNING: CamShiftTracker::start: idx is greater than the number of currently tracked objects." 
			"Adding a new object instead. Did you really want to do this?"	<< std::endl;
		idx = shapes.size();
	}

	if(idx < 0)
		idx = shapes.size();

	for(size_t i = 0; i < ti->shapes.size(); i++) {
		// CamShift prep
		cv::Mat& newMask = updateListElement(masks, idx + i, 
			static_cast<cv::Mat>(cv::Mat::zeros(ti->img.rows, ti->img.cols, CV_8UC1)));	
		
		cv::Mat hsv;
		cv::cvtColor(ti->img, hsv, CV_RGB2HSV);		

		// Threshold both saturation and value. See constructor docs for details.

		// In 8-bit images, OpenCV has hues from 0 to 180.
		// Upper bounds are exclusive.
		cv::inRange(hsv, cv::Scalar(0, _sMin, _vMin, 0), cv::Scalar(181, 256, _vMax, 0), newMask);
	
		// Calculate the hue histogram for the object's region
		cv::MatND& newHist = updateListElement(hists, idx + i, cv::MatND());
		Rect searchWindow = ti->shapes[0]->boundingRect();
		sanitizeWindow(searchWindow, ti->img.cols, ti->img.rows);
		cv::Mat maskROI = newMask(searchWindow);
		cv::Mat roi = hsv(searchWindow);

		const int channel[] = {0};
		float range[] = {0, 181};
		const float* ranges[] = {range};
		cv::calcHist(&roi, 1, channel, maskROI, newHist, 1, &_bins, ranges);

		// normalize it
		double histMax;
		cv::minMaxLoc(newHist, NULL, &histMax);
		newHist *= histMax > 0 ? 255.0 / histMax : 0.0;

		updateListElement(shapes, idx + i, RotatedRect(searchWindow));
	}

	started = true;

	return ti->shapes.size();
}

/*! \sa Tracker::feed
*/
int CamShiftTracker::feed(const cv::Mat& img) {
	assert(shapes.size() == masks.size() && masks.size() == hists.size());

	if(!started) {
		std::cerr << "ERROR: CamShiftTracker::feed: need to call start() first." << std::endl;
		return NO_HINT;
	}

	cv::Mat hsv;
	cv::cvtColor(img, hsv, CV_RGB2HSV);

	std::list<cv::MatND>::const_iterator hIterator = hists.begin();
	std::list<cv::Mat>::iterator mIterator = masks.begin();
	std::list<RotatedRect>::iterator sIterator = shapes.begin();
	for(; hIterator != hists.end(); hIterator++, mIterator++, sIterator++) {	
		cv::inRange(hsv, cv::Scalar(0, _sMin, _vMin, 0), 
				cv::Scalar(181, 256, _vMax, 0), *mIterator);

		int channel = 0;
		float range[] = {0, 256};
		const float* ranges[] = {range};
		cv::Mat bp;
		cv::calcBackProject(&hsv, 1, &channel, *hIterator, bp, ranges);	
		cv::bitwise_and(bp, *mIterator, bp);
		Rect searchWindow = (*sIterator).boundingRect();
		sanitizeWindow(searchWindow, hsv.cols, hsv.rows);
		
		RotatedRect foundObject = cv::CamShift(bp, searchWindow, 
			cv::TermCriteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 10, 1));

		*sIterator = foundObject;
	}
	
	return shapes.size();
}

void CamShiftTracker::stopTrackingSingleObject(size_t idx) {
	assert(shapes.size() == masks.size() && 
		masks.size() == hists.size() && idx >= 0 && idx < shapes.size());

	eraseListElement(hists, idx);
	eraseListElement(masks, idx);
	eraseListElement(shapes, idx);
	
	assert(shapes.size() == masks.size() && masks.size() == hists.size());	
}

void CamShiftTracker::stopTracking() {
	hists.clear();
	shapes.clear();
	hists.clear();
	
	trained = started = false;
}

int CamShiftTracker::bins() const {
	return _bins;
}

/*! Sets the number of histogram bins.
	Will only take effect once train() is called again.
	\sa train()
*
void CamShiftTracker::setBins(int numBins) {
	_bins = std::max(0, numBins);
}*/

int CamShiftTracker::sMin() const {
	return _sMin;
}

/*! Sets the mimimum saturation. 

	\param sMin The minimum (inclusive) saturation. It should be between 
		0 and 255 (inclusively), but will be clamped if it isn't.

	\sa train()
*/
void CamShiftTracker::setSMin(int sMin) {
	_sMin = sMin < 0 ? 0 : std::min(sMin, 255);
}

int CamShiftTracker::vMin() const {
	return _vMin;
}

/*! Sets the minimum color value. 
	
	\param vMin The minimum (inclusive) value. It should be between 0 and 255 
		(inclusively), but will be clamped if it isn't. If the new minimum 
		value is greater than or equal to the maximum value, the maximum 
		value will be reset to 256.
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

/*! Sets the maximum color value. 
	
	\param vMax The maximum (exclusive) value. It should be between 1 and 256
		(inclusively), but will be clamped if it isn't.If the new maximum value is less than the 
		minimum value, the minimum value is reset to 0.
*/
void CamShiftTracker::setVMax(int vMax) {
	if(vMax > 256) 
		vMax = 256;
	else if(vMax < _vMin)
		_vMin = 0;
	
	_vMax = std::max(0, vMax);
}

/*! See \ref Tracker::objectShapes. 
	Additional info: The returned shapes are \ref RotatedRect "RotatedRects". 
*/
void CamShiftTracker::objectShapes(std::vector<const Shape*>& out) const {
	out.reserve(out.size() + shapes.size());
	for(std::list<RotatedRect>::const_iterator i = shapes.begin(); i != shapes.end(); i++)
		out.push_back(static_cast<const Shape*>(&(*i)));
}

/*! Clips the initial search window to inside the video boundaries.
*/
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