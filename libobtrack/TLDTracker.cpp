#include "TLDTracker.h"
#include "TLD.h"
#include "Rect.h"
#include <stdint.h>
#include <cv.h>
#include <iostream>

namespace obt {

/*! Constructs a new TLDTracker.
*/
TLDTracker::TLDTracker():
		Tracker(false, true) {
}

int TLDTracker::start(const TrainingInfo* ti, int idx) {
	if(ti == NULL || ti->img.rows <= 0 || ti->img.cols <= 0 || ti->shapes.empty()) {
		std::cerr << "ERROR: TLDTracker::start: TrainingInfo has "
			"no objects." << std::endl;
		return NO_HINT;
	}

	if(idx > static_cast<int>(tlds.size())) {
		std::cerr << "WARNING: TLDTracker::start: idx is greater than the number of currently tracked objects." 
			"Adding a new object instead. Did you really want to do this?"	<< std::endl;
		idx = -1;
	}

	int numItemsToAdd;
	if(idx == -1) {
		numItemsToAdd = ti->shapes.size();
		idx = tlds.size();
	}
	else
		numItemsToAdd = ti->shapes.size() - (tlds.size() - idx);

	cv::Mat gray;
	cv::cvtColor(ti->img, gray, CV_RGB2GRAY);

	tlds.reserve(tlds.size() + numItemsToAdd);
	objects.reserve(tlds.size() + numItemsToAdd);
	for(size_t i = 0; i < ti->shapes.size(); i++) {
		cv::Rect curRect = ti->shapes[i]->boundingRect();
		if(idx + i < tlds.size()) {
			tlds[idx + i]->release();
		}
		else
			tlds.push_back(new tld::TLD());
		tlds[idx + i]->detectorCascade->imgWidth = gray.cols;
		tlds[idx + i]->detectorCascade->imgHeight = gray.rows;
		tlds[idx + i]->detectorCascade->imgWidthStep = gray.step;
		tlds[idx + i]->selectObject(gray, &curRect);
	}

	started = true;

	return tlds.size();
}

int TLDTracker::feed(const cv::Mat& img) {
	if(!started)
		return NO_HINT;

	cv::Mat gray;
	cv::cvtColor(img, gray, CV_RGB2GRAY);
	for(size_t i = 0; i < tlds.size(); i++) {
		tlds[i]->processImage(gray, true);
		const Rect& curRect = (tlds[i]->currBB == NULL ? INVALID_RECT : *(tlds[i]->currBB));
		if(i < objects.size())
			objects[i] = curRect;
		else
			objects.push_back(curRect);		
	}
	return tlds.size();
}

void TLDTracker::stopTrackingSingleObject(size_t idx) {
	assert(idx >= 0 && idx < tlds.size());
	if(tlds.size() == 1) {
		stopTracking();
		return;
	}
	tlds.erase(tlds.begin() + idx);
	objects.erase(objects.begin() + idx);
}

void TLDTracker::stopTracking() {
	tlds.clear();
	objects.clear();
	started = false;
}

void TLDTracker::objectShapes(std::vector<const Shape*>& shapes) const {
	shapes.reserve(shapes.size() + objects.size());
	for(size_t i = 0; i < objects.size(); i++)
		shapes.push_back(static_cast<const Shape*>(&(objects[i])));
}

}