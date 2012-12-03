#include "TLDTracker.h"
#include "Rect.h"
#include "tld.h"
#include <stdint.h>
#include <cv.h>
#include <iostream>

namespace obt {

/*! Constructs a new TLDTracker.
*/
TLDTracker::TLDTracker():
		Tracker(false, true) {
}

int TLDTracker::init(){ 
	// TODO
}

int TLDTracker::start(const TrainingInfo* ti, int idx) {
	// TODO
	
	if(ti == NULL || ti->img.rows <= 0 || ti->img.cols <= 0 || ti->shapes.empty()) {
		std::cerr << "ERROR: TLDTracker::start: TrainingInfo has "
			"no objects." << std::endl;
		return NO_HINT;
	}

	/*if(idx > static_cast<int>(tld.size())) {
		std::cerr << "WARNING: TLDTracker::start: idx is greater than the number of currently tracked objects." 
			"Adding a new object instead. Did you really want to do this?"	<< std::endl;
		idx = -1;
	}

	int numItemsToAdd;
	if(idx == -1) {
		numItemsToAdd = ti->shapes.size();
		idx = tld.size();
	}
	else
		numItemsToAdd = ti->shapes.size() - (tld.size() - idx);*/

	// TODO
}

int TLDTracker::feed(const cv::Mat& img) {
	// TODO
}

void TLDTracker::stopTrackingSingleObject(int idx) {
	assert(idx >= 0 && idx < tld.size());

	//TODO
}

void TLDTracker::stopTracking() {
	// TODO
}

void TLDTracker::objectShapes(std::vector<const Shape*>& shapes) const {
	// TODO
}

TLDTracker::~TLDTracker() {
	// TODO?
}


}