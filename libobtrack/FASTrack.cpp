#include "FASTrack.h"
#include "Rect.h"
#include <cv.h>
#include <string>
#include <iostream>

namespace obt {

FASTrack::FASTrack(std::string featureDetector, 
				std::string descriptorExtractor,
				std::string descriptorMatcher):
	Tracker(false, true),
	detector(cv::FeatureDetector::create(featureDetector)),
	extractor(cv::DescriptorExtractor::create(descriptorExtractor)),
	matcher(cv::DescriptorMatcher::create(descriptorMatcher)),
	prevKeyPoints(&(keys[0])),
	keyPoints(&(keys[1])),
	prevDescriptors(&(descriptors[0])),
	descriptors(&(descs[1])) {
}

/*bool FASTrack::trainForSingleObject(const std::vector<TrainingInfo>& ti, int idx) {
	return trainForSingleObject(ti[0], idx); // No more than 1 TrainingInfo needed for this tracker.
	// TODO: Log this.
}

bool FASTrack::trainForSingleObject(const TrainingInfo& ti, int idx) {
	cv::FAST(
}*/


int FASTrack::start(const cv::Mat& img) {
	/*if(mask.rows != img.rows || mask.cols != img.cols || started) {
		std::clog << "ERROR: Can't detect objects. Training images";
		return -1;
	}*/
	detector->detect(img, *keyPoints, mask);
	extractor->compute(img, *keyPoints, *descriptors);
	started = true;
	return keyPoints->size();
}

int FASTrack::feed(const cv::Mat& img) {
	if(!started)
		return start(img);

	std::swap(keyPoints, prevKeyPoints);	
	std::swap(descriptors, prevDescriptors);
	std::cout << "Old descriptor rows: " << descriptors->rows << std::endl;

	// Testing shows that the keypoint vector and descriptor matrix
	// are cleared before any new stuff is added to them.
	// The official docs could also use this, though.
	detector->detect(img, *keyPoints, mask);
	extractor->compute(img, *keyPoints, *descriptors);
	std::cout << "New descriptor rows: " << descriptors->rows << std::endl;
	matcher->match(*descriptors, *prevDescriptors, latestMatch);
	keyPointShapes.clear();
	for(std::vector<cv::DMatch>::size_type i = 0; i < latestMatch.size(); i++) {
		cv::Point2f pt = (*keyPoints)[latestMatch[i].queryIdx].pt;
		keyPointShapes.push_back(Rect((int)(pt.x - 2), (int)(pt.y - 2), 4, 4));
	}
	// TODO: Threshold descriptor distance
	return keyPointShapes.size();
}

void FASTrack::objectShapes(std::vector<const Shape*>& shapes) const {
	for(std::vector<Rect>::size_type i = 0; i < keyPointShapes.size(); i++)
		shapes.push_back(&(keyPointShapes[i]));
}

FASTrack::~FASTrack() {
}

}