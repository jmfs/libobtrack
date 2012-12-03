#ifndef _OBTRACK_FASTRACK_H
#define _OBTRACK_FASTRACK_H

#include <cv.h>
#include "Tracker.h"
#include "TrainingInfo.h"
#include "Rect.h"

namespace obt {

/*! FASTrack: Tracking by point correspondence. By default, it extracts FAST [Rosten2006] points from an image,
builds SURF [Bay2006] descriptors out of them, and corresponds them using the FLANN library [Muja2009] */
class FASTrack : public Tracker {
public:
	FASTrack(std::string featureDetector = "FAST", 
			std::string descriptorExtractor = "SURF", 
			std::string descriptorMatcher = "FlannBased");

	//virtual bool trainForSingleObject(const std::vector<TrainingInfo>& ti, int idx = -1);	
	//virtual bool trainForSingleObject(const TrainingInfo& ti, int idx = -1);

	virtual int start(const cv::Mat& img);
	virtual int feed(const cv::Mat& img);

	//virtual void stopTracking();

	virtual void objectShapes(std::vector<const Shape*>& shapes) const;

	virtual ~FASTrack();

	/*virtual bool train(const std::vector<TrainingInfo>& ti) {
		return trainForSingleObject(ti); // TODO
	}

	virtual bool train(const TrainingInfo& ti); {
		return trainForSingleObject(ti); // TODO
	}
	
	virtual void stopTrackingSingleObject(int idx);
	virtual void stopTracking();*/
	
private:
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::Ptr<cv::DescriptorMatcher> matcher;

	cv::Mat mask;

	std::vector<cv::KeyPoint> keys[2];
	std::vector<cv::KeyPoint>* keyPoints;
	std::vector<cv::KeyPoint>* prevKeyPoints;

	std::vector<cv::DMatch> latestMatch;

	cv::Mat descs[2];	
	cv::Mat* descriptors;
	cv::Mat* prevDescriptors;

	std::vector<Rect> keyPointShapes;
};

}

#endif
