#ifndef _OBTRACK_FASTRACK_H
#define _OBTRACK_FASTRACK_H

#include <cv.h>
#include "Tracker.h"
#include "TrainingInfo.h"
#include "Rect.h"

namespace obt {

/*! FASTrack: Tracking by point correspondence. By default, it extracts FAST [Rosten2006] points from an image,
builds BRIEF-32 descriptors out of them, and corresponds them using Hamming distance */
class FASTrack : public Tracker {
public:	
	FASTrack(cv::Ptr<cv::FeatureDetector> featureDetector = //cv::FeatureDetector::create("GridFAST"),
		new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector, 500, 4, 4),
		cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::DescriptorExtractor::create("BRIEF"),
		cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming"),
		float scaleX = 1.0f, float scaleY = 1.0f);

	virtual bool trainForSingleObject(const std::vector<TrainingInfo>& ti, int idx = -1);	
	virtual bool trainForSingleObject(const TrainingInfo& ti, int idx = -1);

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

	std::vector<cv::DMatch> latestMatch;
	std::vector<cv::KeyPoint>* keyPoints;
	std::vector<cv::KeyPoint>* prevKeyPoints;
private:
	static void matches2points(const vector<cv::KeyPoint>& train, const vector<cv::KeyPoint>& query,
                    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
                    std::vector<cv::Point2f>& pts_query);
	static void warpKeypoints(const cv::Mat& H, 
						const std::vector<cv::KeyPoint>& in, 
						std::vector<cv::KeyPoint>& out);
	static void keypoints2points(const std::vector<cv::KeyPoint>& in, 
								std::vector<cv::Point2f>& out);
	static void points2keypoints(const std::vector<cv::Point2f>& in, 
								std::vector<cv::KeyPoint>& out);
	void FASTrack::movementStats(cv::Point2d& avg, cv::Point2d& sigma) const;	
	cv::Rect getNewMaskRect() const;

	static const cv::Mat DEFAULT_H;
	static const float MAX_MOVEMENT;
	cv::Mat defaultMask;	
	
	cv::Ptr<cv::FeatureDetector> detector;
	cv::Ptr<cv::DescriptorExtractor> extractor;
	cv::Ptr<cv::DescriptorMatcher> matcher;

	/*! Factors by which to scale the image 
		before extracting key points. Smaller scales have
		better performance */
	float scaleX, scaleY;		

	cv::Mat mask;
	obt::Rect prevMaskRect;

	std::vector<cv::KeyPoint> keys[2];	
	cv::Mat descs[2];
	cv::Mat* descriptors;
	cv::Mat* prevDescriptors;
	cv::Mat HPrev;

	std::vector<Rect> keyPointShapes;
};

}

#endif
