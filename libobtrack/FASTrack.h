#ifndef _OBTRACK_FASTRACK_H
#define _OBTRACK_FASTRACK_H

#include <cv.h>
#include "Tracker.h"
#include "Rect.h"

class TrainingInfo;

namespace obt {

/*! FASTrack: Tracking by point correspondence. By default, it extracts FAST [Rosten2006] points from an image,
builds BRIEF-32 descriptors out of them, and corresponds them using Hamming distance */
class FASTrack : public Tracker {
public:	
	FASTrack(cv::Ptr<cv::FeatureDetector> featureDetector = //cv::FeatureDetector::create("GridFAST"),
		new cv::GridAdaptedFeatureDetector(new cv::FastFeatureDetector, DEFAULT_NUM_KEYPOINTS, 
			DEFAULT_GRID_SIZE, DEFAULT_GRID_SIZE),
		cv::Ptr<cv::DescriptorExtractor> descriptorExtractor = cv::DescriptorExtractor::create("BRIEF"),
		cv::Ptr<cv::DescriptorMatcher> descriptorMatcher = cv::DescriptorMatcher::create("BruteForce-Hamming"),
		float scaleX = 1.0f, float scaleY = 1.0f);

	virtual int start(const TrainingInfo* ti = NULL, int idx = -1);
	virtual int feed(const cv::Mat& img);

	virtual void stopTrackingSingleObject(size_t idx);
	virtual void stopTracking();

	virtual void objectShapes(std::vector<const Shape*>& shapes) const;
	
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
	static void FASTrack::movementStats(const std::vector<cv::KeyPoint>& keyPoints, 
		const std::vector<cv::KeyPoint>& prevKeyPoints, 
		const std::vector<cv::DMatch>& latestMatch, cv::Point2d& avg, cv::Point2d& sigma);	
	static cv::Rect getNewMaskRect(const std::vector<cv::KeyPoint>& keyPoints, const Rect& prevMaskRect);

	static const cv::Mat DEFAULT_H; //! Default Homography Matrix
	static const float MAX_MOVEMENT; //! Keypoint movement threshold
	static const int DEFAULT_NUM_KEYPOINTS = 500; //! Default number of keypoints to detect
	static const int DEFAULT_GRID_SIZE = 4; //! default grid size for GridAdaptedFeatureDetector
	/*! In order to be matched in the next frame, a point's movement must differ from 
		the movement average less than STD_DEV_MULTIPLIER times the movement standard deviation.
	*/
	static const int STD_DEV_MULTIPLIER = 1;

	void sanityCheck();

	cv::Mat defaultMask; //! Default point detection mask
	
	cv::Ptr<cv::FeatureDetector> detector; //! Feature point detector to use. See \ref FASTrack() for defaults.
	cv::Ptr<cv::DescriptorExtractor> extractor; //! Descriptor extractor. See \ref FASTrack() for defaults.
	cv::Ptr<cv::DescriptorMatcher> matcher; //! Descriptor matcher. See \ref FASTrack() for defaults.

	/*! Factors by which to scale the image 
		before extracting key points. Smaller scales have
		better performance, but not by much */
	float scaleX, scaleY;		

	std::list<cv::Mat> masks; //! Rectangular masks for the object detectors, one per tracked object
	std::list<Rect> prevMaskRects; //! The previous frame's mask rectangle, for each tracked object

	std::list<std::vector<cv::KeyPoint> > keyPoints[2]; //! Keypoints (current Frame and previous) for each tracked object
	std::list<cv::Mat> descriptors[2]; //! Descriptors (current Frame and previous) for each tracked object
	/*! Holds the current frame's index in the \ref keyPoints and \ref descriptors arrays. The 
		remaining index will have the previous frame's data.
	*/
	int curDescIndex; 

	std::list<cv::Mat> HPrevs; //! Previous frame's Homography matrix.

	std::list<Rect> keyPointShapes; //! Holds detected shapes.
	std::list<std::vector<cv::DMatch> > latestMatches; //! previous frame's matched keypoints
};

}

#endif
