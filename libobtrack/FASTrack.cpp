#include "Shape.h"
#include "FASTrack.h"
#include "Rect.h"
#include <cv.h>
#include <string>
#include <iostream>
#include <fstream>

static std::ostream& logger = std::ofstream("log.txt", ios::out);

namespace obt {


const cv::Mat FASTrack::DEFAULT_H = cv::Mat::eye(3, 3, CV_32FC1);
const float FASTrack::MAX_MOVEMENT = 25.0f;

/*! Creates a new point tracker, using the supplied feature detector,
	descriptor extractor, and descriptor matcher.

	The images supplied to the tracker be scaled by the scaleX
	and scaleY factors, before tracking is done.
*/
FASTrack::FASTrack(cv::Ptr<cv::FeatureDetector> featureDetector,
			cv::Ptr<cv::DescriptorExtractor> descriptorExtractor,
			cv::Ptr<cv::DescriptorMatcher> descriptorMatcher, 
			float scaleX, float scaleY):
		Tracker(false, true),
		scaleX(scaleX),
		scaleY(scaleY),
		detector(featureDetector),
		extractor(descriptorExtractor),
		matcher(descriptorMatcher),
		curDescIndex(0) {
}


int FASTrack::start(const TrainingInfo* ti, int idx) {
	bool validImage = ti != NULL && ((!started && ti->img.rows > 0 && ti->img.cols > 0) || 
		(started && ti->img.rows == defaultMask.rows && ti->img.cols == defaultMask.cols));

	if(ti == NULL || !validImage || ti->shapes.empty()) {
		return NO_HINT;
	}

	sanityCheck();

	if(idx > static_cast<int>(masks.size())) {
		std::cerr << "WARNING: FASTrack::start: idx is greater than the number of currently tracked objects." 
			"Adding a new object instead. Did you really want to do this?"	<< std::endl;
		idx = masks.size();
	}

	if(idx < 0)
		idx = masks.size();

	if(defaultMask.rows == 0)
		defaultMask = cv::Mat::zeros(ti->img.rows, ti->img.cols, CV_8UC1);
	cv::Mat& mask = updateListElement(masks, idx, defaultMask);

	Rect& prevMaskRect = updateListElement(prevMaskRects, idx, Rect(ti->shapes[0]->boundingRect()));
	cv::rectangle(mask, prevMaskRect, cv::Scalar::all(1));
	
	for(int i = 0; i < 2; i++) {
		updateListElement(keyPoints[i], idx, std::vector<cv::KeyPoint>());
		updateListElement(descriptors[i], idx, cv::Mat());
	}

	updateListElement(HPrevs, idx, DEFAULT_H);

	detector->detect(ti->img, getListElement(keyPoints[curDescIndex], idx), mask);
	extractor->compute(ti->img, 
			getListElement(keyPoints[curDescIndex], idx), 
			getListElement(descriptors[curDescIndex], idx));

	if(idx == keyPointShapes.size())
		keyPointShapes.push_back(prevMaskRect);
	else
		updateListElement(keyPointShapes, idx, prevMaskRect);

	updateListElement(latestMatches, idx, std::vector<cv::DMatch>());

	started = true;
	return masks.size();
}

/*! Returns a cv::Rect containing the region over which to draw ones in the 
	point detectors mask. This rectangle is the result of taking an Axis-Aligned
	Bounding Box of all points, and enlarging it MAX_MOVEMENT pixels in all directions.
	If there are no points were detected in the previous frame, the previous
	rectangle is enlarged.
*/
cv::Rect FASTrack::getNewMaskRect(const std::vector<cv::KeyPoint>& keyPoints, const Rect& prevMaskRect) {
	const float maxFloat = std::numeric_limits<float>::max();
	const float minFloat = std::numeric_limits<float>::min();
	float minX = maxFloat;
	float minY = maxFloat;
	float maxX = minFloat;
	float maxY = minFloat;

	if(keyPoints.empty()) {
		minX = static_cast<float>(prevMaskRect.x);
		minY = static_cast<float>(prevMaskRect.y);
		maxX = static_cast<float>(prevMaskRect.x + prevMaskRect.width);
		maxY = static_cast<float>(prevMaskRect.y + prevMaskRect.height);
	}
	else {	
		for(std::vector<cv::KeyPoint>::size_type i = 0; i < keyPoints.size(); i++) {
			minX = std::min(keyPoints[i].pt.x, minX);
			minY = std::min(keyPoints[i].pt.y, minY);
			maxX = std::max(keyPoints[i].pt.x, maxX);
			maxY = std::max(keyPoints[i].pt.y, maxY);		
		}
	}
	
	return cv::Rect(static_cast<int>(minX - MAX_MOVEMENT),
			static_cast<int>(minY - MAX_MOVEMENT),
			static_cast<int>(maxX - minX + 2 * MAX_MOVEMENT),
			static_cast<int>(maxY - minY + 2 * MAX_MOVEMENT));
}

/*! Calculates movement average and standard deviation. 
*/
void FASTrack::movementStats(const std::vector<cv::KeyPoint>& keyPoints, 
		const std::vector<cv::KeyPoint>& prevKeyPoints, 
		const std::vector<cv::DMatch>& latestMatch, 
		cv::Point2d& avg, cv::Point2d& sigma) {
	avg.x = avg.y = sigma.x = sigma.y = 0;
	logger << "Size: " << latestMatch.size() << std::endl;
	if(latestMatch.empty())
		return;
	for(std::vector<cv::DMatch>::size_type i = 0; i < latestMatch.size(); i++) {
		cv::Point2f trainPt = prevKeyPoints[latestMatch[i].trainIdx].pt;
		cv::Point2f queryPt = keyPoints[latestMatch[i].queryIdx].pt;
		avg.x += queryPt.x - trainPt.x;
		avg.y += queryPt.y - trainPt.y;
	}
	avg.x /= latestMatch.size();
	avg.y /= latestMatch.size();
	logger << "Mean: " << avg.x << ' ' << avg.y << std::endl;

	for(std::vector<cv::DMatch>::size_type i = 0; i < latestMatch.size(); i++) {
		cv::Point2f trainPt = prevKeyPoints[latestMatch[i].trainIdx].pt;
		cv::Point2f queryPt = keyPoints[latestMatch[i].queryIdx].pt;
		double xBase = queryPt.x - trainPt.x - avg.x;
		double yBase = queryPt.y - trainPt.y - avg.y;
		sigma.x += xBase * xBase;
		sigma.y += yBase * yBase;
	}
	
	sigma.x = std::sqrt(sigma.x / (latestMatch.size() - 1));
	sigma.y = std::sqrt(sigma.y / (latestMatch.size() - 1));
	logger << "Sigma: "<< sigma.x << ' ' << sigma.y << std::endl;
}

int FASTrack::feed(const cv::Mat& img) {
	if(!started)
		return NO_HINT;

	assert(img.rows == defaultMask.rows && img.cols == defaultMask.cols);
	sanityCheck();
	
	curDescIndex = 1 - curDescIndex;
	keyPointShapes.clear();

	std::list<cv::Mat>::iterator masksIt, HPrevsIt, curDescsIt, prevDescsIt;
	masksIt = masks.begin();
	HPrevsIt = HPrevs.begin(); 
	curDescsIt = descriptors[curDescIndex].begin();
	prevDescsIt = descriptors[1 - curDescIndex].begin();
		
	std::list<std::vector<cv::KeyPoint> >::iterator curKpIt, prevKpIt;
	curKpIt = keyPoints[curDescIndex].begin();
	prevKpIt = keyPoints[1 - curDescIndex].begin();

	std::list<Rect>::iterator prevMaskRectsIt = prevMaskRects.begin();
	std::list<std::vector<cv::DMatch> >::iterator matchesIt = latestMatches.begin(); 
	for( ; masksIt != masks.end(); 
			masksIt++, HPrevsIt++, curDescsIt++, prevDescsIt++,
			curKpIt++, prevKpIt++, prevMaskRectsIt++, matchesIt++) {
		cv::Mat& mask = *masksIt;
		cv::Mat& HPrev = *HPrevsIt;
		cv::Mat& descs = *curDescsIt;
		cv::Mat& prevDescs = *prevDescsIt;
		std::vector<cv::KeyPoint>& kps = *curKpIt;
		std::vector<cv::KeyPoint>& prevKps = *prevKpIt;
		Rect& prevMaskRect = *prevMaskRectsIt;
		std::vector<cv::DMatch>& latestMatch = *matchesIt;

		defaultMask.copyTo(mask);
		cv::Rect bounding = prevMaskRect = getNewMaskRect(kps, prevMaskRect);
		cv::rectangle(mask, bounding, cv::Scalar::all(1));
	
		cv::Mat scaledImg;
		if(scaleX == 1.0f && scaleY == 1.0f)
			scaledImg = img;
		else
			cv::resize(img, scaledImg, cv::Size(), scaleX, scaleY);

		// Testing shows that the keypoint vector and descriptor matrix
		// are cleared before any new stuff is added to them.
		// The official docs could also say this, though...
		detector->detect(scaledImg, kps, mask);
		extractor->compute(scaledImg, kps, descs);

		std::vector<cv::Point2f> prevPoints, points;
		std::vector<unsigned char> HMask;

		if (!prevKps.empty()) {
			std::vector<cv::KeyPoint> testKeyPoints;
			warpKeypoints(HPrev.inv(), kps, testKeyPoints);

			cv::Mat matchingMask = cv::windowedMatchingMask(testKeyPoints, prevKps, 25, 25);
			matcher->match(descs, prevDescs, latestMatch, matchingMask);

			matches2points(prevKps, kps, latestMatch, prevPoints, points);

			if (latestMatch.size() > 5) {
				cv::Mat H = cv::findHomography(cv::Mat(prevPoints), cv::Mat(points), HMask, CV_RANSAC, 4);
				if (cv::countNonZero(cv::Mat(HMask)) > 15)
					HPrev = H;
				else
					HPrev = DEFAULT_H;
			}
			else
				HPrev = DEFAULT_H;
		}		

		if(!kps.empty()) {
			if(prevKps.empty()) {
				keyPointShapes.push_back(getNewMaskRect(kps, prevMaskRect));
				continue;
			}
			// Calculate the movement average and standard deviation...
			cv::Point2d avgMovement, movementStdDev;
			movementStats(kps, prevKps, latestMatch, avgMovement, movementStdDev);
	
			std::vector<cv::KeyPoint> newKeyPoints;
			std::vector<cv::DMatch> newMatch;
			cv::Mat newDescriptors(descs.rows, descs.cols, descs.type());
			int numPoints = 0;
			int prevNumPoints = latestMatch.size();
			// ...so it is possible to eliminate outliers from the key points
			for(std::vector<cv::DMatch>::size_type i = 0; i < latestMatch.size(); i++) {
				cv::Point2f trainPt = prevKps[latestMatch[i].trainIdx].pt;
				cv::Point2f queryPt = kps[latestMatch[i].queryIdx].pt;
				float xMovement = queryPt.x - trainPt.x;
				float yMovement = queryPt.y - trainPt.y;
				double xDiff = std::fabs(xMovement - avgMovement.x);
				double yDiff = std::fabs(yMovement - avgMovement.y);
			
				if(xDiff <= STD_DEV_MULTIPLIER * movementStdDev.x &&
						yDiff <= STD_DEV_MULTIPLIER * movementStdDev.y) {
					newKeyPoints.push_back(kps.at(latestMatch[i].queryIdx));
					descs.row(latestMatch[i].queryIdx).copyTo(newDescriptors.row(numPoints));
					latestMatch[i].queryIdx = newKeyPoints.size() - 1;
					newMatch.push_back(latestMatch[i]);
					numPoints++;
				}
			}

			kps = newKeyPoints;
			latestMatch = newMatch;
			// We've allocated an upper bound for the number of rows, now we 
			// discard the rows we haven't used
			descs = newDescriptors(cv::Range(0, numPoints), cv::Range(0, descs.cols));
		} // if(!kps.empty())

	
		keyPointShapes.push_back(getNewMaskRect(kps, prevMaskRect));
	} // for( ; masksIt != masks.end(); (...)

	return masks.size();
}

/*! See \ref Tracker::objectShapes. 
	Additional info: The returned shapes are \ref Rect "Rects". 
*/
void FASTrack::objectShapes(std::vector<const Shape*>& shapes) const {
	const int oldSize = shapes.size();
	shapes.resize(shapes.size() + keyPointShapes.size());
	std::list<Rect>::const_iterator it;
	int i;
	for(i = 0, it = keyPointShapes.begin(); 
			it != keyPointShapes.end(); i++, it++)
		shapes[oldSize + i] = &(*it);
}

/*! Converts matching indices to xy points
*/
void FASTrack::matches2points(const vector<cv::KeyPoint>& train, const vector<cv::KeyPoint>& query,
                    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
                    std::vector<cv::Point2f>& pts_query) {
	pts_train.clear();
	pts_query.clear();
	pts_train.reserve(matches.size()); // matches.size() is an upper bound for the vector's size
	pts_query.reserve(matches.size());

	for (size_t i = 0; i < matches.size(); i++)	{
		const cv::DMatch& dmatch = matches[i];

		pts_query.push_back(query[dmatch.queryIdx].pt);
		pts_train.push_back(train[dmatch.trainIdx].pt);
  }
}

/*! Uses computed homography H to warp original input points to new planar position
*/
void FASTrack::warpKeypoints(const cv::Mat& H, const std::vector<cv::KeyPoint>& in, std::vector<cv::KeyPoint>& out)
{
	cv::vector<cv::Point2f> pts;
	keypoints2points(in, pts);
	cv::vector<cv::Point2f> pts_w(pts.size());
	cv::Mat m_pts_w(pts_w);
	perspectiveTransform(cv::Mat(pts), m_pts_w, H);
	points2keypoints(pts_w, out);
}

/*! Takes a descriptor and turns it into an xy point
*/
void FASTrack::keypoints2points(const std::vector<cv::KeyPoint>& in, std::vector<cv::Point2f>& out) {
	out.clear();
	out.reserve(in.size());
	for (size_t i = 0; i < in.size(); ++i) {
		out.push_back(in[i].pt);
	}
}

/*! Takes an xy point and appends that to a keypoint structure
*/
void FASTrack::points2keypoints(const std::vector<cv::Point2f>& in, std::vector<cv::KeyPoint>& out) {
	out.clear();
	out.reserve(in.size());
	for (size_t i = 0; i < in.size(); ++i) {
		out.push_back(cv::KeyPoint(in[i], 1));
	}
}

void FASTrack::stopTrackingSingleObject(size_t idx) {
	sanityCheck();

	if(masks.size() == 1) {
		stopTracking();
		return;
	}

	eraseListElement(masks, idx);
	eraseListElement(prevMaskRects, idx);
	for(int i = 0; i < 2; i++) {
		eraseListElement(keyPoints[i], idx);
		eraseListElement(descriptors[i], idx);
	}	
	eraseListElement(HPrevs, idx);
	eraseListElement(keyPointShapes, idx);
	eraseListElement(latestMatches, idx);

	sanityCheck();
}

void FASTrack::stopTracking() {
	masks.clear();
	prevMaskRects.clear();
	for(int i = 0; i < 2; i++) {
		keyPoints[i].clear();
		descriptors[i].clear();
	}	
	HPrevs.clear();
	keyPointShapes.clear();
	latestMatches.clear();
	defaultMask = cv::Mat();
	started = false;

	sanityCheck();
}

void FASTrack::sanityCheck() {
	assert(masks.size() == prevMaskRects.size() && 
			masks.size() == keyPoints[0].size() && 
			masks.size() == keyPoints[1].size() && 
			masks.size() == descriptors[0].size() &&
			masks.size() == descriptors[1].size() && 
			masks.size() == HPrevs.size() && 
			masks.size() == keyPointShapes.size() && 
			masks.size() == latestMatches.size() &&
			defaultMask.rows == 0 && defaultMask.cols == 0
	);
}

}