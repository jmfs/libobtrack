#include "Shape.h"
#include "FASTrack.h"
#include "Rect.h"
#include <cv.h>
#include <string>
#include <iostream>
#include <fstream>

static std::ostream& logger = std::ofstream("log.txt", ios::out);
//std::ostream& logger = std::cout;

namespace obt {


const cv::Mat FASTrack::DEFAULT_H = cv::Mat::eye(3, 3, CV_32FC1);
const float FASTrack::MAX_MOVEMENT = 25.0f;

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
		prevKeyPoints(&(keys[0])),
		keyPoints(&(keys[1])),
		prevDescriptors(&(descs[0])),
		descriptors(&(descs[1])),
		HPrev(DEFAULT_H) {
}


int FASTrack::start(const TrainingInfo* ti, int idx) {
	/*if(mask.rows != img.rows || mask.cols != img.cols || started) {
		std::cerr << "ERROR: Can't detect objects. Training images";
		return -1;
	}*/
	/*cv::Mat gray;
	cv::cvtColor(img, gray, CV_RGB2GRAY);*/
	bool validImage = ti != NULL && ((!started && ti->img.rows > 0 && ti->img.cols > 0) || 
		(started && ti->img.rows == defaultMask.rows && ti->img.cols == defaultMask.cols));

	if(ti == NULL || !validImage || ti->shapes.empty()) {
		return NO_HINT;
	}
	
	defaultMask = mask = cv::Mat::zeros(ti->img.rows, ti->img.cols, CV_8UC1);

	//TODO: Handle multiple objects and non-negative indexes
	prevMaskRect = ti->shapes[0]->boundingRect();	
	cv::rectangle(mask, prevMaskRect, cv::Scalar::all(1));
	detector->detect(ti->img, *keyPoints, mask);
	extractor->compute(ti->img, *keyPoints, *descriptors);
	started = true;
	return feed(ti->img);
}

/*! Returns a cv::Rect containing the region over which to draw ones in the 
	point detectors mask. This rectangle is the result of taking an Axis-Aligned
	Bounding Box of all points, and enlarging it MAX_MOVEMENT pixels in all directions.
	If there are no points were detected in the previous frame, the previous
	rectangle is enlarged.
*/
cv::Rect FASTrack::getNewMaskRect() const {
	const float maxFloat = std::numeric_limits<float>::max();
	const float minFloat = std::numeric_limits<float>::min();
	float minX = maxFloat;
	float minY = maxFloat;
	float maxX = minFloat;
	float maxY = minFloat;

	if(keyPoints->empty()) {
		minX = static_cast<float>(prevMaskRect.x);
		minY = static_cast<float>(prevMaskRect.y);
		maxX = static_cast<float>(prevMaskRect.x + prevMaskRect.width);
		maxY = static_cast<float>(prevMaskRect.y + prevMaskRect.height);		
	}
	else {	
		for(std::vector<cv::KeyPoint>::size_type i = 0; i < keyPoints->size(); i++) {
			minX = std::min(keyPoints->at(i).pt.x, minX);
			minY = std::min(keyPoints->at(i).pt.y, minY);
			maxX = std::max(keyPoints->at(i).pt.x, maxX);
			maxY = std::max(keyPoints->at(i).pt.y, maxY);		
		}
	}
	
	return cv::Rect(static_cast<int>(minX - MAX_MOVEMENT),
			static_cast<int>(minY - MAX_MOVEMENT),
			static_cast<int>(maxX - minX + 2 * MAX_MOVEMENT),
			static_cast<int>(maxY - minY + 2 * MAX_MOVEMENT));
}

/*! Calculates movement average and standard deviation. */
void FASTrack::movementStats(cv::Point2d& avg, cv::Point2d& sigma) const{
	avg.x = avg.y = sigma.x = sigma.y = 0;
	logger << "Size: " << latestMatch.size() << std::endl;
	if(latestMatch.empty())
		return;
	for(std::vector<cv::DMatch>::size_type i = 0; i < latestMatch.size(); i++) {
		cv::Point2f trainPt = (*prevKeyPoints)[latestMatch[i].trainIdx].pt;
		cv::Point2f queryPt = (*keyPoints)[latestMatch[i].queryIdx].pt;
		avg.x += queryPt.x - trainPt.x;
		avg.y += queryPt.y - trainPt.y;
	}
	avg.x /= latestMatch.size();
	avg.y /= latestMatch.size();
	logger << "Mean: " << avg.x << ' ' << avg.y << std::endl;

	for(std::vector<cv::DMatch>::size_type i = 0; i < latestMatch.size(); i++) {
		cv::Point2f trainPt = (*prevKeyPoints)[latestMatch[i].trainIdx].pt;
		cv::Point2f queryPt = (*keyPoints)[latestMatch[i].queryIdx].pt;
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
	//TODO: Check image size
	if(!started)
		started = true;

	defaultMask.copyTo(mask);
	cv::Rect bounding = prevMaskRect = getNewMaskRect();
	cv::rectangle(mask, bounding, cv::Scalar::all(1));
	
	std::swap(keyPoints, prevKeyPoints);	
	std::swap(descriptors, prevDescriptors);

	/*cv::Mat gray;
	cv::cvtColor(img, gray, CV_RGB2GRAY);*/
	cv::Mat scaledImg;
	if(scaleX == 1.0f && scaleY == 1.0f)
		scaledImg = img;
	else
		cv::resize(img, scaledImg, cv::Size(), scaleX, scaleY);

	//cv::medianBlur(scaledImg, scaledImg, 3);

	// Testing shows that the keypoint vector and descriptor matrix
	// are cleared before any new stuff is added to them.
	// The official docs could also say this, though...
	detector->detect(scaledImg, *keyPoints, mask);
	extractor->compute(scaledImg, *keyPoints, *descriptors);

	std::vector<cv::Point2f> prevPoints, points;
	std::vector<unsigned char> HMask;

	if (!prevKeyPoints->empty()) {
		std::vector<cv::KeyPoint> testKeyPoints;
		warpKeypoints(HPrev.inv(), *keyPoints, testKeyPoints);

		cv::Mat matchingMask = cv::windowedMatchingMask(testKeyPoints, *prevKeyPoints, 25, 25);
		matcher->match(*descriptors, *prevDescriptors, latestMatch, matchingMask);

		matches2points(*prevKeyPoints, *keyPoints, latestMatch, prevPoints, points);

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
			
	//keyPointShapes.clear();
	keyPointShapes.clear();

	if(!keyPoints->empty()) {
		if(prevKeyPoints->empty()) {
			keyPointShapes.push_back(getNewMaskRect());
			return 1;
		}
		// Calculate the movement average and standard deviation
		cv::Point2d avgMovement, movementStdDev;
		movementStats(avgMovement, movementStdDev);
	
		std::vector<cv::KeyPoint> newKeyPoints;
		std::vector<cv::DMatch> newMatch;
		cv::Mat newDescriptors(descriptors->rows, descriptors->cols, descriptors->type());
		int numPoints = 0;
		int prevNumPoints = latestMatch.size();
		// So it is possible to eliminate outliers from the key points
		for(std::vector<cv::DMatch>::size_type i = 0; i < latestMatch.size(); i++) {
			cv::Point2f trainPt = (*prevKeyPoints)[latestMatch[i].trainIdx].pt;
			cv::Point2f queryPt = (*keyPoints)[latestMatch[i].queryIdx].pt;
			float xMovement = queryPt.x - trainPt.x;
			float yMovement = queryPt.y - trainPt.y;
			double xDiff = std::fabs(xMovement - avgMovement.x);
			double yDiff = std::fabs(yMovement - avgMovement.y);
			
			if(xDiff <= movementStdDev.x &&
					yDiff <= movementStdDev.y) {
				newKeyPoints.push_back(keyPoints->at(latestMatch[i].queryIdx));
				descriptors->row(latestMatch[i].queryIdx).copyTo(newDescriptors.row(numPoints));
				latestMatch[i].queryIdx = newKeyPoints.size() - 1;
				newMatch.push_back(latestMatch[i]);
				numPoints++;
			}
		}

		if(numPoints == 0) {
			numPoints = numPoints; // breakpoint fodder
		}

		*keyPoints = newKeyPoints;
		latestMatch = newMatch;
		// We've allocated an upper bound for the number of rows, now we 
		// discard the rows we haven't used
		*descriptors = newDescriptors(cv::Range(0, numPoints), cv::Range(0, descriptors->cols));
	}

	
	keyPointShapes.push_back(getNewMaskRect());

	return 1;
}

/*! See \ref Tracker::objectShapes. 
	Additional info: The returned shapes are \ref Rect "Rects". 
*/
void FASTrack::objectShapes(std::vector<const Shape*>& shapes) const {
	for(std::vector<Rect>::size_type i = 0; i < keyPointShapes.size(); i++)
		shapes.push_back(&(keyPointShapes[i]));
}

//! Converts matching indices to xy points
void FASTrack::matches2points(const vector<cv::KeyPoint>& train, const vector<cv::KeyPoint>& query,
                    const std::vector<cv::DMatch>& matches, std::vector<cv::Point2f>& pts_train,
                    std::vector<cv::Point2f>& pts_query) {
	pts_train.clear();
	pts_query.clear();
	pts_train.reserve(matches.size());
	pts_query.reserve(matches.size());

	for (size_t i = 0; i < matches.size(); i++)	{
		const cv::DMatch & dmatch = matches[i];

		pts_query.push_back(query[dmatch.queryIdx].pt);
		pts_train.push_back(train[dmatch.trainIdx].pt);
  }
}

//Uses computed homography H to warp original input points to new planar position
void FASTrack::warpKeypoints(const cv::Mat& H, const std::vector<cv::KeyPoint>& in, std::vector<cv::KeyPoint>& out)
{
	cv::vector<cv::Point2f> pts;
	keypoints2points(in, pts);
	cv::vector<cv::Point2f> pts_w(pts.size());
	cv::Mat m_pts_w(pts_w);
	perspectiveTransform(cv::Mat(pts), m_pts_w, H);
	points2keypoints(pts_w, out);
}

//! Takes a descriptor and turns it into an xy point
void FASTrack::keypoints2points(const std::vector<cv::KeyPoint>& in, std::vector<cv::Point2f>& out) {
	out.clear();
	out.reserve(in.size());
	for (size_t i = 0; i < in.size(); ++i) {
		out.push_back(in[i].pt);
	}
}

//! Takes an xy point and appends that to a keypoint structure
void FASTrack::points2keypoints(const std::vector<cv::Point2f>& in, std::vector<cv::KeyPoint>& out) {
	out.clear();
	out.reserve(in.size());
	for (size_t i = 0; i < in.size(); ++i) {
		out.push_back(cv::KeyPoint(in[i], 1));
	}
}


FASTrack::~FASTrack() {
}

}