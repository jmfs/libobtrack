#ifndef _OBTSHAPES_BLOB_H
#define _OBTSHAPES_BLOB_H

#include <list>
#include <cv.h>
#include "Rect.h"
#include "RotatedRect.h"

namespace obt {

class RotatedRect;

class Blob : public Shape {
	typedef std::vector<cv::Point> Pixels;
	
	explicit Blob(int capacity = 100);

	cv::Point2f centroid() const;
	cv::Rect boundingRect() const;
	cv::RotatedRect boundingRotatedRect() const;

	void getPixels(std::vector<cv::Point>& result) const;
	const std::vector<cv::Point>& getPixelsRef() const;
	std::list<cv::Point>::size_type size();

	void addPoint(int x, int y); 
	void clear();

protected:
	static const int DEFAULT_CAPACITY;

	std::vector<cv::Point> pixels; //! The pixel coordinates of the pixels in this Blob.

	/*! The bounds for this Blob. If there are no points in the Blob, at least minX will be equal to 
		std::numeric_limits<int>::min(). Not really an invalid value, just very unlikely to happen.
	*/
	mutable int minX, minY, maxX, maxY;

	/*! Both the centroid and the bounding rotated rectangle (the smallest possible rectangle
		containing all of the Blob's pixels) are pretty hard to calculate, so they are cached
		so any subsequent calls to centroid() or boundingRotatedRect() return them at once.
	*/
	mutable cv::Point2f* cachedCentroid;
	mutable RotatedRect cachedRR; //! /sa cachedCentroid
};

}

#endif