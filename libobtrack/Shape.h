#ifndef OBTRACK_SHAPE_H
#define OBTRACK_SHAPE_H

#include <cv.h>

namespace obt {

class Shape {
public:
	virtual cv::Point2f centroid() const = 0;
	virtual cv::Rect boundingRect() const = 0;
	virtual cv::RotatedRect boundingRotatedRect() const = 0;

	virtual void getPixels(std::vector<cv::Point>& result) const = 0;
	
	// TODO: Think what kind of operations are really needed here
	// Intersection? Union? Add pixels?
};

}

#endif