#ifndef _OBTSHAPES_SHAPE_H
#define _OBTSHAPES_SHAPE_H

#include <vector>
#include <cv.h>

namespace obt {

class Shape {
public:
	virtual cv::Point3f centroid() const = 0;
	virtual cv::Rect boundingRect() const = 0;
	virtual cv::RotatedRect boundingRotatedRect() const = 0;

	virtual bool isComposite() const;
	virtual bool isAlternative() const ;
	virtual bool isSkeleton() const;
	virtual bool is3D() const;

	/*! Puts all the pixels contained by this Shape into result.
		Does not clear result before the operation.
	*/
	virtual void getPixels(std::vector<cv::Point>& result) const = 0;
};

}

#endif