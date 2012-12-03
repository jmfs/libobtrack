#ifndef _OBTSHAPES_SHAPE_H
#define _OBTSHAPES_SHAPE_H

#include <vector>
#include <cv.h>

namespace obt {

/*! Base class for representing abstract shapes.
	Each shape must derive from this.
*/
class Shape {
public:
	/*! Returns the Shape's centroid.
		If the Shape is 2D, the centroid's Z coordinate will be 0.0f.
	*/
	virtual cv::Point3f centroid() const = 0;
	/*! Returns an axis-aligned bounding rectangle for the shape.
	*/
	virtual cv::Rect boundingRect() const = 0;
	/*! Returns the smallest possible bounding rectangle for this shape
		(may not be axis-aligned).
	*/
	virtual cv::RotatedRect boundingRotatedRect() const = 0;

	virtual bool isInvalid() const;

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