#ifndef _OBTSHAPES_SHAPE_ALTERNATIVES_H
#define _OBTSHAPES_SHAPE_ALTERNATIVES_H

#include "CompositeShape.h"

namespace obt {

/*
	\sa ShapeUnion
	\sa ShapeIntersection
	\sa ShapeDifference
*/
class ShapeAlternatives : public CompositeShape {
public:
	ShapeAlternatives();
	explicit ShapeAlternatives(int capacity);
	ShapeAlternatives(const Shape* s1, const Shape* s2);

	virtual bool isAlternative() const;

	virtual cv::Point3f centroid() const;
	virtual cv::Rect boundingRect() const;
	virtual cv::RotatedRect boundingRotatedRect() const;

	virtual void getPixels(std::vector<cv::Point>& result) const;
};

}

#endif
