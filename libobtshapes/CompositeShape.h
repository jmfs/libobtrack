#ifndef _OBTSHAPES_COMPOSITE_SHAPE_H
#define _OBTSHAPES_COMPOSITE_SHAPE_H

#include "Shape.h"
#include <vector>

namespace obt {

/*! Base class for forming a Shape from various other Shapes.
	Subclasses define what to do with those shapes.

	\sa ShapeAlternatives
	\sa ShapeUnion
	\sa ShapeIntersection
	\sa ShapeDifference
*/
class CompositeShape : public Shape {
public:
	CompositeShape();
	explicit CompositeShape(int capacity);
	CompositeShape(const Shape* s1, const Shape* s2);

	virtual bool isComposite() const;

	virtual cv::Point3f centroid() const = 0;
	virtual cv::Rect boundingRect() const = 0;
	virtual cv::RotatedRect boundingRotatedRect() const = 0;

	virtual void getPixels(std::vector<cv::Point>& result) const = 0;

	/*! Members of this CompositeShape. The meaning of these will be defined by subclasses.
		It would be too cumbersome for \ref Tracker "Trackers" to work with \ref Shape "Shapes" 
		in this representation. Therefore, there is no invariant to maintain, and so
		this member is public.
		
		Trackers should pack their Shapes into a CompositeShape only when returning them to 
		callers via \ref Tracker::objectShapes.
	*/
	std::vector<const Shape*> members; 
};

}

#endif