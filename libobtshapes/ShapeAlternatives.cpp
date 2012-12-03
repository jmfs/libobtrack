#include "ShapeAlternatives.h"

#include "helpers.h"

namespace obt {

ShapeAlternatives::ShapeAlternatives():
		CompositeShape() {
}

ShapeAlternatives::ShapeAlternatives(int capacity):
		CompositeShape(capacity) {
}

ShapeAlternatives::ShapeAlternatives(const Shape* s1, const Shape* s2):
		CompositeShape(s1, s2) {
}

cv::Point3f ShapeAlternatives::centroid() const {
	if(members.empty())
		return INVALID_POINT_3D;
	else
		return members[0]->centroid();
}

bool ShapeAlternatives::isAlternative() const {
	return true;
}

/*! Gets the first member's centroid.
	\sa Shape::centroid
*/
cv::Rect ShapeAlternatives::boundingRect() const {
	if(members.empty())
		return INVALID_RECT;
	else
		return members[0]->boundingRect();
}

/*! Gets the first member's bounding rectangle.
	\sa Shape::boundingRect
*/
cv::RotatedRect ShapeAlternatives::boundingRotatedRect() const {
	if(members.empty())
		return INVALID_ROTATED_RECT;
	else
		return members[0]->boundingRotatedRect();
}

/*! Gets the first member's bounding not-necessarily-axis-aligned rectangle.
	\sa Shape::centroid
*/
void ShapeAlternatives::getPixels(std::vector<cv::Point>& result) const {
	if(!members.empty())
		members[0]->getPixels(result);
}

} // namespace obt