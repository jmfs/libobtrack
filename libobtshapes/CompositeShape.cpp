#include "CompositeShape.h"

namespace obt {

CompositeShape::CompositeShape() {
}

/*! Initializes \ref CompositeShape::members with vector(capacity).
*/
CompositeShape::CompositeShape(int capacity) {
	members.reserve(capacity);
}

/*! Shorthand for adding two shapes to \ref CompositeShape::members.
*/
CompositeShape::CompositeShape(const Shape* s1, const Shape* s2):
		members(2) {
	members.push_back(s1);
	members.push_back(s2);
}

bool CompositeShape::isComposite() const {
	return true;
}

}