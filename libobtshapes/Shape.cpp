#include "Shape.h"

namespace obt {

bool Shape::isInvalid() const {
	return false;
}

bool Shape::isComposite() const {
	return false;
}

bool Shape::isAlternative() const {
	return false;
}

bool Shape::isSkeleton() const {
	return false;
}

bool Shape::is3D() const {
	return false;
}

}