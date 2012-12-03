#include "Object.h"

namespace obt {

Object::Object(Shape* shape):
	_shape(shape) {	
}

cv::Point Object::centroid() const {
	return _shape->centroid();
}

cv::Rect Object::boundingRect() const {
	return _shape->boundingRect();
}

const Shape& Object::shape() const {
	return *_shape;
}

void Object::setShape(Shape* other) {
	_shape = std::auto_ptr<Shape>(other);
}

}
