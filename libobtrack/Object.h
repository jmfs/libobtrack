#ifndef _OBTRACK_OBJECT_H
#define _OBTRACK_OBJECT_H

#include "Shape.h"
#include <cv.h>

namespace obt {

/*! This class represents an object to track.
 * It only defines the object's shape and some auxiliary functions.
 * Individual trackers should subclass it in order to use their
 * particular appearance model.
 */
class Object {
public:
	/*! Initializes the object with the new shape.
	\param shape The object's shape. This class becomes responsible
		for freeing the data in shape.
	*/
	Object(Shape* shape);
	Object(const Object& other);
	
	virtual cv::Point centroid() const;
	virtual cv::Rect boundingRect() const;
	const Shape& shape() const;

	/*! Changes the object's shape. 
		Should be used *only* by trackers. If you call this from
		outside a tracker, all bets are off.

		\param The new shape. This class becomes responsible for
			freeing the data in shape.
	*/
	void setShape(Shape* shape);

	// TODO: is saving the object's appearance here needed or possible?
	
protected:
	/*ShapeModel shape;
	AppearanceModel appearance;*/
};

}

#endif