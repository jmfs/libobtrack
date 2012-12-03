#include "Tracker.h"

namespace obt {

Tracker::Tracker(bool needsTraining, bool needsHint):
		_needsTraining(needsTraining),
		_needsHint(needsHint),
		trained(!needsTraining),
		started(false) {
}

/*! Trains an object tracker, according to sample images, or
	known image/objects pairs. If a tracker needs training, it should
	always overload this function.

	\param ti A vector of images, or (image, object) pairs.

	\return Whether the training has been successful. 
		If the method hasn't been overriden, always returns true.
*/
bool Tracker::train(const std::vector<TrainingInfo>& ti) {
	/*if(singleObject)
		return trainForSingleObject(ti);*/

	trained = true;
	return true;
}

/*! Trains an object tracker, according to a single sample image, or
	known (image, shapes) pair. If a tracker needs training, it should
	always overload this function.

	\param ti An image, or (image, shapes) pair

	\return Whether the training has been successful. 
		If the method hasn't been overriden, always returns true.

	\sa Tracker::trainForSingleObject()
*/
bool Tracker::train(const TrainingInfo& ti) {
	/*if(singleObject)
		return trainForSingleObject(ti);*/

	trained = true;
	return true;
}

/*! Trains an object tracker to track an object, using multiple sample images or 
	(image, object shape) pairs. If there are too many sample images for the 
	tracker, the first ones will prevail. If the tracker is already trained 
	for this object, re-does the training. A negative object index tells 
	the tracker the object is new.

	\param ti Vector with the training info.
	\param idx The index of the object to change, as reported by objectShapes().
		If negative, or equal to the current number of tracked objects, a new object will be tracked.

	\return Whether the training has been successful. 
		If the method hasn't been overriden, always returns true.
	
	\sa objectShapes()
/
bool Tracker::trainForSingleObject(const std::vector<TrainingInfo>& ti, int idx) {
	trained = true;
	return true;
}

/*! Trains an object tracker to track an object, using a single sample image or
	(image, object shape) pair. If there are too many sample images for the 
	tracker, the first ones will prevail. If the tracker is already trained 
	for this object, re-does the training. A negative object index tells 
	the tracker the object is new.

	\param ti Vector with the training info.
	\param idx The index of the object to change, as reported by objectShapes().
		If negative, or equal to the current number of tracked objects, a new object will be tracked.

	\return Whether the training has been successful. 
		If the method hasn't been overriden, always returns true.
	
	\sa objectShapes()
/
bool Tracker::trainForSingleObject(const TrainingInfo& ti, int idx) {
	trained = true;
	return true;
}*/

/*! Stops tracking a single object, deleting its training data.
		
	\param idx The index of the object to stop tracking, as reported by objectShapes()

	\return Whether there was success.

	\sa objectShapes()
*/
void Tracker::stopTrackingSingleObject(int idx) {
}

/*! Stops all tracking. Forgets any prior training.
*/
void Tracker::stopTracking() {
	trained = false;
	started = false;
}

bool Tracker::needsTraining() const {
	return _needsTraining;
}

bool Tracker::isTrained() const {
	return trained;
}

bool Tracker::isStarted() const {
	return started;
}


/*! Appends 2D versions of the shapes found to a vector. This is the 2D rendition
	from image number forImage. The shapes must be in the same order as the ones in
	objectShapes3D, and shapes not present in image forImage must be invalid ones.
	Invalid shapes include, but are not limited to, an empty Blob, an INVALID_RECT or 
	INVALID_ROTATED_RECT.
	The contents are only guaranteed to be valid pointers until the next call to feed().
	By default, calls objectShapes(shapes).

	\param shapes Output. The found shapes will be appended to this vector.
	\param forImage In multi-camera trackers, the image's index. Default: 0.

	\sa INVALID_RECT
	\sa INVALID_ROTATED_RECT
	\sa Shape
	\sa feed
*/
void Tracker::objectShapes2D(std::vector<const Shape*>& shapes, int forImage) const {
	objectShapes(shapes);
}

}