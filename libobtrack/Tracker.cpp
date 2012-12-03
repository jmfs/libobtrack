#include "Tracker.h"

namespace obt {

Tracker::Tracker(bool needsTraining, bool needsHint, bool singleObject):
		_needsTraining(needsTraining),
		_needsHint(needsHint),
		trained(false),
		started(false),
		singleObject(singleObject) {
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

bool Tracker::isSingleObjectTracker() const {
	return singleObject;
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

}