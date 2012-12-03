#ifndef _OBTRACK_TRACKER_H
#define _OBTRACK_TRACKER_H

#include "TrainingInfo.h"
#include  "Skeleton.h"
#include <list>
#include <vector>
#include <limits>

namespace obt {

/*! The base class for all object trackers.
	Can handle both single-object and multi-object trackers.
	To keep the interface consistent, single-object trackers will
	be run once for each object, whenever they are tracking more than a single object.
*/
class Tracker {
public:
	//! Error codes for the start and feed methods.
	enum Errors {
		//! Returned for a tracker which needs a hint, but doesn't receive one
		NO_HINT = -1, 
		//! Returned in case the data passed is invalid
		INVALID_DATA = -2, 
		/*! Returned if further initialization is needed
		 *	Having a need for it should be avoided whenever possible, in individual trackers.
		 */
		INIT_NEEDED = -3,
		/*! Values below this one can be used by tracker developers with whatever
			meaning they wish to attribute to them.
		*/
		FIRST_USER_ERROR = -4 
	};
	Tracker(bool needsTraining, bool needsHint);

	virtual int init();

	/*! Performs an initial object detection, with or without hints,
		or updates one or more object's current shape and/or appearance models.

		If needsHint is true, ti should contain data, otherwise, NO_HINT is returned.
		If needsHint is false, what to do with any data in ti is up to each individual 
		tracker's implementation.
	
		\param ti Hints about the objects in the image.
			If ti is NULL, or if it doesn't contain data, and needsHint == true, 
			NO_HINT is returned.
		\param idx If non-negative, indicates the first object index to update. If negative,
			new objects are tracked.

		\return The number of detected objects in the image, or a negative error code.
	 */
	virtual int start(const TrainingInfo* ti = NULL, int idx = -1) = 0;

	/*! Feeds a new image to the object tracker. If no initial object detection has been
		done yet, start() is called.

		\param img A new image.
		\return The number of detected objects in the image.

		\sa start
	 */
	virtual int feed(const cv::Mat& img) = 0;

	bool needsTraining() const;
	bool needsHint() const;
	bool isTrained() const;
	bool isStarted() const;

	virtual bool train(const std::vector<TrainingInfo>& ti);	
	virtual bool train(const TrainingInfo& ti);	
	
	virtual void stopTrackingSingleObject(size_t idx);
	virtual void stopTracking();

	/*! Appends the shapes found to a vector.
		The contents are only guaranteed to be valid pointers until the next call to feed().
		\param shapes Output. The found shapes will be appended to this vector.
	*/
	virtual void objectShapes(std::vector<const Shape*>& shapes) const = 0;
	virtual void objectShapes2D(std::vector<const Shape*>& shapes, int forImage = 0) const;

protected:
	template<typename T> static T& updateListElement(
		std::list<T>& list, size_t i, const T& newValue);
	template<typename T> static T& getListElement(std::list<T>& list, size_t i);
	template<typename T> static void eraseListElement(std::list<T>& list, size_t i);


	bool trained; //! If true, this tracker has already been trained and it is ready to start tracking objects
	bool started; //! If true, initial object detection has been done

private:	
	bool _needsTraining; //! Specifies if this tracker needs to be trained by the train() function
	/*! Whether this tracker needs an initial hint to the object's position.
	*/
	bool _needsHint; 
};

/*! Updates a list element. If idx == l.size(), adds a value to the end of the list,
*/
template<typename T> T& Tracker::updateListElement(
				std::list<T>& l, size_t idx, const T& newValue) {
	assert(idx >= 0 && idx <= l.size());
	if(idx == l.size()) {
		l.push_back(newValue);
		return l.back();
	}
	else {
		std::list<T>::iterator it = l.begin();
		std::advance(it, idx);
		*it = newValue;
		return *it;
	}
}

/*! Gets a list's element by index.
*/
template<typename T> T& Tracker::getListElement(std::list<T>& list, size_t idx) {
	assert(idx >= 0 && idx < list.size());
	std::list<T>::iterator it = list.begin();
	std::advance(it, idx);
	return *it;
}

/*! Erases a list's element by index.
*/
template<typename T> void Tracker::eraseListElement(std::list<T>& list, size_t idx) {
	assert(idx >= 0 && idx < list.size());
	std::list<T>::iterator it = list.begin();
	std::advance(it, idx);
	list.erase(it);
}

}


#endif