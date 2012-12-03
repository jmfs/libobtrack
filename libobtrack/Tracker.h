#ifndef _OBTRACK_TRACKER_H
#define _OBTRACK_TRACKER_H

#include "Object.h"
#include "TrainingInfo.h"
#include <list>
#include <vector>
#include <limits>

namespace obt {

/*! The base class for all based trackers.
	Can handle both single-object and multi-object trackers.
	To keep the interface consistent, single-object trackers will
	be run once for each object, whenever they are tracking more than a single object.
*/
class Tracker {
public:
	enum Errors {
		NO_HINT = -1, //! Returned for a tracker which needs a hint, but doesn't receive one
		INVALID_DATA = -2 //! Returned in case the data passed is invalid
	};
	explicit Tracker(bool needsTraining, bool needsHint, bool singleObject);

	/*! Performs an initial object detection, with or without hints,
	 *  or updates one or more object's current shape and/or appearance models.
	 *	If needsHint is true, ti should contain data, otherwise, NO_HINT is returned.
	 *  If needsHint is false, what to do with any data in ti is up to each individual 
	 *  tracker's implementation.
	 *
	 * \param img The initial image.
	 * \param ti Hints about the objects in the image.
	 *		If ti is NULL, or if it doesn't contain data, and needsHint == true, 
	 *		NO_HINT is returned.
	 * \param idx If non-negative, indicates the first object index to update. If negative,
	 *		new objects are tracked.
	 * \return The number of detected objects in the image, or a negative error code.
	 */
	virtual int start(const TrainingInfo* ti = NULL, int idx = -1) = 0;

	/*! Feeds a new image to the object tracker. If no initial object detection has been
	 * done yet, start(img, NULL) is called.
	 * \param img A new image.
	 * \return The number of detected objects in the image.
	 */
	virtual int feed(const cv::Mat& img) = 0;

	bool needsTraining() const;
	bool needsHint() const;
	bool isTrained() const;
	bool isStarted() const;
	bool isSingleObjectTracker() const;

	virtual bool train(const std::vector<TrainingInfo>& ti);	
	virtual bool train(const TrainingInfo& ti);	
	
	virtual void stopTrackingSingleObject(int idx);
	virtual void stopTracking();

	/*! Appends the shapes found to a vector.
		\param shapes Output. The found shapes will be appended to this vector.
	*/
	virtual void objectShapes(std::vector<const Shape*>& shapes) const = 0;

protected:
	bool trained; //! If true, this tracker has already been trained and it is ready to start tracking objects
	bool started; //! If true, initial object detection has been done

private:
	bool singleObject; //! Whether this is a single object tracker.
	bool _needsTraining; //! Specifies if this tracker needs to be trained by the train() function
	/*! Whether this tracker needs an initial hint to the object's position and/or category.
	*/
	bool _needsHint; 
};

}

#endif