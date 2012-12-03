#ifndef _OBTRACK_TRACKER_BASE_H
#define _OBTRACK_TRACKER_BASE_H

#include <cv.h>

namespace obt {


/*! The base class for all object trackers.
 * You shouldn't derive from this class. Derive from 
 * Tracker or PointTracker instead
*/
class TrackerBase {
public:
	explicit TrackerBase(bool needsTraining = false);

	/*! Performs an initial object detection.
	 * \param img The initial image.
	 * \return The number of detected objects in the image.
	 */
	virtual int start(const cv::Mat& img) = 0;

	/*! Feeds a new image to the object tracker.
	 * \param img A new image.
	 * \return The number of detected objects in the image.
	 */
	virtual int feed(const cv::Mat& img) = 0;

	/*! Forces a tracker to stop tracking the nth object in this tracker.
	 * \param n The object's index in this tracker.
	 */
	//virtual void untrackObject(int n) = 0;

	bool needsTraining() const;
	bool isTrained() const;
	bool isStarted() const;

protected:
	bool _needsTraining; //! Specifies if this tracker needs to be trained by the train() function
	bool trained; //! If true, this tracker has already been trained and it is ready to start tracking objects
	bool started; //! If true, initial object detection has been done
};

}


#endif
