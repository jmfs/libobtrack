#ifndef _OBTRACK_TRACKER_H
#define _OBTRACK_TRACKER_H

#include "TrackerBase.h"
#include "Object.h"
#include "TrainingInfo.h"
#include <list>
#include <vector>

namespace obt {

/*! The base class for all shape- (not point-) based trackers.
	Can handle both single-object and multi-object trackers.
	To keep the interface consistent, single-object trackers will
	be run once for each object, if trained for more than a single object.
*/
class Tracker : public TrackerBase {
public:
	explicit Tracker(bool needsTraining = false, bool singleObject = false);

	
	virtual bool train(const std::vector<TrainingInfo>& ti);	
	virtual bool train(const TrainingInfo& ti);	
	
	virtual bool trainForSingleObject(const std::vector<TrainingInfo>& ti, int idx = -1);	
	virtual bool trainForSingleObject(const TrainingInfo& ti, int idx = -1);
	
	virtual void stopTrackingSingleObject(int idx);
	virtual void stopTracking();

	/*! Appends the shapes found to a vector.
		\param shapes Output. The found shapes will be appended to this vector.
	*/
	virtual void objectShapes(std::vector<const Shape*>& shapes) const = 0;

	bool isSingleObjectTracker();
private:
	bool singleObject; //! Whether this is a single object tracker.
};

}

#endif