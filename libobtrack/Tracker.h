#ifndef _OBTRACK_TRACKER_H
#define _OBTRACK_TRACKER_H

#include "TrackerBase.h"
#include "Object.h"
#include "TrainingInfo.h"
#include <list>
#include <vector>

namespace obt {

class Tracker : public TrackerBase {
public:
	explicit Tracker(bool needsTraining = false);

	/*! Trains an object tracker, according to sample images, or
		known image/objects pairs. If a tracker needs training, it should
		always overload this function.

		\return Whether the training has been successful. 
			If the method hasn't been overriden, always returns true.
	*/
	virtual bool train(const std::vector<TrainingInfo>& ti);

	const std::list<Object>& objects() const;

protected:
	std::list<Object> _objects; //! A list of currently tracked objects
};

}

#endif