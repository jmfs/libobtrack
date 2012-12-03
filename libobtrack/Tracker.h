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

	/*  ! Redoes the training for the object in index n.

		\return Whether the training has been successful. 
			If the method hasn't been overriden, always returns true.
	*/
	//virtual bool retrain(int n, const std::vector<TrainingInfo>& ti);

	/*! Trains an object tracker, according to sample images, or
		known image/objects pairs. If a tracker needs training, it should
		always overload this function.

		\return Whether the training has been successful. 
			If the method hasn't been overriden, always returns true.
	*/
	virtual bool train(const std::vector<TrainingInfo>& ti);

	/*! Appends the shapes found to a vector.
		\param shapes Output. The found shapes will be appended to this vector.
	*/
	virtual void objectShapes(std::vector<const Shape*>& shapes) const = 0;
};

}

#endif