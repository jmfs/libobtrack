#include "Tracker.h"

namespace obt {

Tracker::Tracker(bool needsTraining):
		TrackerBase(needsTraining) {
}

bool Tracker::train(const std::vector<TrainingInfo>& ti) {
	trained = true;
	return true;
}

const std::list<Object>& Tracker::objects() const {
	return _objects;
}

}