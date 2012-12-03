#include "Tracker.h"

namespace obt {

Tracker::Tracker(bool needsTraining):
		TrackerBase(needsTraining) {
}

/*bool Tracker::retrain(int n, const std::vector<TrainingInfo>& ti) {
	trained = true;
	return true;
}*/

bool Tracker::train(const std::vector<TrainingInfo>& ti) {
	trained = true;
	return true;
}

}