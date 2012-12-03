#include "TrackerBase.h"

namespace obt {

TrackerBase::TrackerBase(bool needsTraining):
		_needsTraining(needsTraining),
		trained(false),
		started(false) {
}

bool TrackerBase::needsTraining() const {
	return _needsTraining;
}

bool TrackerBase::isTrained() const {
	return trained;
}

bool TrackerBase::isStarted() const {
	return started;
}

}
