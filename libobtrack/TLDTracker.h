#ifndef _OBTRACK_TLD_TRACKER_H
#define _OBTRACK_TLD_TRACKER_H

#include "Tracker.h"
#include "TLD.h"
#include "Rect.h"
#include <cv.h>

namespace obt {

class Shape;
class TrainingInfo;

/*! Adaptation of the OpenTLD tracker, originally by Zdenek Kalal.

	This tracker keeps learning the tracked object's shape and appearance, 
	thus adapting to changing conditions.

	You can find the tracker's homepage here: http://info.ee.surrey.ac.uk/Personal/Z.Kalal/tld.html.
	The code in this class is based on Georg Nebehay's C++ implementation: https://github.com/gnebehay/OpenTLD.
*/
class TLDTracker : public Tracker {
public:
	TLDTracker();
	int start(const TrainingInfo* ti = NULL, int idx = -1);
	int feed(const cv::Mat& img);
	void stopTrackingSingleObject(int idx);
	void stopTracking();
	void objectShapes(std::vector<const Shape*>& shapes) const;

private:
	std::vector<tld::TLD*> tlds;
	std::vector<Rect> objects;
};

}

#endif