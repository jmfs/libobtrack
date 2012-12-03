#ifndef _OBTRACK_CAMSHIFT_TRACKER_H
#define _OBTRACK_CAMSHIFT_TRACKER_H

#include <cv.h>
#include "Tracker.h"
#include "Rect.h"

namespace obt {

/*! An object tracker, using the CAMSHIFT algorithm, as provided by OpenCV.
	Uses the HSV color space.
*/
class CamShiftTracker : public Tracker {
public:			
	explicit CamShiftTracker(int bins = 16, int sMin = 32, int vMin = 20, int vMax = 235);

	virtual bool train(const std::vector<const TrainingInfo>& ti);
	virtual int start(const cv::Mat& img);
	virtual int feed(const cv::Mat& img);

protected:
	/*! Number of histogram bins, minimum saturation, minimum value and maximum value.
		See constructor for details.
		\sa CamShiftTracker()
	*/
	int bins, sMin, vMin, vMax;	
	cv::MatND hist; //! The hue histogram
	Rect searchWindow; //! CamShift's search window
};

}

#endif