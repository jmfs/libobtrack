#ifndef _OBTRACK_CAMSHIFT_TRACKER_H
#define _OBTRACK_CAMSHIFT_TRACKER_H

#include <vector>
#include <cv.h>
#include "Tracker.h"
#include "RotatedRect.h"
#include "Rect.h"

namespace obt {


/*! An object tracker, using the CAMSHIFT algorithm, as provided by OpenCV.
	Uses the HSV color space.
*/
class CamShiftTracker : public Tracker {
public:			
	explicit CamShiftTracker(int bins = 16, int sMin = 32, int vMin = 20, int vMax = 235);

	bool train(const std::vector<TrainingInfo>& ti);
	int start(const cv::Mat& img);
	int feed(const cv::Mat& img);

	int bins() const;

	int sMin() const;
	void setSMin(int sMin);

	int vMin() const;
	void setVMin(int vMin);

	int vMax() const;
	void setVMax(int vMax);

	void objectShapes(std::vector<const Shape*>& out) const;

protected:
	void sanitizeWindow(Rect& rect, int width, int height);


	/*! Number of histogram bins, minimum saturation, minimum value and maximum value.
		See constructor for details.
		\sa CamShiftTracker()
	*/
	int _bins, _sMin, _vMin, _vMax;	
	cv::MatND hist; //! The hue histogram
	std::vector<RotatedRect> shapes;
	cv::Mat mask;
	
};

}

#endif