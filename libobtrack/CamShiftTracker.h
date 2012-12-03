#ifndef _OBTRACK_CAMSHIFT_TRACKER_H
#define _OBTRACK_CAMSHIFT_TRACKER_H

#include <vector>
#include <cv.h>
#include "Tracker.h"
#include "RotatedRect.h"

namespace obt {

template <typename T> class Rect_;
typedef Rect_<int> Rect;

/*! An object tracker, using the CAMSHIFT algorithm, as provided by OpenCV.
	Uses the HSV color space.
*/
class CamShiftTracker : public Tracker {
public:	
	explicit CamShiftTracker(int bins = 16, int sMin = 32, int vMin = 20, int vMax = 235);
	
	virtual void stopTrackingSingleObject(size_t idx);
	virtual void stopTracking();

	int start(const TrainingInfo* ti = NULL, int idx = -1);
	int feed(const cv::Mat& img);

	int bins() const;

	int sMin() const;
	/*! Sets the mimimum saturation. 

		\param sMin The minimum (inclusive) saturation. It should be between 
			0 and 255 (inclusively), but will be clamped if it isn't.

		\sa train()
	*/
	void setSMin(int sMin);

	int vMin() const;
	/*! Sets the minimum color value. 
	
		\param vMin The minimum (inclusive) value. It should be between 0 and 255 
			(inclusively), but will be clamped if it isn't. If the new minimum 
			value is greater than or equal to the maximum value, the maximum 
			value will be reset to 256.
	*/
	void setVMin(int vMin);

	int vMax() const;
	/*! Sets the maximum color value. 
	
		\param vMax The maximum (exclusive) value. It should be between 1 and 256
			(inclusively), but will be clamped if it isn't.If the new maximum value is less than the 
			minimum value, the minimum value is reset to 0.
	*/
	void setVMax(int vMax);

	void objectShapes(std::vector<const Shape*>& out) const;

protected:
	void sanitizeWindow(Rect& rect, int width, int height);
	
	/*! Number of histogram bins, minimum saturation, minimum value and maximum value.
		See constructor for details.
		\sa CamShiftTracker()
	*/
	int _bins, _sMin, _vMin, _vMax;	
	std::list<cv::MatND> hists; //! The hue histogram
	std::list<RotatedRect> shapes;
	std::list<cv::Mat> masks;	
};

}



#endif