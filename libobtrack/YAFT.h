#ifndef _OBTRACK_YAFT_H
#define _OBTRACK_YAFT_H

#include "Tracker.h"
#include <cv.h>
struct CvPixelBackgroundGMM;

namespace obt {

/*! Yet Another Flow Tracker. A tracker using background subtraction + optical flow. 
It will probably be feature creeped into using a human model. */
class YAFT : public Tracker {
public:
	YAFT();

	virtual int start(const cv::Mat& img);
	virtual int feed(const cv::Mat& img);

	virtual void stopTracking();

	virtual void objectShapes(std::vector<const Shape*>& shapes) const;

	virtual ~YAFT();

	std::vector<std::vector<cv::Point> > contours;
	std::vector<cv::Vec4i> contourHierarchy; //TODO: These are here for testing purposes. Need to remove it later.
	unsigned char* bgData;
	
private:
	CvPixelBackgroundGMM* bgModel;
	
};

}
#endif