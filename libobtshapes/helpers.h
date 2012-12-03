#ifndef _OBTSHAPES_HELPERS_H
#define _OBTSHAPES_HELPERS_H

#include <cv.h>

namespace obt {

extern const cv::Rect INVALID_RECT;
extern const cv::RotatedRect INVALID_ROTATED_RECT;
extern const cv::Point2f INVALID_POINT_2D;
extern const cv::Point3f INVALID_POINT_3D;


bool isRectInvalid(const cv::Rect& rect);
bool isRectInvalid(const cv::RotatedRect& rect);
bool isPointInvalid(const cv::Point2f& point);
bool isPointInvalid(const cv::Point3f& point);

}

#endif