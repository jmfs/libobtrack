#include "helpers.h"
#include <cv.h>

namespace obt {

static const float FLOAT_MAX = std::numeric_limits<float>::max();

const cv::Rect INVALID_RECT(INT_MAX, INT_MAX, 0, 0);
const cv::RotatedRect INVALID_ROTATED_RECT(
		cv::Point2f(FLOAT_MAX, FLOAT_MAX),
		cv::Size2f(0, 0), FLOAT_MAX);

const cv::Point2f INVALID_POINT_2D(FLOAT_MAX, FLOAT_MAX);
const cv::Point3f INVALID_POINT_3D(FLOAT_MAX, FLOAT_MAX, FLOAT_MAX);


bool isRectInvalid(const cv::Rect& rect) {
	return rect.x == INT_MAX && rect.y == INT_MAX && 
		rect.width == 0 && rect.height == 0;
}

bool isRectInvalid(const cv::RotatedRect& rect) {
	return rect.center.x == FLOAT_MAX && rect.center.y == FLOAT_MAX &&
		rect.size.width == 0 && rect.size.height == 0 && rect.angle == FLOAT_MAX;
}

bool isPointInvalid(const cv::Point2f& point) {
	return point.x == FLOAT_MAX && point.y == FLOAT_MAX;
}
bool isPointInvalid(const cv::Point3f& point) {
	return point.x == FLOAT_MAX && point.y == FLOAT_MAX && point.z == FLOAT_MAX;
}

}