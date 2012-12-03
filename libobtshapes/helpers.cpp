#include "helpers.h"
#include <cv.h>

namespace obt {

static const float FLOAT_MAX = std::numeric_limits<float>::max();

const cv::Rect INVALID_RECT(INT_MAX, INT_MAX, 0, 0); //! Invalid rectangle constant.
const cv::RotatedRect INVALID_ROTATED_RECT(
		cv::Point2f(FLOAT_MAX, FLOAT_MAX),
		cv::Size2f(0, 0), FLOAT_MAX); //! Invalid roated rectangle constant.

const cv::Point2f INVALID_POINT_2D(FLOAT_MAX, FLOAT_MAX); //! Invalid 2D point constant.
const cv::Point3f INVALID_POINT_3D(FLOAT_MAX, FLOAT_MAX, FLOAT_MAX); //! Invalid 3D point constant.

/*! Checks whether a 2D point is invalid.
*/
bool isPointInvalid(const cv::Point2f& point) {
	return point.x == INVALID_POINT_2D.x && point.y == INVALID_POINT_2D.y;
}

/*! Checks whether a 3D point is invalid.
*/
bool isPointInvalid(const cv::Point3f& point) {
	return point.x == INVALID_POINT_3D.x && 
		point.y == INVALID_POINT_3D.y && 
		point.z == INVALID_POINT_3D.z;
}

}