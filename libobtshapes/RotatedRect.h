#ifndef _OBTSHAPES_ROTATED_RECT_H
#define _OBTSHAPES_ROTATED_RECT_H

#include <cv.h>
#include "Shape.h"

namespace obt {

template<typename T> class Rect_;

/*! A rotated rectangle class. Like cv::RotatedRect, but with Shape's operations.
	See http://opencv.willowgarage.com/documentation/cpp/basic_structures.html#rotatedrect
	for more details.
*/
class RotatedRect : public Shape, public cv::RotatedRect {
public:
    RotatedRect();
	RotatedRect(const cv::Point2f& center, const cv::Size2f& size, float angle);
    RotatedRect(const CvBox2D& box);
	RotatedRect(const cv::RotatedRect cvRotRect);

	//! Convert a Rect_ into a RotatedRect
	template<typename T> RotatedRect(const Rect_<T>& rect);

	operator cv::RotatedRect() const;
    operator CvBox2D() const;

	/*! Undocumented (as of 2010-10-02) cv::RotatedRect function, to get the rectangle's corners.
		\param pts Output parameter. Should be an array of 4 "cv::Point2f"s.
			On return, it will be filled with the four corners of the rectangle.
	*/
	void points(cv::Point2f pts[]) const;

	cv::Point2f centroid() const;
	cv::Rect boundingRect() const;
	cv::RotatedRect boundingRotatedRect() const;


	/*! Adds to result the pixels being inside the RotatedRect.
		\param result Output vector for the pixels
	*/
	void getPixels(std::vector<cv::Point>& result) const;
};

template<typename T> 
RotatedRect::RotatedRect(const Rect_<T>& rect):
		cv::RotatedRect(
			cv::Point2f(
				static_cast<float>(rect.x + rect.width / 2.0f), 
				static_cast<float>(rect.y + rect.height / 2.0f)
			),
			cv::Size2f(
				static_cast<float>(rect.width), 
				static_cast<float>(rect.height)
			),
			0.0f
		) {
}

}

#endif