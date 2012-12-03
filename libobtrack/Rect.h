#ifndef OBTRACK_RECT_H
#define OBTRACK_RECT_H

#include <cv.h>
#include "Shape.h"

namespace obt {

/*! A rectangle class. Like cv::Rect, but with Shape's operations.
	See http://opencv.willowgarage.com/documentation/cpp/basic_structures.html#rect
	for more details.
*/
template<typename T> class Rect_ : public Shape, public cv::Rect_<T> {
public:
	Rect_<T>();
    Rect_<T>(T x, T y, T width, T height);
	Rect_<T>(const cv::Rect_<T>& other);
    Rect_<T>(const Rect_& other);
    Rect_<T>(const CvRect& other);
	Rect_<T>(const cv::Point_<T>& org, const cv::Size_<T>& sz);
	Rect_<T>(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2);

	Rect_<T>& operator = (const cv::Rect_<T>& r );
    Rect_<T>& operator = (const Rect_<T>& r );
    
	cv::Point_<T> tl() const;     
	cv::Point_<T> br() const;

	cv::Size_<T> size() const;
    
	T area() const;
	
	operator Rect_<int>() const;
    operator Rect_<float>() const;
    operator Rect_<double>() const;
	operator cv::Rect_<T>() const;
    operator CvRect() const;

    // x <= pt.x && pt.x < x + width &&
    // y <= pt.y && pt.y < y + height ? true : false
	bool contains(const cv::Point_<T>& pt) const;


	virtual cv::Point2f centroid() const;
	virtual cv::Rect boundingRect() const;

	// Is this enough? What about sub-pixel stuff?
	virtual void getPixels(std::vector<cv::Point>& result) const;

	virtual bool operator ==(const Rect_<T>& other) const;
};

typedef Rect_<int> Rect; //! For convenience.

}

#endif