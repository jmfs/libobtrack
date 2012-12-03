#ifndef _OBTSHAPES_RECT_H
#define _OBTSHAPES_RECT_H

#include <cv.h>
#include <cmath>
#include "Shape.h"
#include "RotatedRect.h"

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
    Rect_<T>(const Rect_<T>& other);
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
	virtual cv::RotatedRect boundingRotatedRect() const;

	virtual void getPixels(std::vector<cv::Point>& result) const;

	virtual bool operator ==(const Rect_<T>& other) const;
};

typedef Rect_<int> Rect; //! For convenience.



template<typename T>
Rect_<T>::Rect_():
		cv::Rect_<T>() {
}

template<typename T> 
Rect_<T>::Rect_(T x, T y, T width, T height):
		cv::Rect_<T>(x, y, width, height) {
}

template<typename T> 
Rect_<T>::Rect_(const cv::Rect_<T>& cvCppRect):
		cv::Rect_<T>(cvCppRect) {
}

template<typename T> 
Rect_<T>::Rect_(const Rect_<T>& other):
		cv::Rect<T>(other) {
}

template<typename T> 
Rect_<T>::Rect_(const CvRect& cvRect):
		cv::Rect_<T>(cvRect) {
}

template<typename T>
Rect_<T>::Rect_(const cv::Point_<T>& org, const cv::Size_<T>& sz):
		cv::Rect_<T>(org, sz) {
}

template<typename T> 
Rect_<T>::Rect_(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2):
		cv::Rect_<T>(pt1, pt2) {
}

template<typename T> 
Rect_<T>& Rect_<T>::operator = (const cv::Rect_<T>& r ) {	
	cv::Rect_<T>::operator =(r);
	return *this;
}

template<typename T> 
Rect_<T>& Rect_<T>::operator = (const Rect_<T>& other ) {
	cv::Rect_<T>::operator =(other);
	return *this;
}
    
template<typename T> 
cv::Point_<T> Rect_<T>::tl() const {
	return cv::Rect_<T>::tl();
}

template<typename T> 
cv::Point_<T> Rect_<T>::br() const {
	return cv::Rect_<T>::br();
}

template<typename T> 
cv::Size_<T> Rect_<T>::size() const {
	return cv::Rect_<T>::size();
}
    
template<typename T> 
T Rect_<T>::area() const {
	return cv::Rect_<T>::area();
}

template<typename T> 
Rect_<T>::operator Rect_<int>() const {
	return Rect_<int>(x, y, width, height);
}

template<typename T> 
Rect_<T>::operator Rect_<float>() const {
	return Rect_<float>(x, y, width, height);
}

template<typename T> 
Rect_<T>::operator Rect_<double>() const {
	return Rect_<double>(x, y, width, height);
}

template<typename T> 
Rect_<T>::operator cv::Rect_<T>() const {
	return cv::Rect_<T>(*this);
}

template<typename T> 
Rect_<T>::operator CvRect() const {
	return cv::Rect::operator CvRect();
}

template<typename T> 
bool Rect_<T>::contains(const cv::Point_<T>& pt) const {
	return cv::Rect::contains(pt);
}

template<typename T> 
cv::Point2f Rect_<T>::centroid() const {
	return cv::Point2f(
		static_cast<float>(x + width / 2.0f), 
		static_cast<float>(y + height / 2.0f));
}

template<typename T> 
cv::Rect Rect_<T>::boundingRect() const {
	// Make sure x and y are rounded down, while width and height are rounded up,
	// so as to have a rectangle that properly bounds the original, in case
	// we are converting from floating point to integer.
	int newX = static_cast<int>(x);
	int newY = static_cast<int>(y);

	// compensate for the lost space in the previous rounding down, and round that up
	int newWidth = static_cast<int>(width + x - newX + 0.5);
	int newHeight = static_cast<int>(height + y - newY + 0.5);

	return cv::Rect(newX, newY, newWidth, newHeight);	
}

template<typename T>
cv::RotatedRect Rect_<T>::boundingRotatedRect() const {
	return RotatedRect(*this);
}

/*! Gets the pixels that are completely inside the rectangle.
	\param result The pixels will be appended to this vector.
*/
template<typename T> 
void Rect_<T>::getPixels(std::vector<cv::Point>& result) const {
	// only include pixels completely inside the rectangle
	int yStart = static_cast<int>(this->y + 0.5);
	int yEnd = static_cast<int>(this->y + this->height);
	int xStart = static_cast<int>(this->x + 0.5);
	int xEnd = static_cast<int>(this->x + this->width);
	for(int y = yStart; y <= yEnd; y++)
		for(int x = xStart; x <= xEnd; x++)
			result.push_back(cv::Point(x, y));
}

template<typename T> 
bool Rect_<T>::operator ==(const Rect_<T>& other) const {
	return x == other.x && y == other.y && 
		width == other.width && height == other.height;

}

} // namespace obt

#endif