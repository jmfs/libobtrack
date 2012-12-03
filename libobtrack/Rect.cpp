#include "Rect.h"

namespace obt {

template<typename T> 
Rect_<T>::Rect_():
		cv::Rect<T>() {
}

template<typename T> 
Rect_<T>::Rect_(T x, T y, T width, T height):
		cv::Rect<T>(x, y, width, height) {
}

template<typename T> 
Rect_<T>::Rect_(const cv::Rect_<T>& cvCppRect):
		cv::Rect<T>(cvCppRect) {
}

template<typename T> 
Rect_<T>::Rect_(const Rect_<T>& other):
		cv::Rect<T>(other) {
}

template<typename T> 
Rect_<T>::Rect_(const CvRect& cvRect):
		cv::Rect<T>(cvRect) {
}

template<typename T>
Rect_<T>::Rect_(const cv::Point_<T>& org, const cv::Size_<T>& sz):
		cv::Rect<T>(org, sz) {
}

template<typename T> 
Rect_<T>::Rect_(const cv::Point_<T>& pt1, const cv::Point_<T>& pt2):
		cv::Rect<T>(pt1, pt2) {
}

template<typename T> 
Rect_<T>& Rect_<T>::operator = (const cv::Rect_<T>& r ) {
		return Rect_<T>(r);
}

template<typename T> 
Rect_<T>& Rect_<T>::operator = (const Rect_<T>& other ) {
	return Rect_<T>(other);
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
	return cv::Point_<T>((left + right) / 2, (top + bottom) / 2);
}

template<typename T> 
cv::Rect Rect_<T>::boundingRect() const {
	// Make sure x and y are rounded down, while width and height are rounded up,
	// so as to have a rectangle that properly bounds the original, in case
	// we are converting from floating point to integer.
	return cv::Rect(x, y, width + 0.6f, height + 0.6f);
}

template<typename T> 
void Rect_<T>::getPixels(std::vector<cv::Point>& result) const {
	//FIXME: Needs checking for floating point values
	for(int y = top; y <= bottom; y++)
		for(int x = left; x <= right; x++)
			result.push_back(cv::Point(x, y));
}

template<typename T> 
bool Rect_<T>::operator ==(const Rect_<T>& other) const {
	return left == other.left && top == other.top && 
		right == other.right && bottom == other.bottom;

}

} // namespace obt
