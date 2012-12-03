#include "RotatedRect.h"
#include <iostream>

namespace obt {

RotatedRect::RotatedRect():
		cv::RotatedRect() {
}

RotatedRect::RotatedRect(const cv::Point2f& center, const cv::Size2f& size, float angle):
		cv::RotatedRect(center, size, angle) {
}

RotatedRect::RotatedRect(const CvBox2D& box):
		cv::RotatedRect(center, size, angle) {
}

RotatedRect::RotatedRect(const cv::RotatedRect& cvRotRect):
	cv::RotatedRect(cvRotRect) {
}

RotatedRect::operator cv::RotatedRect() const {
	return cv::RotatedRect(*this);
}

RotatedRect::operator CvBox2D() const {
	return cv::RotatedRect::operator CvBox2D();
}

void RotatedRect::points(cv::Point2f pts[]) const {
	cv::RotatedRect::points(pts);
}

cv::Point3f RotatedRect::centroid() const {
	return cv::Point3f(center); 
}

cv::Rect RotatedRect::boundingRect() const {
	return cv::RotatedRect::boundingRect();
}

cv::RotatedRect RotatedRect::boundingRotatedRect() const {
	return *this;
}

void RotatedRect::getPixels(std::vector<cv::Point>& result) const {
	//TODO: stub
	std::cerr << "obt::RotatedRect::getPixels: Not implemented, yet" << std::endl;
	assert(false);
	/*cv::Rect bb = boundingRect();
	
	cv::Point2f corners[4];
	points(corners);

	float m[4], b[4];	// The m and b in the line equation y = mx + b,
						// m being the slope and b the y-intercept.
						// One for each side.

	for(int i = 0; i < 4; i++) {
		int nextPoint = (i + 1) % 4;
		m[i] = (corners[nextPoint].y - corners[i].y) / 
			(corners[nextPoint].x - corners[i].x);
		
		cv::Point2f vec(corners[nextPoint] - corners[i]);
		b[i] = corners[nextPoint].y - corners[nextPoint].x * vec.y / vec.x ;
		// From the vectorial line equation, (x, y) = (x0, y0) + k(vx, vy),
		// for x = 0
	}


	for(int y = bb.y; y < bb.y + bb.height; y++) {


	}*/
}

}