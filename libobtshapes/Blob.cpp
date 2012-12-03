#include "Blob.h"
#include "helpers.h"
#include <algorithm>
#include <limits>
#include <list>
#include <cv.h>

namespace obt {

const int Blob::DEFAULT_CAPACITY = 100;

Blob::Blob(int capacity) {
	minX = minY = std::numeric_limits<int>::max();
	maxX = maxY = std::numeric_limits<int>::min();
	pixels.reserve(capacity);
}

/*! A Blob is considered invalid if it is empty.
	\return true if this Blob has no pixels, false otherwise.
*/
bool Blob::isInvalid() const {
	return pixels.empty();
}

cv::Point3f Blob::centroid() const {
	long long xSum = 0;
	long long ySum = 0;
	Pixels::const_iterator i = pixels.begin();
	Pixels::size_type size = pixels.size();
	for( ; i != pixels.end(); i++) {
		xSum += (*i).x;
		ySum += (*i).y;
	}

	float centroidX = xSum / static_cast<float>(size);
	float centroidY = ySum / static_cast<float>(size);
		
	return cv::Point3f(centroidX, centroidY, 0.0f);
}

cv::Rect Blob::boundingRect() const {
	if(pixels.empty())
		return INVALID_RECT;

	return cv::Rect(minX, minY, maxX - minX, maxY - minY);
}

// This runs _really_ slowly on Debug builds, for some reason. 
// Most of the time is spent at cv::AutoBuffer::AutoBuffer .
cv::RotatedRect Blob::boundingRotatedRect() const {
	if(pixels.empty())
		return INVALID_ROTATED_RECT;

	return cv::minAreaRect(cv::Mat(pixels, false));
}

/*! Adds a point to this Blob
*/
void Blob::addPoint(int x, int y) {
	maxX = std::max(x, maxX);
	minX = std::min(x, minX);
	maxY = std::max(y, maxY);
	minY = std::min(y, minY);
	
	pixels.push_back(cv::Point(x, y));	
}

void Blob::clear() {
	minX = minY = std::numeric_limits<int>::max();
	maxX = maxY = std::numeric_limits<int>::min();
	pixels.clear();
}

/*! Adds this Blob's pixels to result.

	\param result Output. This Blob's pixels will be added to it.
*/
void Blob::getPixels(std::vector<cv::Point>& result) const {
	result.reserve(result.size() + pixels.size());
	result.insert(result.end(), pixels.begin(), pixels.end());
}

/*! Gets a const reference to the actual \ref pixels vector
*/
const std::vector<cv::Point>& Blob::getPixelsRef() const {
	return pixels;
}

/*! Returns the number of pixels in this Blob
*/
std::vector<cv::Point>::size_type Blob::size() const {
	return pixels.size();
}

}