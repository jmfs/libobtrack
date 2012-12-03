#include "Blob.h"
#include <algorithm>
#include <limits>
#include <list>
#include <cv.h>

namespace obt {

const int Blob::DEFAULT_CAPACITY = 100;

Blob::Blob(int capacity):
		pixels(capacity),
		cachedCentroid(NULL),
		cachedRR(cv::Point2f(), cv::Size2f(-1, -1), 0.0f),
		minX(std::numeric_limits<int>::min()) {
}



cv::Point2f Blob::centroid() const {
	if(cachedCentroid)
		return *cachedCentroid;

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

	if(!cachedCentroid)
		cachedCentroid = new cv::Point2f(centroidX, centroidY);
	else {
		cachedCentroid->x = centroidX;
		cachedCentroid->y = centroidY;
	}

	return cv::Point(static_cast<int>(cachedCentroid->x + 0.5f), 
		static_cast<int>(cachedCentroid->y + 0.5f));
}

cv::Rect Blob::boundingRect() const {
	return cv::Rect(minX, minY, maxX - minX, maxY - minY);
}


cv::RotatedRect Blob::boundingRotatedRect() const {
	if(cachedRR.size.width == -1 && !pixels.empty()) {
		cachedRR = cv::minAreaRect(cv::Mat(pixels));
	}

	return cachedRR;
}

void Blob::addPoint(int x, int y) {
	if(cachedCentroid) {
		delete cachedCentroid;
		cachedCentroid = NULL;
	}	
	cachedRR.size.width = -1; // Invalidate the cached RotatedRect

	if(pixels.empty()) {
		maxX = x;
		minX = x;
		maxY = y;
		minY = y;
	}
	else {
		maxX = std::max(x, maxX);
		minX = std::min(x, minX);
		maxY = std::max(y, maxY);
		minY = std::min(y, minY);
	}

	pixels.push_back(cv::Point(x, y));	
}

void Blob::clear() {
	if(cachedCentroid) {
		delete cachedCentroid;
		cachedCentroid = NULL;
	}
	minX = std::numeric_limits<int>::min();
	cachedRR.size.width = -1;
}

void Blob::getPixels(std::vector<cv::Point>& result) const {
	result.reserve(result.size() + pixels.size());
	result.insert(result.end(), pixels.begin(), pixels.end());
}
	
const std::vector<cv::Point>& Blob::getPixelsRef() const {
	return pixels;
}

std::list<cv::Point>::size_type Blob::size() {
	return pixels.size();
}

}