#ifndef _OBTRACK_TRAINING_INFO_H
#define _OBTRACK_TRAINING_INFO_H

#include <vector>
#include <cv.h>

namespace obt {

class Shape;

class TrainingInfo {
public:	
	cv::Mat img; //! An image
	std::vector<Shape*> shapes; //! Info about the objects found in an image
};

}

#endif