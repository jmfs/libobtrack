#include "YAFT.h"
#include "CvPixelBackgroundGMM.h"
#include <cv.h>
#include <iostream>

namespace obt {

YAFT::YAFT():
		Tracker(),
		bgModel(NULL),
		bgData(NULL){
}

int YAFT::start(const cv::Mat& img) {
	if(bgModel != NULL)
		cvReleasePixelBackgroundGMM(&bgModel);
	
	bgModel = cvCreatePixelBackgroundGMM(img.cols, img.rows);
	bgModel->fAlphaT = 0.005f;
	bgModel->fTb = 12;
	bgData = (unsigned char*)malloc(img.rows * img.cols);
	//bgData.create(img.rows, img.cols, CV_8UC1);

	return 1; // TODO: Change this
}

int YAFT::feed(const cv::Mat& img) {
	if(!started) {
		start(img);
		started = true;
	}
		
	cvUpdatePixelBackgroundGMM(bgModel, img.data, bgData);
	
	// Find connected components in the image
	// TODO: Find out if using an adaptive threshold, 
	//		in order to include shadows near to the foreground, works better
	/*cv::threshold(bgData, bgData, 200, 255, CV_THRESH_BINARY); 
	
	contours.clear();
	contourHierarchy.clear();

	cv::findContours(bgData, contours, contourHierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);	*/
	return 1; // TODO: Change this
}

void YAFT::stopTracking() {
	// TODO
}

void YAFT::objectShapes(std::vector<const Shape*>& shapes) const {
	// TODO
}

YAFT::~YAFT() {
	if(bgModel != NULL)
		cvReleasePixelBackgroundGMM(&bgModel);
}

}