#ifndef _OBTRACK_KINECT_H
#define _OBTRACK_KINECT_H

#include "Tracker.h"
#include "Blob.h"
#include <XnCppWrapper.h>
#include <map>

namespace obt {

class KinectTracker : public Tracker {
	KinectTracker();
	
	virtual int start(const cv::Mat& img);
	
	virtual int feed(const cv::Mat& img);

	virtual void objectShapes(std::vector<const Shape*>& shapes) const;

	~KinectTracker();



private:
	static void XN_CALLBACK_TYPE FoundUser(xn::UserGenerator& generator, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE CalibrationEnded(xn::SkeletonCapability& skeleton, XnUserID user, XnBool bSuccess, void* instance);
	xn::Context context;			//! The OpenNI context
	xn::UserGenerator userNode;		//! An OpenNI user generator
	xn::DepthGenerator depthNode;	//! An OpenNI depth generator

	std::vector<Blob> users;		//! The users vector
	/*! A copy of the users vector, which backs up the vector returned in objectShapes.
		This is required, since this tracker works through asynchronous callbacks, and the content of
		users can change between calling objectShapes and feed, in the next frame

		\sa objectShapes
		\sa feed
	*/

	//TODO: Store the skeleton

	std::map<int, int> userCalibrations;
	std::map<int, int> calibrationUsers;
};

}

#endif