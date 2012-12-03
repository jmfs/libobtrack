#ifndef _OBTRACK_KINECT_H
#define _OBTRACK_KINECT_H

#include "Tracker.h"
#include "Blob.h"
#include "Skeleton.h"
#include "ShapeAlternatives.h"

#include <XnCppWrapper.h>
#include <map>

namespace obt {

/*! A tracker which gets user positions from OpenNI.
	Returns object shapes in both Blobs and Skeletons, if available.
	Only one skeleton can be tracked at a time, but as many as 5 blobs
	have been tested to be successfully tracked.
*/
class KinectTracker : public Tracker {
public:
	explicit KinectTracker(xn::Context& context);
	int init();

	const xn::Context& getContext() const;
	
	virtual int start(const TrainingInfo* ti = NULL, int idx = -1);	
	virtual int feed(const cv::Mat& img);

	virtual void objectShapes(std::vector<const Shape*>& shapes) const;
	virtual void objectShapes2D(std::vector<const Shape*>& shapes, int forImage = 0) const;

	~KinectTracker();

private:	
	static void XN_CALLBACK_TYPE FoundUser(xn::UserGenerator& generator, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE LostUser(xn::UserGenerator& generator, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE PoseDetected(xn::PoseDetectionCapability& pose, const XnChar* strPose, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE CalibrationStarted(xn::SkeletonCapability& skeleton, XnUserID user, void* instance);
	static void XN_CALLBACK_TYPE CalibrationEnded(xn::SkeletonCapability& skeleton, XnUserID user, XnBool bSuccess, void* instance);

	void updateSkeleton();

	/* Whether this tracker manages the OpenNI context's lifecycle.
		Defaults to true, but the option exists since the class' user may want
		to manage the context by himself, for instance, to access the data
		from the visible light camera.
	*/
	//bool manageContext; 

	/*! Reference to the OpenNI context.
		It is a reference, since this tracker can't possibly know whether the caller wishes
		to get data from the visible light camera, and because the caller
		knows best when to update the nodes.
	*/
	xn::Context& context;
	
	xn::UserGenerator userNode;		//! An OpenNI user generator
	xn::DepthGenerator depthNode;	//! An OpenNI depth generator

	std::vector<Blob> users;		//! The users vector

	XnCallbackHandle userCBs, calibrationCBs, poseCBs;

	/*! A user's skeleton, if it is detected. The Kinect can only reliably 
		detect and track one skeleton (at least using OpenNI, as of 2011-09-05),
		so only one is saved.
	*/
	Skeleton skel, skel2D;
	ShapeAlternatives userSkelAlternative, user2DSkelAlternative;

	int skelUser;		//! Which user the saved skeleton belongs to, or 0 if none
};

}

#endif