#include "Kinect.h"
#include <iostream>

namespace obt {


void XN_CALLBACK_TYPE KinectTracker::FoundUser(
			xn::UserGenerator& generator, XnUserID user, void* cookie) {
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);	
	std::cerr << "Found user " << user << std::endl;
	if(instance->getSkeleton)
		instance->userNode.GetPoseDetectionCap().StartPoseDetection("Psi", user);
	return;
}

void XN_CALLBACK_TYPE KinectTracker::LostUser(
			xn::UserGenerator& generator, XnUserID user, void* cookie) {
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);
	if(instance->getSkeleton) {
		if(user == instance->skelUser)
			instance->skelUser = 0;
		instance->userNode.GetSkeletonCap().StopTracking(user);
	}
	std::cerr << "Lost user " << user << std::endl;
}

void XN_CALLBACK_TYPE KinectTracker::PoseDetected(
			xn::PoseDetectionCapability& pose, 
			const XnChar* strPose, XnUserID user, void* cookie) { 
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);
	assert(instance->getSkeleton);
	instance->userNode.GetSkeletonCap().RequestCalibration(user, TRUE);
	instance->userNode.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE KinectTracker::CalibrationStarted(
			xn::SkeletonCapability& skeleton, XnUserID user, void* cookie) {
	std::cerr << "Calibrating for skeleton..." << std::endl;
	assert(static_cast<KinectTracker*>(cookie)->getSkeleton);
}

void XN_CALLBACK_TYPE KinectTracker::CalibrationEnded(
			xn::SkeletonCapability& skeleton, 
			XnUserID user, XnBool bSuccess, void* cookie) {
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);
	assert(instance->getSkeleton);
	std::cerr << "Calibration done [" << user << "] " << (bSuccess?"":"un") << "successfully" << std::endl;
	if (bSuccess) {
		instance->userNode.GetSkeletonCap().SaveCalibrationData(user, 0);
		instance->userNode.GetSkeletonCap().StartTracking(user);
		instance->skelUser = user;
		const Shape* userBlob = &(instance->users[user - 1]);
		instance->userSkelAlternative.members[0] = userBlob;
		instance->userSkelAlternative.members[1] = 
			static_cast<const Shape*>(&instance->skel);
		instance->user2DSkelAlternative.members[0] = userBlob;
		instance->user2DSkelAlternative.members[1] = 
			static_cast<const Shape*>(&instance->skel2D);
	}
	else
		instance->userNode.GetPoseDetectionCap().StartPoseDetection("Psi", user);
}


/*! Constructs a new KinectTracker, which will use a reference to
	the provided context to get the	data it will need.

	\param context A reference to an OpenNI context. 
		This context must be initialized prior to the call to init.
	\param wantSkeleton Whether the Kinect should try to detect a user's skeleton.
*/
KinectTracker::KinectTracker(xn::Context& context, bool wantSkeleton):
		Tracker(false, false),
		context(context),
		getSkeleton(wantSkeleton),
		userCBs(NULL),
		poseCBs(NULL),
		calibrationCBs(NULL),
		skelUser(0), // OpenNI users start from 1, so using 0 for "no user" is safe.
		skel(true),
		skel2D(false),
		wasInit(false),
		userSkelAlternative(2),
		user2DSkelAlternative(2) {
	users.reserve(4); // Paraphrasing infamous words, this should be enough for everyone.
				// There isn't a big problem if it's exceeded anyway.
}

/*! Initializes OpenNI.
	\return XN_STATUS_OK if everything went right, -XnStatus (see OpenNI docs) if something went wrong, or 
		\ref SKELETON_NOT_AVAILABLE, if the device doesn't support pose estimation/skeleton detection. 
		In this last case, the tracker can still be used, albeit without that capability. */
int KinectTracker::init() {
	XnStatus status = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthNode);
	if(status != XN_STATUS_OK) 
		goto initFailure;

	status = context.FindExistingNode(XN_NODE_TYPE_USER, userNode);
	if(status != XN_STATUS_OK)
		goto initFailure;

	wasInit = true;
	if(getSkeleton == true && (!userNode.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
			!userNode.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION))) {
		getSkeleton = false;
		status = SKELETON_NOT_AVAILABLE;
	}

	userNode.RegisterUserCallbacks(
		KinectTracker::FoundUser, KinectTracker::LostUser, this, userCBs);

	if(getSkeleton) {
		// try to detect all joints
		userNode.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL); 

		/*	OpenNI API clarifications-that-should-have-been-in-the-docs:
	
			FoundUser is called when a user is found in the frame for the first time.
			LostUser is called either when the sensor is obscured, or ~10 seconds 
			after the user leaves the frame.
		
			When the user leaves the frame, her center of mass is reset to (0, 0, 0).

			In NITE's "Players" example, although skeleton data appears to be lost 
			when the user leaves the frame, this is not the case.

			The matrix for joint orientation is a 9-element array. Is the array row-major,
			or column-major? This post (http://groups.google.com/group/openni-dev/browse_thread/thread/18ee244d608fa642)
			seems to imply it is row-major.
		*/
	
		userNode.GetSkeletonCap().RegisterCalibrationCallbacks(
			KinectTracker::CalibrationStarted, KinectTracker::CalibrationEnded, this, calibrationCBs);
		userNode.GetPoseDetectionCap().RegisterToPoseCallbacks(
			KinectTracker::PoseDetected, NULL, this, poseCBs);
	}

	return status;

initFailure:
	wasInit = false;
	return -static_cast<int>(status);
}

KinectTracker::~KinectTracker() {
	if(getSkeleton) {
		userNode.GetSkeletonCap().UnregisterCalibrationCallbacks(calibrationCBs);
		userNode.GetPoseDetectionCap().UnregisterFromPoseCallbacks(poseCBs);
	}
	userNode.UnregisterUserCallbacks(userCBs);
}

int KinectTracker::start(const TrainingInfo* ti, int idx) {
	if(!wasInit)
		return INIT_NEEDED;
	started = true;
	return userNode.GetNumberOfUsers();
}

void KinectTracker::updateSkeleton() {
	assert(getSkeleton);
	if(skelUser == 0)
		return;
	XnSkeletonJoint openNIJoints[Skeleton::MAX_JOINTS];
	XnUInt16 numJoints = Skeleton::MAX_JOINTS;
	xn::SkeletonCapability cap = userNode.GetSkeletonCap();
	cap.EnumerateActiveJoints(openNIJoints, numJoints);
	std::map<Skeleton::Joint, JointInfo>& joints = skel._getJointMap();
	std::map<Skeleton::Joint, JointInfo>& joints2D = skel2D._getJointMap();
	/*	Nuke the maps from orbit. It's not the only way to be sure, and probably not 
		the most efficient thing to do, but it's easier than finding out which joints 
		have become inactive and deleting just those. */
	joints.clear(); 
	joints2D.clear();
	for(int i = 0; i < numJoints; i++) {
		XnSkeletonJointTransformation jointTransform;
		cap.GetSkeletonJoint(skelUser, openNIJoints[i], jointTransform);

		XnVector3D& openNIPosition = jointTransform.position.position;
		const cv::Point3f position(openNIPosition.X, openNIPosition.Y, openNIPosition.Z);
		depthNode.ConvertRealWorldToProjective(1, &openNIPosition, &openNIPosition);
		const cv::Point3f pos2D(openNIPosition.X, openNIPosition.Y, 0.0f);		
		const float posConfidence = jointTransform.position.fConfidence;
		
		const XnFloat* openNIRotation = jointTransform.orientation.orientation.elements;
		cv::Mat rotation(3, 3, CV_32FC1);
		// Both the OpenNI and the OpenCV matrices are stored in row-major order, so the
		// following works out nicely
		memcpy(rotation.data, openNIRotation, 9*sizeof(float)); 
		const float rotConfidence = jointTransform.orientation.fConfidence;

		/* Skeleton joints were taken from OpenNI, but OpenNI's start at 1, 
			while libobtrack's start at the C++ default of 0, hence the -1.
		*/
		const Skeleton::Joint curJoint = static_cast<Skeleton::Joint>(openNIJoints[i] - 1);
		joints.insert(std::make_pair(curJoint, 
			JointInfo(position, posConfidence, rotation, rotConfidence)));
		// TODO: Find a way to convert the rotations to 2D and pass them on
		joints2D.insert(std::make_pair(curJoint,
			JointInfo(pos2D, posConfidence, cv::Mat(), 0.0f)));
	}
}

/*! This version of feed is peculiar, in that it ignores the
	provided image.

	\param img This parameter is ignored.
	\sa Tracker::feed
*/
int KinectTracker::feed(const cv::Mat& img) {
	if(!wasInit)
		return INIT_NEEDED;
	XnUInt16 numUsers = userNode.GetNumberOfUsers();	
	if(numUsers == 0) {
		users.clear();
		return 0;
	}

	std::vector<XnUserID> userIDs(numUsers);
	userNode.GetUsers(userIDs.data(), numUsers);

	XnUserID maxUserID = *std::max_element(userIDs.begin(), userIDs.end());

	users.reserve(maxUserID);
	while(maxUserID < users.size())
		users.pop_back();
	for(size_t i = 0; i < users.size(); i++)
		users[i].clear();
	while(maxUserID > users.size())
		users.push_back(Blob());
	
	xn::SceneMetaData smd;
	userNode.GetUserPixels(0, smd);
	unsigned int maxX = smd.XRes();
	unsigned int maxY = smd.YRes();
	
	const XnLabel* labels = smd.Data();	
	for(size_t x = 0; x < maxX; x++) {
		for(size_t y = 0; y < maxY; y++) {
			int userInThisPixel = labels[y * maxX + x];
			if(userInThisPixel != 0)				
				users[userInThisPixel - 1].addPoint(x, y);
		}
	}

	if(skelUser != 0)
		updateSkeleton();

	return numUsers;
}

void KinectTracker::objectShapes(std::vector<const Shape*>& shapes) const {
	shapes.reserve(shapes.size() + users.size());
	for(size_t i = 0; i < users.size(); i++) {
		if(skelUser > 0 && i == skelUser - 1)
			shapes.push_back(&userSkelAlternative);
		else
			shapes.push_back(&(users[i]));
	}
}

void KinectTracker::objectShapes2D(std::vector<const Shape*>& shapes, int forImage) const {
	shapes.reserve(shapes.size() + users.size());
	for(size_t i = 0; i < users.size(); i++) {
		if(skelUser > 0 && i == skelUser - 1)
			shapes.push_back(&user2DSkelAlternative);
		else
			shapes.push_back(&(users[i]));
	}
}

}