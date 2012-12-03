#include "Kinect.h"    
#include "Blob.h"
#include <iostream>


namespace obt {


int KinectTracker::init() {
	XnStatus status;
	xn::EnumerationErrors errors;
	//TODO: Add the possibility for another XML file
	status = context.InitFromXmlFile("../kinect.xml", &errors);	

	if(status != XN_STATUS_OK)
		return status;	
	status = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthNode);
	if(status != XN_STATUS_OK)
		return status;

	status = context.FindExistingNode(XN_NODE_TYPE_USER, userNode);
	if(status != XN_STATUS_OK)
		return status;

	if (!userNode.IsCapabilitySupported(XN_CAPABILITY_SKELETON) ||
			!userNode.IsCapabilitySupported(XN_CAPABILITY_POSE_DETECTION)) {
		return -1; //TODO: Ability to work without these
	}

	// try to detect all joints
	userNode.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL); 

	/*	OpenNI API clarifications-that-should-have-been-in-the-docs:
	
		FoundUser is called when a user is found in the frame for the first time.
		LostUser is called either when the sensor is obscured, or ~10 seconds 
		after the user leaves the frame.
		
		When the user leaves the frame, her center of mass is reset to (0, 0, 0).
		It would also be useful to know if that coordinate system is 
		related to the video's pixel coordinates, though I suspect it isn't.

		Although skeleton data appears to be lost when the user leaves the frame
		in NITE's "Players" example, this is not the case.

		The matrix for joint orientation is a 9-element array. Is the array row-major,
		or column-major? This post (http://groups.google.com/group/openni-dev/browse_thread/thread/18ee244d608fa642)
		seems to imply it is row-major.
	*/
	
	userNode.RegisterUserCallbacks(
		KinectTracker::FoundUser, KinectTracker::LostUser, this, userCBs);
	userNode.GetSkeletonCap().RegisterCalibrationCallbacks(
		KinectTracker::CalibrationStarted, KinectTracker::CalibrationEnded, this, calibrationCBs);
	userNode.GetPoseDetectionCap().RegisterToPoseCallbacks(
		KinectTracker::PoseDetected, NULL, this, poseCBs);

	return status;
}

void XN_CALLBACK_TYPE KinectTracker::FoundUser(
			xn::UserGenerator& generator, XnUserID user, void* cookie) {
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);	

	printf("Found user %d\n", user);
	instance->userNode.GetPoseDetectionCap().StartPoseDetection("Psi", user);
	return;
}

void XN_CALLBACK_TYPE KinectTracker::LostUser(
			xn::UserGenerator& generator, XnUserID user, void* cookie) {
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);
	if(user == instance->skelUser)
		instance->skelUser = 0;
	instance->userNode.GetSkeletonCap().StopTracking(user);
	std::clog << "Lost user " << user << std::endl;
}

void XN_CALLBACK_TYPE KinectTracker::PoseDetected(
			xn::PoseDetectionCapability& pose, 
			const XnChar* strPose, XnUserID user, void* cookie) { 
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);
	instance->userNode.GetSkeletonCap().RequestCalibration(user, TRUE);
	instance->userNode.GetPoseDetectionCap().StopPoseDetection(user);
}

void XN_CALLBACK_TYPE KinectTracker::CalibrationStarted(
			xn::SkeletonCapability& skeleton, XnUserID user, void* cookie) {
	std::clog << "Calibrating for skeleton..." << std::endl;
}

void XN_CALLBACK_TYPE KinectTracker::CalibrationEnded(
			xn::SkeletonCapability& skeleton, 
			XnUserID user, XnBool bSuccess, void* cookie) {
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);
	std::clog << "Calibration done [" << user << "] " << (bSuccess?"":"un") << "successfully" << std::endl;
	if (bSuccess) {
		instance->userNode.GetSkeletonCap().SaveCalibrationData(user, 0);
		instance->userNode.GetSkeletonCap().StartTracking(user);
		instance->skelUser = user;
	}
	else
		instance->userNode.GetPoseDetectionCap().StartPoseDetection("Psi", user);
}


KinectTracker::KinectTracker(xn::Context& context):
		Tracker(false, false),
		context(context),
		userCBs(NULL),
		poseCBs(NULL),
		calibrationCBs(NULL),
		skelUser(0), // OpenNI users start from 1, so using 0 is safe.
		users(4) // Paraphrasing infamous words, this should be enough for everyone.
				// There isn't a problem if it's exceeded anyway.
{
}

KinectTracker::~KinectTracker() {
	userNode.GetSkeletonCap().UnregisterCalibrationCallbacks(calibrationCBs);
	userNode.GetPoseDetectionCap().UnregisterFromPoseCallbacks(poseCBs);
	userNode.UnregisterUserCallbacks(userCBs);
}

int KinectTracker::start(const TrainingInfo* ti, int idx) {
	started = true;
	return userNode.GetNumberOfUsers();
}

/*! This version of feed is peculiar, in that it ignores the
	provided image. \ref(Tracker::feed)

	\param img This parameter is ignored.
	\sa Tracker::feed
*/
int KinectTracker::feed(const cv::Mat& img) {
	int numUsers = userNode.GetNumberOfUsers();	
	if(numUsers == 0) {
		users.clear();
		return 0;
	}

	users.reserve(numUsers);
	for(int i = 0; i < users.size(); i++)
		users[i].clear();
	while(numUsers < users.size())
		users.pop_back();
	while(numUsers > users.size())
		users.push_back(Blob());


	/*bool* wasUserFound = new bool[numUsers]; // whether the user was found in the frame
	// remember that the user is only lost after she's out of the frame
	// for 10 seconds
	for(int i = 0; i < numUsers; i++)
		wasUserFound[i] = false;*/

	xn::SceneMetaData smd;
	userNode.GetUserPixels(0, smd);
	int maxX = smd.XRes();
	int maxY = smd.YRes();
	
	const XnLabel* labels = smd.Data();
	
	for(int x = 0; x < maxX; x++) {
		for(int y = 0; y < maxY; y++) {
			int userInThisPixel = labels[y * maxX + x];
			if(userInThisPixel != 0) {				
				//wasUserFound[userInThisPixel - 1] = true;

				// - 1, since users start at 1 and the array starts at 0
				users[userInThisPixel - 1].addPoint(x, y);
			}			
		}
	}

	/* // What to do about off-camera users? For now, ignore them.
	for(int i = 0; i < numUsers; i++) {
		if(!wasUserFound[i]) {
			XnPoint3D com;
			userNode.GetCoM(i + 1, com);
			// The user's position is (0, 0, 0) while she's off-camera, so this
			// would be pointless... maybe do something else here?
			users[i + 1].addPoint(com.X, com.Y);
			
		}			
	}

	delete[] wasUserFound;*/

	return numUsers;
}

void KinectTracker::objectShapes(std::vector<const Shape*>& shapes) const {
	for(int i = 0; i < users.size(); i++) {
		shapes.push_back(&(users[i]));
	}
}

}