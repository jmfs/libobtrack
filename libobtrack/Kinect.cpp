#include "Kinect.h"    
#include "Blob.h"
#include <iostream>

namespace obt {

int* Skeleton::obtrackOpenNIEquivalents = NULL;

void XN_CALLBACK_TYPE KinectTracker::FoundUser(
			xn::UserGenerator& generator, XnUserID user, void* cookie) {
	KinectTracker* instance = static_cast<KinectTracker*>(cookie);	

	std::clog << "Found user " << user << std::endl;
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
		skelUser(0), // OpenNI users start from 1, so using 0 for "no user" is safe.
		users(4) // Paraphrasing infamous words, this should be enough for everyone.
				// There isn't a problem if it's exceeded anyway.
{
	if(obtrackOpenNIEquivalents[XN_SKEL_HEAD] != Skeleton::Joint::HEAD) {
		obtrackOpenNIEquivalents[XN_SKEL_HEAD] = Skeleton::Joint::HEAD;
		obtrackOpenNIEquivalents[XN_SKEL_NECK] = Skeleton::Joint::NECK;
		obtrackOpenNIEquivalents[XN_SKEL_TORSO] = Skeleton::Joint::TORSO;
		obtrackOpenNIEquivalents[XN_SKEL_WAIST] = Skeleton::Joint::WAIST;

		obtrackOpenNIEquivalents[XN_SKEL_LEFT_COLLAR] = Skeleton::Joint::LEFT_COLLAR;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_SHOULDER] = Skeleton::Joint::LEFT_SHOULDER;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_ELBOW] = Skeleton::Joint::LEFT_ELBOW;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_WRIST] = Skeleton::Joint::LEFT_WRIST;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_HAND] = Skeleton::Joint::LEFT_HAND;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_FINGERTIP] = Skeleton::Joint::LEFT_FINGERTIP;

		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_COLLAR] = Skeleton::Joint::RIGHT_COLLAR;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_SHOULDER] = Skeleton::Joint::RIGHT_SHOULDER;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_ELBOW] = Skeleton::Joint::RIGHT_ELBOW;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_WRIST] = Skeleton::Joint::RIGHT_WRIST;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_HAND] = Skeleton::Joint::RIGHT_HAND;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_FINGERTIP] = Skeleton::Joint::RIGHT_FINGERTIP;

		obtrackOpenNIEquivalents[XN_SKEL_LEFT_HIP] = Skeleton::Joint::LEFT_HIP;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_KNEE] = Skeleton::Joint::LEFT_KNEE;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_ANKLE] = Skeleton::Joint::LEFT_ANKLE;
		obtrackOpenNIEquivalents[XN_SKEL_LEFT_FOOT] = Skeleton::Joint::LEFT_FOOT;

		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_HIP] = Skeleton::Joint::RIGHT_HIP;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_KNEE] = Skeleton::Joint::RIGHT_KNEE;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_ANKLE] = Skeleton::Joint::RIGHT_ANKLE;
		obtrackOpenNIEquivalents[XN_SKEL_RIGHT_FOOT] = Skeleton::Joint::RIGHT_FOOT;
	}
}

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

KinectTracker::~KinectTracker() {
	userNode.GetSkeletonCap().UnregisterCalibrationCallbacks(calibrationCBs);
	userNode.GetPoseDetectionCap().UnregisterFromPoseCallbacks(poseCBs);
	userNode.UnregisterUserCallbacks(userCBs);
}

int KinectTracker::start(const TrainingInfo* ti, int idx) {
	started = true;
	return userNode.GetNumberOfUsers();
}

void KinectTracker::updateSkeleton() {
	if(skelUser == 0)
		return;
	XnSkeletonJoint openNIJoints[Skeleton::MAX_JOINTS];
	XnUInt16 numJoints = Skeleton::MAX_JOINTS;
	xn::SkeletonCapability cap = userNode.GetSkeletonCap();
	cap.EnumerateActiveJoints(openNIJoints, numJoints);
	std::map<Skeleton::Joint, JointInfo>& joints = skel._getJointMap();
	joints.clear(); // nuke the map from orbit. It's not the only way to be sure, nor the most 
	// efficient thing to do, but it's easier than finding out which joints have 
	// become inactive and deleting just those.
	for(int i = 0; i < numJoints; i++) {
		XnSkeletonJointTransformation jointTransform;
		cap.GetSkeletonJoint(skelUser, openNIJoints[i], jointTransform);
		const XnVector3D& openNIPosition = jointTransform.position.position;
		const cv::Point3f position(openNIPosition.X, openNIPosition.Y, openNIPosition.Z);
		const float posConfidence = jointTransform.position.fConfidence;
		const XnFloat* openNIRotation = jointTransform.orientation.orientation.elements;
		cv::Mat rotation(3, 3, CV_32FC1);

		// Both the OpenNI and the OpenCV matrices are stored in row-major order, so the
		// following works out nicely
		memcpy(rotation.data, openNIRotation, 9*sizeof(float)); 
		const float rotConfidence = jointTransform.orientation.fConfidence;

		joints.insert(
			std::make_pair(obtrackOpenNIEquivalents[openNIJoints[i]],
				JointInfo(position, posConfidence, rotation, rotConfidence))
			);
	}
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

	if(skelUser != 0)
		updateSkeleton();

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