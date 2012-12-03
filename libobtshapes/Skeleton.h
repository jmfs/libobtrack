#ifndef _OBTRACK_SKELETON_H	
#define _OBTRACK_SKELETON_H

#include "Shape.h"

namespace obt {

class JointInfo;

/*! Represents a human skeleton shape. It is made out of a number
	of joints, described in the \ref Skeleton::Joint enum.
*/
class Skeleton : public Shape {
public:
	/*! All the possible joints.
		Left and right sides refer to the _user's_ real-world
		left and right.
	*/
	static enum Joint {
		HEAD,
		NECK,  
		TORSO,
		WAIST,  
		LEFT_COLLAR,
		LEFT_SHOULDER,
		LEFT_ELBOW,
		LEFT_WRIST,  
		LEFT_HAND, 
		LEFT_FINGERTIP,
		RIGHT_COLLAR,
		RIGHT_SHOULDER,
		RIGHT_ELBOW,
		RIGHT_WRIST,  
		RIGHT_HAND, 
		RIGHT_FINGERTIP,
		LEFT_HIP,
		LEFT_KNEE,
		LEFT_ANKLE,
		LEFT_FOOT,
		RIGHT_HIP,
		RIGHT_KNEE,
		RIGHT_ANKLE,
		RIGHT_FOOT
	};

	Skeleton(bool is3D);

	/*! The number of possible different joints.
		It's the number of different possibilities on the Joints enum.
	*/
	static const int MAX_JOINTS = 24;

	bool isSkeleton() const;
	bool is3D() const;

	void activeJoints(std::vector<Joint>& out) const;
	const std::map<Joint, JointInfo>& getAllActiveJoints() const;

	bool isJointActive(Joint j) const;

	const JointInfo* getJointInfo(Joint j) const;

	cv::Point3f centroid() const;

	cv::Rect boundingRect() const;
	cv::RotatedRect boundingRotatedRect() const;

	void getPixels(std::vector<cv::Point>& result) const;

	std::map<Joint, JointInfo>& _getJointMap();
private:
	/*! The active joints are backed by this map */
	std::map<Joint, JointInfo> joints;

	bool _is3D;

	// TODO: add confidence thresholds

	void getJointPositionsAndStats(std::vector<cv::Point3f>* points, 
		std::vector<cv::Point2f>* points2D, 
		cv::Point3f* min, cv::Point3f* max, cv::Point3f* avg) const;	
};

/*! Stores information about a single skeleton joint.
	Includes its estimated position and orientation, as well as the confidence
	in each.
*/
class JointInfo {
public:
	JointInfo(const cv::Point3f& position, float positionConfidence, 
		const cv::Mat& orientation, float orientationConfidence);

	const cv::Point3f& position() const;
	float positionConfidence() const;
	const cv::Mat& orientation() const;
	float orientationConfidence() const;

private:
	cv::Point3f pos; //! The joint's position
	float posConfidence; //! The confidence in that position. 0 <= posConfidence <= 1
	/*! Matrix representing the joint's rotation.
		Like in OpenNI, the first column is the X orientation, where the value increases from left to right. 
		The second column is the Y orientation, where the value increases from bottom to top. 
		The third column is the Z orientation, where the value increases from near to far. 
	*/
	cv::Mat rot;
	float rotConfidence; //! The confidence the rotation. 0 <= rotConfidence <= 1

	//friend class Tracker;
	// setters
};

/*! Shorthand for std::map<obt::Skeleton::Joint, obt::JointInfo>
*/
typedef std::map<Skeleton::Joint, JointInfo> JointMap;

}

#endif
