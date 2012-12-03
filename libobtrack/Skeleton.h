#ifndef _OBTRACK_SKELETON_H
#define _OBTRACK_SKELETON_H

#include "Shape.h"

namespace obt {

class JointInfo;

class Skeleton : public Shape {
public:
	/*! All the possible joints.
		Left and right sides refer to the _user's_ real-world
		left and right.
	*/
	enum Joint {
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

	/*! The number of possible different joints.
		It's the number of different possibilities on the Joints enum.
	*/
	static const int MAX_JOINTS = 24;

	/*! A function projecting a 3D point onto a 2D plane. */
	typedef cv::Point2f (*TransformationCallback)(const cv::Point3f&);

	explicit Skeleton(TransformationCallback cb = NULL);

	void activeJoints(std::vector<Joint>& out) const;
	const std::map<Joint, JointInfo>& getAllActiveJoints() const;

	bool isJointActive(Joint j) const;

	const JointInfo* getJointInfo(Joint j) const;

	cv::Point2f centroid() const;
	cv::Point3f centroid3D() const;

	cv::Rect boundingRect() const;
	cv::RotatedRect boundingRotatedRect() const;

	void getPixels(std::vector<cv::Point>& result) const;

private:
	friend class Tracker;

	/*! The active joints are backed by this map */
	std::map<Joint, JointInfo> joints;

	// TODO: add confidence thresholds
	
	/*! A callback for transforming 3D points into their equivalent image
		coordinates. If NULL, the skeleton is assumed to be 2D,
		already in image coordinates.
	*/
	TransformationCallback toVideoCoordinates;

	void getJointPositionsAndStats(std::vector<cv::Point3f>* points, 
		cv::Point3f* min, cv::Point3f* max, cv::Point3f* avg) const;
	void transformJointPositionsAndGetStats(const std::vector<cv::Point3f>& points3D,
		std::vector<cv::Point2f>* points2D, cv::Point2f* min, cv::Point2f* max) const;

	std::map<Joint, JointInfo>& getJointMap();
};

class JointInfo {
public:
	JointInfo(cv::Point3f position, float positionConfidence, cv::Mat orientation, float orientationConfidence);

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

}

#endif