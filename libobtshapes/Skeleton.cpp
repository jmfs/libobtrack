#include "Skeleton.h"
#include "helpers.h"

namespace obt {

Skeleton::Skeleton(bool is3D):
		_is3D(is3D) {
}

bool Skeleton::isSkeleton() const {
	return true;
}

/*! Returns whether this skeleton's joint coordinates are in 3D.
*/
bool Skeleton::is3D() const {
	return _is3D;
}
	
/*! Adds the currently active joints to out. out is cleared before this operation. */
void Skeleton::activeJoints(std::vector<Joint>& out) const {
	out.clear();
	std::map<Joint, JointInfo>::const_iterator it = joints.begin();
	while(it != joints.end()) {
		out.push_back(it->first);
		it++;
	}
}
	
const std::map<Skeleton::Joint, JointInfo>& Skeleton::getAllActiveJoints() const {
	return joints;
}

bool Skeleton::isJointActive(Joint j) const {
	return joints.find(j) != joints.end();
}

/*! Returns the joint info for joint j, or NULL if that joint isn't active.
*/
const JointInfo* Skeleton::getJointInfo(Joint j) const {
	std::map<Joint, JointInfo>::const_iterator it = joints.find(j);
	if(it == joints.end())
		return NULL;
	else
		return &it->second;
}

/*! Gets joint positions and stats. All the parameters are for output, and can be NULL if 
	the caller is not interested in them.
	and only joints for which the confidence is greater than 50% are considered.
	If no joints meeting said criteria are found, the following things symbolize the error:
	*avg will equal INVALID_POINT_3D, *min will be greater than *max, and *points will
	be empty.
*/
void Skeleton::getJointPositionsAndStats(
		std::vector<cv::Point3f>* points,  std::vector<cv::Point2f>* points2D, 
		cv::Point3f* min, cv::Point3f* max, cv::Point3f* avg) const {
	if(points != NULL && min != NULL && max != NULL && avg != NULL)
		return;
		
	double xAccum = 0.0;
	double yAccum = 0.0;
	double zAccum = 0.0;
	int numJoints = 0;
	if(points != NULL)
		points->clear();
	if(points2D != NULL)
		points2D->clear();
	if(min != NULL)
		min->x = min->y = min->z = std::numeric_limits<float>::max();
	if(max != NULL)
		max->x = max->y = max->z = std::numeric_limits<float>::min();

	std::map<Joint, JointInfo>::const_iterator it;
	for(it = joints.begin(); it != joints.end(); it++) {
		if(it->second.positionConfidence() < 0.5)
			continue;

		cv::Point3f pos = it->second.position();
		if(points != NULL) {
			points->push_back(pos);
		}
		if(points2D != NULL) {
			points2D->push_back(cv::Point2f(pos.x, pos.y));
		}
		if(min != NULL) {
			min->x = std::min(min->x, pos.x);
			min->y = std::min(min->y, pos.y);
			min->z = std::min(min->z, pos.z);
		}
		if(max != NULL) {
			max->x = std::max(max->x, pos.x);
			max->y = std::max(max->y, pos.y);
			max->z = std::max(max->z, pos.z);
		}
		if(avg != NULL) {
			xAccum += pos.x;
			yAccum += pos.y;
			zAccum += pos.z;
			numJoints++;
		}		
	}

	if(avg != NULL) {
		if(numJoints == 0) {
			*avg = INVALID_POINT_3D;
		}
		else {		
			avg->x = static_cast<float>(xAccum / numJoints);
			avg->y = static_cast<float>(yAccum / numJoints);
			avg->z = static_cast<float>(zAccum / numJoints);
		}
	}
}


/*! Returns the skeleton's centroid's position.
	Defining a skeleton's centroid is problematic. In this case,
	it is taken as being:
	a) The torso center, if that joint is active.
	b) An estimation of the torso center, if the left and right hips and shoulders
	   are active.
	c) The average of all joint positions, otherwise.

	Note that only joints with at least 50%	confidence in their position 
	are used for these calculations.
*/
cv::Point3f Skeleton::centroid() const {
	const JointInfo* torso = getJointInfo(TORSO);
	if(torso != NULL && torso->positionConfidence() >= 0.5)
		return torso->position();
	
	const JointInfo* ls = getJointInfo(LEFT_SHOULDER);
	const JointInfo* rs = getJointInfo(RIGHT_SHOULDER);
	const JointInfo* lh = getJointInfo(LEFT_HIP);
	const JointInfo* rh = getJointInfo(RIGHT_HIP);

	if(ls != NULL && ls->positionConfidence() >= 0.5 &&
			rs != NULL && rs->positionConfidence() >= 0.5 &&
			lh != NULL && lh->positionConfidence() >= 0.5 &&
			rh != NULL && rh->positionConfidence() >= 0.5) {
		cv::Point3f lsp = ls->position();
		cv::Point3f rsp = rs->position();
		cv::Point3f lhp = lh->position();
		cv::Point3f rhp = rh->position();

		float xAvg = (lsp.x + rsp.x + lhp.x + rhp.x) / 4;
		float yAvg = (lsp.y + rsp.y + lhp.y + rhp.y) / 4;
		float zAvg = (lsp.z + rsp.z + lhp.z + rhp.z) / 4;

		return cv::Point3f(xAvg, yAvg, zAvg);
	}

	cv::Point3f result;
	getJointPositionsAndStats(NULL, NULL, NULL, NULL, &result);
	return result;
}

/*! Gets a bounding rectangle for all active joints with greater than 50% confidence
	in their position. If no such joints exist, returns cv::Rect(INT_MAX, INT_MAX, 0, 0).
*/
cv::Rect Skeleton::boundingRect() const {
	if(_is3D)
		return INVALID_RECT;

	cv::Point3f min, max;
	getJointPositionsAndStats(NULL, NULL, &min, &max, NULL);
	if(min.x > max.x) // no joints with >= 0.5 confidence were found
		return INVALID_RECT;
	// Make sure the rectangle always contains both points
	return cv::Rect(
		static_cast<int>(std::floor(min.x)), 
		static_cast<int>(std::floor(min.y)), 
		static_cast<int>(std::ceil(max.x - min.x)), 
		static_cast<int>(std::ceil(max.y - min.y)));
}

/*! Gets the smallest (not necessarily axis-aligned) rectangle containing all active joints with 
	greater than 50% confidence	in their position. If no such joints exist, 
	returns INVALID_ROTATED_RECT.
*/
cv::RotatedRect Skeleton::boundingRotatedRect() const {
	if(_is3D)
		return INVALID_ROTATED_RECT;

	std::vector<cv::Point2f> points;
	
	getJointPositionsAndStats(NULL, &points, NULL, NULL, NULL);
	if(points.empty())
		return INVALID_ROTATED_RECT;

	return cv::minAreaRect(cv::Mat(points, false));
}

void Skeleton::getPixels(std::vector<cv::Point>& result) const {
	// TODO: stub
}

/*! Gets a writable reference to the joint map.
	This shouldn't be used outside of a \ref Tracker.
	Use \ref Skeleton::getAllActiveJoints instead.
*/
std::map<Skeleton::Joint, JointInfo>& Skeleton::_getJointMap() {
	return joints;
}

JointInfo::JointInfo(const cv::Point3f& position, float positionConfidence, 
			const cv::Mat& orientation, float orientationConfidence):
		pos(position),
		posConfidence(positionConfidence),
		rot(orientation),
		rotConfidence(orientationConfidence) {
}

const cv::Point3f& JointInfo::position() const {
	return pos;
}

float JointInfo::positionConfidence() const {
	return posConfidence;
}

const cv::Mat& JointInfo::orientation() const {
	return rot;
}

float JointInfo::orientationConfidence() const {
	return rotConfidence;
}

}