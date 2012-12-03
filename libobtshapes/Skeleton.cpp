#include "Skeleton.h"

namespace obt {

//const int Skeleton::MAX_JOINTS = 24;

Skeleton::Skeleton(TransformationCallback cb):
		toVideoCoordinates(cb) {	
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
	*avg will be (FLOAT_MAX, FLOAT_MAX, FLOAT_MAX), *min will be greater than *max, and *points will
	be empty.
*/
void Skeleton::getJointPositionsAndStats(std::vector<cv::Point3f>* points, 
		cv::Point3f* min, cv::Point3f* max, cv::Point3f* avg) const {
	if(points != NULL && min != NULL && max != NULL && avg != NULL)
		return;
		
	double xAccum = 0.0;
	double yAccum = 0.0;
	double zAccum = 0.0;
	int numJoints = 0;
	if(points != NULL)
		points->clear();
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
			avg->x = avg->y = avg->z = std::numeric_limits<float>::max();
		}
		else {		
			avg->x = xAccum / numJoints;
			avg->y = yAccum / numJoints;
			avg->z = zAccum / numJoints;
		}
	}
}


/*! Returns the skeleton's centroid's 2D position.
	If \ref Skeleton::toVideoCoordinates is not NULL,
	it is used to transform the 3D into 2D coordinates.
	For more info, see \ref Skeleton::centroid3D.
*/
cv::Point2f Skeleton::centroid() const {
	cv::Point3f c3d = centroid3D();
	if(toVideoCoordinates != NULL)
		return toVideoCoordinates(c3d);
	else
		return cv::Point2f(c3d.x, c3d.y);
}

/*! Returns the skeleton's centroid's 3D position.
	Defining a skeleton's centroid is problematic. In this case,
	it is taken as being:
	a) The torso center, if that joint is active.
	b) An estimation of the torso center, if the left and right hips and shoulders
	   are active.
	c) The average of all joint positions, otherwise.

	Note that only joints with at least 50%	confidence in their position 
	are used for these calculations.
*/
cv::Point3f Skeleton::centroid3D() const {
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
	getJointPositionsAndStats(NULL, NULL, NULL, &result);
	return result;
}

/*! Transforms the points in points3D, calling toVideoCoordinates on each one,
	and gets puts the resulting points in points2D, the minimum coordinates on min,
	and the maximum on max. If points2D, min, or max are NULL, they are ignored.
*/
void Skeleton::transformJointPositionsAndGetStats(
		const std::vector<cv::Point3f>& points3D,
		std::vector<cv::Point2f>* points2D, 
		cv::Point2f* min, cv::Point2f* max) const {
	assert(toVideoCoordinates != NULL && !points3D.empty());
	
	if(points2D != NULL)
		points2D->clear();
	if(min != NULL)
		min->x = min->y = std::numeric_limits<float>::max();
	if(max != NULL)
		max->x = max->y = std::numeric_limits<float>::min();
	
	for(int i = 0; i < points3D.size(); i++) {
		cv::Point2f pos2D = toVideoCoordinates(points3D[i]);
		if(points2D != NULL)
			points2D->push_back(pos2D);

		if(min != NULL) {
			min->x = std::min(min->x, pos2D.x);
			min->y = std::min(min->y, pos2D.y);
		}
		if(max != NULL) {
			max->x = std::max(max->x, pos2D.x);
			max->y = std::max(max->y, pos2D.y);
		}
	}
}

/*! Gets a bounding rectangle for all active joints with greater than 50% confidence
	in their position. If no such joints exist, returns cv::Rect(INT_MAX, INT_MAX, 0, 0).
*/
cv::Rect Skeleton::boundingRect() const {
	cv::Point3f min, max;
	std::vector<cv::Point3f> pointVector;
	std::vector<cv::Point3f>* points;
	if(toVideoCoordinates != NULL)
		points = &pointVector;
	else
		points = NULL;
	getJointPositionsAndStats(points, &min, &max, NULL);
	if(min.x > max.x) { // no joints with > 0.5 confidence were found
		return cv::Rect(INT_MAX, INT_MAX, 0, 0);
	}
	if(toVideoCoordinates == NULL)
		return cv::Rect(min.x, min.y, max.x - min.x, max.y - min.y);
	else {
		cv::Point2f min2D;
		cv::Point2f max2D;

		transformJointPositionsAndGetStats(pointVector, NULL, &min2D, &max2D);
		
		return cv::Rect(min2D, max2D);
	}
}

/*! Gets the smallest (not necessarily axis-aligned) rectangle containing all active joints with 
	greater than 50% confidence	in their position. If no such joints exist, 
	returns cv::RotatedRectRect(cv::Point2f(std::numeric_limits<float>::max(),
	std::numeric_limits<float>::max()), cv::Size2f(0, 0), 0).
*/
cv::RotatedRect Skeleton::boundingRotatedRect() const {
	std::vector<cv::Point3f> pointVector;
	std::vector<cv::Point3f>* points;
	if(toVideoCoordinates != NULL)
		points = &pointVector;
	else
		points = NULL;
	getJointPositionsAndStats(points, NULL, NULL, NULL);
	if(pointVector.empty()) {
		const float FLOAT_MAX = std::numeric_limits<float>::max();
		return cv::RotatedRect(
			cv::Point2f(FLOAT_MAX, FLOAT_MAX), cv::Size2f(0, 0), 0);
	}

	std::vector<cv::Point2f> points2D;
	transformJointPositionsAndGetStats(pointVector, &points2D, NULL, NULL);

	return cv::minAreaRect(cv::Mat(points2D, false));
}

void Skeleton::getPixels(std::vector<cv::Point>& result) const {
	// TODO: stub
}

/*! Gets a writable reference to the joint map.
	This shouldn't be used outside of a \ref Tracker.
	Use \ref Skeleton::getAllActiveJoints instead.
*/
std::map<Skeleton::Joint, Skeleton::JointInfo>& _getJointMap() {
	return joints;
}

}