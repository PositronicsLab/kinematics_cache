#include <ros/ros.h>

#include <kinematics_cache/kinematics_cache.h>
#include <kinematics_cache/IKv2.h>
#include <kinematics_cache/OcTreeJointAngles.h>
#include <kinematics_cache/kinematics_cache.h>

namespace kinematics_cache
{

using namespace std;
using namespace octomap;
using namespace kinematics_cache;

static const double RESOLUTION_DEFAULT = 0.01;

KinematicsCache::KinematicsCache(double aMaxDistance, const string& aBaseFrame, const string& aLeftArmDataName, const string& aRightArmDataName)
    : maxDistance(aMaxDistance), baseFrame(aBaseFrame)
{
    // Read in the data
    ROS_INFO("Loading octomap data from files [%s, %s]", aLeftArmDataName.c_str(), aRightArmDataName.c_str());
    leftArmData.reset(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(aLeftArmDataName)));

    if (leftArmData.get() == NULL)
    {
        ROS_ERROR("Failed to read left arm data");
    }

    rightArmData.reset(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(aRightArmDataName)));
    if (rightArmData.get() == NULL)
    {
        ROS_ERROR("Failed to read right arm data");
    }

    ROS_INFO("Cache initialized. Loaded %lu nodes to left and %lu nodes to right", leftArmData->getNumLeafNodes(), rightArmData->getNumLeafNodes());
}

static double calcDistance(const geometry_msgs::PointStamped& point)
{
    return sqrt(pow(point.point.x, 2) +
                pow(point.point.y, 2) +
                pow(point.point.z, 2));
}

const OcTreeJointAngles* KinematicsCache::which(const string& arm) const
{
    if (arm == "left_arm")
    {
        return leftArmData.get();
    }
    else if (arm == "right_arm")
    {
        return rightArmData.get();
    }
    ROS_ERROR("Invalid arm %s", arm.c_str());
    return NULL;
}

bool KinematicsCache::query(const std::string group,
                           const geometry_msgs::PointStamped point, IKList& results) const
{
    ROS_DEBUG("Querying for group %s at position %f %f %f", group.c_str(),
              point.point.x, point.point.y, point.point.z);

    // Do not perform transform, as it is a footgun for performance
    if (point.header.frame_id != baseFrame)
    {
        ROS_ERROR("Queries must be specified in %s not %s", baseFrame.c_str(), point.header.frame_id.c_str());
        return false;
    }

    if (calcDistance(point) > maxDistance + RESOLUTION_DEFAULT)
    {
        ROS_DEBUG("Position at distance %f beyond the maximum reach of the arm %f given error %f",
                  calcDistance(point), maxDistance, RESOLUTION_DEFAULT);
        return false;
    }

    // Check group
    // TODO: This could be optimized to avoid the vector
    vector<string> groups;
    if (group.empty())
    {
        queryForGroup(point, "left_arm", results);
        queryForGroup(point, "right_arm", results);
    }
    else
    {
        queryForGroup(point, group, results);
    }
    ROS_DEBUG("Query succeeded. Found %lu results.", results.size());
    return true;
}

void KinematicsCache::queryForGroup(const geometry_msgs::PointStamped point, const string& group, vector<IKv2>& results) const
{
    const OcTreeNodeJointAngles* node = which(group)->search(point.point.x, point.point.y, point.point.z);

    if (node == NULL)
    {
        return;
    }

    // 0th spot is the number of results
    for (unsigned int k = 0; k < node->getValue()[0]; ++k)
    {
        kinematics_cache::IKv2 result;
        result.positions.resize(7);
        result.point = point;
        result.group = group;
        for (unsigned int i = 0; i < 7; ++i)
        {
            result.positions[i] = node->getValue()[i + 1 + k * octomap::NUM_JOINTS];
        }
        results.push_back(result);
    }
}
}
