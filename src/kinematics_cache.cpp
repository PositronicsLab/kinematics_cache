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

KinematicsCache::KinematicsCache(double aMaxDistance, const string& aBaseFrame, const string& cacheName)
    : maxDistance(aMaxDistance), baseFrame(aBaseFrame)
{
    // Read in the data
    ROS_INFO("Loading octomap data from file [%s]", cacheName.c_str());
    cache.reset(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(cacheName)));

    if (cache.get() == NULL)
    {
        ROS_ERROR("Failed to read cache data");
    }

    ROS_INFO("Cache initialized. Loaded %lu nodes to cache", cache->getNumLeafNodes());
}

static double calcDistance(const geometry_msgs::PointStamped& point)
{
    return sqrt(pow(point.point.x, 2) +
                pow(point.point.y, 2) +
                pow(point.point.z, 2));
}

bool KinematicsCache::query(const geometry_msgs::PointStamped point, IKList& results) const
{
    ROS_DEBUG("Querying at position %f %f %f",
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

    queryForGroup(point, results);

    ROS_DEBUG("Query succeeded. Found %lu results.", results.size());
    return true;
}

bool KinematicsCache::list(IKList& results) const {
    ROS_INFO("Listing all %lu cache nodes", cache->getNumLeafNodes());
    for (octomap::OcTreeJointAngles::leaf_iterator i = cache->begin_leafs(); i != cache->end_leafs(); ++i) {
        // No stored data
        if (i->getValue()[0] == 0) {
            continue;
        }

        kinematics_cache::IKv2 result;
        result.point.point.x = i.getX();
        result.point.point.y = i.getY();
        result.point.point.z = i.getZ();
        results.push_back(result);
    }
    ROS_DEBUG("Listing nodes complete");
    return true;
}

void KinematicsCache::queryForGroup(const geometry_msgs::PointStamped point, vector<IKv2>& results) const
{
    const OcTreeNodeJointAngles* node = cache->search(point.point.x, point.point.y, point.point.z);

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
        for (unsigned int i = 0; i < 7; ++i)
        {
            result.positions[i] = node->getValue()[i + 1 + k * octomap::NUM_JOINTS];
        }
        results.push_back(result);
    }
}
}
