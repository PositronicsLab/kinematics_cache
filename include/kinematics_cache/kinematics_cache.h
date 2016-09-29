#ifndef KINEMATICS_CACHE_H
#define KINEMATICS_CACHE_H

#include <geometry_msgs/PointStamped.h>
#include <vector>
#include <kinematics_cache/IKv2.h>
#include <kinematics_cache/OcTreeJointAngles.h>

namespace kinematics_cache
{
    typedef std::vector<IKv2> IKList;

class KinematicsCache
{

private:

    //! Maximum arm distance
    double maxDistance;

    //! Base frame for IK
    std::string baseFrame;

    //! Cache
    std::auto_ptr<octomap::OcTreeJointAngles> cache;

public:
    KinematicsCache(double aMaxDistance, const std::string& aBaseFrame, const std::string& cacheName);
    bool query(const geometry_msgs::PointStamped point, IKList& results) const;

private:
    void queryForGroup(const geometry_msgs::PointStamped point, std::vector<IKv2>& results) const;
};

} // end namespace

#endif
