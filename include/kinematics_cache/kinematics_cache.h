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

    //! Left arm
    std::auto_ptr<octomap::OcTreeJointAngles> leftArmData;

    //! Right arm
    std::auto_ptr<octomap::OcTreeJointAngles> rightArmData;

public:
    KinematicsCache(double aMaxDistance, const std::string& aBaseFrame, const std::string& aLeftArmDataName, const std::string& aRightArmDataName);
    bool query(const std::string group, const geometry_msgs::PointStamped point, IKList& results) const;

private:
    void queryForGroup(const geometry_msgs::PointStamped point, const std::string& group, std::vector<IKv2>& results) const;
    const octomap::OcTreeJointAngles* which(const std::string& arm) const;
};

} // end namespace

#endif
