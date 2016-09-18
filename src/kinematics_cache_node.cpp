#include <ros/ros.h>
#include <kinematics_cache/IKQueryv2.h>
#include <kinematics_cache/kinematics_cache.h>

namespace
{

using namespace std;
using namespace kinematics_cache;

class KinematicsCacheNode
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! IK Service
    ros::ServiceServer ikService;

    auto_ptr<KinematicsCache> cache;

public:
    KinematicsCacheNode() :
        pnh("~")
    {
        double maxDistance;
        pnh.param("max_distance", maxDistance, 1000.0 /* Large number to disable check */);

        string baseFrame;
        pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");

        ikService = nh.advertiseService("/kinematics_cache/ik",
                                        &KinematicsCacheNode::query, this);

        string leftArmDataName;
        pnh.param<string>("left_arm_data", leftArmDataName, "./octree/left_arm.ot");

        string rightArmDataName;
        pnh.param<string>("right_arm_data", rightArmDataName, "./octree/right_arm.ot");

        // Read in the data
        ROS_INFO("Loading octomap data from files [%s, %s]", leftArmDataName.c_str(), rightArmDataName.c_str());

        cache.reset(new KinematicsCache(maxDistance, baseFrame, leftArmDataName, rightArmDataName));

        ROS_INFO("Service initialized");
    }

public:

    bool query(kinematics_cache::IKQueryv2::Request& req,
               kinematics_cache::IKQueryv2::Response& res)
    {

        IKList results;
        if (!cache->query(req.group, req.point, results))
        {
            ROS_DEBUG("Failed to find IK result for service call");
            return false;
        }

        for (IKList::iterator i = results.begin(); i != results.end(); ++i)
        {
            if (i->positions.size() == 0)
            {
                ROS_WARN("IK result was invalid");
                continue;
            }
            res.results.push_back(*i);
        }

        return true;
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kinematics_cache_node");
    KinematicsCacheNode kcn;
    ros::spin();
}
