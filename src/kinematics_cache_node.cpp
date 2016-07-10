#include <ros/ros.h>

#include <kinematics_cache/IKv2.h>
#include <kinematics_cache/IKQueryv2.h>
#include <kinematics_cache/OcTreeJointAngles.h>

namespace
{

using namespace std;
using namespace octomap;
using namespace kinematics_cache;

static const double RESOLUTION_DEFAULT = 0.01;

typedef vector<IKv2> IKList;

class KinematicsCacheNode
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! IK Service
    ros::ServiceServer ikService;

    //! Maximum arm distance
    double maxDistance;

    //! Base frame for IK
    string baseFrame;

    //! Left arm
    auto_ptr<OcTreeJointAngles> leftArmData;

    //! Right arm
    auto_ptr<OcTreeJointAngles> rightArmData;

public:
    KinematicsCacheNode() :
        pnh("~")
    {
        pnh.param("max_distance", maxDistance, 1000.0 /* Large number to disable check */);
        pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");

        ikService = nh.advertiseService("/kinematics_cache/ik",
                                        &KinematicsCacheNode::query, this);

        string leftArmDataName;
        pnh.param<string>("left_arm_data", leftArmDataName, "/home/jlurz/octree/left_arm.ot");

        string rightArmDataName;
        pnh.param<string>("right_arm_data", rightArmDataName, "/home/jlurz/octree/right_arm.ot");

        // Read in the data
        ROS_INFO("Loading octomap data from files [%s, %s]", leftArmDataName.c_str(), rightArmDataName.c_str());
        leftArmData.reset(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(leftArmDataName)));

        if (leftArmData.get() == NULL)
        {
            ROS_ERROR("Failed to read left arm data");
        }

        rightArmData.reset(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(rightArmDataName)));
        if (rightArmData.get() == NULL)
        {
            ROS_ERROR("Failed to read right arm data");
        }

        ROS_INFO("Service initialized. Loaded %lu nodes to left and %lu nodes to right", leftArmData->getNumLeafNodes(), rightArmData->getNumLeafNodes());
    }

public:

    bool query(kinematics_cache::IKQueryv2::Request& req,
               kinematics_cache::IKQueryv2::Response& res)
    {

        IKList results;
        if (!query(req.group, req.point, results))
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

    static double calcDistance(const geometry_msgs::PointStamped& point)
    {
        return sqrt(pow(point.point.x, 2) +
                    pow(point.point.y, 2) +
                    pow(point.point.z, 2));
    }

    OcTreeJointAngles* which(const string& arm)
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

    bool query(const std::string group,
               const geometry_msgs::PointStamped point, IKList& results)
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
            groups.push_back("left_arm");
            groups.push_back("right_arm");
        }
        else
        {
            groups.push_back(group);
        }

        for (unsigned int i = 0; i < groups.size(); ++i)
        {
            const OcTreeNodeJointAngles* node = which(groups[i])->search(point.point.x, point.point.y, point.point.z);
            if (node == NULL)
            {
                continue;
            }



            // 0th spot is the number of results
            for (unsigned int k = 0; k < node->getValue()[0]; ++k)
            {
                kinematics_cache::IKv2 result;
                result.positions.resize(7);
                result.point = point;
                result.group = groups[i];
                for (unsigned int i = 0; i < 7; ++i)
                {
                    result.positions[i] = node->getValue()[i + 1 + k * octomap::NUM_JOINTS];
                }
                results.push_back(result);
            }
        }
        ROS_DEBUG("Query succeeded. Found %lu results.", results.size());
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
