#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

// MoveIt
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/move_group_interface/move_group.h>

#include <tf/transform_listener.h>
#include <kinematics_cache/IKv2.h>
#include <kinematics_cache/IKQueryv2.h>
#include <boost/math/constants/constants.hpp>
#include <kinematics_cache/OcTreeJointAngles.h>

namespace {
using namespace std;
using namespace kinematics_cache;
using namespace octomap;

static const double RESOLUTION_DEFAULT = 0.02;
static const double IK_SEARCH_RESOLUTION_DEFAULT = 0.01;
static const double MAX_VELOCITY = 100;
static const double MAX_ACCELERATION = 100;
static const double MAX_EXEC_TIME = 60 * 60;

static const double pi = boost::math::constants::pi<double>();

class KinematicsCacheLoader {

private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    tf::TransformListener tf;

    double resolution;

    double ikSearchResolution;

    string kinematicsSolverName;

    string groupName;

    string baseFrame;

    string tipLink;

    robot_model::RobotModelPtr kinematicModel;

    boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematicsLoader;

    kinematics::KinematicsBasePtr kinematicsSolver;

    const robot_model::JointModelGroup* jointModelGroup;

    ros::Publisher searchPub;

    bool moveArmToBase;

    //! Cached IK client.
    ros::ServiceClient ik;

    #if ROS_VERSION_MINIMUM(1, 10, 12)
        // Method not required
    #else
    static vector<string> getActiveJointModelNames(const robot_model::JointModelGroup* jointModelGroup) {
      vector<string> activeJointModels;
      for (unsigned int i = 0; i < jointModelGroup->getJointModels().size(); ++i)
      {
         if (jointModelGroup->getJointModels()[i]->getMimic() != NULL) {
           ROS_WARN("Passive joint model found");
           continue;
         }
         activeJointModels.push_back(jointModelGroup->getJointModels()[i]->getName());

      }
      return activeJointModels;
    }
    #endif // ROS_VERSION_MINIMUM

public:
	KinematicsCacheLoader() :
		pnh("~") {
        pnh.param("resolution", resolution, RESOLUTION_DEFAULT);
        pnh.param("ik_search_resolution", ikSearchResolution, IK_SEARCH_RESOLUTION_DEFAULT);
        pnh.param<string>("kinematics_solver_name", kinematicsSolverName, "pr2_right_arm_kinematics/IKFastDistanceKinematicsPlugin");
        pnh.param<string>("group_name", groupName, "right_arm");
        pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");
        pnh.param<string>("tip_link", tipLink, "r_wrist_roll_link");
        pnh.param<bool>("move_arm_to_base", moveArmToBase, false);

        string cacheServiceName;
        pnh.param<string>("cache_service_name", cacheServiceName, "/kinematics_cache/ik");

        ros::service::waitForService(cacheServiceName);
        ik = nh.serviceClient<kinematics_cache::IKQueryv2>(cacheServiceName, true /* persistent */);

        searchPub = nh.advertise<visualization_msgs::Marker>("search_position", 10);
        kinematicsLoader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
        try {
            kinematicsSolver = kinematicsLoader->createInstance(kinematicsSolverName);
        } catch(pluginlib::PluginlibException& ex) //handle the class failing to load
        {
            ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
            throw ex;
        }

        if(!kinematicsSolver->initialize("robot_description", groupName, baseFrame, tipLink, ikSearchResolution)) {
            ROS_ERROR("Could not initialize solver");
        }

        rdf_loader::RDFLoader rdfLoader;
        const boost::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
        const boost::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();
        kinematicModel.reset(new robot_model::RobotModel(urdfModel, srdf));

        jointModelGroup =  kinematicModel->getJointModelGroup(kinematicsSolver->getGroupName());
	}

private:
    void publishSearchLocation(const string& frame, const geometry_msgs::Point& target) {
        ros::Time now = ros::Time::now();
        visualization_msgs::Marker points;
        points.header.frame_id = frame;
        points.header.stamp = now;
        points.ns = "kinematics_cache";
        points.action = visualization_msgs::Marker::ADD;
        points.type = visualization_msgs::Marker::POINTS;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.scale.x = 0.05;
        points.scale.y = 0.05;
        points.color.g = 1.0f;
        points.color.a = 1.0;
        points.points.push_back(target);
        searchPub.publish(points);
    }

    double calculateMaxDistance(const string& searchFrame) {
        // Calculate length of arm by setting all angle to 0 and performing IK
        #if ROS_VERSION_MINIMUM(1, 10, 12)
        vector<double> zeroPositions(jointModelGroup->getActiveJointModels().size());
        #else
        vector<double> zeroPositions(getActiveJointModelNames(jointModelGroup).size());
        #endif
        vector<geometry_msgs::Pose> results;
        vector<string> endEffector;
        endEffector.push_back(tipLink);
        kinematicsSolver->getPositionFK(endEffector, zeroPositions, results);
        ros::Time now = ros::Time::now();
        tf.waitForTransform(searchFrame, baseFrame, now, ros::Duration(10.0));

        // Convert to search frame.
        geometry_msgs::PoseStamped resultInBaseFrameStamped;
        resultInBaseFrameStamped.pose = results[0];
        resultInBaseFrameStamped.header.stamp = now;
        resultInBaseFrameStamped.header.frame_id = baseFrame;
        geometry_msgs::PoseStamped resultInSearchFrame;
        tf.transformPose(searchFrame, resultInBaseFrameStamped, resultInSearchFrame);

        // Calculate the distance from the zero point.
        return sqrt(pow(resultInSearchFrame.pose.position.x, 2) +
                    pow(resultInSearchFrame.pose.position.y, 2) +
                    pow(resultInSearchFrame.pose.position.z, 2));
    }

public:
    void load() {
        ROS_INFO("Loading kinematics cache");

        OcTreeJointAngles tree(resolution);

        const string& searchFrame = jointModelGroup->getLinkModelNames()[0];
        ROS_INFO("Search frame for loading is %s", searchFrame.c_str());

 #if ROS_VERSION_MINIMUM(1, 10, 12)
        unsigned int numActiveJoints = jointModelGroup->getActiveJointModels().size();
#else
        unsigned int numActiveJoints = getActiveJointModelNames(jointModelGroup).size();
 #endif

        // Move to outstretched positions for easier visualization
        if (moveArmToBase) {
          moveit::planning_interface::MoveGroup group(groupName);
          vector<double> zeroPositions(numActiveJoints);
          group.setJointValueTarget(zeroPositions);
          group.move();
        }

        // The search frame is not perfect because it is not exactly the intersection at the top of the joint.
        double maxDistance = calculateMaxDistance(searchFrame);

        ROS_INFO("Maximum distance is %f", maxDistance);

        // Iterate through grid. Start at negative x,y,z and iterate
        // to end at positive x,y,z. All locations are in the frame
        // of the base of the arm.
        kinematics::KinematicsQueryOptions opts;
        double timeout = 180;

        unsigned int numLoaded = 0;
        vector<double> initialPositions(numActiveJoints);

        // Frames never move so we can use a constant transform
        ros::Time now = ros::Time::now();
        tf.waitForTransform(searchFrame, baseFrame, now, ros::Duration(10.0));

        for (double x = maxDistance; x >= -maxDistance; x -= resolution) {
            ROS_INFO("Decrementing the x search parameter. Current X value is %f. Maximum value is: %f. Current found solutions is %u", x, maxDistance, numLoaded);
            for (double y = maxDistance; y >= -maxDistance; y -= resolution) {
                ROS_INFO("Decrementing the y search parameter. Current Y value is %f. Maximum value is: %f. Current found solutions is %u", y, maxDistance, numLoaded);
                for (double z = maxDistance; z >= -maxDistance; z -= resolution) {
                    if (!ros::ok()) {
                        ROS_WARN("Interrupt requested");
                        return;
                    }

                    if (sqrt(x * x + y * y + z * z) > maxDistance) {
                        ROS_DEBUG("Skipping point exceeding max distance");
                        continue;
                    }

                    ROS_DEBUG("Attempting to create IK solution at %f, %f, %f", x, y, z);
                    geometry_msgs::Pose target;
                    target.position.x = x;
                    target.position.y = y;
                    target.position.z = z;

                    if (groupName == "left_arm") {
                        target.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, pi / 2.0);
                    } else if (groupName == "right_arm") {
                        target.orientation = tf::createQuaternionMsgFromRollPitchYaw(0, 0, -pi / 2.0);
                    }

                    publishSearchLocation(searchFrame, target.position);

                    // Transform to the base frame of the robot where all IK occurs
                    geometry_msgs::PoseStamped targetInBaseFrame;
                    geometry_msgs::PoseStamped targetStamped;
                    targetStamped.pose = target;
                    targetStamped.header.stamp = now;
                    targetStamped.header.frame_id = searchFrame;

                    tf.transformPose(baseFrame, targetStamped, targetInBaseFrame);

                    // Check if the pose is in the cache.
                    kinematics_cache::IKQueryv2 ikQuery;
                    ikQuery.request.point.point = targetInBaseFrame.pose.position;
                    ikQuery.request.point.header = targetInBaseFrame.header;

                    if (ik.call(ikQuery)) {
                        ROS_INFO("Skipping already computed pose");
                        continue;
                    }

                    vector<double> solution(numActiveJoints);
                    moveit_msgs::MoveItErrorCodes error;
                    kinematicsSolver->searchPositionIK(targetInBaseFrame.pose, initialPositions, timeout, solution, error, opts);
                    if(error.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                        ROS_DEBUG("Solution found");
                    } else {
                        ROS_DEBUG("IK failed %i", error.val);
                        continue;
                    }

                    numLoaded++;
                    assert(solution.size() == 7);
                    octomap::angles_t angles;
                    for (unsigned int i = 0; i < solution.size(); ++i)
                    {
                        angles[i] = (float) solution[i];
                    }

                    octomap::point3d point((float) ikQuery.request.point.point.x,
                                   (float) ikQuery.request.point.point.y,
                                   (float) ikQuery.request.point.point.z);
                    OcTreeNodeJointAngles* node = tree.updateNode(point);
                    node->setValue(angles);
                }
            }
        }
        ROS_INFO("Completed loading kinematics cache. Loaded %u entries.", numLoaded);
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, ros::this_node::getName());

    KinematicsCacheLoader kcl;
    kcl.load();
}
