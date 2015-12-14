#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

// Ensure this is defined before boost since mongodb uses version
// 2 and both cannot be simulataneously defined.
#define BOOST_FILESYSTEM_VERSION 2

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
#include <mongodb_store/message_store.h>
#include <kinematics_cache/IK.h>
#include <kinematics_cache/IKQuery.h>

namespace {
using namespace std;
using namespace mongodb_store;

static const double RESOLUTION_DEFAULT = 0.01;
static const double IK_SEARCH_RESOLUTION_DEFAULT = 0.01;

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

    robot_state::RobotStatePtr kinematicState;

    const robot_model::JointModelGroup* jointModelGroup;

    vector<double> basePositions;

    vector<double> jointMaxVelocities;

    vector<double> jointMaxAccelerations;

    ros::Publisher searchPub;

    bool moveArmToBase;

    //! Cached IK client.
    ros::ServiceClient ik;
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

        ros::service::waitForService("/kinematics_cache/ik");
        ik = nh.serviceClient<kinematics_cache::IKQuery>("/kinematics_cache/ik", true /* persistent */);

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
        kinematicState.reset(new robot_state::RobotState(kinematicModel));

        // TODO: Make base positions configurable
        basePositions.resize(jointModelGroup->getActiveJointModels().size());

        // Load limit data
        ROS_DEBUG("Loading joint limits");
        for (unsigned int i = 0; i < jointModelGroup->getActiveJointModels().size(); ++i) {
            const string& jointName = jointModelGroup->getActiveJointModels()[i]->getName();
            const string prefix = "robot_description_planning/joint_limits/" + jointName + "/";
            ROS_DEBUG_NAMED("kinematics_cache_loader", "Loading velocity and accleration limits for joint %s", jointName.c_str());

            bool has_vel_limits;
            double max_velocity;
            if (nh.getParam(prefix + "has_velocity_limits", has_vel_limits) && has_vel_limits && nh.getParam(prefix + "max_velocity", max_velocity)) {
                ROS_DEBUG_NAMED("kinematics_cache_loader", "Setting max velocity to %f", max_velocity);
                jointMaxVelocities.push_back(max_velocity);
            } else {
                ROS_DEBUG_NAMED("kinematics_cache_loader", "Setting max velocity to default");
                jointMaxVelocities.push_back(100); /** TODO: use constant */
            }

            bool has_acc_limits;
            double max_acc;
            if (nh.getParam(prefix + "has_acceleration_limits", has_acc_limits) && has_acc_limits && nh.getParam(prefix + "max_acceleration", max_acc)) {
                ROS_DEBUG_NAMED("ikfast", "Setting max acceleration to %f", max_acc);
                jointMaxAccelerations.push_back(max_acc);
            } else {
                ROS_DEBUG_NAMED("ikfast", "Setting max acceleration to default");
                jointMaxAccelerations.push_back(100); /** TODO: use constant **/
            }
        }
	}

    // TODO: This shares a lot of code with the moveit_plugin
    ros::Duration calcExecutionTime(const vector<double>& solution) {
        double longestTime = 0.0;
        for(unsigned int i = 0; i < solution.size(); ++i) {
            ROS_DEBUG_NAMED("kinematics_cache_loader", "Calculating distance for joint %u", i);
            double d = fabs(basePositions[i] - solution[i]);
            ROS_DEBUG_NAMED("kinematics_cache_loader", "Distance to travel is %f", d);

            // Determine the max "bang-bang" distance.
            double x = 2.0 * jointMaxVelocities[i] / jointMaxAccelerations[i];
            double max_tri_distance = 0.5 * jointMaxVelocities[i] * x;
            ROS_DEBUG_NAMED("kinematics_cache_loader", "Maximum triangular distance given max_vel %f and max_accel %f is %f", jointMaxVelocities[i], jointMaxAccelerations[i], max_tri_distance);

            double t;
            if (d <= max_tri_distance) {
                t = sqrt(d / jointMaxAccelerations[i]);
                ROS_DEBUG_NAMED("kinematics_cache_loader", "Triangular solution for t is %f", t);
            }
            else {
                // Remove acceleration and deacceleration distance and calculate the trapezoidal base distance
                double d_rect = d - max_tri_distance;
                double x_rect = d_rect / jointMaxVelocities[i];
                t = sqrt(max_tri_distance / jointMaxAccelerations[i]) + x_rect;
                ROS_DEBUG_NAMED("ikfast", "Trapezoidal solution for t is %f", t);
            }

            longestTime = max(longestTime, t);
        }

        ROS_DEBUG_NAMED("kinematics_cache_loader", "Execution time is %f", longestTime);
        return ros::Duration(longestTime);
    }

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
        vector<double> zeroPositions(jointModelGroup->getActiveJointModels().size());
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

    void load() {
        ROS_INFO("Loading kinematics cache");
        MessageStoreProxy mdb(nh);
        const string& searchFrame = jointModelGroup->getLinkModelNames()[0];
        ROS_INFO("Search frame for loading is %s", searchFrame.c_str());

        // Move to outstretched positions for easier visualization
        if (moveArmToBase) {
          moveit::planning_interface::MoveGroup group(groupName);
          vector<double> zeroPositions(jointModelGroup->getActiveJointModels().size());
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
        vector<double> initialPositions(jointModelGroup->getActiveJointModels().size());

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
                    target.orientation.x = 0.0;
                    target.orientation.y = 0.0;
                    target.orientation.z = 0.0;
                    target.orientation.w = 1.0;

                    publishSearchLocation(searchFrame, target.position);

                    // Transform to the base frame of the robot where all IK occurs
                    geometry_msgs::PoseStamped targetInBaseFrame;
                    geometry_msgs::PoseStamped targetStamped;
                    targetStamped.pose = target;
                    targetStamped.header.stamp = now;
                    targetStamped.header.frame_id = searchFrame;

                    tf.transformPose(baseFrame, targetStamped, targetInBaseFrame);

                    // Reset the pose to be vertical in the robot base frame.
                    targetInBaseFrame.pose.orientation.x = 0.0;
                    targetInBaseFrame.pose.orientation.y = 0.0;
                    targetInBaseFrame.pose.orientation.z = 0.0;
                    targetInBaseFrame.pose.orientation.w = 1.0;

                    // Check if the pose is in the cache.
                    kinematics_cache::IKQuery ikQuery;
                    ikQuery.request.group = groupName;
                    ikQuery.request.pose = targetInBaseFrame;
                    if (ik.call(ikQuery)) {
                        ROS_INFO("Skipping already computed pose");
                        continue;
                    }

                    vector<double> solution(jointModelGroup->getActiveJointModels().size());
                    moveit_msgs::MoveItErrorCodes error;
                    kinematicsSolver->searchPositionIK(targetInBaseFrame.pose, initialPositions, timeout, solution, error, opts);
                    if(error.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                        ROS_DEBUG("Solution found");
                    } else {
                        ROS_DEBUG("IK failed %i", error.val);
                        continue;
                    }

                    numLoaded++;

                    kinematics_cache::IK msg;
                    msg.positions = solution;
                    msg.group = groupName;
                    msg.pose = targetInBaseFrame;
                    msg.execution_time = calcExecutionTime(solution);
                    mdb.insert(msg);
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
