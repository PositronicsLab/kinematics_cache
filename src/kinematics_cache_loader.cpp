#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

// Ensure this is defined before boost since mongodb uses version
// 2 and both cannot be simulataneously defined.
#define BOOST_FILESYSTEM_VERSION 2

// MoveIt!
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model/joint_model_group.h>

#include <tf/transform_listener.h>
#include <mongodb_store/message_store.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>

namespace {
using namespace std;
using namespace mongodb_store;

static const double RESOLUTION_DEFAULT = 0.1; /* 0.01 */
static const double IK_SEARCH_RESOLUTION = 0.01; /** 0.001 */

class KinematicsCacheLoader {

private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

    tf::TransformListener tf;
    
    double resolution;
    
    string kinematicsSolverName;
            
    string groupName;
    
    string baseFrame;
    
    string tipLink;
    
    robot_model::RobotModelPtr kinematicModel;
    
    boost::shared_ptr<pluginlib::ClassLoader<kinematics::KinematicsBase> > kinematicsLoader;
    
    kinematics::KinematicsBasePtr kinematicsSolver;
    
    robot_state::RobotStatePtr kinematicState;
        
    const robot_model::JointModelGroup* jointModelGroup;
    
public:
	KinematicsCacheLoader() :
		pnh("~") {
        pnh.param("resolution", resolution, RESOLUTION_DEFAULT);
        pnh.param<string>("kinematics_solver_name", kinematicsSolverName, "pr2_right_arm_kinematics/IKFastDistanceKinematicsPlugin");
        pnh.param<string>("group_name", groupName, "right_arm");
        pnh.param<string>("base_frame", baseFrame, "torso_lift_link");
        pnh.param<string>("tip_link", tipLink, "r_wrist_roll_link");
                
        kinematicsLoader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("moveit_core", "kinematics::KinematicsBase"));
        try {
            kinematicsSolver = kinematicsLoader->createInstance(kinematicsSolverName);
        } catch(pluginlib::PluginlibException& ex) //handle the class failing to load
        {
            ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
            throw ex;
        }

        if(!kinematicsSolver->initialize("robot_description", groupName, baseFrame, tipLink, IK_SEARCH_RESOLUTION)) {
            ROS_ERROR("Could not initialize solver");   
        }
  
        rdf_loader::RDFLoader rdfLoader;
        const boost::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
        const boost::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();
        kinematicModel.reset(new robot_model::RobotModel(urdfModel, srdf));

        jointModelGroup =  kinematicModel->getJointModelGroup(kinematicsSolver->getGroupName());
        kinematicState.reset(new robot_state::RobotState(kinematicModel));
	}
    
    void load() {
        ROS_INFO("Loading kinematics cache");
        MessageStoreProxy mdb(nh);
        
        // TODO: Calculate dynamically from the model.
        double maxDistance = 0.708;
        
        // Iterate through grid. Start at negative x,y,z and iterate
        // to end at positive x,y,z. All locations are in the frame
        // of the base of the arm.
        kinematics::KinematicsQueryOptions opts;
        double timeout = 180;
        
        // TODO: Not exactly the right source frame
        vector<double> initialPositions(jointModelGroup->getActiveJointModels().size());
        const string& searchFrame = jointModelGroup->getLinkModelNames()[0];
        for (double x = -maxDistance; x <= maxDistance; x += resolution) {
            for (double y = -maxDistance; y <= maxDistance; y += resolution) {
                for (double z = -maxDistance; z <= maxDistance; z += resolution) {
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
                    
                    ros::Time now = ros::Time::now();

                    tf.waitForTransform(searchFrame, baseFrame,
                              now, ros::Duration(10.0));
                              
                    // Transform
                    geometry_msgs::PoseStamped targetInBaseFrame;
                    geometry_msgs::PoseStamped targetStamped;
                    targetStamped.pose = target;
                    targetStamped.header.stamp = now;
                    targetStamped.header.frame_id = searchFrame;
                    
                    tf.transformPose(baseFrame, targetStamped, targetInBaseFrame);
                    
                    vector<double> solution(jointModelGroup->getActiveJointModels().size());
                    moveit_msgs::MoveItErrorCodes error;
                    kinematicsSolver->searchPositionIK(targetInBaseFrame.pose, initialPositions, timeout, solution, error, opts);
                    if(error.val == moveit_msgs::MoveItErrorCodes::SUCCESS) {
                        ROS_DEBUG("Solution found");
                    } else {
                        ROS_DEBUG("IK failed %i", error.val);
                        continue;
                    }
                    
                    trajectory_msgs::JointTrajectoryPoint msg;
                    msg.positions = solution;
                    mdb.insert(msg);
                    
                    // TODO: Store motion plan?
                    // TODO: Store duration?
                }
            }
        }
        ROS_INFO("Completed loading kinematics cache");
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinematics_cache_loader");

    KinematicsCacheLoader kcl;
    kcl.load();
}
