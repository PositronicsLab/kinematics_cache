#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

// MoveIt!
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model/joint_model_group.h>

namespace {
using namespace std;

static const double RESOLUTION_DEFAULT = 0.01;
    
class KinematicsCacheLoader {

private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

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
        pnh.param<string>("kinematics_solver_name", kinematicsSolverName, "pr2_arm_kinematics/PR2ArmKinematicsPlugin");
        pnh.param<string>("group_name", groupName, "right_arm");
        pnh.param<string>("base_frame", baseFrame, "torso_lift_link");
        pnh.param<string>("tip_link", tipLink, "r_wrist_roll_link");
            
        kinematicsLoader.reset(new pluginlib::ClassLoader<kinematics::KinematicsBase>("kinematics_base", "kinematics::KinematicsBase"));
        try {
            kinematicsSolver.reset(kinematicsLoader->createClassInstance(kinematicsSolverName));
        } catch(pluginlib::PluginlibException& ex) //handle the class failing to load
        {
            ROS_ERROR("The plugin failed to load. Error: %s", ex.what());
            throw ex;
        }

        if(!kinematicsSolver->initialize("robot_description", groupName, baseFrame, tipLink, RESOLUTION_DEFAULT)) {
            ROS_ERROR("Could not initialize solver");   
        }
  
        rdf_loader::RDFLoader rdfLoader;
        const boost::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
        const boost::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();
        kinematicModel.reset(new robot_model::RobotModel(urdfModel, srdf));

        jointModelGroup =  kinematicModel->getJointModelGroup(kinematicsSolver->getGroupName());
        kinematicState.reset(new robot_state::RobotState(kinematicModel));
	}
    
    double calculateArmLength() {
        // TODO: Calculate arm length here
        return 3;
    }
    
    void load() {
        ROS_INFO("Loading kinematics cache");

        double discretization;
        pnh.param<double>("resolution", resolution, RESOLUTION_DEFAULT);
        
        // TODO: Fetch the robot model
        
        // Determine length of arm
        double maxDistance = calculateArmLength();
        
        // Iterate through grid. Start at negative x,y,z and iterate
        // to end at positive x,y,z. All locations are in the frame
        // of the base of the arm.
        for (double x = -maxDistance; x <= maxDistance; x += resolution) {
            for (double y = -maxDistance; y <= maxDistance; y += resolution) {
                for (double z = -maxDistance; z <= maxDistance; z += resolution) {
                    ROS_INFO("Attempting to create IK solution at %f, %f, %f", x, y, z);
                    // TODO: Execute IK
                    // TODO: Store result in Mongo
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
	ros::spin();
}
