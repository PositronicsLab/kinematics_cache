#include <ros/ros.h>

namespace {
using namespace std;

static const double DISCRETIZATION_DEFAULT = 0.01;
    
class KinematicsCacheLoader {

private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

public:
	KinematicsCacheLoader() :
		pnh("~") {
	}
    
    double calculateArmLength() {
        // TODO: Calculate arm length here
        return 3;
    }
    
    void load() {
        ROS_INFO("Loading kinematics cache");

        double discretization;
        pnh.param<double>("discretization", discretization, DISCRETIZATION_DEFAULT);
        
        // Fetch the robot model
        
        // Determine length of arm
        double maxDistance = calculateArmLength();
        
        // Iterate through grid. Start at negative x,y,z and iterate
        // to end at positive x,y,z. All locations are in the frame
        // of the base of the arm.
        for (double x = -maxDistance; x <= maxDistance; x += discretization) {
            for (double y = -maxDistance; y <= maxDistance; y += discretization) {
                for (double z = -maxDistance; z <= maxDistance; z += discretization) {
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
