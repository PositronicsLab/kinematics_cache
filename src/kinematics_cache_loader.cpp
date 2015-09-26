#include <ros/ros.h>

namespace {
using namespace std;

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
    
    void load() {
        ROS_INFO("Loading kinematics cache");

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
