#include <ros/ros.h>

namespace {
using namespace std;

class KinematicsCacheNode {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;

public:
	KinematicsCacheNode() :
		pnh("~") {
	}
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinematics_cache_node");

	KinematicsCacheNode kcn;
	ros::spin();
}
