#include <ros/ros.h>
#include <kinematics_cache/IK.h>
#include <mongodb_store/message_store.h>

namespace {
using namespace std;
using namespace mongodb_store;
using namespace mongo;

static const double RESOLUTION_DEFAULT = 0.1; /* Default value: 0.01 */

class KinematicsCacheNode {
private:

	//! Node handle
	ros::NodeHandle nh;

	//! Private nh
	ros::NodeHandle pnh;
    
    //! Database
    MessageStoreProxy mdb;
    
    //! Cache resolution
    double resolution;
    
    //! Base frame for IK
    string baseFrame;
    
public:
	KinematicsCacheNode() :
		pnh("~"), mdb(nh) {
         pnh.param("resolution", resolution, RESOLUTION_DEFAULT);
         pnh.param<string>("base_frame", baseFrame, "torso_lift_link");
	}
    
    boost::shared_ptr<kinematics_cache::IK> query(const std::string group,
        const geometry_msgs::PoseStamped pose) {
        // Do not perform IK, as it is a footgun for performance
        if (pose.header.frame_id != baseFrame) {
            ROS_ERROR("Queries must be specified in the base frame");
            return boost::shared_ptr<kinematics_cache::IK>();
        }
        
        vector< boost::shared_ptr<kinematics_cache::IK> > results;
        double halfResolution = resolution / 2.0;
        
        BSONObjBuilder b;
        b.append("group", group);
        
        // Position
        b << "pose.pose.position.x" << GT << (pose.pose.position.x - halfResolution) << LT << (pose.pose.position.x + halfResolution);
        b << "pose.pose.position.y" << GT << (pose.pose.position.y - halfResolution) << LT << (pose.pose.position.y + halfResolution);
        b << "pose.pose.position.z" << GT << (pose.pose.position.z - halfResolution) << LT << (pose.pose.position.z + halfResolution);
        
        // Orientation
        b << "pose.pose.orientation.x" << GT << (pose.pose.orientation.x - halfResolution) << LT << (pose.pose.orientation.x + halfResolution);
        b << "pose.pose.orientation.y" << GT << (pose.pose.orientation.y - halfResolution) << LT << (pose.pose.orientation.y + halfResolution);
        b << "pose.pose.orientation.z" << GT << (pose.pose.orientation.z - halfResolution) << LT << (pose.pose.orientation.z + halfResolution);
        b << "pose.pose.orientation.w" << GT << (pose.pose.orientation.w - halfResolution) << LT << (pose.pose.orientation.w + halfResolution);
      
        mongo::BSONObj query = b.obj();
        cout << query << endl;
        mongo::BSONObj metaDataQuery;
        
        ROS_DEBUG_STREAM("Executing query: " << query);
        if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, true)) {
            assert(results.size() == 1);
            ROS_DEBUG("Query succeeded");
            return results[0];
        }
        ROS_WARN("Query failed. Returned empty result");
        return boost::shared_ptr<kinematics_cache::IK>();
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinematics_cache_node");

	KinematicsCacheNode kcn;
	ros::spin();
}
