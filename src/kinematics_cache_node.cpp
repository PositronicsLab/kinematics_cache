#include <ros/ros.h>
#include <kinematics_cache/IK.h>
#include <kinematics_cache/IKQuery.h>
#include <mongodb_store/message_store.h>
#include <visualization_msgs/Marker.h>

namespace {
using namespace std;
using namespace mongodb_store;
using namespace mongo;

static const double RESOLUTION_DEFAULT = 0.1; /* Default value: 0.01 */

class KinematicsCacheNode {
private:
	//! Publisher for the dog position visualization.
	ros::Publisher allIkPub;
    
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
    
    //! IK Service
    ros::ServiceServer ikService;
public:
	KinematicsCacheNode() :
		pnh("~"), mdb(nh) {
         pnh.param("resolution", resolution, RESOLUTION_DEFAULT);
         pnh.param<string>("base_frame", baseFrame, "torso_lift_link");
         
         allIkPub = nh.advertise<visualization_msgs::Marker>(
				"/kinematics_cache_node/known_ik_positions_perm", 1);
        
        ikService = nh.advertiseService("kinematics_cache/ik",
            &KinematicsCacheNode::query, this);
	}
    
private:
    void visualize(const string group, const double maxTime = 60.0) {
        vector<boost::shared_ptr<kinematics_cache::IK> > ikPositions = queryAll(group, maxTime);
        ROS_INFO("Found %lu results", ikPositions.size());
        
        visualization_msgs::Marker points;
        points.header.frame_id = baseFrame;
        points.header.stamp = ros::Time::now();
        points.ns = "ik_positions";
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.pose.orientation.w = 1.0;
        points.scale.x = 0.01;
        points.scale.y = 0.01;
        
        // Points are green
        points.color.g = 1.0f;
        points.color.a = 1.0;
        
        for (unsigned int i = 0; i < ikPositions.size(); ++i) {
            ROS_DEBUG("%f, %f, %f", ikPositions[i]->pose.pose.position.x,ikPositions[i]->pose.pose.position.y,ikPositions[i]->pose.pose.position.z);
            points.points.push_back(ikPositions[i]->pose.pose.position);
        }
        allIkPub.publish(points);
    }
    vector<boost::shared_ptr<kinematics_cache::IK> > queryAll(const string group, const double maxTime = 60.0) {
                 vector< boost::shared_ptr<kinematics_cache::IK> > results;
        
        BSONObjBuilder b;
        b.append("group", group);
        // TODO: This must use nsecs too
        b << "execution_time.secs" << LT << maxTime << GT << 0.0;
        
        mongo::BSONObj query = b.obj();
        mongo::BSONObj metaDataQuery;
        
        ROS_DEBUG_STREAM("Executing query: " << query);
        if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, false)) {
            ROS_DEBUG("Query succeeded");
            return results;
        }
        ROS_WARN("Query failed. Returned empty result");
        return vector<boost::shared_ptr<kinematics_cache::IK> >();
    }
    
    bool query(kinematics_cache::IKQuery::Request& req,
               kinematics_cache::IKQuery::Response& res) {
        boost::shared_ptr<kinematics_cache::IK> result = query(req.group, req.pose);
        if (result.get() == NULL) {
            ROS_INFO("Failed to find IK result for service call");
            return false;
        }
        res.positions = result->positions;
        return true;
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
