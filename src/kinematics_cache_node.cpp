#include <ros/ros.h>
#include <kinematics_cache/IK.h>
#include <kinematics_cache/IKQuery.h>
#include <mongodb_store/message_store.h>
#include <visualization_msgs/Marker.h>

namespace {
struct PoseCompare {
    bool operator() (const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) const {
        if (a.position.x != b.position.x) {
            return a.position.x < b.position.x;
        }
        if (a.position.y != b.position.y) {
            return a.position.y < b.position.y;
        }
        if (a.position.z != b.position.z) {
            return a.position.z < b.position.z;
        }
        if (a.orientation.x != b.orientation.x) {
            return a.orientation.x < b.orientation.x;
        }
        if (a.orientation.y != b.orientation.y) {
            return a.orientation.y < b.orientation.y;
        }
        if (a.orientation.z != b.orientation.z) {
            return a.orientation.z < b.orientation.z;
        }
        return a.orientation.w < b.orientation.w;
    }
};

using namespace std;
using namespace mongodb_store;
using namespace mongo;

static const double RESOLUTION_DEFAULT = 0.01;

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
         pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");

         allIkPub = nh.advertise<visualization_msgs::Marker>(
			"/kinematics_cache/known_ik_positions_perm", 1);

        // Detetermine if we should clean on startup
        bool shouldClean;
        pnh.param<bool>("clean", shouldClean, false);
        if(shouldClean) {
            ROS_INFO("Cleaning database. This may take a few moments.");
            clean();
        }

        ikService = nh.advertiseService("/kinematics_cache/ik",
            &KinematicsCacheNode::query, this);
	}

private:
    void visualize(const string group, const double maxTime = 60.0) {
        vector<boost::shared_ptr<kinematics_cache::IK> > ikPositions = queryAll(group, maxTime);
        ROS_DEBUG("Found %lu results", ikPositions.size());

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
        res.execution_time = result->execution_time;
        return true;
    }

    bool withinResolution(double a, double b) const {
        return fabs(a - b) < resolution / 2.0;
    }

    bool posesEqual(const geometry_msgs::Pose& a, const geometry_msgs::Pose& b) const {

        return withinResolution(a.position.x, b.position.x) &&
               withinResolution(a.position.y, b.position.y) &&
               withinResolution(a.position.z, b.position.z) &&
               withinResolution(a.orientation.x, b.orientation.x) &&
               withinResolution(a.orientation.y, b.orientation.y) &&
               withinResolution(a.orientation.z, b.orientation.z) &&
               withinResolution(a.orientation.w, b.orientation.w);
    }

    void clean() {
        ROS_INFO("Cleaning the IK cache.");
        // Brute force. Search all and load multimap.

        // Clean one group at a time
        const string groups[] = {"left_arm", "right_arm"};
        for (unsigned int i = 0; i < boost::size(groups); ++i) {
            ROS_INFO("Cleaning group %s", groups[i].c_str());
            multimap<geometry_msgs::Pose, string, PoseCompare> poses;
            vector< std::pair<boost::shared_ptr<kinematics_cache::IK>, mongo::BSONObj> > results;

            BSONObjBuilder b;
            b.append("group", groups[i]);

            mongo::BSONObj query = b.obj();
            mongo::BSONObj metaDataQuery;

            ROS_DEBUG_STREAM("Executing query: " << query);
            if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, false)) {
                ROS_INFO("Query for all entries succeeded");
            } else {
                ROS_ERROR("Query for all entries failed");
                return;
            }

            // Create a map from pose to id
            ROS_INFO("Loading the multimap with %lu results", results.size());
            for (unsigned int j = 0; j < results.size(); ++j) {
                // Extract object ID
                string objectId = results[j].second["_id"].OID().toString();
                poses.insert(make_pair(results[j].first->pose.pose, objectId));
            }
            ROS_INFO("Multimap loaded successfully");

            geometry_msgs::Pose lastPose;
            vector<string> idsToDelete;
            ROS_INFO("Searching for duplicates");
            for (multimap<geometry_msgs::Pose, string>::iterator iter = poses.begin(); iter != poses.end(); ++iter) {
                if (posesEqual(lastPose, iter->first)) {
                    ROS_INFO("Found duplicate pose. Adding id %s to delete", iter->second.c_str());
                    idsToDelete.push_back(iter->second);
                }
                lastPose = iter->first;
            }
            ROS_INFO("Duplicate search completed. Found %lu entries to delete", idsToDelete.size());

            for (vector<string>::iterator id = idsToDelete.begin(); id != idsToDelete.end(); ++id) {
                mdb.deleteID(*id);
            }
            ROS_INFO("Completed deleting entries. Completed cleaning cache for group");
        }
    }

    boost::shared_ptr<kinematics_cache::IK> query(const std::string group,
        const geometry_msgs::PoseStamped pose) {
        // Do not perform transform, as it is a footgun for performance
        if (pose.header.frame_id != baseFrame) {
            ROS_ERROR("Queries must be specified in %s not %s", baseFrame.c_str(), pose.header.frame_id.c_str());
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
        mongo::BSONObj metaDataQuery;

        ROS_DEBUG_STREAM("Executing query: " << query);
        if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, true)) {
            if (results.size() > 1) {
                ROS_WARN("Multiple results returned for query. Prune database.");
            }
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
