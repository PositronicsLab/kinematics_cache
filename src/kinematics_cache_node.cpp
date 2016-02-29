#include <ros/ros.h>

// Ensure this is defined before boost since mongodb uses version
// 2 and both cannot be simulataneously defined.
#define BOOST_FILESYSTEM_VERSION 2

#include <kinematics_cache/IK.h>
#include <kinematics_cache/IKQuery.h>
#include <mongodb_store/message_store.h>
#include <visualization_msgs/Marker.h>
#include <tf/transform_listener.h>

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

typedef vector<boost::shared_ptr<kinematics_cache::IK> > IKList;
typedef vector< std::pair<boost::shared_ptr<kinematics_cache::IK>, mongo::BSONObj> > IKResultsList;

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

    //! Maximum arm distance
    double maxDistance;

    //! Base frame for IK
    string baseFrame;

    //! IK Service
    ros::ServiceServer ikService;

    //! TF
    tf::TransformListener tf;
public:
	KinematicsCacheNode() :
		pnh("~"), mdb(nh) {
         pnh.param("resolution", resolution, RESOLUTION_DEFAULT);
         pnh.param("max_distance", maxDistance, 1000.0 /* Large number to disable check */);
         pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");

        // Publish the object location
        ros::SubscriberStatusCallback connectCB = boost::bind(&KinematicsCacheNode::startListening, this);
        ros::SubscriberStatusCallback disconnectCB = boost::bind(&KinematicsCacheNode::stopListening, this);


         allIkPub = nh.advertise<visualization_msgs::Marker>(
			"/kinematics_cache/known_ik_positions_perm", 1, connectCB, disconnectCB);

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
    void startListening(){
        ROS_INFO("Receiving a registration request for visualization");
        // Broadcast to the new listener
        visualize("left_arm", 0);
        visualize("right_arm", 0);
    }

    void stopListening(){
        ROS_INFO("Receiving a registration stop request for visualization");
    }

    void visualize(const string group, const double maxTime = 60.0) {
        ROS_INFO("Querying for all results");
        IKList ikPositions = queryAll(group, maxTime);
        ROS_INFO("Found %lu results", ikPositions.size());

        visualization_msgs::Marker points;
        points.header.frame_id = baseFrame;
        points.header.stamp = ros::Time::now();
        points.ns = "ik_positions_" + group;
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

    IKList queryAll(const string group, const int maxTime = 60) {

        IKList results;
        BSONObjBuilder b;
        b.append("group", group);

        if (maxTime > 0) {
            b << "execution_time.secs" << LT << floor(maxTime);
        }

        mongo::BSONObj query = b.obj();
        mongo::BSONObj metaDataQuery;

        ROS_DEBUG_STREAM("Executing query: " << query);
        if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, false)) {
            ROS_DEBUG("Query succeeded");
            return results;
        }
        ROS_WARN("Query failed. Returned empty result");
        return IKList();
    }

    bool query(kinematics_cache::IKQuery::Request& req,
               kinematics_cache::IKQuery::Response& res) {

        IKList results;
        bool success = query(req.group, req.error, req.pose, results);
        if (success) {
            ROS_INFO("Failed to find IK result for service call");
            return false;
        }

        for (IKList::iterator i = results.begin(); i != results.end(); ++i) {
            if ((*i)->positions.size() == 0) {
                ROS_WARN("IK result was invalid");
                continue;
            }
            res.results.push_back(**i);
        }

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
            IKResultsList results;

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

            vector<string> idsToDelete;

            // Remove all poses that have a zero execution time.
            for (IKResultsList::iterator iter = results.begin(); iter != results.end(); ++iter) {
                if (iter->first->execution_time.toSec() <= 0.0) {
                    ROS_INFO("Found pose with zero execution time: %f. Removing.", iter->first->execution_time.toSec());
                    idsToDelete.push_back(iter->second["_id"].OID().toString());
                    results.erase(iter);
                }
            }

            // Remove poses with invalid frame
            for (IKResultsList::iterator iter = results.begin(); iter != results.end(); ++iter) {
                if (iter->first->pose.header.frame_id != baseFrame) {
                    ROS_DEBUG("Found pose with invalid frame: %s. Converting.", iter->first->pose.header.frame_id.c_str());

                    // Transform the frame
                    geometry_msgs::PoseStamped targetInBaseFrame;

                    // Assume that the pose was created in the base configuration of the robot, which is the same
                    // as the current configuration
                    ros::Time now = ros::Time::now();
                    iter->first->pose.header.stamp = now;
                    tf.waitForTransform(iter->first->pose.header.frame_id, baseFrame, now, ros::Duration(10.0));
                    tf.transformPose(baseFrame, iter->first->pose, targetInBaseFrame);

                    // Now update the database
                    boost::shared_ptr<kinematics_cache::IK> msg = iter->first;
                    msg->pose = targetInBaseFrame;
                    mdb.updateID(iter->second["_id"].OID().toString(), *msg);
                }
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

    static double calcDistance(const geometry_msgs::PoseStamped& pose) {
        return sqrt(pow(pose.pose.position.x, 2) +
                    pow(pose.pose.position.y, 2) +
                    pow(pose.pose.position.z, 2));
    }

    bool query(const std::string group,
        double error,
        const geometry_msgs::PoseStamped pose, IKList& results) {

        ROS_INFO("Querying for group %s with error %f at position %f %f %f", group.c_str(), error,
                 pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);

        // Do not perform transform, as it is a footgun for performance
        if (pose.header.frame_id != baseFrame) {
            ROS_ERROR("Queries must be specified in %s not %s", baseFrame.c_str(), pose.header.frame_id.c_str());
            return false;
        }

        if (error == 0) {
            ROS_INFO("Setting error to default value");
            error = resolution / 2.0;
        }


        if (calcDistance(pose) > maxDistance + error) {
            ROS_INFO("Position beyond the maximum reach of the arm");
            return true;
        }

        BSONObjBuilder b;

        // Query any group if it is not set
        if (!group.empty()) {
            ROS_DEBUG("Executing query for group %s", group.c_str());
            b.append("group", group);
        }

        // Position
        b << "pose.pose.position.x" << GT << (pose.pose.position.x - error) << LT << (pose.pose.position.x + error);
        b << "pose.pose.position.y" << GT << (pose.pose.position.y - error) << LT << (pose.pose.position.y + error);
        b << "pose.pose.position.z" << GT << (pose.pose.position.z - error) << LT << (pose.pose.position.z + error);

        // Orientation
        // TODO: Enable orientation
        // b << "pose.pose.orientation.x" << GT << (pose.pose.orientation.x - error) << LT << (pose.pose.orientation.x + error);
        // b << "pose.pose.orientation.y" << GT << (pose.pose.orientation.y - error) << LT << (pose.pose.orientation.y + error);
        // b << "pose.pose.orientation.z" << GT << (pose.pose.orientation.z - error) << LT << (pose.pose.orientation.z + error);
        // b << "pose.pose.orientation.w" << GT << (pose.pose.orientation.w - error) << LT << (pose.pose.orientation.w + error);

        mongo::BSONObj query = b.obj();
        mongo::BSONObj metaDataQuery;

        ROS_DEBUG_STREAM("Executing query: " << query);
        if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, false)) {
            ROS_INFO("Query succeeded. Found %lu results.", results.size());
            return true;
        }
        ROS_WARN("Query failed");
        return false;
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinematics_cache_node");
	KinematicsCacheNode kcn;
	ros::spin();
}
