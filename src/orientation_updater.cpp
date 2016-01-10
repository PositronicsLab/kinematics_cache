#include <ros/ros.h>
#include <kinematics_cache/IK.h>
#include <kinematics_cache/IKQuery.h>
#include <mongodb_store/message_store.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <boost/math/constants/constants.hpp>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/move_group_interface/move_group.h>

namespace {

using namespace std;
using namespace mongodb_store;
using namespace mongo;

typedef vector<boost::shared_ptr<kinematics_cache::IK> > IKList;
typedef vector< std::pair<boost::shared_ptr<kinematics_cache::IK>, mongo::BSONObj> > IKResultsList;

// Flex joint
const unsigned int JOINT_TO_UPDATE = 5;
static const double pi = boost::math::constants::pi<double>();

class OrientationUpdater {
private:

   //! Node handle
   ros::NodeHandle nh;

   //! Private nh
   ros::NodeHandle pnh;

    //! Database
    MessageStoreProxy mdb;

    //! Arm
    string armName;

    //! TF
    tf::TransformListener tf;

public:
	OrientationUpdater() :
		pnh("~"), mdb(nh) {
         pnh.param<string>("arm", armName, "left_arm");
	}

private:

    void poseToRPY(const geometry_msgs::Pose& pose, double& roll, double& pitch, double& yaw) {
        tf::Pose poseTf;
        tf::poseMsgToTF(pose, poseTf);
        tf::Matrix3x3 m(poseTf.getRotation());
        m.getRPY(roll, pitch, yaw);
    }

    public:
        void update() {
        ROS_INFO("Updating the orientations for %s", armName.c_str());

        ROS_INFO("Querying for all records");
        BSONObjBuilder b;
        b.append("group", armName);

        // TEMP
        b << "pose.pose.position.y" << GT << -0.05 << LT << 0.05;
        // END TEMP

        mongo::BSONObj query = b.obj();
        mongo::BSONObj metaDataQuery;

        ROS_DEBUG_STREAM("Executing query: " << query);
        IKResultsList results;
        if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, false)) {
            ROS_INFO("Query for all entries succeeded");
        } else {
            ROS_ERROR("Query for all entries failed");
            return;
        }

        // Construct the move group for forward kinematics
        robot_model_loader::RobotModelLoader rml("robot_description");
        robot_model::RobotModelPtr model = rml.getModel();
        robot_state::RobotStatePtr state(new robot_state::RobotState(model));
        const robot_state::JointModelGroup* jmg = model->getJointModelGroup(armName);
        moveit::planning_interface::MoveGroup group(armName);

        // Update any configurations where the total joint distance is less than for the current estimate
        for (IKResultsList::iterator iter = results.begin(); iter != results.end(); ++iter) {

            ROS_INFO_STREAM("Joints :");
            for (unsigned int i = 0; i < group.getJoints().size(); ++i) {
                ROS_INFO_STREAM(group.getJoints()[i]);
            }
            ROS_INFO_STREAM(endl);

            double roll, pitch, yaw;
            poseToRPY(iter->first->pose.pose, roll, pitch, yaw);
            ROS_INFO("Before: roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
            // group.setJointValueTarget(iter->first->positions);
            // group.move();

            // We want to set the pose based on whether this is the left or right end effector
            double updateVal = 0.0;
            if (armName == "left_arm") {
                ROS_INFO_STREAM("Positions before update: ");
                for (unsigned int i = 0; i < iter->first->positions.size(); ++i) {
                    ROS_INFO_STREAM(iter->first->positions[i]);
                }
                ROS_INFO_STREAM(endl);

                iter->first->positions[JOINT_TO_UPDATE] += pi / 2.0;
                ROS_INFO_STREAM("Positions after update: ");
                for (unsigned int i = 0; i < iter->first->positions.size(); ++i) {
                    ROS_INFO_STREAM(iter->first->positions[i]);
                }
                ROS_INFO_STREAM(endl);
                iter->first->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0, 0, pi / 2.0);
            }
            else if (armName == "right_arm") {
                iter->first->positions[JOINT_TO_UPDATE] -= pi / 2.0;
                iter->first->pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw (0, 0, pi / -2.0);
            }
            else {
                assert(false && "Unknown arm name");
            }

            state->setJointGroupPositions(jmg, iter->first->positions);
            const Eigen::Affine3d &endEffectorState = state->getGlobalLinkTransform(group.getEndEffectorLink());

            // Convert pose to tf.
            tf::Pose poseTf;
            tf::poseEigenToTF(endEffectorState, poseTf);
            geometry_msgs::Pose pose;
            poseTFToMsg(poseTf, pose);

            // Convert to the same frame as the pose in the db
            ROS_INFO_STREAM("PLANNING FRAME: " << group.getPlanningFrame());

            ros::Time now = ros::Time::now();
            tf.waitForTransform(group.getPlanningFrame(), iter->first->pose.header.frame_id, now, ros::Duration(10.0));

            geometry_msgs::PoseStamped poseInPlanningFrame;
            poseInPlanningFrame.pose = pose;
            poseInPlanningFrame.header.stamp = now;
            poseInPlanningFrame.header.frame_id = group.getPlanningFrame();

            geometry_msgs::PoseStamped poseInBaseFrame;
            tf.transformPose(iter->first->pose.header.frame_id, poseInPlanningFrame, poseInBaseFrame);

            // Print RPY before and after
            poseToRPY(iter->first->pose.pose, roll, pitch, yaw);
            ROS_INFO("After (planned): roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);

            poseToRPY(poseInBaseFrame.pose, roll, pitch, yaw);
            ROS_INFO("After (actual): roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);

            // Test moving it
            group.setJointValueTarget(iter->first->positions);
            group.move();
        }

        // Now perform final updates
        ROS_INFO("Updating records");
        for (IKResultsList::iterator iter = results.begin(); iter != results.end(); ++iter) {
            // TODO: Enable
            // mdb.updateID(iter->second["_id"].OID().toString(), *iter->first);
        }
        ROS_INFO("Completed updating entries. Completed updating orientation for group");
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "orientation_updater");
	OrientationUpdater ou;
	ou.update();
}
