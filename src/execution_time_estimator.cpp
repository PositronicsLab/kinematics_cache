#include <ros/ros.h>
#include <kinematics_cache/IK.h>
#include <kinematics_cache/IKQuery.h>
#include <mongodb_store/message_store.h>
#include <tf/transform_listener.h>
#include <moveit/move_group_interface/move_group.h>
#include <actionlib/client/simple_action_client.h>
#include <human_catching/MoveArmFastAction.h>

namespace {

using namespace std;
using namespace mongodb_store;
using namespace mongo;

typedef vector<boost::shared_ptr<kinematics_cache::IK> > IKList;
typedef vector< std::pair<boost::shared_ptr<kinematics_cache::IK>, mongo::BSONObj> > IKResultsList;
typedef actionlib::SimpleActionClient<human_catching::MoveArmFastAction> ArmClient;

class ExecutionTimeEstimator {
private:

   //! Node handle
   ros::NodeHandle nh;

   //! Private nh
   ros::NodeHandle pnh;

    //! Database
    MessageStoreProxy mdb;

    //! Number of trials per configuration
    int trialsPerConfiguration;

    //! Number of trials
    int trials;
public:
	ExecutionTimeEstimator() :
		pnh("~"), mdb(nh) {
         pnh.param<int>("trialsPerConfiguration", trialsPerConfiguration, 3);
         pnh.param<int>("trials", trials, 10);
        updateExecutionTimes();
	}

private:

    double distance(const vector<double>& jointPositionsA, const vector<double>& jointPositionsB) const {
        assert (jointPositionsA.size() == jointPositionsB.size());
        double d = 0;
        for (unsigned int i = 0; i < jointPositionsA.size(); ++i) {
            d += pow(jointPositionsA[i] - jointPositionsB[i], 2);
        }
        return d;
    }

    void updateExecutionTimes() {
        ROS_INFO("Updating the execution times");

        // Update one group at a time
        const string groups[] = {"left_arm", "right_arm"};
        for (unsigned int i = 0; i < boost::size(groups); ++i) {
            moveit::planning_interface::MoveGroup group(groups[i]);

            ArmClient arm(groups[i] + "_move_arm_fast_action_server", true);
            ROS_INFO("Waiting for %s", (groups[i] + "_move_arm_fast_action_server").c_str());
            arm.waitForServer();

            ROS_INFO("Updating group %s", groups[i].c_str());

            ROS_INFO("Resetting arm to zero position");
            human_catching::MoveArmFastGoal baseGoal;
            baseGoal.joint_positions = vector<double>(7);
            arm.sendGoal(baseGoal);

            // TODO: Move other arm out of the way.

            ROS_INFO("Querying for all records");
            BSONObjBuilder b;
            b.append("group", groups[i]);

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

            for (unsigned int i = 0; i < trials; ++i) {
                ROS_INFO("Beginning trial %u", i);

                // Initialize to random pose
                // TODO: Use a resolution complete method
                vector<double> randomJointAngles = group.getRandomJointValues();

                // Simulate the movement and average the time
                ros::Duration totalTime(0);
                for (unsigned int i = 0; i < trialsPerConfiguration; ++ i) {

                    // Move the arm
                    ROS_INFO("Moving arm to random position.");
                    human_catching::MoveArmFastGoal goal;
                    goal.joint_positions = randomJointAngles;
                    ros::Time begin = ros::Time::now();
                    arm.sendGoal(goal);
                    ros::Time end = ros::Time::now();
                    ROS_INFO("Arm movement completed");

                    // Calculate duration
                    totalTime += (end - begin);

                    // Reset the arm to the zero configuration
                    ROS_INFO("Resetting arm to zero position");
                    arm.sendGoal(baseGoal);
                    ROS_INFO("Reset completed");
                }

                // Average the time
                ros::Duration timePerTrial(totalTime.toSec() / trialsPerConfiguration);

                ROS_INFO("Simulated estimated is %f", timePerTrial.toSec());

                // Update any configurations where the total joint distance is less than for the current estimate
                for (IKResultsList::iterator iter = results.begin(); iter != results.end(); ++iter) {
                    double currDistance = distance(iter->first->positions, randomJointAngles);
                    ROS_DEBUG("Distance to current configuration is %f", currDistance);
                    if (currDistance < iter->first->minimum_distance) {
                        ROS_DEBUG("Updating due to %f lower than %f. Calculated estimate was %f.", currDistance,
                                 iter->first->minimum_distance, iter->first->execution_time.toSec());
                        boost::shared_ptr<kinematics_cache::IK> msg = iter->first;
                        msg->minimum_distance = currDistance;
                        msg->simulated_execution_time = timePerTrial;
                    }
                }
            }

            // Now perform final updates
            ROS_INFO("Updating records");
            for (IKResultsList::iterator iter = results.begin(); iter != results.end(); ++iter) {
                mdb.updateID(iter->second["_id"].OID().toString(), *iter->first);
            }
            ROS_INFO("Completed updating entries. Completed updating execution time for group");
        }
        ROS_INFO("All updates complete");
    }
};
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "execution_time_estimator");
	ExecutionTimeEstimator ete;
	ros::spin();
}
