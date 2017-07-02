#include <ros/ros.h>
#include <boost/shared_ptr.hpp>

// MoveIt
#include <moveit/rdf_loader/rdf_loader.h>
#include <urdf/model.h>
#include <srdfdom/model.h>
#include <moveit/kinematics_base/kinematics_base.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <pluginlib/class_loader.h>
#include <moveit/robot_model/joint_model_group.h>
#include <moveit/move_group_interface/move_group.h>

#include <tf/transform_listener.h>
#include <kinematics_cache/IKv2.h>
#include <kinematics_cache/IKQueryv2.h>
#include <boost/math/constants/constants.hpp>
#include <kinematics_cache/OcTreeJointAngles.h>

namespace
{
using namespace std;
using namespace kinematics_cache;
using namespace octomap;

static const double pi = boost::math::constants::pi<double>();
static const double SEARCH_RESOLUTION = 0.1;

class KinematicsCacheFKLoader
{

private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    tf::TransformListener tf;

    string input;

    string output;

    string groupName;

    string baseFrame;

    string globalFrame;

    string endEffectorFrame;

    static bool is_close(double a, double b, double epsilon = 1e-5)
    {
        return std::fabs(a - b) < epsilon;
    }

public:
    KinematicsCacheFKLoader() :
        pnh("~")
    {
        ROS_INFO("Initializing the kinematics cache fk loader");
        pnh.param<string>("group_name", groupName, "left_arm");
        pnh.param<string>("base_frame", baseFrame, "/torso_lift_link");
        pnh.param<string>("globalFrame", globalFrame, "/odom_combined");
        pnh.param<string>("endEffectorFrame", endEffectorFrame, "l_wrist_roll_link");

        if (!pnh.getParam("input", input))
        {
            ROS_ERROR("Input must be specified");
        }

        if (!pnh.getParam("output", output))
        {
            ROS_ERROR("Output must be specified");
        }
        ROS_INFO("Initialization complete");
    }

    void load()
    {
        ROS_INFO("Loading kinematics cache");

        // Load the octtree
        auto_ptr<OcTreeJointAngles> tree(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(input)));

        // Construct the FK solver
        rdf_loader::RDFLoader rdfLoader;
        const boost::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
        const boost::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();

        robot_model::RobotModelPtr kinematicModel(new robot_model::RobotModel(urdfModel, srdf));
        robot_state::RobotStatePtr kinematicState(new robot_state::RobotState(kinematicModel));
        kinematicState->setToDefaultValues();
        const robot_model::JointModelGroup* jointModelGroup = kinematicModel->getJointModelGroup(groupName);

        // Check the root transform
        ROS_INFO("Root transform in %f %f %f", kinematicState->getRootTransform().translation().x(), kinematicState->getRootTransform().translation().y(), kinematicState->getRootTransform().translation().z());

        vector<string> activeJointModelNames;
        for (unsigned int i = 0; i < jointModelGroup->getJointModels().size(); ++i)
        {
            activeJointModelNames.push_back(jointModelGroup->getJointModels()[i]->getName());
        }

        unsigned int numLoaded = 0;

        // Frames never move so we can use a constant transform
        tf::StampedTransform globalToBaseTransform;
        tf.waitForTransform(baseFrame, globalFrame, ros::Time::now(), ros::Duration(10.0));
        tf.lookupTransform(baseFrame /* target */, globalFrame /* source */, ros::Time(0), globalToBaseTransform);
        assert(globalToBaseTransform.child_frame_id_ == globalFrame);
        assert(globalToBaseTransform.frame_id_ == baseFrame);

        vector<double> angles;
        angles.resize(7);

        ROS_INFO("Beginning. Maximum cells is %u", octomap::MAX_SOLUTIONS);
        for (double i = 4; i >= 1; i /= 2)
        {
            double searchResolution = SEARCH_RESOLUTION * i;
            ROS_INFO("Setting search resolution to %f", searchResolution);
            for (angles[0] = jointModelGroup->getJointModels()[0]->getVariableBounds()[0].first; angles[0] <= jointModelGroup->getJointModels()[0]->getVariableBounds()[0].second; angles[0] += searchResolution)
            {
                ROS_INFO("Incrementing outer-most joint to %f in range[%f, %f]", angles[0], jointModelGroup->getJointModels()[0]->getVariableBounds()[0].first, jointModelGroup->getJointModels()[0]->getVariableBounds()[0].second);
                for (angles[1] = jointModelGroup->getJointModels()[1]->getVariableBounds()[0].first; angles[1] <= jointModelGroup->getJointModels()[1]->getVariableBounds()[0].second; angles[1] += searchResolution)
                {
                    for (angles[2] = jointModelGroup->getJointModels()[2]->getVariableBounds()[0].first; angles[2] <= jointModelGroup->getJointModels()[2]->getVariableBounds()[0].second; angles[2] += searchResolution)
                    {
                        for (angles[3] = jointModelGroup->getJointModels()[3]->getVariableBounds()[0].first; angles[3] <= jointModelGroup->getJointModels()[3]->getVariableBounds()[0].second; angles[3] += searchResolution)
                        {
                            for (angles[4] = jointModelGroup->getJointModels()[4]->getVariableBounds()[0].first; angles[4] <= jointModelGroup->getJointModels()[4]->getVariableBounds()[0].second; angles[4] += searchResolution)
                            {

                                // Compute forward kinematics
                                ROS_DEBUG("Executing FK for values %f %f %f %f %f %f %f", angles[0], angles[1], angles[2], angles[3], angles[4], angles[5], angles[6]);
                                kinematicState->setStateValues(activeJointModelNames, angles);
                                kinematicState->enforceBounds();
                                kinematicState->updateLinkTransforms();

                                assert(kinematicState->knowsTransform(endEffectorFrame));
                                const Eigen::Affine3d& endEffectorPosition = kinematicState->getFrameTransform(endEffectorFrame);

                                ROS_DEBUG("Computed FK value in global frame of [%f %f %f]", endEffectorPosition.translation().x(), endEffectorPosition.translation().y(), endEffectorPosition.translation().z());

                                // Construct a point
                                geometry_msgs::PointStamped resultInGlobalFrameStamped;
                                resultInGlobalFrameStamped.point.x = endEffectorPosition.translation().x();
                                resultInGlobalFrameStamped.point.y = endEffectorPosition.translation().y();
                                resultInGlobalFrameStamped.point.z = endEffectorPosition.translation().z();
                                resultInGlobalFrameStamped.header.stamp = ros::Time(0);
                                resultInGlobalFrameStamped.header.frame_id = globalFrame;

                                tf::Stamped<tf::Point> tfGlobalPoint;
                                tf::pointStampedMsgToTF(resultInGlobalFrameStamped, tfGlobalPoint);
                                tf::Point tfBase = globalToBaseTransform * tfGlobalPoint;

                                ROS_DEBUG("Computed FK value in torso frame of [%f %f %f]", tfBase.x(), tfBase.y(), tfBase.z());

                                // Save the results
                                numLoaded++;
                                octomap::point3d point((float) tfBase.x(),
                                                       (float) tfBase.y(),
                                                       (float) tfBase.z());

                                OcTreeNodeJointAngles* node = tree->updateNode(point);

                                // Update the solutions array
                                angles_t currentAngles = node->getValue();
                                unsigned int k = currentAngles[0];

                                if (k == octomap::MAX_SOLUTIONS)
                                {
                                    ROS_DEBUG("Cell is at maximum. Cannot add solution");
                                    continue;
                                }
                                else
                                {
                                    ROS_DEBUG("Adding solution number %u", k + 1);
                                }

                                // Ensure the value does not match an existing value
                                bool matches = false;
                                for (unsigned int i = 0; i < k; ++i) {
                                    bool isDifferent = false;
                                    for (unsigned int m = 0; m < octomap::NUM_JOINTS; ++m)
                                    {
                                        if(!is_close(angles[m], currentAngles[1 + i * octomap::NUM_JOINTS + m])) {
                                            isDifferent = true;
                                            break;
                                        }
                                    }
                                    if (!isDifferent) {
                                        matches = true;
                                        break;
                                    }
                                }

                                if (matches) {
                                    ROS_INFO("Matches existing entry. Skipping.");
                                    continue;
                                }

                                // Copy over the values
                                for (unsigned int m = 0; m < octomap::NUM_JOINTS; ++m)
                                {
                                    currentAngles[1 + k * octomap::NUM_JOINTS + m] = angles[m];
                                }

                                // Update the solution count
                                k++;
                                currentAngles[0] = k;
                                node->setValue(currentAngles);
                            }
                        }
                    }
                }
            }
        }
        ROS_INFO("Completed updating entries. Writing tree with %u additional entries", numLoaded);
        tree->write(output);
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, ros::this_node::getName());

    KinematicsCacheFKLoader kcl;
    kcl.load();
}
