#include <ros/ros.h>
#include <kinematics_cache/OcTreeJointAngles.h>
#include <octomap/OcTreeIterator.hxx>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/collision_detection/collision_matrix.h>

namespace
{

using namespace std;
using namespace octomap;

class OcTreeCleaner
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Input file
    string input;

    //! Output file
    string output;

    //! Arm
    string arm;

    //! Other arm
    string otherArm;

public:
    OcTreeCleaner() :
        pnh("~")
    {
        if (!pnh.getParam("input", input))
        {
            ROS_ERROR("Input must be specified");
        }

        if (!pnh.getParam("output", output))
        {
            ROS_ERROR("Output must be specified");
        }

        if (!pnh.getParam("arm", arm))
        {
            ROS_ERROR("Arm name must be specified");
        }
        if (!pnh.getParam("other_arm", otherArm))
        {
            ROS_ERROR("Other arm name must be specified");
        }
    }

private:

#if ROS_VERSION_MINIMUM(1, 10, 12)
    // Method not required
#else
    static std::vector<std::string> getActiveJointModelNames(const robot_model::JointModelGroup* jointModelGroup)
    {
        vector<string> activeJointModels;
        for (unsigned int i = 0; i < jointModelGroup->getJointModels().size(); ++i)
        {
            if (jointModelGroup->getJointModels()[i]->getMimic() != NULL)
            {
                ROS_WARN("Passive joint model found");
                continue;
            }
            activeJointModels.push_back(jointModelGroup->getJointModels()[i]->getName());

        }
        return activeJointModels;
    }
#endif // ROS_VERSION_MINIMUM

    static bool is_close(double a, double b, double epsilon = 1e-5)
    {
        return std::fabs(a - b) < epsilon;
    }

    class IsArmLink {
        const robot_model::JointModelGroup* jm1;
        const robot_model::JointModelGroup* jm2;

        public:
            IsArmLink(const robot_model::JointModelGroup* jm1, const robot_model::JointModelGroup* jm2) :
                jm1(jm1), jm2(jm2){}

    bool operator()(const string& x) const {
        return isInJMG(jm1, x) || isInJMG(jm2, x);
    }

    static bool isInJMG(const robot_model::JointModelGroup* jm, const string& x) {
        const std::vector<const robot_model::JointModel*>& roots = jm->getJointRoots();
        vector<string> checked;

        for (unsigned int i = 0; i < roots.size(); ++i) {
            if (isDescendentOfLink(roots[i]->getChildLinkModel(), x, checked)) {
                return true;
            }
        }
        return false;
    }

    static bool isDescendentOfLink(const robot_model::LinkModel* link, const string& x, vector<string>& checked) {
        if (link->getName() == x) {
            return true;
        }

        if (find(checked.begin(), checked.end(), link->getName()) != checked.end()) {
            return false;
        }
        checked.push_back(link->getName());

        const vector<robot_model::JointModel*>& roots = link->getChildJointModels();
        for (unsigned int i = 0; i < roots.size(); ++i) {
            if (isDescendentOfLink(roots[i]->getChildLinkModel(), x, checked)) {
                return true;
            }
        }
        return false;
    }
};
public:
    void clean()
    {
        ROS_INFO("Loading oc_tree from source %s", input.c_str());
        // Load the octtree
        auto_ptr<OcTreeJointAngles> tree(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(input)));

        ROS_INFO("Tree loaded successfully containing %lu nodes", tree->getNumLeafNodes());

        unsigned int updates = 0;
        unsigned int validEntries = 0;

        // Check for self-collision with this solution
        planning_scene_monitor::PlanningSceneMonitor planningScene("robot_description");
        planning_scene::PlanningScenePtr currentScene = planningScene.getPlanningScene();
        robot_state::RobotState currentRobotState = currentScene->getCurrentState();

        // Load the RDF model
        rdf_loader::RDFLoader rdfLoader;
        const boost::shared_ptr<srdf::Model> &srdf = rdfLoader.getSRDF();
        const boost::shared_ptr<urdf::ModelInterface>& urdfModel = rdfLoader.getURDF();

        robot_model::RobotModelPtr kinematicModel(new robot_model::RobotModel(urdfModel, srdf));
        ROS_INFO("Robot model initialized successfully");

        const robot_model::JointModelGroup* jointModelGroup =  kinematicModel->getJointModelGroup(arm);
        const robot_model::JointModelGroup* otherJointModelGroup =  kinematicModel->getJointModelGroup(otherArm);

#if ROS_VERSION_MINIMUM(1, 10, 12)
        const vector<string> jointNames = jointModelGroup->getActiveJointModelNames();
#else
        const vector<string> jointNames = getActiveJointModelNames(jointModelGroup);
        const vector<string> otherJointNames = getActiveJointModelNames(otherJointModelGroup);
#endif

        // Iterate over all the nodes in the tree
        for (OcTreeJointAngles::leaf_iterator leaf = tree->begin_leafs(); leaf != tree->end_leafs(); ++leaf)
        {
            angles_t angles = leaf->getValue();
            angles_t updatedAngles;

            unsigned int k = angles[0];
            unsigned int newK = 0;

            for (unsigned int i = 0; i < k; ++i)
            {
                bool valid = true;

                vector<double> positions;
                positions.resize(octomap::NUM_JOINTS);
                for (unsigned int n = 0; n < octomap::NUM_JOINTS; ++n)
                {
                    positions[n] = leaf->getValue()[n + 1 + i * octomap::NUM_JOINTS];
                }
#if ROS_VERSION_MINIMUM(1, 10, 12)
                currentRobotState.setVariablePositions(jointNames, positions);
#else
                currentRobotState.setStateValues(jointNames, positions);
#endif

                if (currentScene->isStateColliding(currentRobotState, arm, false))
                {
                    vector<string> collidingLinks;
                    currentScene->getCollidingLinks(collidingLinks, currentRobotState);

                    // Remove arm links
                    ROS_INFO("Potential collision found. Checking for arm links. Current size is %lu", collidingLinks.size());
                    collidingLinks.erase(remove_if(collidingLinks.begin(),
                              collidingLinks.end(), IsArmLink(jointModelGroup, otherJointModelGroup)), collidingLinks.end());

                    if (!collidingLinks.empty()) {
                        valid = false;
                        ROS_INFO("State in collision. Colliding links size %lu: ", collidingLinks.size());
                        for (unsigned int l = 0; l < collidingLinks.size(); ++l) {
                            ROS_INFO("%s", collidingLinks[l].c_str());
                        }
                    }
                }

                for (int j = i - 1; j >= 0; --j)
                {
                    // Check for equivalence
                    bool notEqual = false;
                    for (unsigned int l = 0; l < octomap::NUM_JOINTS; ++l)
                    {
                        if (!is_close(angles[l + 1 + i * octomap::NUM_JOINTS], angles[l + 1 + j * octomap::NUM_JOINTS]))
                        {
                            notEqual = true;
                            break;
                        }
                    }

                    if (!notEqual)
                    {
                        ROS_INFO("Removing duplicate");
                        ROS_INFO("Node i: %f %f %f %f %f %f %f", angles[0 + 1 + i * octomap::NUM_JOINTS], angles[1 + 1 + i * octomap::NUM_JOINTS],
                                 angles[2 + 1 + i * octomap::NUM_JOINTS], angles[3 + 1 + i * octomap::NUM_JOINTS], angles[4 + 1 + i * octomap::NUM_JOINTS],
                                 angles[5 + 1 + i * octomap::NUM_JOINTS], angles[6 + 1 + i * octomap::NUM_JOINTS]);
                        ROS_INFO("Node j: %f %f %f %f %f %f %f", angles[0 + 1 + j * octomap::NUM_JOINTS], angles[1 + 1 + j * octomap::NUM_JOINTS],
                                 angles[2 + 1 + j * octomap::NUM_JOINTS], angles[3 + 1 + j * octomap::NUM_JOINTS], angles[4 + 1 + j * octomap::NUM_JOINTS],
                                 angles[5 + 1 + j * octomap::NUM_JOINTS], angles[6 + 1 + j * octomap::NUM_JOINTS]);
                        valid = false;
                        break;
                    }
                }

                if (!valid) {
                    updates++;
                } else {
                    validEntries++;
                    // Copy over the values
                    for (unsigned int m = 0; m < octomap::NUM_JOINTS; ++m) {
                        updatedAngles[1 + newK * octomap::NUM_JOINTS + m] = angles[1 + i * octomap::NUM_JOINTS + m];
                    }
                    newK++;
                }
            }

            // Update k
            updatedAngles[0] = newK;
            leaf->setValue(updatedAngles);
        }

        // Write tree to file
        if (updates > 0)
        {
            ROS_INFO("Completed updating entries. Writing tree with %u valid entries", validEntries);
            tree->write(output);
        }

        ROS_INFO("Cleanup complete. Cleaned %u entries and kept %u valid entries", updates, validEntries);
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oc_tree_cleaner");
    OcTreeCleaner otc;
    otc.clean();
}
