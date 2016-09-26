#include <ros/ros.h>
#include <kinematics_cache/OcTreeJointAngles.h>
#include <octomap/OcTreeIterator.hxx>

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

    //! Resolution
    double resolution;

public:
    OcTreeCleaner() :
        pnh("~")
    {
        pnh.param<string>("input", input, "");
        pnh.param<string>("output", output, "");
    }

private:

static bool is_close(double a, double b, double epsilon = 1e-5) {
    return std::fabs(a - b) < epsilon;
}

public:
    void clean() {

        // Load the octtree
        auto_ptr<OcTreeJointAngles> tree(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(input)));

        unsigned int updates = 0;

        // Iterate over all the nodes in the tree
        for (OcTreeJointAngles::leaf_iterator leaf = tree->begin_leafs(); leaf != tree->end_leafs(); ++leaf) {

            // Loop backward to find duplicates
            angles_t angles = leaf->getValue();
            unsigned int k = angles[0];
            for (unsigned int i = k - 1; i > 0; --i) {
                for (int j = i - 1; j >= 0; --j) {

                    // Check for equivalence
                    bool notEqual = false;
                    for (unsigned int l = 0; l < octomap::NUM_JOINTS; ++l) {
                        if (!is_close(angles[l + 1 + i * octomap::NUM_JOINTS], angles[l + 1 + j * octomap::NUM_JOINTS])) {
                            notEqual = true;
                            break;
                        }
                    }

                    if (!notEqual) {
                        ROS_INFO("Removing duplicate");
                        ROS_INFO("Node i: %f %f %f %f %f %f %f", angles[0 + 1 + i * octomap::NUM_JOINTS], angles[1 + 1 + i * octomap::NUM_JOINTS],
                                 angles[2 + 1 + i * octomap::NUM_JOINTS], angles[3 + 1 + i * octomap::NUM_JOINTS], angles[4 + 1 + i * octomap::NUM_JOINTS],
                                 angles[5 + 1 + i * octomap::NUM_JOINTS], angles[6 + 1 + i * octomap::NUM_JOINTS]);
                        ROS_INFO("Node j: %f %f %f %f %f %f %f", angles[0 + 1 + j * octomap::NUM_JOINTS], angles[1 + 1 + j * octomap::NUM_JOINTS],
                                 angles[2 + 1 + j * octomap::NUM_JOINTS], angles[3 + 1 + j * octomap::NUM_JOINTS], angles[4 + 1 + j * octomap::NUM_JOINTS],
                                 angles[5 + 1 + j * octomap::NUM_JOINTS], angles[6 + 1 + j * octomap::NUM_JOINTS]);
                        if (i == k - 1){
                            --k;
                        } else {
                            // TODO: Move entries.
                        }
                        ++updates;
                        break;
                    }
                 }
            }

            // Update k
            angles[0] = k;
            leaf->setValue(angles);
        }

        // Write tree to file
        if (updates > 0) {
            ROS_INFO("Completed updating entries. Writing tree");
            tree->write(output);
        }

        ROS_INFO("Cleanup complete. Cleaned %u entries", updates);
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oc_tree_cleaner");
    OcTreeCleaner otc;
    otc.clean();
}
