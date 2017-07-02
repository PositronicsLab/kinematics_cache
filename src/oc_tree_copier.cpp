#include <ros/ros.h>
#include <kinematics_cache/OcTreeJointAngles.h>
#include <kinematics_cache/OcTreeJointAngles2.h>
#include <octomap/OcTreeIterator.hxx>

namespace
{

using namespace std;
using namespace octomap;

class OcTreeCopier
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
    OcTreeCopier() :
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

        pnh.param("resolution", resolution, 0.02);
    }

    void copy()
    {
        // Load the octtree
        auto_ptr<OcTreeJointAngles> originalTree(dynamic_cast<OcTreeJointAngles*> (OcTreeJointAngles::read(input)));

        // Create the new tree
        OcTreeJointAngles2 newTree(resolution);

        // Iterate over all the nodes in the tree
        for (OcTreeJointAngles::leaf_iterator leaf = originalTree->begin_leafs(); leaf != originalTree->end_leafs(); ++leaf)
        {
            angles_t angles = leaf->getValue();
            angles2_t angles2;

            for (unsigned int i = 0; i < angles.size(); ++i) {
                angles2[i] = angles[i];
            }

            OcTreeNodeJointAngles2* node = newTree.updateNode(leaf.getCoordinate());
            node->setValue(angles2);
        }

        ROS_INFO("Completed updating entries.");
        newTree.write(output);
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oc_tree_copier");
    OcTreeCopier otc;
    otc.copy();
}
