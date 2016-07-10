#include "kinematics_cache/OcTreeJointAngles.h"

namespace octomap
{

OcTreeJointAngles::OcTreeJointAngles(double resolution)
    : OcTreeBase<OcTreeNodeJointAngles>(resolution)
{
}

bool OcTreeNodeJointAngles::createChild(unsigned int i)
{
    if (children == NULL)
    {
        allocChildren();
    }
    assert (children[i] == NULL);
    children[i] = new OcTreeNodeJointAngles();
    return true;
}

OcTreeNodeJointAngles* OcTreeJointAngles::updateNode(const point3d& value)
{
    OcTreeKey key;
    if (!coordToKeyChecked(value, key)) return NULL;
    return updateNode(key);
}

OcTreeNodeJointAngles* OcTreeJointAngles::updateNode(const OcTreeKey& k)
{
    if (root == NULL){
      root = new OcTreeNodeJointAngles();
      tree_size++;
    }

    OcTreeNodeJointAngles* currNode(root);

    // follow or construct nodes down to last level...
    for (int i = (tree_depth - 1); i >= 0; i--)
    {
        unsigned int pos = computeChildIdx(k, i);

        // requested node does not exist
        if (!currNode->childExists(pos))
        {
            currNode->createChild(pos);
            tree_size++;
        }

        // descend tree
        currNode = static_cast<OcTreeNodeJointAngles*> (currNode->getChild(pos));
        assert(currNode);
    }

    return currNode;
}

OcTreeJointAngles::StaticMemberInitializer OcTreeJointAngles::ocTreeJointAnglesMemberInit;

} // end namespace
