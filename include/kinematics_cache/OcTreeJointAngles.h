#ifndef OCTOMAP_OCTREE_JOINT_ANGLES_H
#define OCTOMAP_OCTREE_JOINT_ANGLES_H

#include <octomap/OcTreeNode.h>
#include <octomap/OcTreeBase.h>
#include <boost/array.hpp>

namespace octomap
{

static const unsigned int MAX_SOLUTIONS = 100;
static const unsigned int NUM_JOINTS = 7;

typedef boost::array<float, NUM_JOINTS * MAX_SOLUTIONS + 1> angles_t;

// node definition
class OcTreeNodeJointAngles : public OcTreeDataNode<angles_t>
{

public:
    OcTreeNodeJointAngles() : OcTreeDataNode() {
        value.assign(0.0);
    }

    ~OcTreeNodeJointAngles() {}

    inline OcTreeNodeJointAngles* getChild(unsigned int i) {
      return static_cast<OcTreeNodeJointAngles*> (OcTreeDataNode<angles_t>::getChild(i));
    }

    inline const OcTreeNodeJointAngles* getChild(unsigned int i) const {
      return static_cast<const OcTreeNodeJointAngles*> (OcTreeDataNode<angles_t>::getChild(i));
    }

    bool createChild(unsigned int i);
};


// tree definition
class OcTreeJointAngles : public OcTreeBase<OcTreeNodeJointAngles>
{

public:
    /// Default constructor, sets resolution of leafs
    OcTreeJointAngles(double resolution);
    virtual OcTreeNodeJointAngles* updateNode(const point3d& value);
    OcTreeNodeJointAngles* updateNode(const OcTreeKey& k);
    OcTreeJointAngles* create() const {return new OcTreeJointAngles(resolution); }
    std::string getTreeType() const {return "OcTreeNodeJointAngles2";}
protected:
    /**
     * Static member object which ensures that this OcTree's prototype
     * ends up in the classIDMapping only once. You need this as a
     * static member in any derived octree class in order to read .ot
     * files through the AbstractOcTree factory. You should also call
     * ensureLinking() once from the constructor.
     */
    class StaticMemberInitializer
    {
    public:
        StaticMemberInitializer()
        {
            OcTreeJointAngles* tree = new OcTreeJointAngles(0.01);
            AbstractOcTree::registerTreeType(tree);
        }
    };
    /// to ensure static initialization (only once)
    static StaticMemberInitializer ocTreeJointAnglesMemberInit;

};

} // end namespace

#endif
