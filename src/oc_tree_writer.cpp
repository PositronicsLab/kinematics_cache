#include <ros/ros.h>
#include <kinematics_cache/IK.h>
#include <kinematics_cache/IKQuery.h>
#include <mongodb_store/message_store.h>
#include <kinematics_cache/OcTreeJointAngles.h>
#include <map>

namespace std
{
    template<> struct less<octomap::point3d>
    {
       bool operator() (const octomap::point3d& lhs, const octomap::point3d& rhs) const
       {
        return lhs.x() < rhs.x() || lhs.y() < rhs.y() || lhs.z() < rhs.z();
       }
    };
}

namespace
{

using namespace std;
using namespace mongodb_store;
using namespace mongo;
using namespace octomap;

typedef vector<boost::shared_ptr<kinematics_cache::IK> > IKList;
typedef vector< std::pair<boost::shared_ptr<kinematics_cache::IK>, mongo::BSONObj> > IKResultsList;

static const double RESOLUTION_DEFAULT = 0.02;

class OcTreeWriter
{
private:

    //! Node handle
    ros::NodeHandle nh;

    //! Private nh
    ros::NodeHandle pnh;

    //! Database
    MessageStoreProxy mdb;

    //! Arm
    string armName;

    //! Output file
    string output;

    //! Resolution
    double resolution;

public:
    OcTreeWriter() :
        pnh("~"), mdb(nh)
    {
        pnh.param<string>("arm", armName, "left_arm");
        pnh.param<string>("output", output, "arm.oct");
        pnh.param<double>("resolution", resolution, RESOLUTION_DEFAULT);
    }

private:

public:
    void update()
    {
        ROS_INFO("Converting joint angles to OcTree for %s", armName.c_str());

        ROS_INFO("Querying for all records");
        BSONObjBuilder b;
        b.append("group", armName);

        mongo::BSONObj query = b.obj();
        mongo::BSONObj metaDataQuery;

        ROS_DEBUG_STREAM("Executing query: " << query);
        IKResultsList results;
        if (mdb.query<kinematics_cache::IK>(results, query, metaDataQuery, false))
        {
            ROS_INFO("Query for all entries succeeded");
        }
        else
        {
            ROS_ERROR("Query for all entries failed");
            return;
        }

        // Construct the octree
        OcTreeJointAngles tree(resolution);

        // Write all records
        for (IKResultsList::iterator iter = results.begin(); iter != results.end(); ++iter)
        {
            ROS_DEBUG("Writing record for position: %f %f %f", iter->first->pose.pose.position.x,
                     iter->first->pose.pose.position.y, iter->first->pose.pose.position.z);
            octomap::point3d point((float) iter->first->pose.pose.position.x,
                                   (float) iter->first->pose.pose.position.y,
                                   (float) iter->first->pose.pose.position.z);

            octomap::angles_t angles;
            assert(iter->first->positions.size() == 7);

            OcTreeNodeJointAngles* node = tree.updateNode(point);

            // Angles stores up to 10 records.
            angles_t value = node->getValue();
            if (value[0] == octomap::MAX_SOLUTIONS) {
                ROS_WARN("Array is full");
            }
            else {
                for (unsigned int i = 0; i < iter->first->positions.size(); ++i)
                {
                    int idx = value[0] * octomap::NUM_JOINTS + 1 + i;
                    value[idx] = (float) iter->first->positions[i];
                }
                value[0]++;
                node->setValue(value);
            }
        }

        ROS_INFO_STREAM("Tree size: " << tree.getNumLeafNodes());

        // Write tree to file
        ROS_INFO("Completed updating entries. Writing tree");
        tree.write(output);

        ROS_INFO("Output complete");
    }
};
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "oc_tree_writer");
    OcTreeWriter otw;
    otw.update();
}
