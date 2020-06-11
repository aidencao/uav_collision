#include <ros/ros.h>

//pcl
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

//octomap
#include <octomap/octomap.h>
#include <octomap/ColorOcTree.h>

//msg
#include <sensor_msgs/PointCloud2.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace std;

// 定义参数
double octo_resolution;
double bound_xy;
double bound_lowz;
double bound_highz;
string sub_topic;
string pub_topic;

//发布者
ros::Publisher octo_Pub;

void pointToOcto(const sensor_msgs::PointCloud2::ConstPtr &msg)
{
    //获取点云数据
    pcl::PointCloud<pcl::PointXYZ> pc;
    pcl::fromROSMsg(*msg, pc);

    //建立过滤
    pcl::PassThrough<pcl::PointXYZ> pass_x;
    pass_x.setFilterFieldName("x");
    pass_x.setFilterLimits(-bound_xy, bound_xy);
    pcl::PassThrough<pcl::PointXYZ> pass_y;
    pass_y.setFilterFieldName("y");
    pass_y.setFilterLimits(-bound_xy, bound_xy);
    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(bound_lowz, bound_highz);

    //完成点云过滤
    pass_x.setInputCloud(pc.makeShared());
    pass_x.filter(pc);
    pass_y.setInputCloud(pc.makeShared());
    pass_y.filter(pc);
    pass_z.setInputCloud(pc.makeShared());
    pass_z.filter(pc);

    //创建octotree对象
    octomap::OcTree tree(octo_resolution);

    for (auto p : pc.points)
    {
        // 将点云里的点插入到octomap中
        tree.updateNode(octomap::point3d(p.x, p.y, p.z), true);
    }

    // 更新octomap
    tree.updateInnerOccupancy();

    //发布数据
    octomap_msgs::Octomap map;
    map.header.frame_id = msg->header.frame_id;
    map.header.stamp = ros::Time::now();
    if (octomap_msgs::fullMapToMsg(tree, map))
        octo_Pub.publish(map);
    else
        ROS_ERROR("Error serializing OctoMap");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "trans_pointcloud");
    ros::NodeHandle n;

    // 获取参数
    n.param("trans_pointcloud/octo_resolution", octo_resolution, 0.5);
    n.param("trans_pointcloud/bound_xy", bound_xy, 8000.0);
    n.param("trans_pointcloud/bound_lowz", bound_lowz, 0.0);
    n.param("trans_pointcloud/bound_highz", bound_highz, 100.0);
    n.param("trans_pointcloud/sub_topic", sub_topic, std::string("/points_raw"));
    n.param("trans_pointcloud/pub_topic", pub_topic, std::string("/octomap_full"));

    ros::Subscriber point_sub = n.subscribe<sensor_msgs::PointCloud2>(sub_topic, 1, pointToOcto);

    octo_Pub = n.advertise<octomap_msgs::Octomap>(pub_topic, 1);

    ros::spin();

    return 0;
}