#include <ros/ros.h>

//fcl
#include <fcl/fcl.h>

//msg
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>

using namespace fcl;

std::shared_ptr<CollisionGeometryd> Quadcopter; //无人机碰撞模型
std::shared_ptr<CollisionGeometryd> tree_obj;   //地图碰撞模型

//初始化
void init(double uavl, double uavw, double uavh)
{
    Quadcopter = std::shared_ptr<CollisionGeometryd>(new fcl::Boxd(uavl, uavw, uavh));
}

//碰撞检测
bool isCollisionFunction()
{
    CollisionObjectd treeObj((tree_obj));
    CollisionObjectd aircraftObject(Quadcopter);

    // 检测当前位置是否会碰撞
    // Vector3d translation(0, 0, 0);
    // Quaterniond rotation(0, 0, 0, 0);
    // aircraftObject.setTransform(rotation, translation);
    CollisionRequestd requestType(1, false, 1, false);
    CollisionResultd collisionResult;
    collide(&aircraftObject, &treeObj, requestType, collisionResult);

    return collisionResult.isCollision();
}

//更新地图
void octomapCallback(const octomap_msgs::Octomap::ConstPtr &msg)
{
    octomap::OcTree *tree_oct = dynamic_cast<octomap::OcTree *>(octomap_msgs::msgToMap(*msg));
    fcl::OcTreed *tree = new fcl::OcTreed(std::shared_ptr<const octomap::OcTree>(tree_oct));

    // 更新地图碰撞信息
    tree_obj = std::shared_ptr<fcl::CollisionGeometryd>(tree);

    bool ans = isCollisionFunction();

    if(ans){
        ROS_ERROR("collision happen!");
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "collision_detection");
    ros::NodeHandle n;

    // 定义参数
    double uavl;
    double uavw;
    double uavh;
    std::string sub_topic;
    std::string pub_topic;

    // 获取参数
    n.param("collision_detection/uavl", uavl, 0.8);
    n.param("collision_detection/uavw", uavw, 0.8);
    n.param("collision_detection/uavh", uavh, 0.3);
    n.param("collision_detection/sub_topic", sub_topic, std::string("/octomap_full"));
    n.param("collision_detection/pub_topic", pub_topic, std::string("/is_collision"));

    init(uavl, uavw, uavh);

    //关注主题
    ros::Subscriber octree_sub = n.subscribe<octomap_msgs::Octomap>(sub_topic, 1, octomapCallback);

    ros::spin();

    return 0;
}