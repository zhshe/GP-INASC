#include <ros/ros.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <chrono>


int main(int argc, char** argv) {
    
    ros::init(argc, argv, "gp_insac_test");
    ros::NodeHandle node("~");
    
    // SegNode seg_node(node);
    // ROS_INFO("------TEST-----------");
    // std::string cloud_path = "/home/zs/zs/kitti/data/cloud_origin.ply";
    // std::string cloud_save = "/home/zs/zs/kitti/data/cloud_origin_";
    // ROS_INFO("cloud_path: %s", cloud_path.c_str());
    // ROS_INFO("cloud_path: %s", cloud_save.c_str());
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ground(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr obstacle(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::io::loadPLYFile(cloud_path, *cloud);
    // auto start = std::chrono::steady_clock::now();
    // seg_node.groundSeg(cloud, *obstacle, *ground);
    // auto end = std::chrono::steady_clock::now();
    // std::chrono::duration<double, std::milli> dur = end - start;
    // ROS_INFO("run time: %f ms", dur.count());
    // pcl::io::savePLYFile(cloud_save + "ground.ply", *ground);
    // pcl::io::savePLYFile(cloud_save + "obstalce.ply", *obstacle);
    ros::shutdown();
    return 0;
}