#include <ros/ros.h>
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>

int main(int argc, char** argv)
{

    ros::init(argc, argv, "localmap");
    ros::NodeHandle nh;
    ros::Publisher cloud_pub=nh.advertise<sensor_msgs::PointCloud2>("/voxel_map",1);

    pcl::PCLPointCloud2::Ptr cloud(new pcl::PCLPointCloud2);
    pcl::PCLPointCloud2::Ptr cloud_filterd(new pcl::PCLPointCloud2);

    sensor_msgs::PointCloud2 output;

    pcl::PCDReader reader;
    reader.read("/home/xzx/pmppc_NoCollision_qpOASE/src/pmppc_planner/include/map_gen/PCD/scans_test.pcd",*cloud);

    std::cout << "过滤前的点个数：" << cloud->width * cloud->height << std::endl;
    std::cout << "数据类型：" << pcl::getFieldsList(*cloud)<< std::endl;

    float leafSize = 0.1f;
    // 执行降采样------------------------
    pcl::VoxelGrid<pcl::PCLPointCloud2> voxel;
    // 设置输入点云
    voxel.setInputCloud(cloud);
    // 设置体素网格的大小
    voxel.setLeafSize(leafSize, leafSize, leafSize);
    // 执行滤波, 并将结果保存到cloud_filterd
    voxel.filter(*cloud_filterd);

    std::cout << "---------------------------------------------" << std::endl;
    std::cout << "过滤后的点个数：" << cloud_filterd->width * cloud_filterd->height << std::endl;
    std::cout << "数据类型：" << pcl::getFieldsList(*cloud_filterd)<< std::endl;

    pcl_conversions::moveFromPCL(*cloud_filterd,output);

    //设置输出点云的坐标系
    output.header.frame_id = std::string("odom");

    ros::Rate loop_rate(1);

    int num = 0;
    bool pub_ret = true;

    while(ros::ok() && pub_ret) {

        if(num>10)
            break;
        cloud_pub.publish(output);

        num++;

        ros::spinOnce();

        loop_rate.sleep();

    }

}