#ifndef GlobalPlannerUtils_HPP
#define GlobalPlannerUtils_HPP

#include<iostream>
#include <Eigen/Dense>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <quadrotor_msgs/PositionCommand.h>

#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/cloud_viewer.h>

#include <visualization_msgs/Marker.h>


class GlobalPlanner{
public:
    ros::NodeHandle nh;
    ros::Subscriber mapSub;
    ros::Subscriber stateSub;
    ros::Subscriber odomSub;
    ros::Publisher pcd_pub;
    ros::Publisher pcd_view_pub;
    ros::Publisher routePub;
    ros::Publisher trajectory_pub;
    ros::Publisher pointsPub;
    ros::Publisher jerkPub;

    bool mapInitialized;
    bool debug;
    int idx = 0;
    double startTime;
    Vector3d odomPos;
    Vector3d Pos;
    Vector3d Vel;
    Vector3d Acc;
    Vector3d next_pos;
    Vector3d next_vel;
    Vector3d next_acc;
    Vector3d Jerk;

    voxel_map::VoxelMap voxelMap;
    sensor_msgs::PointCloud2 output;
    sensor_msgs::PointCloud2 outputView;
    Visualizer visualizer;


    GlobalPlanner(ros::NodeHandle &nh_,bool debug_)
    :visualizer(nh_)
    {
        nh = nh_;
        mapInitialized = false;
        debug = debug_;
        startTime = ros::Time::now().toSec();

        Pos<<0,0,0.0;
        Vel<<0,0,0;
        Acc<<0,0,0;
        next_pos<<0,0,0;
        next_vel<<0,0,0;
        next_acc<<0,0,0;
        Jerk<<0,0,0;

        std::string mapTopic = "/voxel_map";

        const Eigen::Vector3d xyzub(50.0,50.0,5);

        const Eigen::Vector3i xyz(xyzub(0) / 0.25,
                                  xyzub(1) / 0.25,
                                  xyzub(2) / 0.25);
        
        const Eigen::Vector3d offset(-25, -25, 0);

        voxelMap = voxel_map::VoxelMap(xyz, offset, 0.25);


        pcd_pub=nh.advertise<sensor_msgs::PointCloud2>("/voxel_map",10);
        pcd_view_pub=nh.advertise<sensor_msgs::PointCloud2>("/voxel_map_view",10);
    };

    void testMap()
    {
        voxelMap.setOccupied(Eigen::Vector3d(0.1,0.15,-0.1));
        voxelMap.setOccupied(Eigen::Vector3d(0.1,0.2,0.1));
        voxelMap.setOccupied(Eigen::Vector3d(0.4,0.2,0.1));
        voxelMap.setOccupied(Eigen::Vector3d(0.1,0.6,0.1));
        voxelMap.setOccupied(Eigen::Vector3d(0.7,0.6,0.3));
        voxelMap.dilate(std::ceil(0.2 / voxelMap.getScale()));
        for(int i=0; i<5; i++)
        {
            for(int j=0; j<5; j++)
                for(int k=0; k<2; k++)
                    cout<<" "<<voxelMap.query(Eigen::Vector3i(i,j,k));
            cout<<" "<<endl;
        }
        
    }

    void viewCloud()
    {
        Vector3i mapsize = voxelMap.getSize();
        pcl::PointCloud<pcl::PointXYZ> cloudView;
        int idx=0;

        for(int i=0; i<mapsize(0); i++)
            for(int j=0; j<mapsize(1); j++)
                for(int k=0; k<mapsize(2); k++)
                {
                    if(voxelMap.query(Eigen::Vector3i(i,j,k))==0)
                        continue;
                    else{
                        pcl::PointXYZ point;
                        Vector3d cp = voxelMap.posI2D(Eigen::Vector3i(i,j,k));

                        point.x = cp(0);
                        point.y = cp(1);
                        point.z = cp(2);
                        cloudView.points.push_back(point);
                    } 
                }

        pcl::toROSMsg(cloudView,outputView);
        outputView.header.frame_id = std::string("odom");
    }

    void loadPCD(string filename)
    {

        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::io::loadPCDFile<pcl::PointXYZ>(filename, cloud);

	    pcl::toROSMsg(cloud,output);
        output.header.frame_id = std::string("odom");
        int idx=0;

        for (int i=0; i<cloud.size(); i++)
        {

            if (std::isnan(cloud.points[i].x) || std::isinf(cloud.points[i].x) ||
                std::isnan(cloud.points[i].y) || std::isinf(cloud.points[i].y) ||
                std::isnan(cloud.points[i].z) || std::isinf(cloud.points[i].z) )
            {
                continue;
            }
            voxelMap.setOccupied(Eigen::Vector3d(cloud.points[i].x,
                                                    cloud.points[i].y,
                                                    cloud.points[i].z));

        };

        //地图膨胀
        voxelMap.dilate(std::ceil(0.25 / voxelMap.getScale()));

        viewCloud();

        pcd_pub.publish(outputView);
        pcd_view_pub.publish(outputView);

        mapInitialized = true;

    };

    inline void mapCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg)
    {
        if (!mapInitialized)
        {
            size_t cur = 0;
            const size_t total = msg->data.size() / msg->point_step;
            float *fdata = (float *)(&msg->data[0]);
            cout<<"total: "<<total<<endl;
            for (size_t i = 0; i < total; i++)
            {
                cur = msg->point_step / sizeof(float) * i;

                if (std::isnan(fdata[cur + 0]) || std::isinf(fdata[cur + 0]) ||
                    std::isnan(fdata[cur + 1]) || std::isinf(fdata[cur + 1]) ||
                    std::isnan(fdata[cur + 2]) || std::isinf(fdata[cur + 2]))
                {
                    continue;
                }
                voxelMap.setOccupied(Eigen::Vector3d(fdata[cur + 0],
                                                     fdata[cur + 1],
                                                     fdata[cur + 2]));
            }

            //地图膨胀
            voxelMap.dilate(std::ceil(0.25 / voxelMap.getScale()));

            mapInitialized = true;
        }
    };

    void hPolySet(const Vector3d &nowPos, const Vector3d &secPos, Eigen::MatrixX4d &hpoly)
    {
        // std::vector<Eigen::MatrixX4d> hPolys;
        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);

        sfc_gen::convexCoverNowPos(nowPos,
                                 secPos,
                                 pc,
                                 voxelMap.getOrigin(),
                                 voxelMap.getCorner(),
                                 7.0,
                                 8.0,
                                 hpoly);
    };

    void hPolysSet(const std::vector<Eigen::Vector3d> &path, std::vector<Eigen::MatrixX4d> &hPolys, const Eigen::Vector3d nowPos)
    {
        std::vector<Eigen::Vector3d> pc;
        voxelMap.getSurf(pc);
        if(path.empty())
            return;
        for(int i=0; i<path.size();i=i+1)
        {
            Eigen::MatrixX4d hpoly;
            if(i==0)
                hPolySet(nowPos,path[i+2],hpoly);
            else if (i<path.size()-2)
            {
                hPolySet(path[i],path[i+1],hpoly);
            }else{
                hPolySet(path[i],path[i],hpoly);
            };
            
                
            hPolys.push_back(hpoly);
        }
        std::vector<Eigen::MatrixX4d> thpolys = hPolys;
        sfc_gen::shortCut(thpolys);
        visualizer.visualizePolytope(thpolys);
        // cout<<"hPolys Size: "<<hPolys.size()<<endl;
    };

    bool pathPlanning(std::vector<Eigen::Vector3d> &routes, const Eigen::Vector3d &startPos, const Eigen::Vector3d &endPos)
    {
        if(voxelMap.query(endPos) == 0)
        {
            sfc_gen::planPath<voxel_map::VoxelMap>(startPos,
                                                   endPos,
                                                   voxelMap.getOrigin(),
                                                   voxelMap.getCorner(),
                                                   &voxelMap, 0.01,
                                                   routes);

        }else{
            ROS_WARN("Infeasible Position Selected !!!\n");
            return false;
        }
        return true;
        
    };

    void visualizeSpheres(const std::vector<Eigen::Vector3d> &path)
    {
        visualization_msgs::Marker sphereMarkers, sphereDeleter;

        sphereMarkers.id = 0;
        sphereMarkers.type = visualization_msgs::Marker::SPHERE_LIST;
        sphereMarkers.header.stamp = ros::Time::now();
        sphereMarkers.header.frame_id = "odom";
        sphereMarkers.pose.orientation.w = 1.00;
        sphereMarkers.action = visualization_msgs::Marker::ADD;
        sphereMarkers.ns = "spheres";
        sphereMarkers.color.r = 0.00;
        sphereMarkers.color.g = 1.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = 0.5;
        sphereMarkers.scale.y = 0.5;
        sphereMarkers.scale.z = 0.5;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        for(int i=0; i<path.size(); i++)
        {
            geometry_msgs::Point point;
            Eigen::Vector3d temp = path[i];
            point.x = temp(0);
            point.y = temp(1);
            point.z = temp(2);
            sphereMarkers.points.push_back(point);
        }
        

        pointsPub.publish(sphereDeleter);
        pointsPub.publish(sphereMarkers);
    };

    void visuslizeRoute(const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::Marker routeMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "odom";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 0.00;
        routeMarker.color.g = 1.00;
        routeMarker.color.b = 0.00;
        routeMarker.color.a = 1.00;
        routeMarker.scale.x = 0.1;

        bool first = true;
        Eigen::Vector3d last;
        for (auto it : route)
        {
            if (first)
            {
                first = false;
                last = it;
                continue;
            }
            geometry_msgs::Point point;

            point.x = last(0);
            point.y = last(1);
            point.z = last(2);
            routeMarker.points.push_back(point);
            point.x = it(0);
            point.y = it(1);
            point.z = it(2);
            routeMarker.points.push_back(point);
            last = it;
        }

        routePub.publish(routeMarker);
    };
    
    void stateFeed(quadrotor_msgs::PositionCommandConstPtr pMsg)
    {
        quadrotor_msgs::PositionCommand msg;
        // idx++;
        // if(idx%100 == 0)
        //     cout<<ros::Time::now().toSec()-startTime<<" "<<idx<<endl;
        msg = *pMsg;

        Pos(0) = msg.position.x;
        Pos(1) = msg.position.y;
        Pos(2) = msg.position.z;

        Vel(0) = msg.velocity.x;
        Vel(1) = msg.velocity.y;
        Vel(2) = msg.velocity.z;

        Acc(0) = msg.acceleration.x;
        Acc(1) = msg.acceleration.y;
        Acc(2) = msg.acceleration.z;
    };

    void pubJerk()
    {
        quadrotor_msgs::PositionCommand cmd;

        cmd.header.stamp = ros::Time::now();

        cmd.position.x = next_pos(0);
        cmd.position.y = next_pos(1);
        cmd.position.z = next_pos(2);

        cmd.velocity.x = next_vel(0);
        cmd.velocity.y = next_vel(1);
        cmd.velocity.z = next_vel(2);

        cmd.acceleration.x = next_acc(0);
        cmd.acceleration.y = next_acc(1);
        cmd.acceleration.z = next_acc(2);

        cmd.jerk.x = Jerk(0);
        cmd.jerk.y = Jerk(1);
        cmd.jerk.z = Jerk(2);


        cmd.yaw = 0;

        cmd.yaw_dot = 0;

        cmd.kx = {0.0,0.0,0.0};
        cmd.kv = {0.0,0.0,0.0};
        cmd.trajectory_id = 0;
        cmd.trajectory_flag = 0;

        jerkPub.publish(cmd);
    };

    void odomCallback(nav_msgs::OdometryConstPtr pMsg)
    {
        nav_msgs::Odometry msg;
        msg = *pMsg;

        odomPos(0) = msg.pose.pose.position.x;
        odomPos(1) = msg.pose.pose.position.y;
        odomPos(2) = msg.pose.pose.position.z;

    }

};

#endif