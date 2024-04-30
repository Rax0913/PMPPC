#ifndef SolverTest_HPP
#define SolverTest_HPP

#include<ros/ros.h>
#include <Eigen/Dense>
#include <string>
#include "ros/console.h"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Vector3.h>
#include <iostream>


using namespace Eigen;
class SolverTest{
public:
    SolverTest(int Ns){
        N = Ns;
        Pos<<0,0,0.0;
        Vel<<0,0,0;
        Acc<<0,0,0;
        Jerk<<0,0,0;
        odomp<<0,0,0;
        odomv<<0,0,0;
        odoma<<0,0,0;
        lastTime = ros::Time::now().toSec();
        };

    void initPos(double x, double y, double z)
    {
        Pos<<x,y,z;
        odomp<<x,y,z;
    }

    void paramInit(const ros::NodeHandle &nh_priv)
    {
        nh_priv.getParam("usePointModel",usePointModel);
        nh_priv.getParam("odomTopic",odomTopic);
        nh_priv.getParam("accmin",accmin);
        nh_priv.getParam("accmax",accmax);
        nh_priv.getParam("velmin",velmin);
        nh_priv.getParam("velmax",velmax);
        nh_priv.getParam("posmin",posmin);
        nh_priv.getParam("posmax",posmax);
        nh_priv.getParam("startPosition",startPosition);
        nh_priv.getParam("run_in_sim",run_in_sim);
    }


    void cmdCal()
    {
        if(run_in_sim)
            traj.push_back(Pos);
        else
            traj.push_back(odomp);
        
        quasim();
        visuslizeTraj(traj);
        visualizeSphere(); 
    }

    void updateOdom()
    {
        Vector3d Pos;
        for(int i=0; i<3; i++)
        {
            Pos(i) = odomp(i);
            Vel(i) = odomv(i);
            if(solverRet)
                lastPos(i) = odomp(i);
        }
    };

    void quasim()
    {
        if(initSet)
        {
            lastTime = ros::Time::now().toSec();
            initSet = false;
        }
        double Ts = ros::Time::now().toSec() - lastTime;
        for(int i=0; i<3; i++)
        {
            Pos(i) = Pos(i) + Vel(i)*Ts + 0.5*Acc(i)*Ts*Ts + Jerk(i)*Ts*Ts*Ts/6;
            Vel(i) = Vel(i) + Acc(i)*Ts + 0.5*Jerk(i)*Ts*Ts;
            Acc(i) = Acc(i) + Jerk(i)*Ts;
        }

        quadrotor_msgs::PositionCommand cmd;

        cmd.header.stamp = ros::Time::now();

        cmd.position.x = Pos(0);
        cmd.position.y = Pos(1);
        cmd.position.z = Pos(2);

        cmd.velocity.x = Vel(0);
        cmd.velocity.y = Vel(1);
        cmd.velocity.z = Vel(2);

        cmd.acceleration.x = Acc(0);
        cmd.acceleration.y = Acc(1);
        cmd.acceleration.z = Acc(2);

        cmd.jerk.x = Jerk(0);
        cmd.jerk.y = Jerk(1);
        cmd.jerk.z = Jerk(2);


        cmd.yaw = 0;

        cmd.yaw_dot = 0;

        cmd.kx = {0.0,0.0,0.0};
        cmd.kv = {0.0,0.0,0.0};
        cmd.trajectory_id = 0;
        cmd.trajectory_flag = 0;

        cmdPub.publish(cmd);

        lastTime = ros::Time::now().toSec();
    };

    void pubstate()
    {
        quadrotor_msgs::PositionCommand cmd;

        cmd.header.stamp = ros::Time::now();

        if(usePointModel)
        {
            cmd.position.x = Pos(0);
            cmd.position.y = Pos(1);
            cmd.position.z = Pos(2);

            cmd.velocity.x = Vel(0);
            cmd.velocity.y = Vel(1);
            cmd.velocity.z = Vel(2);
            // std::cout<<"state odom0: "<<cmd.position.x<<" "<<cmd.position.y<<" "<<cmd.position.z<<std::endl;
        }else{
            cmd.position.x = odomp(0);
            cmd.position.y = odomp(1);
            cmd.position.z = odomp(2);

            cmd.velocity.x = odomv(0);
            cmd.velocity.y = odomv(1);
            cmd.velocity.z = odomv(2);
        }



        cmd.acceleration.x = Acc(0);
        cmd.acceleration.y = Acc(1);
        cmd.acceleration.z = Acc(2);

        cmd.jerk.x = Jerk(0);
        cmd.jerk.y = Jerk(1);
        cmd.jerk.z = Jerk(2);


        cmd.yaw = 0;

        cmd.yaw_dot = 0;

        cmd.kx = {0.0,0.0,0.0};
        cmd.kv = {0.0,0.0,0.0};
        cmd.trajectory_id = 0;
        cmd.trajectory_flag = 0;

        

        statePub.publish(cmd);                             
    }

    void visuslizeTraj(const std::vector<Eigen::Vector3d> &route)
    {
        visualization_msgs::Marker routeMarker;

        routeMarker.id = 0;
        routeMarker.type = visualization_msgs::Marker::LINE_LIST;
        routeMarker.header.stamp = ros::Time::now();
        routeMarker.header.frame_id = "odom";
        routeMarker.pose.orientation.w = 1.00;
        routeMarker.action = visualization_msgs::Marker::ADD;
        routeMarker.ns = "route";
        routeMarker.color.r = 1.00;
        routeMarker.color.g = 0.00;
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

        trajPub.publish(routeMarker);
    };

    void imuFeed(sensor_msgs::ImuConstPtr pMsg)
    {
        sensor_msgs::Imu msg;
        msg = *pMsg;

        Eigen::Quaterniond q1{msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z};
    
        Eigen::Vector3d v1{msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z};

        Eigen::Vector3d v2 = q1*v1;

        odoma(0) = v2(0);
        odoma(1) = v2(1);
        odoma(2) = v2(2)-9.81;

    }

    void odomFeed(nav_msgs::OdometryConstPtr pMsg)
    {
        nav_msgs::Odometry msg;
        msg = *pMsg;

        odomp(0) = msg.pose.pose.position.x;
        odomp(1) = msg.pose.pose.position.y;
        odomp(2) = msg.pose.pose.position.z;

        odomv(0) = msg.twist.twist.linear.x;
        odomv(1) = msg.twist.twist.linear.y;
        odomv(2) = msg.twist.twist.linear.z;

        // updateOdom();


    }

    void retFeed(std_msgs::BoolConstPtr pMsg){
        std_msgs::Bool msg;
        msg = *pMsg;
        startControl = msg.data;
    }

    void jerkFeed(quadrotor_msgs::PositionCommandConstPtr pMsg)
    {
        startControl = true;
        quadrotor_msgs::PositionCommand msg;
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

        Jerk(0) = msg.jerk.x;
        Jerk(1) = msg.jerk.y;
        Jerk(2) = msg.jerk.z;
        // updateOdom();

    }

    void visualizeSphere()
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
        sphereMarkers.color.g = 0.00;
        sphereMarkers.color.b = 1.00;
        sphereMarkers.color.a = 1.00;
        sphereMarkers.scale.x = 0.5;
        sphereMarkers.scale.y = 0.5;
        sphereMarkers.scale.z = 0.5;

        sphereDeleter = sphereMarkers;
        sphereDeleter.action = visualization_msgs::Marker::DELETE;

        geometry_msgs::Point point;
        point.x = Pos(0);
        point.y = Pos(1);
        point.z = Pos(2);
        sphereMarkers.points.push_back(point);

        spherePub.publish(sphereDeleter);
        spherePub.publish(sphereMarkers);
    };

    ros::Publisher trajectory_pub;
    ros::Publisher setpoint_pub;
    ros::Publisher position_pub;
    ros::Publisher routePub;
    ros::Publisher spherePub;
    ros::Publisher pointsPub;
    ros::Publisher trajPub;
    ros::Publisher cmdPub;
    ros::Publisher statePub;

    ros::Subscriber odomSub;
    ros::Subscriber imuSub;
    ros::Subscriber jerkSub;
    ros::Subscriber retSub;


    double lastTime;

    ros::Timer timer1;
    bool initSet = true;
    bool solverRet = true;
    bool usePointModel;
    bool startControl = false;
    bool errorHappend = false;
    bool run_in_sim;
    bool startPub=false;
    int N;

    std::string odomTopic;

    Vector3d Pos;
    Vector3d Vel;
    Vector3d Acc;
    Vector3d Jerk;
    Vector3d Snap;
    Vector3d odomp;
    Vector3d odomv;
    Vector3d odoma;
    Vector3d lastPos;
    std::vector<double> accmin;
    std::vector<double> accmax;
    std::vector<double> velmin;
    std::vector<double> velmax;
    std::vector<double> posmin;
    std::vector<double> posmax;
    std::vector<double> startPosition;

    std::vector<Eigen::Vector3d> traj;
};



#endif