#include "misc/visualizer.hpp"
#include "gcopter/trajectory.hpp"
#include "gcopter/gcopter.hpp"
#include "gcopter/firi.hpp"
#include "gcopter/flatness.hpp"
#include "gcopter/voxel_map.hpp"
#include "gcopter/sfc_gen.hpp"

#include<pmppcSolver/pmppcSolver.hpp>
#include<pmppcSolver/SolverTest.hpp>
#include<pmppcSolver/GlobalPlannerUtils.hpp>
#include<ros/ros.h>
#include "ros/console.h"
#include "boost/bind.hpp"
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>


#include <cmath>
#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <chrono>
#include <random>
#include <queue>





int main(int argc, char **argv)
{
    ros::init(argc, argv, "pmppcCtrl");
    ros::NodeHandle nh;

    // ros::Duration(1.0).sleep();

    int N_step = 20;
    double Ts  = 0.2;
    int iter_num = 0;
    int maxhpoly=0;
    int rate_idx = 0;
    bool firstInit = true;
    bool initAcc=false;
    bool runRet = false;
    double global_start_time = ros::Time::now().toSec();
    double last_time;

    PmppcPlanner pmppcPlanner(N_step, Ts);
    GlobalPlanner globalPlanner(nh,false);
    

    ros::Duration(1.0).sleep();
    pmppcPlanner.globalParamInit(ros::NodeHandle("~"));


    MatrixXd qpSloution;
    MatrixXd s_hat_init;
    Vector3d posTraj;
    Vector3d jerk;
    Vector3d jerk_temp;
    Vector3d vel_temp;
    Vector3d acc_temp;
    Vector3d pos_temp;
    Vector3d pos_k;

    Vector3d pre_pos;
    Vector3d pre_vel;
    Vector3d pre_acc;
    Vector3d pre_jerk;
    Vector3d last_jerk;

    std::vector<Eigen::Vector3d> route;
    std::vector<Eigen::MatrixX4d> hPolys;

    pos_k<<0,0,0;

    qpSloution = MatrixXd::Zero(N_step*4,1);
    s_hat_init = MatrixXd::Ones(N_step,1);


    globalPlanner.trajectory_pub = nh.advertise<nav_msgs::Path>("/pmppc/trajectory", 10);                    //predict trajectory
    globalPlanner.routePub = nh.advertise<visualization_msgs::Marker>("/pmppc/visualizer/route", 10);         //path
    globalPlanner.pointsPub = nh.advertise<visualization_msgs::Marker>("/pmppc/visualizer/pointsPub", 1000);  //predict points
    globalPlanner.jerkPub = nh.advertise<quadrotor_msgs::PositionCommand>("/pmppc/jerk", 1000);  //predict points
    ros::Publisher CmdPub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);


    globalPlanner.stateSub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("/pmppc/state", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       1,
                                       boost::bind(&GlobalPlanner::stateFeed, &globalPlanner, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    globalPlanner.mapSub = nh.subscribe<sensor_msgs::PointCloud2>("/voxel_map", 
                                        1, 
                                        boost::bind(&GlobalPlanner::mapCallBack, &globalPlanner, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    globalPlanner.odomSub = nh.subscribe<nav_msgs::Odometry>(pmppcPlanner.odomTopic, 
                                        1, 
                                        boost::bind(&GlobalPlanner::odomCallback, &globalPlanner, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    
    if(!pmppcPlanner.SolverInit(nh))
        cout<<"ERROR: Init Failed!!!"<<endl;
    else
        cout<<"Init Successfully!!"<<endl;
    for(int i=0; i<pmppcPlanner.waypoints.size();i++)
    {
        Vector3d pt;
        pmppcPlanner.waypoints.PointQuery(i,pt);
        route.push_back(pt);
    }

    ros::Rate rate(200);

    if(pmppcPlanner.usePCD){
         // globalPlanner.testMap();
        cout<<"load pcd map"<<endl;
        globalPlanner.loadPCD("/home/xzx/pmppc_nuc/src/pmppc_planner/include/map_gen/PCD/scans_3_2_planning_small.pcd");
    
    }else
    {
        while((!globalPlanner.mapInitialized) && ros::ok())
        {
            // cout<<"map init"<<endl;
            ros::spinOnce();
            rate.sleep();
        }
    }


     cout<<globalPlanner.Pos(0)<<" "<<pmppcPlanner.startPosition[0]<<
    " "<<globalPlanner.Pos(1)<<" "<<pmppcPlanner.startPosition[1]
    <<" "<<globalPlanner.Pos(2)<<" "<<pmppcPlanner.startPosition[2]<<endl;
    if(!pmppcPlanner.run_in_sim)
    {
        while(!(abs(globalPlanner.odomPos(0)-pmppcPlanner.startPosition[0])<0.1 &&
        abs(globalPlanner.odomPos(1)-pmppcPlanner.startPosition[1])<0.1 &&
        abs(globalPlanner.odomPos(2)-pmppcPlanner.startPosition[2])<0.05) && ros::ok()){
            quadrotor_msgs::PositionCommand cmd;

            cmd.header.stamp = ros::Time::now();

            cmd.position.x = pmppcPlanner.startPosition[0];
            cmd.position.y = pmppcPlanner.startPosition[1];
            cmd.position.z = pmppcPlanner.startPosition[2];

            cmd.velocity.x = 0.0;
            cmd.velocity.y = 0.0;
            cmd.velocity.z = 0.0;

            cmd.acceleration.x = 0.0;
            cmd.acceleration.y = 0.0;
            cmd.acceleration.z = 0.0;

            cmd.jerk.x = 0.0;
            cmd.jerk.y = 0.0;
            cmd.jerk.z = 0.0;


            cmd.yaw = 0;

            cmd.yaw_dot = 0;

            cmd.kx = {0.0,0.0,0.0};
            cmd.kv = {0.0,0.0,0.0};
            cmd.trajectory_id = 0;
            cmd.trajectory_flag = 0;

            CmdPub.publish(cmd);
            ros::spinOnce();
            rate.sleep();

        }
    }
    // solverTest.updateOdom();

    
    ros::Duration(2.0).sleep();
    if(firstInit && ros::ok())
    {
        cout<<"||||||||||||||||  Start ||||||||||||||||"<<endl;
        while(iter_num<10  && (s_hat_init - pmppcPlanner.s_hat).norm()>0.001)
        {    
    
    
            pmppcPlanner.x_k<<globalPlanner.Pos(0), globalPlanner.Pos(1),globalPlanner.Pos(2),
            globalPlanner.Vel(0),globalPlanner.Vel(1),globalPlanner.Vel(2),
            globalPlanner.Acc(0),globalPlanner.Acc(1),globalPlanner.Acc(2),pmppcPlanner.thk;

            s_hat_init = pmppcPlanner.s_hat;

            std::vector<Eigen::MatrixX4d> hPolys;
            globalPlanner.hPolysSet(pmppcPlanner.pointsRoute, hPolys, globalPlanner.Pos);
            if(!pmppcPlanner.QPSolver(hPolys))
                cout<<"ERROR: Solve Failed!!!"<<endl;

            
            iter_num ++;
            cout<<"Debug: "<<iter_num<<endl    
    
    ;
            globalPlanner.pcd_pub.publish(globalPlanner.outputView);
            globalPlanner.pcd_view_pub.publish(globalPlanner.outputView);
        }
        firstInit = false;
    }
    iter_num = 0;

    cout<<"|||||||||||||||"<<(s_hat_init - pmppcPlanner.s_hat).norm()<<"  |||"<<endl;
    last_time = ros::Time::now().toSec()-Ts;
    while(ros::ok())
    {
        if(pmppcPlanner.targetPoints.size()!=0)
        {
            int iterIdx=0;
            std::vector<Eigen::Vector3d> tempRoute;
            Vector3d sPos,ePos,oPos;
            while(pmppcPlanner.targetPoints.size()!=0)
            {
                tempRoute.clear();
                if(iterIdx == 0)
                    pmppcPlanner.waypoints.PointQuery(pmppcPlanner.waypoints.size()-1,sPos);
                else
                    sPos = oPos;
                ePos = pmppcPlanner.targetPoints.front();
                globalPlanner.pathPlanning(tempRoute, sPos, ePos);
                for(int i=1; i<tempRoute.size(); i++)
                {
                    pmppcPlanner.waypoints.push(tempRoute[i]);
                }
                iterIdx++;
                oPos = ePos;
                pmppcPlanner.targetPoints.pop();
            }

        }
        if(ros::Time::now().toSec()-last_time >Ts-0.0025)
        {
            globalPlanner.next_pos = globalPlanner.Pos;
            globalPlanner.next_vel = globalPlanner.Vel;
            globalPlanner.next_acc = globalPlanner.Acc;
            globalPlanner.pubJerk();

            runRet = false;
            last_jerk = globalPlanner.Jerk;
            for(int k=0; k<3; k++)
            {
                pre_pos(k) = globalPlanner.Pos(k) + globalPlanner.Vel(k)*Ts + globalPlanner.Acc(k)*Ts*Ts/2 + last_jerk(k)*Ts*Ts*Ts/6;
                pre_vel(k) = globalPlanner.Vel(k) + globalPlanner.Acc(k)*Ts + last_jerk(k)*Ts*Ts/2;
                pre_acc(k) = globalPlanner.Acc(k) + last_jerk(k)*Ts;
            }
            nav_msgs::Path path;
            geometry_msgs::PoseStamped pose;
            path.header.stamp = ros::Time::now();
            path.header.frame_id = "odom";
            pos_temp = globalPlanner.Pos;
            vel_temp = globalPlanner.Vel;
            acc_temp = globalPlanner.Acc;
            for(int i=0; i<N_step; i++)
            {
                double t=Ts;
                for(int j=0; j<3; j++)
                {
                    pos_temp(j) = pos_temp(j) + vel_temp(j)*t +acc_temp(j)*t*t/2 + pmppcPlanner.QPSolution(4*i+j)*t*t*t/6;
                    vel_temp(j) = vel_temp(j) +acc_temp(j)*t + pmppcPlanner.QPSolution(4*i+j)*t*t/2;
                    acc_temp(j) = acc_temp(j) + pmppcPlanner.QPSolution(4*i+j)*t;
                }
            
                pose.pose.position.x = pos_temp(0);
                pose.pose.position.y = pos_temp(1);
                pose.pose.position.z = pos_temp(2);

                pose.pose.orientation.x = 0;
                pose.pose.orientation.y = 0;
                pose.pose.orientation.z = 0;
                pose.pose.orientation.w = 1;

                path.poses.push_back(pose);
                last_time = ros::Time::now().toSec();
            }

            globalPlanner.trajectory_pub.publish(path);

            
        }
        if((ros::Time::now().toSec()-last_time)>0.05 && (!runRet))
        {
            runRet = true;
            if(pmppcPlanner.getTarget)
            {
                cout<<"||||||||||||||||  Start ||||||||||||||||"<<endl;
                std::vector<Eigen::MatrixX4d> hPolys;
                globalPlanner.hPolysSet(pmppcPlanner.pointsRoute, hPolys, globalPlanner.Pos);
                while(iter_num<20 && (s_hat_init - pmppcPlanner.s_hat).norm()>0.001)
                {

                    pmppcPlanner.x_k<<pre_pos(0), pre_pos(1),pre_pos(2),
                        pre_vel(0),pre_vel(1),pre_vel(2),
                        pre_acc(0),pre_acc(1),pre_acc(2),pmppcPlanner.thk;
                    // cout<<"p: "<<pre_pos.transpose()<<" v: "<<pre_vel.transpose()
                    // <<" a: "<<pre_acc.transpose()<<endl;
                    s_hat_init = pmppcPlanner.s_hat;
                    if(!pmppcPlanner.QPSolver(hPolys)){
                        cout<<"ERROR: Solve Failed!!!"<<endl;
                        // if(pmppcPlanner.startPlanning)
                        //     solverTest.solverRet = false;
                    }
                    globalPlanner.Jerk<<pmppcPlanner.QPSolution(0),pmppcPlanner.QPSolution(1),pmppcPlanner.QPSolution(2);
                    
                    iter_num ++;
                }
                pmppcPlanner.getTarget = false;
            }else{
                std::vector<Eigen::MatrixX4d> hPolys;
                globalPlanner.hPolysSet(pmppcPlanner.pointsRoute, hPolys, globalPlanner.Pos);

                while(iter_num<1 ){
                    pmppcPlanner.x_k<<pre_pos(0), pre_pos(1),pre_pos(2),
                        pre_vel(0),pre_vel(1),pre_vel(2),
                        pre_acc(0),pre_acc(1),pre_acc(2),pmppcPlanner.thk;
                    double start_planning_time = ros::Time::now().toSec();
                    if(!pmppcPlanner.QPSolver(hPolys)){
                        cout<<"ERROR: Solve Failed!!!"<<endl;
                    }
                    
                    globalPlanner.Jerk<<pmppcPlanner.QPSolution(0),pmppcPlanner.QPSolution(1),pmppcPlanner.QPSolution(2);
                        
                    iter_num++;
                };
            }

            
            globalPlanner.visualizeSpheres(pmppcPlanner.pointsRoute);

            std::vector<Eigen::Vector3d> route;
            for(int i=0; i<pmppcPlanner.waypoints.size();i++)
            {
                Vector3d pt;
                pmppcPlanner.waypoints.PointQuery(i,pt);
                route.push_back(pt);
            };
            globalPlanner.visuslizeRoute(route);
        }
        rate_idx++;

        
        iter_num = 0;
        
        ros::spinOnce();
        rate.sleep();
    }
    
    // cout<<"maxHpolys: "<<maxhpoly<<endl;
    cout<<"Solver Time: "<<pmppcPlanner.allSolverTime<<" N: "
    <<pmppcPlanner.allSolverN<<" average: "<<pmppcPlanner.allSolverTime/pmppcPlanner.allSolverN<<endl;


    return 0;
}
