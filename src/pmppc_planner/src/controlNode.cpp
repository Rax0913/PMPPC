#include<pmppcSolver/SolverTest.hpp>
#include<ros/ros.h>
#include "ros/console.h"
#include <string>

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "control_node");
    ros::NodeHandle nh;

    int N_step = 20;
    bool usepointmodel;
    SolverTest solverTest(N_step);

    solverTest.paramInit(ros::NodeHandle("~"));

    solverTest.trajPub = nh.advertise<visualization_msgs::Marker>("/pmppc/visualizer/trajectory", 10);     //trajectory
    solverTest.spherePub = nh.advertise<visualization_msgs::Marker>("/pmppc/visualizer/spheres", 1000);    // now point
    solverTest.statePub = nh.advertise<quadrotor_msgs::PositionCommand>("/pmppc/state", 10);
    solverTest.cmdPub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);
   
    solverTest.retSub = nh.subscribe<std_msgs::Bool>("/pmppc/startControl",
                                         100,
                                         boost::bind(&SolverTest::retFeed, &solverTest, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    solverTest.odomSub =
        nh.subscribe<nav_msgs::Odometry>(solverTest.odomTopic,
                                         100,
                                         boost::bind(&SolverTest::odomFeed, &solverTest, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());
    solverTest.imuSub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data",
                                       100,
                                       boost::bind(&SolverTest::imuFeed, &solverTest, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    solverTest.jerkSub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("/pmppc/jerk",
                                       100,
                                       boost::bind(&SolverTest::jerkFeed, &solverTest, _1),
                                           
    
    ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());

    

    ros::Rate rate(200);

    while(ros::ok() )
    {
        if(solverTest.startControl)
            solverTest.cmdCal();
        // std::cout<<"odom topic: "<<solverTest.odomTopic<<std::endl;
        solverTest.pubstate();
        ros::spinOnce();
        rate.sleep();
    };
}
