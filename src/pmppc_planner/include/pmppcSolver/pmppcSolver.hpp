#ifndef PmppcSolver_HPP
#define PmppcSolver_HPP

#include <ros/ros.h>
#include <iostream>
#include <Eigen/Dense>
#include <qpOASES.hpp>
#include <pmppcSolver/myQueue.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <queue>

using namespace Eigen;
using namespace std;
using namespace qpOASES;

#define M_PI 3.14159265358979323846



class PmppcPlanner
{
public:
    PmppcPlanner(int N, double Ts_);
    bool SolverInit(ros::NodeHandle &nh_);
    void WaypointsInit();
    void printArray(string msg, const real_t* array, int n);
    void targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg);
    void popPoints();
    bool QPSolver(std::vector<Eigen::MatrixX4d> &hpolys);
    void globalParamInit(const ros::NodeHandle &nh_priv);
    double Max(double a1, double a2)
    {
        if(a1>a2)
            return a1;
        else
            return a2;
    };
    double Min(double a1, double a2)
    {
        if(a1<a2)
            return a1;
        else
            return a2;
    }

    SQProblem solver;
    myQueue waypoints{1000};

    bool debug;
    bool initSolver = false;
    bool getTarget;
    bool sloverRet = false;
    bool usePointModel;
    bool continuousTrajectory;
    bool usePCD; 
    bool startPlanning = false;
    bool fixedPoint = false;
    bool run_in_sim;

    int pointNum;
    int N;
    int nWSR = 200;
    int cPointNum = 20;
    int stateN;
    double Ts;
    double thk;
    double allSolverTime = 0;
    double allSolverN = 0;
    
    double qcmin,qcmax,ql,quj,quv,qs,jmin,jmax,vsmin,vsmax,qc_dis;
   
    std::vector<double> accmin;
    std::vector<double> accmax;
    std::vector<double> velmin;
    std::vector<double> velmax;
    std::vector<double> posmin;
    std::vector<double> posmax;
    std::vector<double> startPosition;
    string odomTopic;

    std::vector<Eigen::Vector3d> pointsRoute;
    queue<Vector3d> targetPoints;
    Vector3d startPos;
    Vector3d endPos;
    VectorXd u_k_1;
    VectorXd U;
    VectorXd s_hat;
    // VectorXd s_hat_init;
    VectorXd Posx;
    VectorXd Posy;
    VectorXd Posz;
    VectorXd dh;
    VectorXd d_hat_s;
    VectorXd s_idx;
    VectorXd gh;
    VectorXd lb;
    VectorXd ub;
    VectorXd QPlb;
    VectorXd QPub;
    VectorXd p_hat;
    VectorXd p_pre;
    VectorXd p_next;
    VectorXd G_set;
    VectorXd S_set;
    VectorXd State;
    VectorXd s_constraint;
    VectorXd Gradient;
    VectorXd QPSolution;

    Eigen::Matrix<double,10,1> x_k;
    Eigen::Matrix<double,10,10> A;
    Eigen::Matrix<double,10,4> B;
    MatrixXd Pm;
    MatrixXd Mm;
    MatrixXd D1m;
    MatrixXd D2m;
    MatrixXd D3m;
    MatrixXd A1m;
    MatrixXd A2m;
    MatrixXd A3m;
    MatrixXd E1m;
    MatrixXd E2m;
    MatrixXd E3m;
    MatrixXd E4m;
    MatrixXd E5m;
    MatrixXd E6m;
    MatrixXd Qc;
    MatrixXd Ql;
    MatrixXd Qu;
    MatrixXd Qs;
    MatrixXd Fm;


    ros::Subscriber targetSub;

};





#endif