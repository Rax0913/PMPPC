#ifndef myQueue_HPP
#define myQueue_HPP

#include <iostream>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

class myQueue{
public:
    MatrixXd pos_q;
    MatrixXd G_q;
    MatrixXd S_q;
    int s_idx;
    int e_idx;
    int sizeNum;
    bool beWrong = false;
    bool debug = false;

    myQueue(int nn)
    {
        pos_q = MatrixXd::Zero(nn,3);
        G_q = MatrixXd::Zero(nn,1);
        S_q = MatrixXd::Zero(nn,1);
        s_idx = 0;
        e_idx = 0;
        sizeNum = nn;
    };

    void waypointInit(double x, double y, double z)
    {
        pos_q(0,0) = x;
        pos_q(0,1) = y;
        pos_q(0,2) = z;
    }

    void push(const Vector3d &point)
    {
        e_idx++;
        if(e_idx==sizeNum)
            e_idx -= sizeNum;
        if(e_idx+1 == s_idx)
            beWrong = true;
        if(beWrong)
            cout<<"Queue be wrong!!!"<<endl;
        pos_q(e_idx,0) = point(0);
        pos_q(e_idx,1) = point(1);
        pos_q(e_idx,2) = point(2);

        if(e_idx==0){
            G_q(e_idx) = pow(point(0)-pos_q(sizeNum-1,0),2) + pow(point(1)-pos_q(sizeNum-1,1),2) + pow(point(2)-pos_q(sizeNum-1,2),2);
            S_q(e_idx) = S_q(sizeNum-1) + pow(G_q(e_idx),0.5);
        }else{
            G_q(e_idx) = pow(point(0)-pos_q(e_idx-1,0),2) + pow(point(1)-pos_q(e_idx-1,1),2) + pow(point(2)-pos_q(e_idx-1,2),2);
            S_q(e_idx) = S_q(e_idx-1) + pow(G_q(e_idx),0.5);
        }
        
        if(debug)
        {
            cout<<"push successfully!!!"<<endl;
        }
    };

    void pop()
    {
        //pop之前s_hat先减掉S_q(s_idx);
        
        cout<<"s_idx: "<<s_idx<<endl;  
        s_idx++;
        if(s_idx == sizeNum)
            s_idx -= sizeNum;
        int i = s_idx+1;
        if(i==sizeNum)
            i -= sizeNum;
        
        while(i!=e_idx)
        {
            S_q(i) = S_q(i) - S_q(s_idx);
            i++;
            if(i==sizeNum)
                i -= sizeNum;
        };
        S_q(e_idx) -= S_q(s_idx);
        S_q(s_idx) = 0;
        
        
    };

    int size()
    {
        int sizeQ;
        if(e_idx>=s_idx)
            sizeQ = e_idx-s_idx + 1;
        else
            sizeQ = e_idx+sizeNum-s_idx+1;
        return sizeQ;     
    };

    double Gquery(int idx)
    {
        if(idx>size()-1)
            cout<<"G query error!!!"<<endl;
        if(debug)
        {
            cout<<"Gquery: "<<G_q(s_idx + idx)<<endl;
        }
        if(s_idx+idx>sizeNum-1)
            return G_q(s_idx - sizeNum + idx);
        else
            return G_q(s_idx + idx);
    };

    double Squery(int idx)
    {
        if(idx>size()-1)
            cout<<"S query error!!!"<<endl;
        if(debug)
        {
            cout<<"Squery: "<<S_q(s_idx + idx)<<endl;
        }
        if(s_idx+idx>sizeNum-1)
            return S_q(s_idx - sizeNum + idx);
        else
            return S_q(s_idx + idx);
    };

    void PointQuery(int idx, Vector3d &point)
    {
        if(idx>size()-1)
            cout<<"Point query error!!!"<<endl;
        if(s_idx+idx>sizeNum-1){
            point(0) = pos_q(s_idx - sizeNum + idx,0);
            point(1) = pos_q(s_idx - sizeNum + idx,1);
            point(2) = pos_q(s_idx - sizeNum + idx,2);
        }else{
            point(0) = pos_q(s_idx+idx,0);
            point(1) = pos_q(s_idx+idx,1);
            point(2) = pos_q(s_idx+idx,2);
        };
        if(debug)
        {
            cout<<"Pointquery: "<<point.transpose()<<endl;
        }
    } 
};




#endif