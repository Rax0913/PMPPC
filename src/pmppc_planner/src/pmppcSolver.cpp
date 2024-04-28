#include<pmppcSolver/pmppcSolver.hpp>

PmppcPlanner::PmppcPlanner(int N_, double Ts_)
{
   N = N_;
   Ts = Ts_;
   thk = 0;
   debug = false;
   getTarget = false;
   startPos<<0,0,0;
   endPos<<0,0,0;
   stateN = 10;

    x_k = MatrixXd::Zero(stateN,1);
    u_k_1 = MatrixXd::Zero(4,1);
    s_hat = MatrixXd::Zero(N,1);
    d_hat_s = MatrixXd::Zero(N,1);
    s_idx = MatrixXd::Zero(N,1);
    s_constraint = MatrixXd::Zero(N,1);
    dh = MatrixXd::Zero(N,1);
    gh = MatrixXd::Zero(N,1);
    lb = MatrixXd::Zero(5*N,1);
    ub = MatrixXd::Zero(5*N,1);
    QPlb = MatrixXd::Zero(5*N+60*cPointNum,1);
    QPub = MatrixXd::Zero(5*N+60*cPointNum,1) ;
    p_hat = MatrixXd::Zero(3*N,1);
    p_pre = MatrixXd::Zero(3*N,1);
    p_next = MatrixXd::Zero(3*N,1);
    State = MatrixXd::Zero(stateN*N,1);
    U = MatrixXd::Zero(4*N,1);
    Gradient = MatrixXd::Zero(N*4,1);
    QPSolution = MatrixXd::Zero(N*4,1);

    Pm = MatrixXd::Zero(N*stateN,4*N);
    Mm = MatrixXd::Zero(N*stateN,stateN);
    D1m = MatrixXd::Zero(N,stateN*N);
    D2m = MatrixXd::Zero(N*3,stateN*N);
    D3m = MatrixXd::Zero(N,4*N);
    A1m = MatrixXd::Zero(N,3*N);
    A2m = MatrixXd::Zero(N,3*N);
    A3m = MatrixXd::Zero(N*3,N);
    E1m = MatrixXd::Zero(N*3,stateN*N);
    E2m = MatrixXd::Zero(N,1);
    E3m = MatrixXd::Zero(N,stateN*N);
    E4m = MatrixXd::Zero(N,1);
    E5m = MatrixXd::Zero(N*4,4*N);
    E6m = MatrixXd::Zero(N*4,4);
    Qc = MatrixXd::Zero(N*3,3*N);
    Ql = MatrixXd::Zero(N,N);
    Qu = MatrixXd::Zero(4*N,4*N);
    Qs = MatrixXd::Zero(N,N);
    Fm = MatrixXd::Ones(1,N);



    A<<1, 0, 0, Ts, 0, 0, 0.5*pow(Ts,2), 0, 0, 0,
       0, 1, 0, 0, Ts, 0, 0, 0.5*pow(Ts,2), 0, 0,
       0, 0, 1, 0, 0, Ts, 0, 0, 0.5*pow(Ts,2), 0,
       0, 0, 0, 1, 0, 0, Ts, 0, 0, 0,
       0, 0, 0, 0, 1, 0, 0, Ts, 0, 0,
       0, 0, 0, 0, 0, 1, 0, 0, Ts, 0,
       0, 0, 0, 0, 0, 0, 1, 0, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 1, 0, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 1, 0,
       0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

    B<<pow(Ts,3)/6, 0, 0, 0,
       0, pow(Ts,3)/6, 0, 0,
       0, 0, pow(Ts,3)/6, 0,
       pow(Ts,2)/2, 0, 0, 0,
       0, pow(Ts,2)/2, 0, 0,
       0, 0, pow(Ts,2)/2, 0,
       Ts, 0, 0, 0,
       0, Ts, 0, 0,
       0, 0, Ts, 0,
       0, 0, 0, Ts;
}

void PmppcPlanner::globalParamInit(const ros::NodeHandle &nh_priv)
{
   nh_priv.getParam("continuousTrajectory",continuousTrajectory);
   nh_priv.getParam("usePointModel",usePointModel);
   nh_priv.getParam("usePCD",usePCD);
   nh_priv.getParam("qcmin",qcmin);
   nh_priv.getParam("qcmax",qcmax);
   nh_priv.getParam("ql",ql);
   nh_priv.getParam("quj",quj);
   nh_priv.getParam("quv",quv);
   nh_priv.getParam("qs",qs);
   nh_priv.getParam("jmin",jmin);
   nh_priv.getParam("jmax",jmax);
   nh_priv.getParam("vsmin",vsmin);
   nh_priv.getParam("vsmax",vsmax);
   nh_priv.getParam("qc_dis",qc_dis);
   nh_priv.getParam("accmin",accmin);
   nh_priv.getParam("accmax",accmax);
   nh_priv.getParam("velmin",velmin);
   nh_priv.getParam("velmax",velmax);
   nh_priv.getParam("posmin",posmin);
   nh_priv.getParam("posmax",posmax);
   nh_priv.getParam("startPosition",startPosition);
   nh_priv.getParam("odomTopic",odomTopic);
   nh_priv.getParam("fixedPoint",fixedPoint);
   nh_priv.getParam("run_in_sim",run_in_sim);
   cout<<"accmin: "<<accmin[0]<<accmin[1]<<accmin[2]<<endl;
}

bool PmppcPlanner::SolverInit(ros::NodeHandle &nh_)
{

   targetSub = nh_.subscribe("/move_base_simple/goal", 5, &PmppcPlanner::targetCallBack, this,
                                 ros::TransportHints().tcpNoDelay());

   Eigen::Matrix<double,10,10> APow = A;
   
   MatrixXd s_min;
   MatrixXd s_max;

   s_min = MatrixXd::Zero(N,1);
   s_max = MatrixXd::Zero(N,1);


   for(int i = 0; i<N; i++)
   {
      Mm.block<10,10>(i*10,0) = APow;
      D1m(i,10*i+10-1) = 1;
      D2m.block<3,3>(i*3,i*10) <<1, 0, 0, 0, 1, 0, 0, 0, 1;
      D3m.block<1,4>(i,i*4)<<0,0,0,1;
      E5m.block<4,4>(i*4,i*4)<<1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1;
      if(i==0)
         E6m.block<4,4>(0,0)<<-1,0,0,0, 0,-1,0,0, 0,0,-1,0, 0,0,0,-1;
      else
         E5m.block<4,4>(i*4,i*4-4)<<-1,0,0,0, 0,-1,0,0, 0,0,-1,0, 0,0,0,-1;
      Ql(i,i) = ql;
      Qu.block<4,4>(i*4,i*4)<<quj,0,0,0, 0,quj,0,0, 0,0,quj,0, 0,0,0,quv;
      lb.block<4,1>(i*4,0)<<jmin, jmin, jmin, vsmin;
      ub.block<4,1>(i*4,0)<<jmax, jmax, jmax, vsmax;
      QPlb.block<4,1>(i*4,0)<<jmin, jmin, jmin, vsmin;
      QPub.block<4,1>(i*4,0)<<jmax, jmax, jmax, vsmax;
      Pm.block<10,4>(10*i,4*i) = B;
      for(int j=i+1; j<N; j++)
      {
         Pm.block<10,4>(10*j,4*i) = A*Pm.block<10,4>(10*j-10,4*i);
      }
      APow = A*APow;
   }


   Vector3d point;
   waypoints.waypointInit(startPosition[0],startPosition[1],startPosition[2]);
   point<<startPosition[0],startPosition[1],startPosition[2];waypoints.push(point);

   for(int i =0; i<20; i++){
      point<<0,0,0;
      pointsRoute.push_back(point);
   }

   SQProblem temp(4*N,10*N+cPointNum*60);
   solver = temp;

   return true;

};

void PmppcPlanner::popPoints()
{
   if(waypoints.size()>6 && s_hat(0)>waypoints.Squery(6))
   {
      int pop_idx;
      cout<<"s_idx: "<<waypoints.s_idx<<" e_idx"<<waypoints.e_idx<<endl;
      for(int j=5; j<waypoints.size(); j++)
      {
         if(s_hat(0)>=waypoints.Squery(j) && s_hat(0)<waypoints.Squery(j+1))
         {
            pop_idx = j;
            break;
         }
      };
      for(int i=0; i<N; i++)
         s_hat(i) -= waypoints.Squery(pop_idx - 5);
      thk -= waypoints.Squery(pop_idx - 5);
      for(int i=0; i<pop_idx-5; i++)
      {
         waypoints.pop();
      }
   };
};

void PmppcPlanner::targetCallBack(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
   if(!fixedPoint){
      Vector3d posGoal;
      posGoal(0) = msg->pose.position.x;
      posGoal(1) = msg->pose.position.y;
      posGoal(2) = 2*(msg->pose.orientation.z + msg->pose.orientation.w);
      posGoal(2) = posGoal(2)>0.5 ? posGoal(2) : 0.5;
      posGoal(2) = posGoal(2)<2.0 ? posGoal(2) : 2.0;
      cout<<"setGoal:  "<<posGoal(0)<<" "<<posGoal(1)<<" "<<posGoal(2)<<endl;
      targetPoints.push(posGoal);
      popPoints();
      // waypoints.push(posGoal);
      getTarget = true;
   }else{
      if(!startPlanning)
      {
         Vector3d point;
         // point<<1.2,-0.6,1.5; waypoints.push(point);
         // point<<5.5,-1.1,1.5; waypoints.push(point);
         // point<<6.8,-1.2,1.5; waypoints.push(point);
         // point<<6.8,-17,1.5; waypoints.push(point);
         // point<<17.6,-17,1.5; waypoints.push(point);
         // point<<18,6,1.5; waypoints.push(point);
         // point<<7,6,1.5; waypoints.push(point);
         // point<<7.5,27.8,1.5; waypoints.push(point);
         // point<<18.4,27.5,1.5; waypoints.push(point);
         // point<<18,6,1.5; waypoints.push(point);
         // point<<7,6,1.5; waypoints.push(point);
         // point<<6.6,-1.1,1.5; waypoints.push(point);
         // point<<1.2,-0.6,1.5; waypoints.push(point);

         point<<0,0,1.5; waypoints.push(point);
         point<<14.5,0,1.5; waypoints.push(point);
         point<<15.5,6,1.5; waypoints.push(point);
         point<<-8,6,1.5; waypoints.push(point);
         point<<-9,23,2; waypoints.push(point);
         point<<14.5,24.0,1.5; waypoints.push(point);
         point<<15.5,6,1.5; waypoints.push(point);
         point<<-8,6,1.5; waypoints.push(point);
         point<<-7.5,0,1.5; waypoints.push(point);
         point<<0,-0,1.5; waypoints.push(point);
         startPlanning = true;
      }
   }
   
};


void PmppcPlanner::printArray(string msg, const real_t* array, int n)
{
   cout<<msg<<": ";
   for(int i=0; i<n;i++)
      cout<<" "<<array[i];
   cout<<" "<<endl;
}

bool PmppcPlanner::QPSolver(std::vector<Eigen::MatrixX4d> &hpolys)
{
   int j;
   pointsRoute.clear();
   real_t Hqp[4*N*4*N];
   real_t Aqp[4*N*(10*N+cPointNum*60)];
   real_t gqp[4*N];
   real_t lbqp[4*N];
   real_t ubqp[4*N];
   real_t lbAqp[10*N+cPointNum*60];
   real_t ubAqp[10*N+cPointNum*60];
   real_t uOpt[4*N];

   int hployRows[20];
   int totalRows = 0;
   if(hpolys.empty())
   {
      for(int i=0; i<20; i++)
      {
         hployRows[i] = 0;
         totalRows += hployRows[i];
      };
   }else{
      for(int i=0; i<20; i++)
      {
         // hployRows[i] = hpolys[i/2].rows();
         hployRows[i] = hpolys[i].rows();
         totalRows += hployRows[i];
      };
   }
   

   MatrixXd B2m  = MatrixXd::Zero(totalRows,3*N);
   MatrixXd B3m  = MatrixXd::Zero(totalRows,4*N);
   MatrixXd hm  = MatrixXd::Zero(totalRows,1);
   MatrixXd hmax  = MatrixXd::Zero(totalRows,1);
   MatrixXd J = MatrixXd::Zero(N*4,4*N);
   MatrixXd s_min;
   MatrixXd state_min;
   MatrixXd state_max;

   s_min = MatrixXd::Zero(10*N,1);
   state_min = MatrixXd::Zero(9,1);
   state_max = MatrixXd::Zero(9,1);
   for(int i=0; i<N; i++)
   {
     Vector3d point_hat;
      if(s_hat(i)>=waypoints.S_q(waypoints.e_idx)){
         j = waypoints.size()-2;
         s_idx(i) = waypoints.size()-2;
      }else{
         if(i==0)
            j=0;
         else
            j=Max(0,s_idx(i-1)-2); 
         while(!(s_hat(i)>=waypoints.Squery(j) && s_hat(i)<waypoints.Squery(j+1))){
            j++;
         }
            
         s_idx(i) = j;
      }
      double s_j = waypoints.Squery(j);
      double s_j_1 = waypoints.Squery(j+1);
      double g_j_1 = waypoints.Gquery(j+1);
      Vector3d pos_j,pos_j_1;
      waypoints.PointQuery(j,pos_j);
      waypoints.PointQuery(j+1,pos_j_1);
      dh(i) = Min(s_j_1- s_hat(i),s_hat(i) - s_j);
      gh(i) = g_j_1 + 0.0001;
      d_hat_s(i) = s_j_1 - s_hat(i)+0.0001;
      // s_hat(i) = Min(s_hat(i),S_set(S_set.size()-1));

      p_hat(i*3) = pos_j(0) + (s_hat(i) - s_j)/(s_j_1 - s_j + 0.0001)*(pos_j_1(0) - pos_j(0));
      p_hat(i*3+1) = pos_j(1) + (s_hat(i) - s_j)/(s_j_1 - s_j + 0.0001)*(pos_j_1(1) - pos_j(1));
      p_hat(i*3+2) = pos_j(2) + (s_hat(i) - s_j)/(s_j_1 - s_j + 0.0001)*(pos_j_1(2) - pos_j(2));
      if(i==0)
         startPos<<p_hat(i*3),p_hat(i*3+1),p_hat(i*3+2);
      if(i==3)
         endPos<<p_hat(i*3),p_hat(i*3+1),p_hat(i*3+2);
      p_pre.block<3,1>(i*3,0)<<pos_j(0), pos_j(1), pos_j(2);
      p_next.block<3,1>(i*3,0)<<pos_j_1(0), pos_j_1(1), pos_j_1(2);
      point_hat<<p_hat(i*3),p_hat(i*3+1),p_hat(i*3+2);
      pointsRoute.push_back(point_hat);
   }

   int hpolyIdx = 0 ;
   for(int i=0; i<N; i++)
   {
      double qc;
      qc = min(qcmax,max(qcmin,-(qcmax-qcmin)/qc_dis*dh(i)+qcmax));
   
      Qc.block<3,3>(i*3,i*3)<<qc,0,0, 0,qc,0, 0,0,qc;
      Qu.block<4,4>(i*4,i*4)<<quj,0,0,0, 0,quj,0,0, 0,0,quj,0, 0,0,0,quv;
      if(i==0)
         Qs(i,i) = qs*1;
      else
         Qs(i,i) = qs;
      A1m.block<1,3>(i,3*i)<<(p_next(i*3)-p_hat(i*3))/d_hat_s(i), (p_next(i*3+1)-p_hat(i*3+1))/d_hat_s(i), (p_next(i*3+2)-p_hat(i*3+2))/d_hat_s(i);
      A2m.block<1,3>(i,3*i)<<(p_next(i*3)-p_pre(i*3))/gh(i), (p_next(i*3+1)-p_pre(i*3+1))/gh(i), (p_next(i*3+2)-p_pre(i*3+2))/gh(i);
      A3m.block<3,1>(3*i,i)<<p_next(i*3)-p_pre(i*3), p_next(i*3+1)-p_pre(i*3+1), p_next(i*3+2)-p_pre(i*3+2);

      
      Eigen::MatrixX4d hpoly;
      hpoly = hpolys[i];

      for(int j=0; j<hployRows[i]; j++)
      {
         B2m(hpolyIdx+j,3*i) = hpoly(j,0);
         B2m(hpolyIdx+j,3*i+1) = hpoly(j,1);
         B2m(hpolyIdx+j,3*i+2) = hpoly(j,2);
         hmax(hpolyIdx+j) = -hpoly(j,3);
      };

      hpolyIdx += hployRows[i];
    
   }

   E1m = A3m*A2m*D2m - D2m;
   E2m = p_pre - A3m*A2m*p_pre;
   E3m = A1m*D2m - D1m;
   E4m = s_hat - A1m*p_hat;
   B3m = B2m*D2m*Pm;
   hm = B2m*D2m*Mm*x_k;


   J = Pm.transpose()*E1m.transpose()*Qc*E1m*Pm + Pm.transpose()*E3m.transpose()*Ql*E3m*Pm + E5m.transpose()*Qu*E5m;
   Gradient = x_k.transpose()*Mm.transpose()*E3m.transpose()*Ql*E3m*Pm + E4m.transpose()*Ql*E3m*Pm 
            + u_k_1.transpose()*E6m.transpose()*Qu*E5m + x_k.transpose()*Mm.transpose()*E1m.transpose()*Qc*E1m*Pm
            + E2m.transpose()*Qc*E1m*Pm - 0.5*Fm*Qs*D1m*Pm;

   s_min = -Mm*x_k;

   state_min<<posmin[0],posmin[1],posmin[2],velmin[0],velmin[1],
               velmin[2],accmin[0],accmin[1],accmin[2];
   state_max<<posmax[0],posmax[1],posmax[2],velmax[0],velmax[1],
               velmax[2],accmax[0],accmax[1],accmax[2];

   for(int i = 0; i<4*N; i++)
   {
      gqp[i] = Gradient(i);
      if((i+1)%4 == 0)
      {
         lbqp[i] = vsmin;
         ubqp[i] = vsmax;
      }else{
         lbqp[i] = jmin;
         ubqp[i] = jmax;
      }
      if(i<N)
      {
         for(int r=0; r<9; r++)
         {
            lbAqp[10*i+r] = state_min(r)+s_min(10*i+r);
            ubAqp[10*i+r] = state_max(r)+s_min(10*i+r);
         }
         lbAqp[10*i+9] = Max(0,s_min(10*i+9));
         ubAqp[10*i+9] = waypoints.S_q(waypoints.e_idx) + s_min(10*i+9);
      }
      for(int j = 0; j<4*N; j++)
      {
         Hqp[i*4*N + j] = J(i,j);
      }

   };

   for(int i=0; i<10*N; i++)
      for(int j=0; j<4*N; j++)
         Aqp[i*4*N+j] = Pm(i,j);
   for(int i=0; i<totalRows; i++)
   {
      lbAqp[10*N+i] = -10000000;
      ubAqp[10*N+i] = hmax(i) - hm(i);
      for(int j=0; j<4*N; j++)
         Aqp[N*4*10*N+i*4*N+j] = B3m(i,j);
   }

   for(int i=totalRows; i<60*cPointNum; i++)
   {
      lbAqp[10*N+i] = -10000000;
      ubAqp[10*N+i] = 10000000;
      for(int j=0; j<4*N; j++)
         Aqp[N*4*10*N+i*4*N+j] = 0;
   }

   double startTime = ros::Time::now().toSec();
   nWSR = 100;
   if(!initSolver)
      sloverRet =  solver.init(Hqp,gqp,Aqp,lbqp,ubqp,lbAqp,ubAqp,nWSR,0);
   else
      sloverRet = solver.hotstart(Hqp,gqp,Aqp,lbqp,ubqp,lbAqp,ubAqp,nWSR,0);   //成功返回零，失败返回1
   cout<<"slover ret: "<<sloverRet<<endl;
   initSolver = true;
   solver.getPrimalSolution(uOpt);
   allSolverTime += (ros::Time::now().toSec() - startTime);
   allSolverN += 1.0;
   for(int i=0;i<4*N;i++)
      QPSolution(i) = uOpt[i];

   s_hat(0) =  thk + Max(0,QPSolution(3))*Ts;
   s_hat(0) = Min(s_hat(0),waypoints.S_q(waypoints.e_idx));
   thk = s_hat(0);
   for(int i = 1; i<N; i++)
   {
      s_hat(i) = s_hat(i-1) + Max(0,QPSolution(4*i+3))*Ts;
      s_hat(i) = Min(s_hat(i),waypoints.S_q(waypoints.e_idx));
      s_hat(i-1) = s_hat(i);
   }
   s_hat(N-1) = 2.5*s_hat(N-2)-2*s_hat(N-3)+0.5*s_hat(N-4);
   s_hat(N-1) = Min(s_hat(N-1),waypoints.S_q(waypoints.e_idx));
   // cout<<"s_hat "<<s_hat.transpose()<<endl;
   u_k_1 <<QPSolution(0),QPSolution(1),QPSolution(2),QPSolution(3);
   if(debug)
   {
      cout<<"QPSolution:   "<<QPSolution<<endl;
   }
   return !sloverRet;

}