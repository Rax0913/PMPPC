#include <iostream>
#include <so3_control/SO3Control.h>

#include <ros/ros.h>

SO3Control::SO3Control()
  : mass_(0.5)
  , g_(9.81)
{
  acc_.setZero();
}

void
SO3Control::setMass(const double mass)
{
  mass_ = mass;
}

void
SO3Control::setGravity(const double g)
{
  g_ = g;
}

void
SO3Control::setPosition(const Eigen::Vector3d& position)
{
  pos_ = position;
}

void
SO3Control::setVelocity(const Eigen::Vector3d& velocity)
{
  vel_ = velocity;
}

void
SO3Control::calculateControl(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const double des_yaw, const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv)
{
  //  ROS_INFO("Error %lf %lf %lf", (des_pos - pos_).norm(),
  //           (des_vel - vel_).norm(), (des_acc - acc_).norm());

  bool flag_use_pos = !(std::isnan(des_pos(0)) || std::isnan(des_pos(1)) || std::isnan(des_pos(2)));
  bool flag_use_vel = !(std::isnan(des_vel(0)) || std::isnan(des_vel(1)) || std::isnan(des_vel(2)));
  bool flag_use_acc = !(std::isnan(des_acc(0)) || std::isnan(des_acc(1)) || std::isnan(des_acc(2)));

  Eigen::Vector3d totalError(Eigen::Vector3d::Zero());
  if ( flag_use_pos ) totalError.noalias() += des_pos - pos_;
  if ( flag_use_vel ) totalError.noalias() += des_vel - vel_;
  if ( flag_use_acc ) totalError.noalias() += des_acc - acc_;

  Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
                     fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
                     fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));

  // std::cout << des_pos.transpose() << std::endl;
  // std::cout << des_vel.transpose() << std::endl;
  // std::cout << des_acc.transpose() << std::endl;
  // std::cout << des_yaw << std::endl;
  // std::cout << pos_.transpose() << std::endl;
  // std::cout << vel_.transpose() << std::endl;
  // std::cout << acc_.transpose() << std::endl;
  

  force_ = mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  if ( flag_use_pos ) force_.noalias() += kx.asDiagonal() * (des_pos - pos_);
  if ( flag_use_vel ) force_.noalias() += kv.asDiagonal() * (des_vel - vel_);
  if ( flag_use_acc ) force_.noalias() += mass_ * ka.asDiagonal() * (des_acc - acc_) + mass_ * (des_acc);

  // Limit control angle to 45 degree
  double          theta = M_PI / 2;
  double          c     = cos(theta);
  Eigen::Vector3d f;
  f.noalias() = force_ - mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
  {
    double nf        = f.norm();
    double A         = c * c * nf * nf - f(2) * f(2);
    double B         = 2 * (c * c - 1) * f(2) * mass_ * g_;
    double C         = (c * c - 1) * mass_ * mass_ * g_ * g_;
    double s         = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
    force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
  }
  // Limit control angle to 45 degree

  Eigen::Vector3d b1c, b2c, b3c;
  Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

  if (force_.norm() > 1e-6)
    b3c.noalias() = force_.normalized();
  else
    b3c.noalias() = Eigen::Vector3d(0, 0, 1);

  b2c.noalias() = b3c.cross(b1d).normalized();
  b1c.noalias() = b2c.cross(b3c).normalized();

  Eigen::Matrix3d R;
  R << b1c, b2c, b3c;

  orientation_ = Eigen::Quaterniond(R);
}


Eigen::Vector3d SO3Control::computeLimitedTotalAcc(
    const Eigen::Vector3d &PIDErrorAcc,
    const Eigen::Vector3d &ref_acc)
{
  Eigen::Vector3d total_acc, Gravity;
  double max_angle = 45.0/180.0*3.1415926;
  Gravity<<0,0,-9.81;
  total_acc = PIDErrorAcc + ref_acc - Gravity;

  // Limit magnitude
  //xzx: kMinNormalizedCollectiveAcc_= 3;
  if (total_acc.norm() < 3.0)
  {
    total_acc = total_acc.normalized() * 3.0;
  }

  // Limit angle

  //xzx: 取z轴加速度
  double z_acc = total_acc.dot(Eigen::Vector3d::UnitZ());
  Eigen::Vector3d z_B = total_acc.normalized();

  //xzx: 防止z轴力太小飞机下掉
  if (z_acc < 3.0)
  {
    z_acc = 3.0; // Not allow too small z-force when angle limit is enabled.
  }
  Eigen::Vector3d rot_axis = Eigen::Vector3d::UnitZ().cross(z_B).normalized();
  //xzx:矢量方向与z轴夹角
  double rot_ang = std::acos(Eigen::Vector3d::UnitZ().dot(z_B) / (1 * 1));
  
  if (rot_ang > max_angle) // Exceed the angle limit
  {
    //xzx: 绕着rot_axis旋转param.max_angle
    std::cout<<"rot_angle: "<<rot_ang<<std::endl;
    Eigen::Vector3d limited_z_B = Eigen::AngleAxisd(max_angle, rot_axis) * Eigen::Vector3d::UnitZ();
    total_acc = z_acc / std::cos(max_angle) * limited_z_B;
  }
  

  return total_acc;
}

void SO3Control::computeFlatInputRotation(const Eigen::Vector3d &acc,
                                  const Eigen::Vector3d &jerk,
                                  const double &yaw,
                                  const double &yaw_dot,
                                  Eigen::Vector3d &omg,
                                  Eigen::Matrix3d &R_d)
{
  static Eigen::Vector3d omg_old(0.0, 0.0, 0.0);
  static Eigen::Vector3d yk(0.0, 1.0, 0.0);

  double u,wx,wy,wz,sinTheta,cosTheta,cosFyi;

  Eigen::Vector3d ZB,XB,YB,XC,hw;

  //ZB.norm() v1.dot(v2) v1.cross(v2)


  //cal desire attitude
  ZB = acc;
  u = ZB.norm();
  ZB = ZB/u;
  XC<<cos(yaw), sin(yaw), 0;
  YB = ZB.cross(XC);
  if(YB.dot(yk)<(-YB).dot(yk))
      YB = -YB;
  yk = YB;
  YB = YB/YB.norm();
  XB = YB.cross(ZB);
  // R_d <<XB(0),YB(0),ZB(0),XB(1),YB(1),ZB(1),XB(2),YB(2),ZB(2);


  //cal desire omega
  hw = (jerk - ZB.dot(jerk)*ZB)/u;
  wx = -(hw.dot(YB));
  wy = hw.dot(XB);
  sinTheta = cos(yaw)*ZB(0) + sin(yaw)*ZB(1);
  cosTheta = cos(yaw)*XB(0) + sin(yaw)*XB(1);
  if(abs(YB(0))+abs(YB(1))>0.0001)
  {
      if(abs(cos(yaw))>abs(sin(yaw)))
          cosFyi = YB(1)/cos(yaw);
      else
          cosFyi = -YB(0)/sin(yaw);
      wz = (cosFyi*yaw_dot+sinTheta*wx)/(cosTheta+0.00000001);
  }else{
      cosFyi = 0;
      wz = (yaw_dot +sinTheta*wx)/(cosTheta+0.00000001);
  }
  omg << wx,wy,wz;

  Eigen::Matrix3d rotM;
  R_d << XB, YB, ZB;
  omg_old = omg;
  // std::cout<<"hhh"<<std::endl;
  return;
}

void
SO3Control::calculateControlSO3(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const Eigen::Vector3d& des_jerk,
                             const double des_yaw, const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv)
{
  //  ROS_INFO("Error %lf %lf %lf", (des_pos - pos_).norm(),
  //           (des_vel - vel_).norm(), (des_acc - acc_).norm());

  bool flag_use_pos = !(std::isnan(des_pos(0)) || std::isnan(des_pos(1)) || std::isnan(des_pos(2)));
  bool flag_use_vel = !(std::isnan(des_vel(0)) || std::isnan(des_vel(1)) || std::isnan(des_vel(2)));
  bool flag_use_acc = !(std::isnan(des_acc(0)) || std::isnan(des_acc(1)) || std::isnan(des_acc(2)));

  Eigen::Vector3d acc_error;
  double pos_error,vel_error;

  for(int i=0; i<3; i++)
  {
    pos_error = std::max(std::min(des_pos(i) - pos_(i), 1.0), -1.0);
    vel_error = std::max(std::min(des_vel(i) - vel_(i) + 2.0*pos_error, 1.0), -1.0);
    acc_error(i) = 2.0*vel_error;
  }

  Eigen::Vector3d total_des_acc = computeLimitedTotalAcc(acc_error, des_acc);

  // force_ = mass_ * g_ * Eigen::Vector3d(0, 0, 1) + mass_ * (des_acc + acc_error);
  force_ = mass_ * total_des_acc;
  
  Eigen::Vector3d omg;
  Eigen::Matrix3d R_d;
  Eigen::Quaterniond desired_attitude;
  computeFlatInputRotation(total_des_acc, des_jerk, des_yaw, des_yaw_dot, 
                omg, R_d);

  omega_ = omg;

  orientation_ = Eigen::Quaterniond(R_d);
}

const Eigen::Vector3d&
SO3Control::getComputedForce(void)
{
  return force_;
}

const Eigen::Vector3d&
SO3Control::getOmega(void)
{
  return omega_;
}

const Eigen::Quaterniond&
SO3Control::getComputedOrientation(void)
{
  return orientation_;
}

void
SO3Control::setAcc(const Eigen::Vector3d& acc)
{
  acc_ = acc;
}
