#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>

class SO3Control
{
public:
  SO3Control();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d& position);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAcc(const Eigen::Vector3d& acc);

  void calculateControl(const Eigen::Vector3d& des_pos,
                        const Eigen::Vector3d& des_vel,
                        const Eigen::Vector3d& des_acc, const double des_yaw,
                        const double des_yaw_dot, const Eigen::Vector3d& kx,
                        const Eigen::Vector3d& kv);

  void calculateControlSO3(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const Eigen::Vector3d& des_jerk,
                             const double des_yaw, const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv);

  void computeFlatInputRotation(const Eigen::Vector3d &acc,
                                  const Eigen::Vector3d &jerk,
                                  const double &yaw,
                                  const double &yaw_dot,
                                  Eigen::Vector3d &omg,
                                  Eigen::Matrix3d &R_d);
  Eigen::Vector3d computeLimitedTotalAcc(
    const Eigen::Vector3d &PIDErrorAcc,
    const Eigen::Vector3d &ref_acc);

  const Eigen::Vector3d&    getComputedForce(void);
  const Eigen::Vector3d&    getOmega(void);
  const Eigen::Quaterniond& getComputedOrientation(void);


  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Inputs for the controller
  double          mass_;
  double          g_;
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;

  // Outputs of the controller
  Eigen::Vector3d    force_;
  Eigen::Vector3d    omega_;
  Eigen::Quaterniond orientation_;
};

#endif
