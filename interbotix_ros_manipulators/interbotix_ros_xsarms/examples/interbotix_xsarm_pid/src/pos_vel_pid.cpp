#include "interbotix_xsarm_pid/pos_vel_pid.h"

PosVelPID::PosVelPID(const double Kp_pos, const double Ki_pos, const double Kd_pos,
                    const double Kp_vel, const double Ki_vel, const double Kd_vel,
                    const double u_min, const double u_max)
  : Kp_pos(Kp_pos), Ki_pos(Ki_pos), Kd_pos(Kd_pos),
    Kp_vel(Kp_vel), Ki_vel(Ki_vel), Kd_vel(Kd_vel),
    u_min(u_min), u_max(u_max)
{
  clear();
}

double PosVelPID::compute_control(const double pos_ref, const double vel_ref, 
                                 const double ff, const double pos_actual, const double vel_actual)
{
  // Position error calculation
  double pos_error = pos_ref - pos_actual;
  double i_pos_error_temp = i_pos_error + pos_error;
  d_pos_error = pos_error - p_pos_error;
  p_pos_error = pos_error;
  
  // Velocity error calculation
  double vel_error = vel_ref - vel_actual;
  double i_vel_error_temp = i_vel_error + vel_error;
  d_vel_error = vel_error - p_vel_error;
  p_vel_error = vel_error;

  // Combine position and velocity control terms
  double u_pos = Kp_pos * p_pos_error + Ki_pos * i_pos_error_temp + Kd_pos * d_pos_error;
  double u_vel = Kp_vel * p_vel_error + Ki_vel * i_vel_error_temp + Kd_vel * d_vel_error;
  
  // Sum position and velocity control with feedforward
  double u = ff + u_pos + u_vel;

  // Prevent integrator windup and cap output
  if (u_min < u && u < u_max) {
    i_pos_error = i_pos_error_temp;
    i_vel_error = i_vel_error_temp;
  }
  else if (u < u_min)
    u = u_min;
  else if (u > u_max)
    u = u_max;

  return u;
}

void PosVelPID::clear(void)
{
  p_pos_error = 0;
  i_pos_error = 0;
  d_pos_error = 0;
  
  p_vel_error = 0;
  i_vel_error = 0;
  d_vel_error = 0;
}

void MultiPosVelPID::init(const uint8_t num, 
                         const std::vector<double> Kp_pos_vec, const std::vector<double> Ki_pos_vec, const std::vector<double> Kd_pos_vec,
                         const std::vector<double> Kp_vel_vec, const std::vector<double> Ki_vel_vec, const std::vector<double> Kd_vel_vec,
                         const std::vector<double> umin_vec, const std::vector<double> umax_vec)
{
  num_joints = num;
  controllers.clear();
  
  for (size_t i = 0; i < num_joints; i++) {
    PosVelPID controller(Kp_pos_vec.at(i), Ki_pos_vec.at(i), Kd_pos_vec.at(i),
                        Kp_vel_vec.at(i), Ki_vel_vec.at(i), Kd_vel_vec.at(i),
                        umin_vec.at(i), umax_vec.at(i));
    controllers.push_back(controller);
    joint_ffs.push_back(0);
  }
  
  // Initialize reference vectors with zeros
  joint_pos_refs.resize(num_joints, 0);
  joint_vel_refs.resize(num_joints, 0);
}

void MultiPosVelPID::set_pos_refs(const std::vector<double> pos_refs)
{
  joint_pos_refs = pos_refs;
}

void MultiPosVelPID::set_vel_refs(const std::vector<double> vel_refs)
{
  joint_vel_refs = vel_refs;
}

void MultiPosVelPID::set_ffs(const std::vector<double> ffs)
{
  joint_ffs = ffs;
}

void MultiPosVelPID::compute_control(double *controller_output, 
                                    const std::vector<double> joint_pos_actuals,
                                    const std::vector<double> joint_vel_actuals)
{
  for (size_t i = 0; i < num_joints; i++) {
    controller_output[i] = controllers.at(i).compute_control(
      joint_pos_refs.at(i), joint_vel_refs.at(i),
      joint_ffs.at(i), 
      joint_pos_actuals.at(i), joint_vel_actuals.at(i)
    );
  }
}

void MultiPosVelPID::clear(void)
{
  for (auto& controller : controllers) {
    controller.clear();
  }
}