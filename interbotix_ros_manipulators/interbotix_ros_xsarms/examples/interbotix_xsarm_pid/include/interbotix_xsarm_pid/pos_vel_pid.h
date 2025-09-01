#ifndef INTERBOTIX_XSARM_POS_VEL_PID_H
#define INTERBOTIX_XSARM_POS_VEL_PID_H

#include <vector>
#include <cstdint>

class PosVelPID
{
public:
  /// @brief Constructor for the position-velocity PID class
  PosVelPID(const double Kp_pos, const double Ki_pos, const double Kd_pos,
            const double Kp_vel, const double Ki_vel, const double Kd_vel,
            const double u_min, const double u_max);

  /// @brief Compute one iteration of the feedback controller
  /// @param pos_ref - position value to track
  /// @param vel_ref - velocity value to track
  /// @param ff - feedforward term
  /// @param pos_actual - observed position value
  /// @param vel_actual - observed velocity value (can be computed or measured)
  double compute_control(const double pos_ref, const double vel_ref, 
                        const double ff, const double pos_actual, const double vel_actual);

  /// @brief Clears the position and velocity p, i, and d errors
  void clear(void);

private:
  // Controller gains for position loop
  double Kp_pos;
  double Ki_pos;
  double Kd_pos;
  
  // Controller gains for velocity loop
  double Kp_vel;
  double Ki_vel;
  double Kd_vel;

  // Position error terms
  double p_pos_error;
  double i_pos_error;
  double d_pos_error;
  
  // Velocity error terms
  double p_vel_error;
  double i_vel_error;
  double d_vel_error;

  // Controller output limits
  double u_min;
  double u_max;
};

class MultiPosVelPID
{
public:
  /// @brief Initialize multiple position-velocity PID controllers
  void init(const uint8_t num, 
           const std::vector<double> Kp_pos_vec, const std::vector<double> Ki_pos_vec, const std::vector<double> Kd_pos_vec,
           const std::vector<double> Kp_vel_vec, const std::vector<double> Ki_vel_vec, const std::vector<double> Kd_vel_vec,
           const std::vector<double> umin_vec, const std::vector<double> umax_vec);
           
  /// @brief Set the desired position reference values
  void set_pos_refs(const std::vector<double> pos_refs);
  
  /// @brief Set the desired velocity reference values
  void set_vel_refs(const std::vector<double> vel_refs);
  
  /// @brief Set the feedforward values
  void set_ffs(const std::vector<double> ffs);
  
  /// @brief Compute control outputs for all controllers
  void compute_control(double *controller_output, 
                      const std::vector<double> joint_pos_actuals,
                      const std::vector<double> joint_vel_actuals);
  
  /// @brief Clear all errors
  void clear(void);

private:
  uint8_t num_joints;
  std::vector<PosVelPID> controllers;
  std::vector<double> joint_pos_refs;
  std::vector<double> joint_vel_refs;
  std::vector<double> joint_ffs;
};

#endif // INTERBOTIX_XSARM_POS_VEL_PID_H