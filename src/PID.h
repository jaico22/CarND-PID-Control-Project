#ifndef PID_H
#define PID_H

#include <vector>
using std::vector;

class PID {
 public:
  /**
   * Constructor
   */
  PID();

  /**
   * Destructor.
   */
  virtual ~PID();

  /**
   * Initialize PID.
   * @param (Kp_, Ki_, Kd_) The initial PID coefficients
   */
  void Init(double Kp_, double Ki_, double Kd_);


  /**
   * Control Steering Wheel Angle
   * @param cte The current cross track erreor
   */
  double steering_controller(double cte);

  /**
   * Update the PID error variables given cross track error.
   * @param cte The current cross track error
   */
  void UpdateError(double cte);

  /**
   * Calculate the total PID error.
   * @output The total PID error
   */
  double TotalError();

 private:
  /**
   * PID Errors
   */
  double p_error;
  double i_error;
  double d_error;
  bool use_dynamic_tuning;
  vector<double> dp;
  /**
   * PID Coefficients
   */ 
  double Kp;
  double Ki;
  double Kd;
  double err;
  double best_err; 
  uint8_t twid_idx; 
  int eval_step = 0;
  int twid_runs_n;
  int twid_state;
  bool has_settled;
};

#endif  // PID_H