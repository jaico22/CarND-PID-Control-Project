#include "PID.h"
#include <vector>
#include <math.h>
#include <iostream>

using std::vector;
/**
 * TODO: Complete the PID class. You may add any additional desired functions.
 */

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {
  /**
   * TODO: Initialize PID coefficients (and errors, if needed)
   */
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;
  dp = {0.05,0.05,0.00001};
  twid_idx = 0; 
  use_dynamic_tuning = false;
  twid_runs_n = 0;
  twid_state = 0;
  best_err = 1000000000000;
  has_settled = false;
}

void PID::UpdateError(double cte) {
  /**
   * TODO: Update PID errors based on cte.
   */
  d_error = p_error - cte;
  p_error = cte;
  i_error = i_error + cte; 
  double N_STEPS;
  if (has_settled==false){
    N_STEPS = 500+500;
    if (eval_step == 500){
      has_settled = true; 
    }
  }else{
    N_STEPS = 500;
  }

  if (has_settled==true){
    err += pow(cte,2); 
  }
  eval_step += 1;
  std::cout << "Best Err: " << best_err << std::endl;
  std::cout << use_dynamic_tuning << " Eval step: " << eval_step << std::endl;
  std::cout << "kp = " << Kp << " kd = " << Kd << " ki = " << Ki << std::endl; 

  if ((eval_step>N_STEPS)&&(use_dynamic_tuning == true)){
    std::cout << "Error = " << err << std::endl;
    if (twid_state == 0){
        if(twid_idx==0){
          Kp += dp[twid_idx];
        }else if(twid_idx==1){
          Kd += dp[twid_idx];
        }else if(twid_idx==2){
          Ki += dp[twid_idx];
        }
        twid_state = 1;      
    }else if(twid_state == 1){
      if (err < best_err){
        best_err = err;
        dp[twid_idx] *= 1.1;
        twid_idx += 1;
        if (twid_idx > 2){
          twid_idx = 0; 
        }
        twid_state = 0;
        twid_runs_n += 1; 
        
      }else{
        if(twid_idx==0){
          Kp -= 2 * dp[twid_idx];
        }else if(twid_idx==1){
          Kd -= 2 * dp[twid_idx];
        }else if(twid_idx==2){
          Ki -= 2 * dp[twid_idx];
        }
        twid_state = 2;
      }
    }else{
      if(err < best_err){
        best_err = err;
        dp[twid_idx] *= 1.1;
      }else{
        if(twid_idx==0){
          Kp += dp[twid_idx];
        }else if(twid_idx==1){
          Kd += dp[twid_idx];
        }else if(twid_idx==2){
          Ki += dp[twid_idx];
        }    
        dp[twid_idx] *= 0.9;
      }
      twid_idx += 1;
      if (twid_idx > 2){
        twid_idx = 0;
      }
      twid_state = 0; 
      twid_runs_n += 1; 
    }
    eval_step = 0;
    err = 0.0;
  }
}

double PID::steering_controller(double cte){
  double steer = -Kp * p_error - Kd * d_error - Ki * i_error;
  return steer; 
}

double PID::TotalError() {
  /**
   * TODO: Calculate and return the total error
   */
  return i_error;  // TODO: Add your total error calc here!
}