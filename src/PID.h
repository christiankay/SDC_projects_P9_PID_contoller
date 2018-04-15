#ifndef PID_H
#define PID_H

#include <vector>

using namespace std;

class PID {
public:
  /*
  * Errors
  */
  double p_error_;
  double i_error_;
  double d_error_;

 

  /* init state*/
  bool is_initialized_;
  bool found_parameter_;
  int count_;

  double prev_cte;
  double d_cte;
  double i_cte;
  /*
  * Coefficients
  */ 
  double Kp_;
  double Ki_;
  double Kd_;




  /*
  * Constructor
  */
  PID();

  /*
  * Destructor.
  */
  virtual ~PID();

  /*
  * Initialize PID.
  */
  void Init(double Kp, double Ki, double Kd);

  /*
  * Update the PID error variables given cross track error.
  */
  double UpdateControl(double cte);

  /*
  * Calculate the total PID error.
  */
  double TotalError();


  void SearchParameters(double tol, double cte);

  /*
  * twiddle init
  */
  double best_error = 999.9;
  int last_optimize_index = 0;
  int current_state = 0; // possible values 0, 1,2, 3
};

#endif /* PID_H */
