#ifndef PID_H
#define PID_H

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
   * @param (Kp, Ki, Kd) The initial PID coefficients
   */
  void Init(double Kp, double Ki, double Kd);

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

  // TODO: NOTICE THIS PART
  void twiddle(double cte);
  void twiddleUpdate(double cte, double *K, double *dK);
  // TODO [END]

 //private:
  /**
   * PID Errors
   */
  double p_error_;
  double i_error_;
  double d_error_;

  /**
   * PID Coefficients
   */
  double Kp_;
  double Ki_;
  double Kd_;

  double prev_cte_;

  int count_;

  bool use_twiddle_;
  int PID_ind_;
  double twi_dKp_;
  double twi_dKi_;
  double twi_dKd_;
  double best_cte_;

  int twiddle_step_;
  int it_;
};

#endif  // PID_H