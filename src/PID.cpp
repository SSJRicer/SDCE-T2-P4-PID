#include "PID.h"
#include <iostream>

using std::cout;
using std::endl;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd)
{
  Kp_ = Kp;
  Ki_ = Ki;
  Kd_ = Kd;

  p_error_ = i_error_ = d_error_ = 0.0;

  prev_cte_ = 0.0;
}

void PID::UpdateError(double cte)
{
  // Proportional error (= CTE):
  p_error_ = cte;

  // Integral error (sum of all CTE):
  i_error_ += cte;

  // Differential error (delta CTE):
  d_error_ = cte - prev_cte_;
  prev_cte_ = cte;
}

double PID::TotalError()
{
  return -Kp_ * p_error_ - Kd_ * d_error_ - Ki_ * i_error_;
}