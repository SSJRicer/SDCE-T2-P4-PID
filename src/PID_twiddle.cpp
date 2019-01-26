#include "PID.h"
#include <iostream>

#define RUN_ITER 100

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

  //////// TWIDDLE PART ////////
  // Twiddle flag:
  use_twiddle_ = false;

  it_ = 1;

  // Best error:
  best_cte_ = 1000;

  // Iteration number:
  count_ = 1;

  PID_ind_ = 1;

  twi_dKp_ = 0.1 * Kp;
  twi_dKi_ = 0.1 * Ki;
  twi_dKd_ = 0.1 * Kd;

  twiddle_step_ = 1;
  //////// TWIDDLE PART ////////
}

void PID::UpdateError(double cte)
{
  p_error_ = cte;
  i_error_ += cte;
  d_error_ = cte - prev_cte_;

  if (use_twiddle_) {
      cout << "Count = " << count_ << endl;
      if (count_ > RUN_ITER && count_ <= RUN_ITER + 100)
          twiddle(cte);
      else if (count_ > RUN_ITER + 100)
          count_ = 1;
  }

  if (cte < best_cte_)
  best_cte_ = cte;
  prev_cte_ = cte;

  count_++;
}

double PID::TotalError()
{
  return Kp_ * p_error_ + Kd_ * d_error_ + Ki_ * i_error_;
}

void PID::twiddle(double cte)
{
    switch (PID_ind_) {
        case 1:
            twiddleUpdate(cte, &Kp_, &twi_dKp_);
            PID_ind_ = 2;
            break;
        case 2:
            twiddleUpdate(cte, &Ki_, &twi_dKi_);
            PID_ind_ = 3;
            break;
        case 3:
            twiddleUpdate(cte, &Kd_, &twi_dKd_);
            PID_ind_ = 1;
            cout << "Iteration [" << it_ << "], best error = " << best_cte_ << endl;
            it_++;
            break;
        default:
            cout << "Something is very wrong here!\n";
    }
}

void PID::twiddleUpdate(double cte, double *K, double *dK)
{
    switch (twiddle_step_) {
        case 1:
            *K += *dK;
            break;
        case 2:
            if (cte < best_cte_) {
                best_cte_ = cte;
                *dK *= 1.1;
            }
            else
                *K -= 2 * (*dK);
            break;
        case 3:
            if (cte < best_cte_) {
                best_cte_ = cte;
                *dK *= 1.1;
            }
            else {
                *K += *dK;
                *dK += 0.9;
            }
            break;
        default:
            cout << "Twiddledumb\n";
    }
}