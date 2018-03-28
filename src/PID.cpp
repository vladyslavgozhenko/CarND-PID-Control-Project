#include "PID.h"
#include <vector>
#include <stdlib.h>
#include <math.h>

using namespace std;

/*
* TODO: Complete the PID class.
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp_, double Ki_, double Kd_) {

  //intializes errors
  p_error = 0;
  i_error = 0;
  d_error = 0;

  // initializes PID coefficients
  Kp = Kp_;
  Ki = Ki_;
  Kd = Kd_;

  //previous cte
  double cte_prev=0.0;

  //total error
  total_error = 0.0;
}

void PID::UpdateError(double cte) {
  //calculates proportional error
  p_error = cte;

  //calculates differential error
  d_error = cte - prev_cte;

  // calculates integral error
  i_error += cte;

  //updates previos cte
  prev_cte = cte;
  return;
}

double PID::TotalError() {
    total_error = -Kp * p_error - Kd * d_error - Ki * i_error;
  return total_error;
}
