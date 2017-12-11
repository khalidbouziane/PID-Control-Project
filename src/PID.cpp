#include "PID.h"

using namespace std;

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
  this->Kp = Kp;
  this->Ki = Ki;
  this->Kd = Kd;

  p_error = 0;
  i_error = 0;
  d_error = 0;
}

void PID::UpdateError(double cte) {
  // previous cte
  d_error = cte - p_error;

  // current cte
  p_error = cte;

  // sum cte
  i_error += cte;
}

double PID::TotalError() {
  double error;
  error = Kp*p_error + Ki*i_error + Kd*d_error;
  return error;
}
