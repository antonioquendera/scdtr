#ifndef PID_H
#define PID_H

class pid {
  float I, D, K, Ti, Td, b, h, y_old, N, Tt, u_old, v;
  bool feedback_enabled; 
  
public:
  explicit pid(float _h, float _K = 31.61, float b_ = 1,  
               float Ti_ = 1, float Td_ = 0, float N_ = 10, float Tt_ = 1);
  ~pid() {};

  float compute_control(float r, float y);
  float housekeep(float r, float y, float u);
  void update_parameters(float _K, float _b, float _Ti, float _Td, float r, float y);
  
  void setFeedbackEnabled(bool enabled) {
    feedback_enabled = enabled;
  }
};

// Constructor
inline pid::pid(float _h, float _K, float b_, 
                float Ti_, float Td_, float N_, float Tt_)
    : h{_h}, K{_K}, b{b_}, Ti{Ti_}, Td{Td_}, 
      N{N_}, Tt{Tt_}, I{0.0}, D{0.0}, y_old{0.0}, u_old{0.0}, v{0.0}, feedback_enabled {true} {}

// Compute Control using Incremental Form
inline float pid::compute_control(float r, float y) {
  float bi = K * h / Ti; 
  float ad = Td / (Td + N * h);
  float bd = K * N * Td / (Td + N * h);
  
  D = ad * D - bd * (y - y_old);  

  float P;
  if (feedback_enabled) {
      // When feedback is enabled, use the measured value y.
      P = K * (b * r - y);
  } else {
      // When feedback is disabled, ignore y (feedforward only).
      P = K * (b * r);
  }
  v = P + I;  

  float u = v;
  if (u < 0) u = 0;
  if (u > 4095) u = 4095;

  y_old = y;

  return u;
}


// Housekeeping (Integral update with anti-windup)
inline float pid::housekeep(float r, float y, float u) {
  float e = r - y;
  if (feedback_enabled) {
      // Update the integral term only when feedback is active
      I += (K * h / Ti) * e + h / Tt * (u - v);  // Anti-windup correction
  }
  y_old = y;

  u_old = u;

  return u_old;
}


// Update Parameters for Bumpless Transfer
inline void pid::update_parameters(float _K, float _b, float _Ti, float _Td, float r, float y) {
  I += K * (b * r - y) - _K * (_b * r - y); // Bumpless Transfer
  K = _K;
  b = _b;
  Ti = _Ti;
  Td = _Td;
}

#endif // PID_H
