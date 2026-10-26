struct PIDController {
  float kp;
  float ki;
  float kd;
  float integral;
  float prev_error;
  float out_min;
  float out_max;

  float hitung(float target, float feedback, float dt) {
    float error = target - feedback;

    // Integral dengan batasan anti-windup
    integral += error * dt;
    if (integral > out_max) integral = out_max;
    else if (integral < out_min) integral = out_min;

    // Derivatif
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    // Kalkulasi Total
    float output = (kp * error) + (ki * integral) + (kd * derivative);

    // Clamp output
    if (output > out_max) output = out_max;
    else if (output < out_min) output = out_min;

    return output;
  }
};