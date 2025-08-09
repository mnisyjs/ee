#include "serial_bridge/pid_controller.h"

PIDController::PIDController(double kp, double ki, double kd, double min_output, double max_output)
    : kp_(kp), ki_(ki), kd_(kd),
      prev_error_(0.0), integral_(0.0),
      min_output_(min_output), max_output_(max_output),
      setpoint_(0.0), first_run_(true) {}

double PIDController::calculate(double error) {
    if (first_run_) {
        prev_error_ = error;
        first_run_ = false;
    }

    // 积分项累加
    integral_ += error;

    // 微分项
    double derivative = error - prev_error_;

    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;

    // 限制输出
    output = std::max(min_output_, std::min(max_output_, output));

    prev_error_ = error;
    return output;
}

void PIDController::reset() {
    prev_error_ = 0.0;
    integral_ = 0.0;
    first_run_ = true;
}

void PIDController::setSetpoint(double setpoint) {
    setpoint_ = setpoint;
}

void PIDController::setParameters(double kp, double ki, double kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}
