#pragma once
#include <algorithm>

class PIDController {
public:
    PIDController(double kp, double ki, double kd, double min_output = -1e6, double max_output = 1e6);

    // 计算输出，参数为当前误差
    double calculate(double error);

    // 支持重置PID积分项等
    void reset();

    // 可以动态设置目标值（Setpoint）
    void setSetpoint(double setpoint);

    // 可动态设置PID参数
    void setParameters(double kp, double ki, double kd);

private:
    double kp_, ki_, kd_;
    double prev_error_;
    double integral_;
    double min_output_, max_output_;
    double setpoint_;
    bool first_run_;
};
