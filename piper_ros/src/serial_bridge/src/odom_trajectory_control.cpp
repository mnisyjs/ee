#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
#include <vector>
#include <cmath>

enum State {
    FORWARD,
    TURN,
    STOP
};

struct Step {
    double distance;   // 直线距离
    double angle;      // 旋转角度（弧度）
};

class OdomTrajectoryControl {
public:
    OdomTrajectoryControl(ros::NodeHandle& nh):
        nh_(nh), state_(FORWARD), current_step_(0), first_odom_(true)
    {
        odom_sub_ = nh_.subscribe("/odom", 10, &OdomTrajectoryControl::odomCallback, this);
        cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

        // 定义轨迹：先直行2m，再左转90度，再直行1m
        traj_ = {
            {7.0, 0.0},            // 直行2米
            {0.0, M_PI/2.0},       // 左转90度
            {3.0, 0.0},            // 直行1米
            {0.0, M_PI/2.0},      // 右转90度
            {7.0, 0.0}             // 直行1米
        };
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;
        double yaw = tf::getYaw(msg->pose.pose.orientation);

        if (first_odom_) {
            start_x_ = x;
            start_y_ = y;
            start_yaw_ = yaw;
            first_odom_ = false;
        }

        if (current_step_ >= traj_.size()) {
            publishStop();
            return;
        }

        Step& step = traj_[current_step_];

        switch (state_) {
            case FORWARD: {
                double dx = x - start_x_;
                double dy = y - start_y_;
                double dist = std::sqrt(dx*dx + dy*dy);
                if (dist < step.distance) {
                    publishForward();
                } else {
                    publishStop();
                    ros::Duration(0.5).sleep();
                    start_x_ = x;
                    start_y_ = y;
                    start_yaw_ = yaw;
                    if (fabs(step.angle) > 1e-3) {
                        state_ = TURN;
                    } else {
                        current_step_++;
                        if (current_step_ < traj_.size()) {
                            state_ = (traj_[current_step_].distance > 1e-3) ? FORWARD : TURN;
                        } else {
                            state_ = STOP;
                        }
                    }
                }
                break;
            }
            case TURN: {
                double dtheta = normalizeAngle(yaw - start_yaw_);
                if (fabs(dtheta) < fabs(step.angle)) {
                    publishTurn(step.angle > 0 ? 1.0 : -1.0);
                } else {
                    publishStop();
                    ros::Duration(0.5).sleep();
                    start_x_ = x;
                    start_y_ = y;
                    start_yaw_ = yaw;
                    current_step_++;
                    if (current_step_ < traj_.size()) {
                        state_ = (traj_[current_step_].distance > 1e-3) ? FORWARD : TURN;
                    } else {
                        state_ = STOP;
                    }
                }
                break;
            }
            case STOP:
            default:
                publishStop();
                break;
        }
    }

    void publishForward() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.15; // 直行速度（可调）
        cmd.angular.z = 0.0;
        cmd_pub_.publish(cmd);
    }

    void publishTurn(double dir) {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = dir * 0.5; // 转向速度（可调）
        cmd_pub_.publish(cmd);
    }

    void publishStop() {
        geometry_msgs::Twist cmd;
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_.publish(cmd);
    }

    double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2*M_PI;
        while (angle < -M_PI) angle += 2*M_PI;
        return angle;
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_pub_;
    State state_;
    size_t current_step_;
    std::vector<Step> traj_;

    double start_x_, start_y_, start_yaw_;
    bool first_odom_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "odom_trajectory_control");
    ros::NodeHandle nh;

    OdomTrajectoryControl controller(nh);

    ros::spin();
    return 0;
}
