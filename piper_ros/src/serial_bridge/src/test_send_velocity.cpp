#include <ros/ros.h>
#include <string>
#include "serial_bridge/uart_Thread.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "stm32_velocity_test");
	ros::NodeHandle nh("~");

	std::string com_name = "/dev/ttyUSB0";
	double vx = 0.1;
	double vy = 0.1;
	double wz = 0.1;
	int rate_hz = 10;
	double duration_s = 3.0;

	nh.param<std::string>("com_name", com_name, com_name);
	nh.param("vx", vx, vx);
	nh.param("vy", vy, vy);
	nh.param("wz", wz, wz);
	nh.param("rate_hz", rate_hz, rate_hz);
	nh.param("duration", duration_s, duration_s);

	Uart_Thread uart;
	int init_ret = uart.InitSerialPort(com_name);
	if (init_ret == -1)
	{
		ROS_ERROR("Failed to open serial port: %s", com_name.c_str());
		return 1;
	}

	ROS_INFO("Starting STM32 velocity test on %s: vx=%.3f m/s, vy=%.3f m/s, wz=%.3f rad/s, rate=%d Hz, duration=%.1f s",
			 com_name.c_str(), vx, vy, wz, rate_hz, duration_s);

	ros::Rate rate(rate_hz);
	ros::Time end_time = ros::Time::now() + ros::Duration(duration_s);
	while (ros::ok() && ros::Time::now() < end_time)
	{
		uart.Mission_Send(Uart_Thread_Space::Lidar_Position, &uart,
					   static_cast<float>(vx),
					   static_cast<float>(vy),
					   static_cast<float>(wz));
		ROS_INFO_THROTTLE(1.0, "Sent velocities to STM32: vx=%.3f, vy=%.3f, wz=%.3f", vx, vy, wz);
		rate.sleep();
	}

	ROS_INFO("STM32 velocity test completed.");
	return 0;
}
