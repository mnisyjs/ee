#ifndef __QUEUE_T_H
#define __QUEUE_T_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <Eigen/Core>

#include <iostream>

// 队列中最长长度
#define QUEUE_MAX_LENGTH 50

// 队列中数据元素的数据类型
typedef uint8_t queue_data_t;

// 队列结构体定义
//循环数组
class Queue_T
{
public:
	/*队列数组*/
	queue_data_t data[QUEUE_MAX_LENGTH + 1] = {0};
	int head = 0, tail = 0;

	/**
	 * @brief 查询队列长度
	 * @return 队列长度
	 */
	int size();

	/**
	 * @brief 查询队列中下标为index的元素的值
	 * @param index 查询下标
	 * @return 队列中下标为index的元素的值
	 */
	queue_data_t value(uint index);

	/**
	 * @brief 入队
	 * @param x 要加入的元素
	 */
	void push(queue_data_t x);

	/**
	 * @brief 出队
	 * @return 出队的元素
	 */
	queue_data_t pop();

	/**
	 * @brief 重载[]运算
	 */
	queue_data_t operator[](uint index);
};

#endif
