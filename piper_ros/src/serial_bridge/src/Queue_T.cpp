//自定义一个队列
#include "serial_bridge/Queue_T.hpp"

#include <cstring>

/**
 * @brief 查询队列长度
 * @return 队列长度
 */
int Queue_T::size()
{
	if (this->tail >= this->head)
	{
		int size = this->tail - this->head;
		return size;
	}
	else
	{
		int size = this->tail + 1 + QUEUE_MAX_LENGTH - this->head;
		return size;
	}
}

/**
 * @brief 查询队列中下标为index的元素的值
 * @param index 查询下标
 * @return 队列中下标为index的元素的值
 */
queue_data_t Queue_T::value(uint index)
{
	// 队伍长度不足，直接返回
	if (this->size() < index + 1)
		return 0;

	index = (this->head + index + 1) % QUEUE_MAX_LENGTH;

	return this->data[index];
}

/**
 * @brief 入队
 * @param x 要加入的元素
 */
void Queue_T::push(queue_data_t x)
{
	// 队列满了就不添加了
	if (this->size() == QUEUE_MAX_LENGTH)
		return;

	this->tail = (this->tail + 1) % QUEUE_MAX_LENGTH;
	this->data[this->tail] = x;
}

/**
 * @brief 出队
 * @return 出队的元素
 */
queue_data_t Queue_T::pop()
{
	// 队列长度为0，错误
	if (this->size() == 0)
		return 0;

	this->head = (this->head + 1) % QUEUE_MAX_LENGTH;

	return this->data[this->head];
}

/**
 * @brief 重载[]运算
 */
queue_data_t Queue_T::operator[](uint index)
{
	return this->value(index);
}
