#ifndef __ROS_COMMUNICATION_H
#define __ROS_COMMUNICATION_H
#include <ros/ros.h>
#include <boost/asio.hpp>

bool ROS_READ_FROM_STM32(float *angle, float *x_action, float *y_action, float *data4, float *data5, float *data6, float *data7, float *data8, float *data9, float *data10);
void ROS_WRITE_TO_STM32(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10);
unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len);
void SerialInit();

#endif 