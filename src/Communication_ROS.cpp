#include "Communication_ROS.h"

using namespace std;
using namespace boost::asio;

const unsigned char serial_header[2] = {0x55,0xaa};
const unsigned char serial_ender[2] = {0x0d,0x0a};

union receiveUnion0
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_angle_union_instance;

union receiveUnion1
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_x_action_union_instance;

union receiveUnion2
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_y_action_union_instance;

union receiveUnion3
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_data4_union_instance;

union receiveUnion4
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_data5_union_instance;

union receiveUnion5
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_data6_union_instance;

union receiveUnion6
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_data7_union_instance;

union receiveUnion7
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_data8_union_instance;

union receiveUnion8
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_data9_union_instance;

union receiveUnion9
{
	float data;
	unsigned char tmp_array[4];
} Stm32ToRos_data10_union_instance;


union sendUnion0
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data1_union_instance;

union sendUnion1
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data2_union_instance;

union sendUnion2
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data3_union_instance;

union sendUnion3
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data4_union_instance;

union sendUnion4
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data5_union_instance;

union sendUnion5
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data6_union_instance;

union sendUnion6
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data7_union_instance;

union sendUnion7
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data8_union_instance;

union sendUnion8
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data9_union_instance;

union sendUnion9
{
	float data;
	unsigned char tmp_array[4];
} RosToStm32_data10_union_instance;

boost::asio::io_service boost_io_service;
boost::asio::serial_port boost_serial_point = boost::asio::serial_port(boost_io_service, "/dev/ttyUSB0");
void SerialInit()
{
	boost_serial_point.set_option(serial_port::baud_rate(115200));
	boost_serial_point.set_option(serial_port::flow_control(serial_port::flow_control::none));
	boost_serial_point.set_option(serial_port::parity(serial_port::parity::none));
	boost_serial_point.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
	boost_serial_point.set_option(serial_port::character_size(8));
}

bool ROS_READ_FROM_STM32(float *angle, float *x_action, float *y_action, float *data4, float *data5, float *data6, float *data7, float *data8, float *data9, float *data10)
{
	unsigned char receive_buf[920] = {0};
	unsigned char length;
	unsigned char check_value;
	
	try
	{
		boost::asio::streambuf response;
		boost::system::error_code boost_error_code;
		boost::asio::read_until(boost_serial_point, response, "\r\n",boost_error_code);
		copy(istream_iterator<unsigned char>(istream(&response)>>noskipws),istream_iterator<unsigned char>(),receive_buf);
	}
	catch(boost::system::system_error &err)
	{
		std::cerr << "read_until_error" << std::endl;
	}
	
	for(int i = 0; i < 2; i++)
	{
		if(receive_buf[i] != serial_header[i])
		{
			std::cerr << "read_header_error" << std::endl;
			return false;
		}
	}
	
	length = receive_buf[2];
	
	check_value = serial_get_crc8_value(receive_buf, 42);
	if(check_value != receive_buf[43])
	{
		std::cerr << "check_value_error" << std::endl;
		return false;
	}
	
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_angle_union_instance.tmp_array[i] = receive_buf[i + 3];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_x_action_union_instance.tmp_array[i] = receive_buf[i + 7];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_y_action_union_instance.tmp_array[i] = receive_buf[i + 11];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_data4_union_instance.tmp_array[i] = receive_buf[i + 15];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_data5_union_instance.tmp_array[i] = receive_buf[i + 19];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_data6_union_instance.tmp_array[i] = receive_buf[i + 23];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_data7_union_instance.tmp_array[i] = receive_buf[i + 27];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_data8_union_instance.tmp_array[i] = receive_buf[i + 31];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_data9_union_instance.tmp_array[i] = receive_buf[i + 35];
	}
	for(int i = 0; i < 4; i++)
	{
		Stm32ToRos_data10_union_instance.tmp_array[i] = receive_buf[i + 39];
	}
	
	*angle = Stm32ToRos_angle_union_instance.data;
	*x_action = Stm32ToRos_x_action_union_instance.data;
	*y_action = Stm32ToRos_y_action_union_instance.data;
	*data4 = Stm32ToRos_data4_union_instance.data;
	*data5 = Stm32ToRos_data5_union_instance.data;
	*data6 = Stm32ToRos_data6_union_instance.data;
	*data7 = Stm32ToRos_data7_union_instance.data;
	*data8 = Stm32ToRos_data8_union_instance.data;
	*data9 = Stm32ToRos_data9_union_instance.data;
	*data10 = Stm32ToRos_data10_union_instance.data;
	
	return true;
}
void ROS_WRITE_TO_STM32(float data1, float data2, float data3, float data4, float data5, float data6, float data7, float data8, float data9, float data10)
{
	unsigned char send_buf[46] = {0};
	unsigned char length = 40;
	
	RosToStm32_data1_union_instance.data = data1;
	RosToStm32_data2_union_instance.data = data2;
	RosToStm32_data3_union_instance.data = data3;
	RosToStm32_data4_union_instance.data = data4;
	RosToStm32_data5_union_instance.data = data5;
	RosToStm32_data6_union_instance.data = data6;
	RosToStm32_data7_union_instance.data = data7;
	RosToStm32_data8_union_instance.data = data8;
	RosToStm32_data9_union_instance.data = data9;
	RosToStm32_data10_union_instance.data = data10;
	
	for(int i = 0; i < 2; i++)
	{
		send_buf[i] = serial_header[i];
	}
	send_buf[2] = length;
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 3] = RosToStm32_data1_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 7] = RosToStm32_data2_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 11] = RosToStm32_data3_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 15] = RosToStm32_data4_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 19] = RosToStm32_data5_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 23] = RosToStm32_data6_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 27] = RosToStm32_data7_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 31] = RosToStm32_data8_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 35] = RosToStm32_data9_union_instance.tmp_array[i];
	}
	for(int i = 0; i < 4; i++)
	{
		send_buf[i + 39] = RosToStm32_data10_union_instance.tmp_array[i];
	}
	send_buf[43] = serial_get_crc8_value(send_buf, 42);
	for(int i = 0; i < 2; i++)
	{
		send_buf[i + 44] = serial_ender[i];
	}
	
	boost::asio::write(boost_serial_point, boost::asio::buffer(send_buf));
}
unsigned char serial_get_crc8_value(unsigned char *data, unsigned char len)
{
	unsigned char crc = 0;
	unsigned char i;
	while(len--)
	{
		crc ^= *data++;
		for(i = 0; i < 8; i++)
		{
			if(crc&0x01)
				crc=(crc>>1)^0x8C;
			else
				crc >>= 1;
		}
	}
	return crc;
}
