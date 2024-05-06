# rc_ros_32

this is a project for us to easily use ros without ros_control
quickly move to stm32 to deal with 2023 robocon.

## ROS programmer should:

1.get action data, imu_data from stm32, push cmd_vel, odom data that has been updated to stm32.
2.you should have radar_link,odom and camera_link at least.



## 5.7 更新变化

1.取消了rc_common的依赖

2.本项目中无构建目标，请自行移植到自己的包

3.移植时将include文件夹下的所有文件和src中的communication_ROS.cpp放在你的项目中

4.代码默认ttyusb0，请自行按需要更改