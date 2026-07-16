# hardware

The `imu_node` executable reads an ST ISM330-family IMU over Linux `i2c-dev`
and publishes `sensor_msgs/Imu`. The low-level I2C sensor access is kept in the
ROS-free `hardware::imu::Ism330` class.

The `pixhawk_imu_node` executable converts PX4
`px4_msgs/msg/SensorCombined` IMU samples to `sensor_msgs/Imu`.
![build test](https://buildbot.hippocampus-robotics.net/plugins/badges/hardware-colcon-amd64.svg?left_text=build%20amd64)
![deb](https://buildbot.hippocampus-robotics.net/plugins/badges/hardware-deb-amd64.svg?left_text=deb%20amd64)
![build test](https://buildbot.hippocampus-robotics.net/plugins/badges/hardware-colcon-arm64.svg?left_text=build%20arm64)
![deb](https://buildbot.hippocampus-robotics.net/plugins/badges/hardware-deb-arm64.svg?left_text=deb%20arm64)
