LSM9DS1 IMU Serial Port ROS Node
================================
Serial Client node in C++ to read IMU data from arduino and embedded systems. It send **transforms** and **sensor_msgs/Imu** messages to ROS.

Parameters
----------
Parameter    | Function     | Default 
------------ | -------------| -------
*serial_port*| Defines the serial port to use | `/dev/ttyUSB0`
*imu*        | Defines the name of the imu for tf_broadcaster           | `imu_link`
*base*       | Defines the name of the base for tf_broadcaster          | `base_link`
*baudrate*   | Defines the Baudrate | `230400`
*x_imu_robot*| Defines the x position of imu with respect to the base   | `0.0`
*y_imu_robot*| Defines the y position of imu with respect to the base   | `0.0`
*z_imu_robot*| Defines the z position of imu with respect to the base   | `0.0`

Examples
--------
The following command has no arguments but default configuration.
```console
foo@bar:~$ rosrun lsm9ds1_imu serial_client
```
If you want to specify a port where your device is sending messages, set imu name to `imu_left_arm` and set its position to `(0,0,0.2)` meters from the base, you can use the next command.
```console
foo@bar:~$ rosrun lsm9ds1_imu serial_client _serial_port:="/dev/ttyACM0" _imu:="imu_left_arm" _z_imu_robot:=.2
```