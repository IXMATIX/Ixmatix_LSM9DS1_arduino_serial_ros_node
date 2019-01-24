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

Serial Port Format
This node reads imu data from serial port with a specific format. You can use [this example](https://github.com/IxmatixRoboticsUniversity/Ixmatix_LSM9DS1_arduino_ros/tree/master/examples/Ixmatix_LSM9DS1_ROS_no_nodehandler) as template for your program.

### Serial port 
#### IMU resolutions and offsets
You have to set resolutions for accelerometer and gyroscope in order to convert raw data to real data. To do this, you have to send a serial port message ending with a `\n` and it has to be defined as follows:
```
aRes: accelerometer_resolution
gRes: gyroscope_resolution
```
**Example:**
```
aRes: 0.00006100
gRes: 0.00875000
```
Similarly, to declare IMU offsets you have to send it as follows:
```
aOff: accel_offset_x accel_offset_y accel_offset_z
gOff: gyros_offset_x gyros_offset_y gyros_offset_z
```
**Example:**
```
aOff: 0.040 -0.100 -0.010
gOff: -0.050 0.350 -0.730
```
#### IMU message definition
Serial Port message has to be ended with a `\n` and has to be defined as follows:
```
Imu: q0 q1 q2 q3 gx gy gz gx_var gy_var gz_var ax ay az ax_var ay_var az_var
```
Name  | Definition
----- | ----------
q0    | w coefficient of quaternion orientation
q1    | x coefficient of quaternion orientation
q2    | y coefficient of quaternion orientation
q3    | z coefficient of quaternion orientation
gx    | gyroscope raw value in x
gy    | gyroscope raw value in y
gz    | gyroscope raw value in z
gx_var| gyroscope covariance in x
gy_var| gyroscope covariance in y
gz_var| gyroscope covariance in z
ax    | accelerometer raw value in x
ay    | accelerometer raw value in y
az    | accelerometer raw value in z
ax_var| accelerometer covariance in x
ay_var| accelerometer covariance in y
az_var| accelerometer covariance in z

**Example**:
```
Imu: 0.8504979 0.5120977 0.0719147 0.0960700 -244 2473 -1172 0.00 0.12 0.53 83 -15494 6300 0.00 0.01 0.00
```
