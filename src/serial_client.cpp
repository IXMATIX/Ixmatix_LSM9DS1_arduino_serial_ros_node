#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

/* You need to install ros-kinetic-serial */

#include <sstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <unistd.h>
#include "serial/serial.h"

#include <boost/algorithm/string.hpp>
#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/io.hpp>

#include <math.h>

using namespace std;
using namespace boost::numeric::ublas;

using serial::Serial;

int run(int argc, char ** argv);

int main(int argc, char **argv){
    ros::init(argc, argv, "serial_client_lsm9ds1");
    ros::NodeHandle n("~");

    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 1000);

    ros::Rate loop_rate(30);
    // cout << "hola";

    string port, imu_name, base_name;
    int baud;
    float x_imu_robot, y_imu_robot, z_imu_robot;

    n.param("serial_port", port, string("/dev/ttyUSB0"));
    n.param("imu", imu_name, string("imu_second_floor_link"));
    n.param("base", base_name, string("base_link"));
    n.param("baudrate", baud, 230400);
    n.param("x_imu_robot", x_imu_robot, 0.f);
    n.param("y_imu_robot", y_imu_robot, 0.f);
    n.param("z_imu_robot", z_imu_robot, 0.20f);

    Serial *my_serial = new Serial();
    try {
        my_serial->setPort(port);
        my_serial->setBaudrate((unsigned long) baud);
        serial::Timeout timeout = serial::Timeout::simpleTimeout(1000);
        my_serial->setTimeout(timeout);
        my_serial->setParity(serial::parity_even);
        my_serial->setStopbits(serial::stopbits_one);
        my_serial->setBytesize(serial::sevenbits);
        my_serial->open();
    } catch (exception &e) {
        cerr << "Unhandled Exception: " << e.what() << endl;
    }
    
    ROS_INFO("Opening %s with %d baudrate ", port.c_str(), baud);
    if(!my_serial->isOpen()){
        ROS_ERROR("Serial port is not open. Try to set serial_port param if this is not your device or change baudrate param");
        return -1;
    }
    
    ROS_INFO("Connected!");

    ROS_INFO("%s is (%f, %f, %f) away from %s", imu_name.c_str(), x_imu_robot, y_imu_robot, z_imu_robot, base_name.c_str());
    
    
    string buff;
    double aRes, gRes, mRes;
    boost::numeric::ublas::vector<double> aOff (3), gOff (3), mOff (3);
    matrix<double> mSoftIron(3, 3);
    
    uint msgs_count=0;
    while(my_serial!= NULL && my_serial->isOpen() && ros::ok()){
        if(msgs_count > 30){
            ROS_INFO("Sent IMU messages");
            msgs_count=0;
        }
        buff = my_serial->readline(65536, "\n");
        std::vector<string> strs;
        boost::split(strs, buff, boost::is_any_of(" "));

        if (strs.size() == 17 && !strs[0].compare("Imu:")) {
            sensor_msgs::Imu imu;
            imu.header.frame_id = imu_name;
            imu.header.stamp = ros::Time::now();
            imu.orientation.w = atof(strs[1].c_str());
            imu.orientation.x = atof(strs[2].c_str());
            imu.orientation.y = atof(strs[3].c_str());
            imu.orientation.z = atof(strs[4].c_str());
            imu.angular_velocity.x = ((float)atoi(strs[5].c_str()) * gRes -gOff(0) ) * M_PI / 180.0f;
            imu.angular_velocity.y = ((float)atoi(strs[6].c_str()) * gRes -gOff(1) ) * M_PI / 180.0f;
            imu.angular_velocity.z = ((float)atoi(strs[7].c_str()) * gRes -gOff(2) ) * M_PI / 180.0f;
            imu.angular_velocity_covariance[0] = atof(strs[8].c_str());
            imu.angular_velocity_covariance[4] = atof(strs[9].c_str());
            imu.angular_velocity_covariance[8] = atof(strs[10].c_str());
            imu.linear_acceleration.x = ((float)atoi(strs[11].c_str()) * aRes -aOff(0) ) * 0.00981;
            imu.linear_acceleration.y = ((float)atoi(strs[12].c_str()) * aRes -aOff(0) ) * 0.00981;
            imu.linear_acceleration.z = ((float)atoi(strs[13].c_str()) * aRes -aOff(0) ) * 0.00981;
            imu.linear_acceleration_covariance[0] = atof(strs[14].c_str());
            imu.linear_acceleration_covariance[4] = atof(strs[15].c_str());
            imu.linear_acceleration_covariance[8] = atof(strs[16].c_str());
            imu_pub.publish(imu);

            static tf::TransformBroadcaster tfbroadcaster;
            tf::Transform tf_msg;
            
            tf::Vector3 origin(x_imu_robot, y_imu_robot, z_imu_robot);
            tf::Quaternion q(atof(strs[2].c_str()), atof(strs[3].c_str()), atof(strs[4].c_str()), atof(strs[1].c_str()));

            tf_msg.setOrigin(origin);
            tf_msg.setRotation(q);
            
            tf::StampedTransform s_trans(tf_msg, imu.header.stamp, base_name, imu_name);
            tfbroadcaster.sendTransform(s_trans);

            msgs_count++;
        }

        // Resolutions
        else if (strs.size() == 2 && !strs[0].compare("aRes:")) {
            aRes = atof(strs[1].c_str());
            ROS_INFO("Set acceleration resolution to %f", aRes);
        } else if (strs.size() == 2 && !strs[0].compare("gRes:")) {
            gRes = atof(strs[1].c_str());
            ROS_INFO("Set gyroscope resolution to %f", gRes);
        } else if (strs.size() == 2 && !strs[0].compare("mRes:")) {
            mRes = atof(strs[1].c_str());
            ROS_INFO("Set magnetometer resolution to %f", mRes);
        }

        // Offsets
        else if (strs.size() == 4 && !strs[0].compare("aOff:")) {
            aOff(0) = atof(strs[1].c_str());
            aOff(1) = atof(strs[2].c_str());
            aOff(2) = atof(strs[3].c_str());
            cout << "Accerelometer offset " << aOff << endl;
        } else if (strs.size() == 4 && !strs[0].compare("gOff:")) {
            gOff(0) = atof(strs[1].c_str());
            gOff(1) = atof(strs[2].c_str());
            gOff(2) = atof(strs[3].c_str());
            cout << "Gyroscope offset " << gOff << endl;
        } else if (strs.size() == 13 && !strs[0].compare("mOff:")) {
            mOff(0) = atof(strs[1].c_str());
            mOff(1) = atof(strs[2].c_str());
            mOff(2) = atof(strs[3].c_str());
            int k=4;
            for(int i=0; i<3; i++)
                for(int j=0; j<3; j++)
                    mSoftIron(i, j) = atof(strs[k++].c_str());
            cout << "Magnetometer Hard Iron " << mOff << endl;
            cout << "Magnetometer Soft Iron " << mSoftIron << endl;
        }
    }
    return 0;
}